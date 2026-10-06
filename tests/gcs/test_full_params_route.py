"""HTTP route tests for `/api/vehicles/{sys_id}/parameters`."""
from __future__ import annotations

import struct
import time
import unittest
from unittest.mock import MagicMock, patch

from fastapi import FastAPI, WebSocket
from fastapi.testclient import TestClient

from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.full_params import FullParamCache, full_param_cache
from gcs.backend.routes import full_params as full_params_route
from navpy.modules.vehicle._mavftp.param_pck import (
    AP_TYPE_FLOAT, FLAG_HAS_DEFAULT, PCK_MAGIC, parse_param_pck,
)
from navpy.modules.vehicle.full_param_snapshot import FullParamSnapshot


def _build_snapshot(target_system=42) -> FullParamSnapshot:
    rec_a = bytes([(0 << 4) | 1, (3 - 1) << 4 | 0]) + b"AAA" + struct.pack("<b", 5)
    rec_b = bytes([(FLAG_HAS_DEFAULT << 4) | AP_TYPE_FLOAT, (1 - 1) << 4 | 2]) \
        + b"B" + struct.pack("<f", 1.5) + struct.pack("<f", 0.0)
    blob = struct.pack("<HHH", PCK_MAGIC, 2, 2) + rec_a + rec_b
    return FullParamSnapshot(
        target_system=target_system,
        fetched_at_unix_s=time.time(),
        with_defaults=True,
        pck=parse_param_pck(blob, defaults_requested=True),
    )


def _make_app(cache: FullParamCache) -> FastAPI:
    app = FastAPI()
    app.include_router(full_params_route.router, prefix="/api/vehicles")
    return app


class _RouteTest(unittest.TestCase):
    """Shared setup: patch the singleton cache + the vehicle manager."""

    def setUp(self):
        self.cache = FullParamCache()
        # Replace the module-level singleton in the route handler.
        cache_patcher = patch.object(
            full_params_route, "full_param_cache", self.cache)
        cache_patcher.start()
        self.addCleanup(cache_patcher.stop)
        # Fake vehicle manager.
        self.vehicle = MagicMock()
        self.vehicle.target_system = 42
        self.vehicle.is_armed = False
        self.vehicle._callbacks = {}
        self.vehicle.on_message.side_effect = (
            lambda name, cb:
            self.vehicle._callbacks.setdefault(name, []).append(cb))
        self.vehicle.fetch_full_param_snapshot.return_value = _build_snapshot(42)

        self.entry = MagicMock()
        self.entry.vehicle = self.vehicle
        self.entry.device = "dev:A"

        vm_patcher = patch.object(
            full_params_route, "vehicle_mgr",
            MagicMock(get_vehicle=MagicMock(return_value=self.entry)),
        )
        vm_patcher.start()
        self.addCleanup(vm_patcher.stop)

        self.app = _make_app(self.cache)
        self.client = TestClient(self.app)


class TestGetParameters(_RouteTest):
    def test_get_returns_snapshot(self):
        r = self.client.get("/api/vehicles/42/parameters")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertEqual(body["sys_id"], 42)
        self.assertEqual(body["num_params"], 2)
        names = [p["name"] for p in body["params"]]
        self.assertIn("AAA", names)
        self.assertIn("AAB", names)
        # Every record carries a read_only flag for the editor to gate on.
        self.assertTrue(all("read_only" in p for p in body["params"]))

    def test_get_404_when_vehicle_missing(self):
        full_params_route.vehicle_mgr.get_vehicle.return_value = None
        r = self.client.get("/api/vehicles/42/parameters")
        self.assertEqual(r.status_code, 404)

    def test_get_then_get_serves_from_cache(self):
        self.client.get("/api/vehicles/42/parameters")
        self.client.get("/api/vehicles/42/parameters")
        self.assertEqual(self.vehicle.fetch_full_param_snapshot.call_count, 1)

    def test_refresh_forces_fetch(self):
        self.client.get("/api/vehicles/42/parameters")
        self.client.get("/api/vehicles/42/parameters?refresh=true")
        self.assertEqual(self.vehicle.fetch_full_param_snapshot.call_count, 2)

    def test_get_wires_progress_callback(self):
        self.client.get("/api/vehicles/42/parameters")
        _, kwargs = self.vehicle.fetch_full_param_snapshot.call_args
        self.assertIn("progress_callback", kwargs)
        self.assertTrue(callable(kwargs["progress_callback"]))


class TestPutParameters(_RouteTest):
    def test_put_writes_each_change(self):
        # Pre-populate cache so write_batch can derive mav types.
        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.set_parameter.return_value = True
        r = self.client.put(
            "/api/vehicles/42/parameters",
            json={"changes": [{"name": "AAB", "value": 2.0}]},
        )
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertTrue(body["snapshot_stale"])
        self.assertTrue(body["results"]["AAB"]["ok"])

    def test_put_with_ws_client_broadcasts_progress(self):
        # The other PUT tests run with NO WS client, so `ws_manager.broadcast`
        # early-returns and the write-progress path is never exercised. Connect
        # a real WS client so the broadcast actually sends, guarding the live
        # write path end-to-end (regression for the per-param write events).
        self.addCleanup(ws_manager._clients.clear)
        app = FastAPI()
        app.include_router(full_params_route.router, prefix="/api/vehicles")

        @app.websocket("/ws/telemetry")
        async def _ws(ws: WebSocket):  # noqa: ANN202
            await ws_manager.connect(ws)
            try:
                while True:
                    await ws.receive_text()
            except Exception:
                await ws_manager.disconnect(ws)

        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.set_parameter.return_value = True

        client = TestClient(app)
        with client.websocket_connect("/ws/telemetry") as wsc:
            r = client.put(
                "/api/vehicles/42/parameters",
                json={"changes": [{"name": "AAB", "value": 2.0},
                                  {"name": "AAA", "value": 6}]},
            )
            self.assertEqual(r.status_code, 200)
            # The PUT completed, so all progress events are buffered. Drain
            # until the terminal `done` event.
            events = []
            while True:
                # broadcast() sends binary frames (send_bytes).
                msg = wsc.receive_json(mode="binary")
                if msg.get("type") == "full_param_write_progress":
                    events.append(msg)
                    if msg.get("done"):
                        break
        self.assertTrue(events)
        self.assertTrue(all(e["sys_id"] == 42 and e["total"] == 2 for e in events))
        self.assertEqual(
            events[-1],
            {"type": "full_param_write_progress",
             "sys_id": 42, "written": 2, "total": 2, "done": True},
        )

    def test_put_fractional_int_is_coerced_to_stored_value(self):
        # Raw PUT of 2.1 to int8 AAA: the boundary stages 2, reports value 2,
        # and never surfaces a misleading echo error.
        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.set_parameter.return_value = True
        r = self.client.put(
            "/api/vehicles/42/parameters",
            json={"changes": [{"name": "AAA", "value": 2.1}]},
        )
        self.assertEqual(r.status_code, 200)
        cell = r.json()["results"]["AAA"]
        self.assertTrue(cell["ok"])
        self.assertEqual(cell["value"], 2)
        self.assertEqual(self.vehicle.set_parameter.call_args.args[1], 2)

    def test_put_then_put_uses_stale_metadata_without_refresh(self):
        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.set_parameter.return_value = True

        first = self.client.put(
            "/api/vehicles/42/parameters",
            json={"changes": [{"name": "AAB", "value": 2.0}]},
        )
        second = self.client.put(
            "/api/vehicles/42/parameters",
            json={"changes": [{"name": "AAB", "value": 3.0}]},
        )

        self.assertEqual(first.status_code, 200)
        self.assertEqual(second.status_code, 200)
        self.assertTrue(self.cache.is_stale(42))
        self.assertEqual(self.vehicle.set_parameter.call_count, 2)

    def test_get_after_put_refetches_stale_snapshot(self):
        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.set_parameter.return_value = True
        self.client.put(
            "/api/vehicles/42/parameters",
            json={"changes": [{"name": "AAB", "value": 2.0}]},
        )
        self.assertTrue(self.cache.is_stale(42))

        r = self.client.get("/api/vehicles/42/parameters")

        self.assertEqual(r.status_code, 200)
        self.assertFalse(self.cache.is_stale(42))
        self.vehicle.fetch_full_param_snapshot.assert_called_once()

    def test_put_armed_without_token_409(self):
        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.is_armed = True
        with self.assertLogs(full_params_route.log, level="WARNING") as logs:
            r = self.client.put(
                "/api/vehicles/42/parameters",
                json={"changes": [{"name": "AAB", "value": 2.0}]},
            )
        self.assertEqual(r.status_code, 409)
        text = "\n".join(logs.output)
        self.assertIn("status=409", text)
        self.assertIn("AAB", text)
        self.assertNotIn("2.0", text)

    def test_put_armed_with_token_200(self):
        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.is_armed = True
        self.vehicle.set_parameter.return_value = True
        token_resp = self.client.post("/api/vehicles/42/parameters/arm-token")
        nonce = token_resp.json()["nonce"]
        r = self.client.put(
            "/api/vehicles/42/parameters",
            json={
                "changes": [{"name": "AAB", "value": 2.0}],
                "armed_token": nonce,
            },
        )
        self.assertEqual(r.status_code, 200)

    def test_put_no_snapshot_400(self):
        # No snapshot in cache → can't derive mav_param_type → 400.
        self.vehicle.is_armed = False
        with self.assertLogs(full_params_route.log, level="WARNING") as logs:
            r = self.client.put(
                "/api/vehicles/42/parameters",
                json={"changes": [{"name": "AAB", "value": 2.0}]},
            )
        self.assertEqual(r.status_code, 400)
        text = "\n".join(logs.output)
        self.assertIn("status=400", text)
        self.assertIn("AAB", text)
        self.assertNotIn("2.0", text)

    def test_put_unknown_param_in_cache_returns_per_cell_error(self):
        self.cache._snapshots[42] = _build_snapshot(42)
        self.vehicle.set_parameter.return_value = True
        r = self.client.put(
            "/api/vehicles/42/parameters",
            json={"changes": [{"name": "DOES_NOT_EXIST", "value": 1}]},
        )
        self.assertEqual(r.status_code, 200)
        self.assertFalse(r.json()["results"]["DOES_NOT_EXIST"]["ok"])

    def test_delete_cache_then_put_returns_400(self):
        self.cache._snapshots[42] = _build_snapshot(42)
        self.client.delete("/api/vehicles/42/parameters/cache")

        r = self.client.put(
            "/api/vehicles/42/parameters",
            json={"changes": [{"name": "AAB", "value": 2.0}]},
        )

        self.assertEqual(r.status_code, 400)


class TestArmTokenRoute(_RouteTest):
    def test_arm_token_returns_nonce(self):
        r = self.client.post("/api/vehicles/42/parameters/arm-token")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertTrue(body["nonce"])
        self.assertGreater(body["expires_at_unix_s"], time.time())


class TestDeleteCacheRoute(_RouteTest):
    def test_delete_invalidates_cache(self):
        self.cache._snapshots[42] = _build_snapshot(42)
        r = self.client.delete("/api/vehicles/42/parameters/cache")
        self.assertEqual(r.status_code, 200)
        self.assertIsNone(self.cache.peek(42))


if __name__ == "__main__":
    unittest.main()
