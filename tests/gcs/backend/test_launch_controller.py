"""Tests for the async LaunchController state machine (prepare/trigger API)."""
import asyncio
import time
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from gcs.backend.launch_controller import LaunchController, VehicleState


class TestVehicleStateEnum(unittest.TestCase):
    """VehicleState enum values."""

    def test_all_states(self):
        self.assertEqual(VehicleState.idle.value, "idle")
        self.assertEqual(VehicleState.queued.value, "queued")
        self.assertEqual(VehicleState.arming.value, "arming")
        self.assertEqual(VehicleState.armed.value, "armed")
        self.assertEqual(VehicleState.launching.value, "launching")
        self.assertEqual(VehicleState.airborne.value, "airborne")
        self.assertEqual(VehicleState.failed.value, "failed")


class TestLaunchControllerInit(unittest.TestCase):
    """Initial state of LaunchController."""

    def test_not_running(self):
        lc = LaunchController()
        self.assertFalse(lc.is_running)

    def test_not_prepared(self):
        lc = LaunchController()
        self.assertFalse(lc.is_prepared)

    def test_empty_states(self):
        lc = LaunchController()
        self.assertEqual(lc.get_states(), {})


class TestUpdateTelemetry(unittest.TestCase):
    """Telemetry feed updates internal state."""

    def test_updates_altitude_and_armed(self):
        lc = LaunchController()
        lc.update_telemetry(1, 25.0, True)
        self.assertEqual(lc._altitudes[1], 25.0)
        self.assertTrue(lc._armed[1])

    def test_multiple_vehicles(self):
        lc = LaunchController()
        lc.update_telemetry(1, 10.0, True)
        lc.update_telemetry(2, 0.0, False)
        self.assertEqual(lc._altitudes[1], 10.0)
        self.assertEqual(lc._altitudes[2], 0.0)
        self.assertTrue(lc._armed[1])
        self.assertFalse(lc._armed[2])


def _make_mock_client(trigger_success=True):
    """Create a mock ESP32 client."""
    client = MagicMock()
    client.health_check.return_value = True
    result = MagicMock()
    result.success = trigger_success
    result.message = "ok" if trigger_success else "trigger failed"
    client.trigger_channel.return_value = result
    return client


def _prepare_args(lc, **overrides):
    """Return default prepare kwargs, with optional overrides."""
    defaults = dict(
        sys_ids=[1],
        channel_map={1: 1},
        esp32_host="127.0.0.1",
        esp32_port=80,
        altitude_threshold=10.0,
        arm_timeout=5.0,
        altitude_timeout=30.0,
        arm_func=lambda sid: None,
        auto_func=lambda sid: None,
    )
    defaults.update(overrides)
    return defaults


class TestPrepare(unittest.IsolatedAsyncioTestCase):
    """Tests for prepare() — session initialization."""

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_prepare_sets_all_idle(self, mock_ws):
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(lc, sys_ids=[1, 2], channel_map={1: 1, 2: 2}))

        self.assertTrue(lc.is_prepared)
        self.assertEqual(lc._states[1], VehicleState.idle)
        self.assertEqual(lc._states[2], VehicleState.idle)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_prepare_esp32_unreachable(self, mock_ws):
        """All vehicles fail when ESP32 is unreachable during prepare."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()
        mock_client.health_check.return_value = False

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(lc, sys_ids=[1, 2], channel_map={}))

        self.assertFalse(lc.is_prepared)
        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertEqual(lc._states[2], VehicleState.failed)
        self.assertIn("not reachable", lc._errors.get(1, ""))

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_prepare_idempotent(self, mock_ws):
        """Second prepare() call is a no-op when already prepared."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(lc, sys_ids=[1]))
            await lc.prepare(**_prepare_args(lc, sys_ids=[1, 2]))

        # Still only has vehicle 1 from first prepare call
        self.assertIn(1, lc._states)
        self.assertNotIn(2, lc._states)


class TestTriggerVehicle(unittest.IsolatedAsyncioTestCase):
    """Tests for trigger_vehicle() — per-vehicle launch."""

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_full_launch_sequence(self, mock_ws):
        """Full sequence: auto -> arm -> trigger -> airborne."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        call_order = []

        def arm_func(sid):
            call_order.append(('arm', sid))
            lc._armed[sid] = True

        def auto_func(sid):
            call_order.append(('auto', sid))

        lc._altitudes[1] = 0.0

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], altitude_timeout=2.0,
                arm_func=arm_func, auto_func=auto_func,
            ))
            await lc.trigger_vehicle(1)

            # Simulate altitude rising
            await asyncio.sleep(0.2)
            lc._altitudes[1] = 15.0

            await lc._tasks[1]

        # AUTO must be called before ARM
        self.assertEqual(call_order, [('auto', 1), ('arm', 1)])
        self.assertEqual(lc._states[1], VehicleState.airborne)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_trigger_not_prepared(self, mock_ws):
        """trigger_vehicle raises RuntimeError when not prepared."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        with self.assertRaises(RuntimeError):
            await lc.trigger_vehicle(1)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_trigger_unknown_vehicle(self, mock_ws):
        """trigger_vehicle raises ValueError for unknown sys_id."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(lc, sys_ids=[1]))
            with self.assertRaises(ValueError):
                await lc.trigger_vehicle(99)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_trigger_already_launching(self, mock_ws):
        """trigger_vehicle raises RuntimeError if vehicle already launching."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(lc, sys_ids=[1], arm_timeout=5.0))
            # First trigger — will wait for arm (never confirmed)
            await lc.trigger_vehicle(1)
            # Attempt second trigger while first is active
            with self.assertRaises(RuntimeError):
                await lc.trigger_vehicle(1)
            # Cleanup: abort to stop the background task
            await lc.abort()

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_arm_timeout(self, mock_ws):
        """Vehicle fails if arm doesn't confirm within timeout."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], arm_timeout=0.3,
            ))
            await lc.trigger_vehicle(1)
            await lc._tasks[1]

        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertIn("Arm timeout", lc._errors.get(1, ""))

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_already_airborne_disarmed_arms(self, mock_ws):
        """Vehicles already above threshold but disarmed get armed."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        lc._altitudes[1] = 50.0
        auto_calls = []
        arm_calls = []

        def arm_func(sid):
            arm_calls.append(sid)
            lc._armed[sid] = True

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1],
                arm_func=arm_func,
                auto_func=lambda sid: auto_calls.append(sid),
            ))
            await lc.trigger_vehicle(1)
            await lc._tasks[1]

        self.assertEqual(lc._states[1], VehicleState.airborne)
        self.assertEqual(auto_calls, [1])
        self.assertEqual(arm_calls, [1])
        mock_client.trigger_channel.assert_not_called()

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_already_airborne_armed_skips_rearm(self, mock_ws):
        """Already airborne + already armed skips re-arm."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        lc._altitudes[1] = 50.0
        lc._armed[1] = True
        arm_calls = []

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1],
                arm_func=lambda sid: arm_calls.append(sid),
            ))
            await lc.trigger_vehicle(1)
            await lc._tasks[1]

        self.assertEqual(lc._states[1], VehicleState.airborne)
        self.assertEqual(arm_calls, [])
        mock_client.trigger_channel.assert_not_called()

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_low_threshold_does_not_skip_esp32_when_grounded(self, mock_ws):
        """Regression: a grounded vehicle with a low/zero confirmation threshold
        must still TRIGGER the ESP32 — it must not be falsely treated as already
        airborne (which skipped the trigger and released the lock early)."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()
        lc._armed[1] = True
        lc._altitudes[1] = 0.0  # grounded
        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], altitude_threshold=0.0, altitude_timeout=1.0, settle_s=0.0,
            ))
            await lc.trigger_vehicle(1)
            await lc._tasks[1]
        # ESP32 must have been triggered (not skipped as "already airborne")
        mock_client.trigger_channel.assert_called_once_with(1)
        self.assertEqual(lc._states[1], VehicleState.airborne)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_trigger_failure(self, mock_ws):
        """Vehicle fails when ESP32 trigger fails."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client(trigger_success=False)

        lc._armed[1] = True  # Already armed

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(lc, sys_ids=[1]))
            await lc.trigger_vehicle(1)
            await lc._tasks[1]

        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertIn("Trigger failed", lc._errors.get(1, ""))

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_trigger_failure_aborts_queued_fleet(self, mock_ws):
        """Regression: a failed ESP32 trigger must prevent every later launch trigger."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client(trigger_success=False)

        lc._armed.update({1: True, 2: True})
        lc._altitudes.update({1: 0.0, 2: 0.0})

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc,
                sys_ids=[1, 2],
                channel_map={1: 1, 2: 2},
                altitude_threshold=0.0,
                settle_s=0.0,
            ))
            # The UI queues the fleet immediately; V2 is already waiting when
            # V1 reports that its physical trigger failed.
            await lc.trigger_vehicle(1)
            await lc.trigger_vehicle(2)
            await asyncio.gather(*lc._tasks.values())

        mock_client.trigger_channel.assert_called_once_with(1)
        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertEqual(lc._states[2], VehicleState.failed)
        self.assertIn("Trigger failed", lc._errors[1])
        self.assertEqual(lc._errors[2], "Aborted after V1 failed")
        self.assertFalse(lc.is_prepared)
        event_types = [c.args[0].get("type") for c in mock_ws.broadcast.call_args_list]
        self.assertEqual(event_types.count("launch_aborted"), 1)
        self.assertNotIn("launch_complete", event_types)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_altitude_timeout(self, mock_ws):
        """Vehicle fails when altitude doesn't reach threshold in time."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        lc._armed[1] = True  # Already armed
        lc._altitudes[1] = 0.0  # Stays at 0

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], altitude_timeout=0.3,
            ))
            await lc.trigger_vehicle(1)
            await lc._tasks[1]

        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertIn("Altitude timeout", lc._errors.get(1, ""))

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_not_airborne_aborts_before_next_trigger(self, mock_ws):
        """If V1 never confirms airborne, V2's ESP32 channel is never triggered."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        lc._armed.update({1: True, 2: True})
        lc._altitudes.update({1: 0.0, 2: 0.0})

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc,
                sys_ids=[1, 2],
                channel_map={1: 1, 2: 2},
                altitude_timeout=0.2,
                settle_s=0.0,
            ))
            await lc.trigger_vehicle(1)
            await lc.trigger_vehicle(2)
            await asyncio.gather(*lc._tasks.values())

        mock_client.trigger_channel.assert_called_once_with(1)
        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertEqual(lc._errors[1], "Altitude timeout")
        self.assertEqual(lc._states[2], VehicleState.failed)
        self.assertEqual(lc._errors[2], "Aborted after V1 failed")

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_channel_map_default(self, mock_ws):
        """When channel not in map, sys_id is used as channel."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        lc._armed[5] = True
        lc._altitudes[5] = 0.0

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[5], channel_map={}, altitude_timeout=0.3,
            ))

            await lc.trigger_vehicle(5)
            await asyncio.sleep(0.1)
            lc._altitudes[5] = 15.0
            await lc._tasks[5]

        # Should have called trigger with channel=5 (the sys_id)
        mock_client.trigger_channel.assert_called_once_with(5)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_multiple_vehicles_sequential(self, mock_ws):
        """Two vehicles launch sequentially — V2 queued while V1 runs,
        launch_complete only after both terminal."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        trigger_order = []

        def arm_func(sid):
            lc._armed[sid] = True

        def auto_func(sid):
            trigger_order.append(sid)

        lc._altitudes[1] = 0.0
        lc._altitudes[2] = 0.0

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1, 2], channel_map={1: 1, 2: 2},
                altitude_timeout=2.0, arm_func=arm_func,
                auto_func=auto_func,
            ))

            # Trigger both — V2 should wait for V1
            await lc.trigger_vehicle(1)
            await lc.trigger_vehicle(2)

            # V2 should be queued while V1 holds the lock
            await asyncio.sleep(0.2)
            self.assertEqual(lc._states[2], VehicleState.queued,
                             "V2 should be queued while V1 is active")

            # V1 gains altitude → becomes airborne
            lc._altitudes[1] = 15.0
            await lc._tasks[1]
            self.assertEqual(lc._states[1], VehicleState.airborne)

            # launch_complete must NOT have fired yet (V2 still queued)
            complete_calls = [
                c for c in mock_ws.broadcast.call_args_list
                if c.args[0].get("type") == "launch_complete"
            ]
            self.assertEqual(len(complete_calls), 0,
                             "launch_complete must not fire while V2 is queued")

            # Now V2 should proceed
            await asyncio.sleep(0.2)
            lc._altitudes[2] = 15.0
            await lc._tasks[2]

        self.assertEqual(lc._states[2], VehicleState.airborne)
        # V1 must have started before V2
        self.assertEqual(trigger_order, [1, 2])
        # launch_complete should have fired exactly once after both terminal
        complete_calls = [
            c for c in mock_ws.broadcast.call_args_list
            if c.args[0].get("type") == "launch_complete"
        ]
        self.assertEqual(len(complete_calls), 1)


class TestTimingAndAirborne(unittest.IsolatedAsyncioTestCase):
    """Step 3: settle / stagger / require_armed."""

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_prepare_stores_timing_params(self, mock_ws):
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()
        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], settle_s=0.5, stagger_s=1.0, require_armed=True,
            ))
        self.assertEqual(lc._settle_s, 0.5)
        self.assertEqual(lc._stagger_s, 1.0)
        self.assertTrue(lc._require_armed)

    def test_is_airborne_predicate(self):
        lc = LaunchController()
        lc._altitude_threshold = 10.0
        # Below threshold → never airborne
        lc._altitudes[1] = 5.0
        self.assertFalse(lc._is_airborne(1))
        # Above threshold, armed not required → airborne
        lc._altitudes[1] = 15.0
        lc._require_armed = False
        self.assertTrue(lc._is_airborne(1))
        # Above threshold, armed required but disarmed → not airborne
        lc._require_armed = True
        lc._armed[1] = False
        self.assertFalse(lc._is_airborne(1))
        # Above threshold, armed required and armed → airborne
        lc._armed[1] = True
        self.assertTrue(lc._is_airborne(1))

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_require_armed_airborne_happy_path(self, mock_ws):
        """With require_armed, an armed vehicle above threshold goes airborne.

        (The 'disarmed blocks airborne' case is covered deterministically by
        test_is_airborne_predicate; gating it through the full sequence races
        with the arm step.)
        """
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        def arm_func(sid):
            lc._armed[sid] = True

        lc._altitudes[1] = 0.0
        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], altitude_timeout=2.0, require_armed=True,
                arm_func=arm_func,
            ))
            await lc.trigger_vehicle(1)
            await asyncio.sleep(0.15)
            lc._altitudes[1] = 25.0  # armed (by arm_func) AND above threshold
            await lc._tasks[1]
        self.assertEqual(lc._states[1], VehicleState.airborne)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_require_armed_altitude_alone_times_out(self, mock_ws):
        """require_armed + altitude up but never armed → altitude timeout.

        Drives the airborne wait directly (armed stays False the whole time)
        to avoid racing the arm step.
        """
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        lc._altitude_threshold = 10.0
        lc._require_armed = True
        lc._altitudes[1] = 25.0   # above threshold
        lc._armed[1] = False      # but not armed
        # Predicate must refuse to call it airborne.
        self.assertFalse(lc._is_airborne(1))
        # And the abort-aware poll times out rather than succeeding.
        result = await lc._poll(lambda: lc._is_airborne(1), timeout=0.2)
        self.assertFalse(result)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_sleep_or_abort_returns_early(self, mock_ws):
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        lc._abort_event.set()
        start = asyncio.get_event_loop().time()
        await lc._sleep_or_abort(5.0)
        elapsed = asyncio.get_event_loop().time() - start
        self.assertLess(elapsed, 1.0)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_abort_during_settle_is_prompt(self, mock_ws):
        """A long settle must not hold up an abort (settle is abort-aware)."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()
        lc._altitudes[1] = 0.0
        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(lc, sys_ids=[1], settle_s=5.0))
            await lc.trigger_vehicle(1)
            await asyncio.sleep(0.1)  # now inside the settle sleep
            start = asyncio.get_event_loop().time()
            await lc.abort()
            elapsed = asyncio.get_event_loop().time() - start
        self.assertLess(elapsed, 1.0)  # didn't wait out the 5s settle
        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertEqual(lc._errors.get(1), "Aborted")
        # ESP32 never triggered (aborted before trigger)
        mock_client.trigger_channel.assert_not_called()

    def test_has_pending_vehicle(self):
        lc = LaunchController()
        lc._states = {1: VehicleState.airborne}
        self.assertFalse(lc._has_pending_vehicle())
        lc._states = {1: VehicleState.airborne, 2: VehicleState.queued}
        self.assertTrue(lc._has_pending_vehicle())
        lc._states = {1: VehicleState.idle}
        self.assertTrue(lc._has_pending_vehicle())
        lc._states = {1: VehicleState.failed, 2: VehicleState.airborne}
        self.assertFalse(lc._has_pending_vehicle())

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_climb_confirmation_end_to_end(self, mock_ws):
        """With climb confirmation on, vehicle goes airborne only after a
        sustained post-trigger climb."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()
        lc._armed[1] = True   # arm step passes instantly
        lc._altitudes[1] = 0.0
        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], settle_s=0.0, altitude_timeout=3.0,
                min_climb_rate_ms=1.0, climb_confirm_s=0.2,
            ))
            await lc.trigger_vehicle(1)
            await asyncio.sleep(0.1)  # past trigger + post-trigger climb-window reset
            # Now climbing above altitude → climb window starts fresh post-trigger
            lc.update_telemetry(1, 15.0, True, 2.0)
            await lc._tasks[1]
        self.assertEqual(lc._states[1], VehicleState.airborne)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_settle_pause_applied(self, mock_ws):
        """A non-zero settle delays ARM after AUTO without breaking the sequence."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()
        order = []

        def auto_func(sid):
            order.append(('auto', asyncio.get_event_loop().time()))

        def arm_func(sid):
            order.append(('arm', asyncio.get_event_loop().time()))
            lc._armed[sid] = True

        lc._altitudes[1] = 0.0
        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], altitude_timeout=2.0, settle_s=0.3,
                auto_func=auto_func, arm_func=arm_func,
            ))
            await lc.trigger_vehicle(1)
            await asyncio.sleep(0.15)
            lc._altitudes[1] = 15.0
            await lc._tasks[1]
        self.assertEqual([o[0] for o in order], ['auto', 'arm'])
        # ARM happened at least ~settle after AUTO
        self.assertGreaterEqual(order[1][1] - order[0][1], 0.25)
        self.assertEqual(lc._states[1], VehicleState.airborne)


class TestClimbConfirmation(unittest.TestCase):
    """Step 4: sustained-climb confirmation for airborne detection."""

    def test_update_telemetry_stores_climb(self):
        lc = LaunchController()
        lc.update_telemetry(1, 5.0, True, 2.5)
        self.assertEqual(lc._climb_rates[1], 2.5)

    def test_update_telemetry_climb_defaults_zero(self):
        """Back-compat: callers without climb still work."""
        lc = LaunchController()
        lc.update_telemetry(1, 5.0, True)
        self.assertEqual(lc._climb_rates[1], 0.0)

    def test_climb_ok_since_tracks_threshold(self):
        lc = LaunchController()
        lc._min_climb_rate_ms = 1.0
        lc.update_telemetry(1, 0.0, False, 0.5)   # below → not tracking
        self.assertIsNone(lc._climb_ok_since.get(1))
        lc.update_telemetry(1, 0.0, False, 1.5)   # above → start tracking
        self.assertIsNotNone(lc._climb_ok_since.get(1))
        lc.update_telemetry(1, 0.0, False, 0.2)   # drop → reset
        self.assertIsNone(lc._climb_ok_since.get(1))

    def test_update_telemetry_stores_throttle(self):
        lc = LaunchController()
        lc.update_telemetry(1, 5.0, True, 0.0, 55.0)
        self.assertEqual(lc._throttles[1], 55.0)

    def test_is_airborne_throttle_gate(self):
        lc = LaunchController()
        lc._altitude_threshold = 10.0
        lc._require_throttle = True
        lc._min_throttle_pct = 20.0
        lc._altitudes[1] = 20.0  # above altitude
        # Throttle below min → not airborne
        lc._throttles[1] = 5.0
        self.assertFalse(lc._is_airborne(1))
        # Throttle at/above min → airborne
        lc._throttles[1] = 40.0
        self.assertTrue(lc._is_airborne(1))

    def test_throttle_gate_disabled_uses_altitude_only(self):
        lc = LaunchController()
        lc._altitude_threshold = 10.0
        lc._require_throttle = False  # disabled
        lc._altitudes[1] = 20.0
        lc._throttles[1] = 0.0
        self.assertTrue(lc._is_airborne(1))

    def test_climb_disabled_does_not_track(self):
        lc = LaunchController()
        lc._min_climb_rate_ms = 0.0  # disabled
        lc.update_telemetry(1, 0.0, False, 5.0)
        self.assertIsNone(lc._climb_ok_since.get(1))

    def test_is_airborne_climb_gate(self):
        lc = LaunchController()
        lc._altitude_threshold = 10.0
        lc._min_climb_rate_ms = 1.0
        lc._climb_confirm_s = 1.0
        lc._altitudes[1] = 20.0  # above altitude
        # Not climbing → not airborne
        lc._climb_ok_since[1] = None
        self.assertFalse(lc._is_airborne(1))
        # Climbing but not long enough → not airborne
        lc._climb_ok_since[1] = time.monotonic()
        self.assertFalse(lc._is_airborne(1))
        # Climbing sustained beyond the confirm window → airborne
        lc._climb_ok_since[1] = time.monotonic() - 2.0
        self.assertTrue(lc._is_airborne(1))

    def test_climb_disabled_uses_altitude_only(self):
        lc = LaunchController()
        lc._altitude_threshold = 10.0
        lc._min_climb_rate_ms = 0.0  # disabled
        lc._altitudes[1] = 20.0
        self.assertTrue(lc._is_airborne(1))

    def test_prepare_resets_climb_state(self):
        lc = LaunchController()
        lc._climb_ok_since[1] = 123.0
        # cleanup also clears it
        lc.cleanup()
        self.assertEqual(lc._climb_ok_since, {})


class TestSequencingOnFailure(unittest.IsolatedAsyncioTestCase):
    """Fail-fast lock behavior when the first vehicle does NOT go airborne."""

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_v2_waits_for_v1_then_aborts_when_v1_fails(self, mock_ws):
        """V2 stays queued during V1's altitude wait and never launches after
        V1 fails to reach altitude."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        def arm_func(sid):
            lc._armed[sid] = True

        lc._altitudes[1] = 0.0
        lc._altitudes[2] = 0.0
        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1, 2], channel_map={1: 1, 2: 2},
                altitude_timeout=0.4, settle_s=0.0, arm_func=arm_func,
            ))
            await lc.trigger_vehicle(1)
            await lc.trigger_vehicle(2)

            # While V1 is in its (doomed) altitude wait, V2 must stay queued.
            await asyncio.sleep(0.15)
            self.assertEqual(lc._states[2], VehicleState.queued,
                             "V2 must not start while V1 holds the lock")
            self.assertEqual(lc._states[1], VehicleState.launching,
                             "V1 should be mid-launch (triggering launch/awaiting altitude)")

            # V1 never climbs → altitude timeout → whole session aborts.
            await lc._tasks[1]
            self.assertEqual(lc._states[1], VehicleState.failed)
            await lc._tasks[2]
        self.assertEqual(lc._states[2], VehicleState.failed)
        self.assertEqual(lc._errors[2], "Aborted after V1 failed")
        mock_client.trigger_channel.assert_called_once_with(1)


class TestAbortAndCleanup(unittest.IsolatedAsyncioTestCase):

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_abort_when_not_running(self, mock_ws):
        """Abort when not running should not raise."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        await lc.abort()

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_abort_cancels_active_tasks(self, mock_ws):
        """Abort stops running vehicle tasks."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], arm_timeout=10.0,
            ))
            # Trigger vehicle — will block waiting for arm
            await lc.trigger_vehicle(1)
            self.assertTrue(lc.is_running)

            # Abort should complete gracefully
            await lc.abort()

        self.assertFalse(lc.is_prepared)
        self.assertFalse(lc.is_running)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_abort_reports_aborted_not_timeout(self, mock_ws):
        """Aborted vehicle shows 'Aborted', not 'Arm timeout'."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1], arm_timeout=10.0,
            ))
            await lc.trigger_vehicle(1)
            await asyncio.sleep(0.2)
            await lc.abort()

        self.assertEqual(lc._states[1], VehicleState.failed)
        self.assertEqual(lc._errors.get(1), "Aborted")

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_abort_emits_aborted_not_complete(self, mock_ws):
        """Abort emits launch_aborted, never launch_complete."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        mock_client = _make_mock_client()

        with patch("navpy.modules.comm.esp32_trigger.Esp32TriggerClient", return_value=mock_client):
            await lc.prepare(**_prepare_args(
                lc, sys_ids=[1, 2], channel_map={1: 1, 2: 2},
                arm_timeout=10.0,
            ))
            await lc.trigger_vehicle(1)
            await lc.trigger_vehicle(2)
            await asyncio.sleep(0.2)
            await lc.abort()

        # Both should be "Aborted"
        self.assertEqual(lc._errors.get(1), "Aborted")
        self.assertEqual(lc._errors.get(2), "Aborted")
        # launch_aborted must fire, launch_complete must NOT
        event_types = [
            c.args[0].get("type") for c in mock_ws.broadcast.call_args_list
            if c.args[0].get("type") in ("launch_complete", "launch_aborted")
        ]
        self.assertIn("launch_aborted", event_types)
        self.assertNotIn("launch_complete", event_types)

    @patch("gcs.backend.launch_controller.ws_manager")
    async def test_get_states(self, mock_ws):
        """get_states returns per-vehicle state dict."""
        mock_ws.broadcast = AsyncMock()
        lc = LaunchController()
        lc._states[1] = VehicleState.armed
        lc._states[2] = VehicleState.failed
        lc._errors[2] = "test error"

        states = lc.get_states()
        self.assertEqual(states[1], {"state": "armed", "error": None})
        self.assertEqual(states[2], {"state": "failed", "error": "test error"})

    def test_cleanup(self):
        """cleanup() resets all state."""
        lc = LaunchController()
        lc._states[1] = VehicleState.airborne
        lc._errors[1] = "some error"
        lc._prepared = True

        lc.cleanup()

        self.assertEqual(lc._states, {})
        self.assertEqual(lc._errors, {})
        self.assertFalse(lc.is_prepared)
        self.assertIsNone(lc._esp32_client)


if __name__ == "__main__":
    unittest.main()
