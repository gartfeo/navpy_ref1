"""Launch AUTO and arm are confirmed and resent until they hold (todo 117, ``radio_budget.md`` round 7)."""
import asyncio
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import AsyncMock, patch

from gcs.backend import launch_commands
from gcs.backend.launch_controller import LaunchController, VehicleState
from tests.gcs.backend.test_launch_controller import _make_mock_client, _prepare_args

RESEND_S = 0.2


@patch.object(launch_commands, "LAUNCH_COMMAND_RESEND_S", RESEND_S)
@patch("gcs.backend.launch_controller.ws_manager")
class TestConfirmedLaunchCommands(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.lc = LaunchController()
        self.lc._altitudes[1] = 0.0
        self.client = _make_mock_client()
        self.auto_calls = []
        self.arm_calls = []

    def _auto_taking_effect_on(self, call: int):
        """AUTO sent; only call number ``call`` reaches the vehicle."""
        def auto(sid):
            self.auto_calls.append(sid)
            mode, count = self.lc._modes.get(sid, (None, 0))
            if len(self.auto_calls) == call:  # its next heartbeat, through the telemetry feed
                self.lc.update_telemetry(sid, self.lc._altitudes[sid], self.lc._armed.get(sid, False),
                                         mode="AUTO", heartbeat_count=count + 1)
            return count
        return auto

    def _arm_taking_effect_on(self, call: int, then=lambda: None):
        def arm(sid):
            self.arm_calls.append(sid)
            if len(self.arm_calls) == call:
                self.lc._armed[sid] = True
                then()
        return arm

    async def _launch(self, mock_ws, **prepare_kw):
        mock_ws.broadcast = AsyncMock()
        kw = dict(sys_ids=[1], settle_s=0.0, altitude_timeout=1.0, arm_timeout=1.5)
        kw.update(prepare_kw)
        if "arm_func" not in kw:
            kw["arm_func"] = self._arm_taking_effect_on(1)
        with patch("gcs.backend.esp32_trigger.Esp32TriggerClient", return_value=self.client):
            await self.lc.prepare(**_prepare_args(self.lc, **kw))
            result = self.client.trigger_channel.return_value

            def release(channel):
                self.lc._altitudes[1] = 15.0  # airborne once released
                return result

            self.client.trigger_channel.side_effect = release
            await self.lc.trigger_vehicle(1)
            await asyncio.gather(*self.lc._tasks.values())

    async def test_a_lost_auto_is_resent_until_the_mode_reads_auto(self, mock_ws):
        await self._launch(mock_ws, raw_auto_func=self._auto_taking_effect_on(3))
        self.assertEqual(self.auto_calls, [1, 1, 1])
        self.client.trigger_channel.assert_called_once()
        self.assertEqual(self.lc._states[1], VehicleState.airborne)

    async def test_a_heartbeat_from_before_the_first_send_does_not_confirm_auto(self, mock_ws):
        self.lc._modes[1] = ("AUTO", 5)  # the vehicle read AUTO before the launch
        await self._launch(mock_ws, raw_auto_func=self._auto_taking_effect_on(0))
        self.assertGreaterEqual(len(self.auto_calls), 3)
        self.assertEqual(self.arm_calls, [])
        self.client.trigger_channel.assert_not_called()
        self.assertEqual(self.lc._states[1], VehicleState.failed)
        self.assertEqual(self.lc._errors[1], "AUTO mode timeout")

    async def test_a_vehicle_not_connected_never_confirms_auto_from_its_cached_mode(self, mock_ws):
        self.lc._modes[1] = ("AUTO", 5)  # cached before the vehicle went away

        def auto(sid):
            self.auto_calls.append(sid)
            return None  # not connected: nothing sent

        await self._launch(mock_ws, raw_auto_func=auto)
        self.assertEqual(self.arm_calls, [])
        self.client.trigger_channel.assert_not_called()
        self.assertEqual(self.lc._errors[1], "AUTO mode timeout")

    async def test_a_lost_arm_is_resent_until_armed(self, mock_ws):
        await self._launch(mock_ws, arm_func=self._arm_taking_effect_on(2))
        self.assertEqual(self.arm_calls, [1, 1])
        self.assertEqual(self.lc._states[1], VehicleState.airborne)

    async def test_an_operator_command_stops_the_resends(self, mock_ws):
        def auto(sid):
            self.auto_calls.append(sid)
            self.lc.revoke_commands([sid])  # the operator acts right after the first send
            return 0
        await self._launch(mock_ws, raw_auto_func=auto)
        self.assertEqual(self.auto_calls, [1])
        self.assertEqual(self.lc._errors[1], "AUTO mode timeout")
        self.client.trigger_channel.assert_not_called()

    async def test_an_operator_command_after_the_arm_blocks_the_release(self, mock_ws):
        await self._launch(mock_ws, arm_func=self._arm_taking_effect_on(1, lambda: self.lc.revoke_commands([1])))
        self.client.trigger_channel.assert_not_called()
        self.assertEqual(self.lc._states[1], VehicleState.failed)
        self.assertEqual(self.lc._errors[1], "Operator command during launch")

    async def test_a_release_queued_behind_busy_workers_is_refused_after_an_operator_command(self, mock_ws):
        loop = asyncio.get_running_loop()
        busy = ThreadPoolExecutor(1)
        self.addCleanup(busy.shutdown)
        freed = threading.Event()
        busy.submit(freed.wait)  # the only worker is busy when the release is queued
        set_state = self.lc._set_state

        def operator_then_free():
            self.lc.revoke_commands([1])
            freed.set()

        async def busy_on_launching(sid, state, error=None):
            await set_state(sid, state, error)
            if state is VehicleState.launching:
                loop.set_default_executor(busy)
                loop.call_later(0.1, operator_then_free)

        self.lc._set_state = busy_on_launching
        await self._launch(mock_ws)
        self.client.trigger_channel.assert_not_called()
        self.assertEqual(self.lc._errors[1], "Operator command during launch")

    async def test_a_release_queued_behind_busy_workers_is_refused_when_the_vehicle_leaves_auto(self, mock_ws):
        loop = asyncio.get_running_loop()
        busy = ThreadPoolExecutor(1)
        self.addCleanup(busy.shutdown)
        freed = threading.Event()
        busy.submit(freed.wait)
        set_state = self.lc._set_state

        def mode_change_then_free():
            _mode, count = self.lc._modes[1]
            self.lc.update_telemetry(1, 0.0, True, mode="MANUAL", heartbeat_count=count + 1)  # RC switch
            freed.set()

        async def busy_on_launching(sid, state, error=None):
            await set_state(sid, state, error)
            if state is VehicleState.launching:
                loop.set_default_executor(busy)
                loop.call_later(0.1, mode_change_then_free)

        self.lc._set_state = busy_on_launching
        await self._launch(mock_ws)
        self.client.trigger_channel.assert_not_called()
        self.assertIn("Not AUTO and armed at release (mode MANUAL", self.lc._errors[1])

    async def test_release_refuses_a_vehicle_no_longer_auto(self, mock_ws):
        def leave_auto():
            _mode, count = self.lc._modes[1]
            self.lc._modes[1] = ("MANUAL", count + 1)
        await self._launch(mock_ws, arm_func=self._arm_taking_effect_on(1, leave_auto))
        self.client.trigger_channel.assert_not_called()
        self.assertIn("Not AUTO and armed at release", self.lc._errors[1])

    async def test_release_refuses_a_vehicle_no_longer_armed(self, mock_ws):
        set_state = self.lc._set_state

        async def disarm_on_launching(sid, state, error=None):
            await set_state(sid, state, error)
            if state is VehicleState.launching:
                self.lc._armed[sid] = False  # disarmed during the launching broadcast

        self.lc._set_state = disarm_on_launching
        await self._launch(mock_ws)
        self.client.trigger_channel.assert_not_called()
        self.assertIn("Not AUTO and armed at release", self.lc._errors[1])

    async def test_the_ekf_is_rechecked_before_every_arm_attempt(self, mock_ws):
        checks = []
        bad = set()

        def recheck(sid):
            checks.append(sid)
            return ["EKF status stale"] if sid in bad else []

        def arm(sid):
            self.arm_calls.append(sid)
            bad.add(sid)  # the EKF turns bad after the first (lost) arm

        await self._launch(mock_ws, arm_func=arm, ekf_recheck=recheck)
        self.assertEqual(self.arm_calls, [1], "an arm was resent past a bad EKF")
        self.assertEqual(len(checks), 2)
        self.assertEqual(self.lc._states[1], VehicleState.failed)
        self.assertIn("EKF status stale", self.lc._errors[1])

    async def test_abort_stops_the_resends(self, mock_ws):
        def auto(sid):
            self.auto_calls.append(sid)
            asyncio.get_running_loop().call_later(RESEND_S / 2, self.lc._abort_event.set)
            return 0
        await self._launch(mock_ws, raw_auto_func=auto)
        self.assertEqual(self.auto_calls, [1])
        self.assertEqual(self.lc._states[1], VehicleState.failed)
        self.assertEqual(self.lc._errors[1], "Aborted")


class TestMissingVehicleTelemetry(unittest.TestCase):
    def test_a_launch_vehicle_without_a_snapshot_reads_no_mode_and_not_armed(self):
        lc = LaunchController()
        for sid in (1, 2):
            lc._states[sid] = VehicleState.arming
            lc.update_telemetry(sid, 0.0, True, mode="AUTO", heartbeat_count=4)
        lc.forget_missing({2})
        self.assertEqual((lc._modes[1], lc._armed[1]), ((None, 4), False))
        self.assertEqual((lc._modes[2], lc._armed[2]), (("AUTO", 4), True))


class TestOperatorCommandsRevokeLaunchCommands(unittest.IsolatedAsyncioTestCase):
    async def test_any_command_route_call_revokes_its_targets(self):
        from gcs.backend.routes import control
        from gcs.backend.models import CommandRequest

        lc = LaunchController()
        with patch.object(control, "launch_controller", lc), \
                patch.dict(control._COMMAND_HANDLERS, {"set_mode": lambda targets, params: {}}):
            await control.send_command(CommandRequest(command="set_mode", sys_ids=[2], params={"mode": "MANUAL"}))
        self.assertEqual(lc._revoked, {2})
