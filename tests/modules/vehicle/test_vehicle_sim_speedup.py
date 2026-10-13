"""Simulator speedup is a non-blocking scheduler-cadence capability."""
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from navpy.plugins.vehicle.ardupilot.sim_autopilot import (
    SimAutopilotCapability,
    SimAutopilotState,
)
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav


class SimSpeedupTest(unittest.TestCase):
    def _capability(self):
        connection = SimpleNamespace(mav=MagicMock())
        transport = MagicMock()
        transport.call.side_effect = lambda action: action(connection)
        repository = ParameterRepository()
        state = SimAutopilotState()
        capability = SimAutopilotCapability(
            SimpleNamespace(target_system=1),
            transport,
            repository,
            MagicMock(),
            state,
        )
        return capability, repository, state, connection

    @staticmethod
    def _probe_calls(connection):
        return connection.mav.param_request_read_send.call_count

    def test_vehicle_adapter_returns_cached_speedup(self):
        capability, repository, _state, _connection = self._capability()
        repository.update("SIM_SPEEDUP", 10.0)
        with patch(
            "navpy.plugins.vehicle.ardupilot.vehicle_mav.build_vehicle_parts",
            return_value=SimpleNamespace(simulation=capability),
        ):
            vehicle = VehicleMav(
                "unused",
                1,
                wait_heartbeat=False,
                send_heartbeat=False,
                skip_mission_download=True,
            )

        self.assertEqual(vehicle.sim_speedup(), 10.0)

    def test_default_one_and_probes_when_uncached(self):
        capability, _repository, _state, connection = self._capability()

        self.assertEqual(capability.speedup(), 1.0)
        self.assertEqual(self._probe_calls(connection), 1)

    def test_does_not_reprobe_within_poll_interval(self):
        capability, _repository, _state, connection = self._capability()

        with patch(
            "navpy.plugins.vehicle.ardupilot.parameter_client.rt.monotonic",
            return_value=100.0,
        ):
            capability.speedup()
            capability.speedup()

        self.assertEqual(self._probe_calls(connection), 1)

    def test_gives_up_probing_on_real_hardware(self):
        capability, _repository, state, connection = self._capability()

        with patch(
            "navpy.plugins.vehicle.ardupilot.parameter_client.rt.monotonic",
            side_effect=[float(index * 2) for index in range(15)],
        ):
            values = [capability.speedup() for _ in range(15)]

        self.assertEqual(values, [1.0] * 15)
        self.assertEqual(state.snapshot()[2], 10)
        self.assertEqual(self._probe_calls(connection), 10)

    def test_ignores_nonpositive_cached_value(self):
        capability, repository, _state, _connection = self._capability()
        repository.update("SIM_SPEEDUP", 0.0)

        self.assertEqual(capability.speedup(), 1.0)

    def test_ivehicle_default_is_one(self):
        self.assertEqual(IVehicle.sim_speedup(MagicMock()), 1.0)


if __name__ == "__main__":
    unittest.main()
