"""Tests for container-boundary launch spacing in LaunchController.

The controller applies the small inter-vehicle stagger within a container and
the larger container gap at container boundaries (last UAV of one container,
before the first of the next). Vehicles are grouped into containers by
ascending sys_id.
"""
from gcs.backend.launch_controller import (
    LaunchController, VehicleState, _container_map, UAVS_PER_CONTAINER,
)


class TestContainerMap:
    def test_default_group_size(self):
        assert UAVS_PER_CONTAINER == 3

    def test_groups_by_ascending_sys_id(self):
        # sorted: [1,3,5,7,9,11,12] -> containers 0,0,0,1,1,1,2
        m = _container_map([12, 3, 7, 1, 9, 5, 11], 3)
        assert m == {1: 0, 3: 0, 5: 0, 7: 1, 9: 1, 11: 1, 12: 2}

    def test_disabled_when_size_zero(self):
        assert _container_map([1, 2, 3], 0) == {}


def _controller(container_of, states, stagger=1.0, gap=5.0):
    c = LaunchController()
    c._container_of = container_of
    c._states = states
    c._stagger_s = stagger
    c._container_gap_s = gap
    return c


class TestPostVehicleDelay:
    # container_of: sys 1,2,3 -> container 0 ; sys 4,5,6 -> container 1
    C = {1: 0, 2: 0, 3: 0, 4: 1, 5: 1, 6: 1}

    def test_no_map_uses_stagger(self):
        c = _controller({}, {1: VehicleState.airborne, 2: VehicleState.idle})
        assert c._post_vehicle_delay(1) == c._stagger_s

    def test_within_container_uses_stagger(self):
        # V1 finished; V2 (same container) still pending -> stagger
        states = {1: VehicleState.airborne, 2: VehicleState.idle,
                  3: VehicleState.idle, 4: VehicleState.idle,
                  5: VehicleState.idle, 6: VehicleState.idle}
        c = _controller(self.C, states)
        assert c._post_vehicle_delay(1) == c._stagger_s

    def test_container_boundary_uses_gap(self):
        # V1,V2 airborne; V3 (last of container 0) just finished; container 1
        # (V4,V5,V6) still pending -> gap
        states = {1: VehicleState.airborne, 2: VehicleState.airborne,
                  3: VehicleState.airborne, 4: VehicleState.idle,
                  5: VehicleState.idle, 6: VehicleState.idle}
        c = _controller(self.C, states)
        assert c._post_vehicle_delay(3) == c._container_gap_s

    def test_last_container_no_later_uses_stagger(self):
        # V6 is the last vehicle overall; nothing pending after -> stagger
        # (the caller also gates on _has_pending_vehicle, so no delay applies)
        states = {1: VehicleState.airborne, 2: VehicleState.airborne,
                  3: VehicleState.airborne, 4: VehicleState.airborne,
                  5: VehicleState.airborne, 6: VehicleState.airborne}
        c = _controller(self.C, states)
        assert c._post_vehicle_delay(6) == c._stagger_s

    def test_boundary_gap_zero_falls_back_to_no_delay(self):
        # If the operator left the gap at 0, a boundary yields 0 (no extra wait).
        states = {1: VehicleState.airborne, 2: VehicleState.airborne,
                  3: VehicleState.airborne, 4: VehicleState.idle,
                  5: VehicleState.idle, 6: VehicleState.idle}
        c = _controller(self.C, states, stagger=0.5, gap=0.0)
        assert c._post_vehicle_delay(3) == 0.0
