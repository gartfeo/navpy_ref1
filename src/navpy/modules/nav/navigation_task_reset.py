"""Navigation-task state and resource teardown transactions."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from dataclasses import dataclass
from typing import Callable, Optional

from navpy.args.nav_args import NavArgs
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.navigation_speedup import NavigationSpeedupLease
from navpy.modules.nav.mission_catalog import MissionCatalog
from navpy.modules.nav.nav_constants import PEER_APPROACH_DIST
from navpy.modules.nav.nav_state import (
    ConfirmGateState,
    NavigationTaskState,
    GeoHoldState,
    NavigationFailureLatch,
)
from navpy.modules.nav.pass_tracker import LegacyPassTracker
from navpy.modules.nav.confirmation_manager import ConfirmationManager
from navpy.modules.nav.target_retry import TargetRetryState
from navpy.modules.nav.vehicle_navigation import LoiterRadiusLease
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detector_ports import (
    DetectorResetPort,
    GeoPointingPort,
    TrackingCommandPort,
)


def _collect_cleanup_errors(
    steps: tuple[tuple[str, Callable[[], object]], ...],
) -> list[Exception]:
    errors: list[Exception] = []
    for _label, action in steps:
        try:
            action()
        except Exception as error:  # noqa: BLE001 - preserve all teardown failures
            errors.append(error)
    return errors


def _run_cleanup_steps(
    context: str,
    steps: tuple[tuple[str, Callable[[], object]], ...],
) -> None:
    errors = _collect_cleanup_errors(steps)
    if errors:
        raise ExceptionGroup(context, errors)


class NavigationTaskStateReset:
    """Clear only authoritative mutable owners for a fresh acquisition."""

    def __init__(
        self,
        navigation_task: NavigationTaskState,
        navigation_failures: NavigationFailureLatch,
        geo_hold: GeoHoldState,
        confirm: ConfirmGateState,
        detections: DetectionSnapshot,
        mission: MissionCatalog,
        retry: TargetRetryState,
        pass_tracker: LegacyPassTracker,
    ) -> None:
        self._navigation_task = navigation_task
        self._navigation_failures = navigation_failures
        self._geo_hold = geo_hold
        self._confirm = confirm
        self._detections = detections
        self._mission = mission
        self._retry = retry
        self._pass_tracker = pass_tracker

    def clear(self) -> None:
        self._navigation_task.nav_mode_observed = False
        self._navigation_task.guided_last_attempt_at = None
        self._navigation_task.guided_request_started_at = None
        # NAV exit consumes terminal navigation task/completion before the next
        # NAV entry resets them.  Teardown must not erase that evidence.
        self._navigation_task.peer_navigation = False
        self._navigation_task.peer_approach_distance_m = PEER_APPROACH_DIST
        self._navigation_task.orbit_radius_m = 0.0
        self._navigation_task.orbit_approach_alt_rel_m = None
        self._navigation_task.peer_target_location = None
        self._navigation_task.navigation_target_location = None
        self._navigation_failures.reset()
        self._confirm.hold_active = False
        self._confirm.loss_started_at = None
        self._confirm.review_started_at = None
        self._geo_hold.active = False
        self._geo_hold.target_location = None
        self._geo_hold.last_own_target_geo = None
        self._geo_hold.acquisition_log_bucket = None
        _run_cleanup_steps(
            "navigation task state reset failed",
            (
                ("mission", self._mission.clear_navigation_task),
                ("detections", self._detections.clear),
                ("retry", self._retry.full_reset),
                ("pass tracker", self._pass_tracker.reset),
            ),
        )


@dataclass(frozen=True)
class NavigationTaskResourcePorts:
    reset_peer_dispatch: Callable[[], None]
    reset_task_actor: Callable[[], None]
    refresh_mission: Callable[[], None]


class NavigationTaskResourceReset:
    """Release side-effecting resources after state transitions."""

    def __init__(
        self,
        ports: NavigationTaskResourcePorts,
        tracking: TrackingCommandPort,
        geo_pointing: GeoPointingPort,
        detector_reset: DetectorResetPort,
        confirmation_manager: ConfirmationManager,
        loiter_radius: LoiterRadiusLease,
        speedup: NavigationSpeedupLease,
        args: NavArgs,
    ) -> None:
        self._ports = ports
        self._tracking = tracking
        self._geo_pointing = geo_pointing
        self._detector_reset = detector_reset
        self._confirmation_manager = confirmation_manager
        self._loiter_radius = loiter_radius
        self._speedup = speedup
        self._args = args

    def clear(self) -> None:
        errors = _collect_cleanup_errors(
            (
                ("tracking", self._tracking.stop_tracking),
                ("geo pointing", self._geo_pointing.stop_geo_tracking),
                ("loiter radius", self._loiter_radius.restore),
                ("SIM_SPEEDUP", self._restore_speedup),
            )
        )
        peer_quiesced = True
        try:
            self._ports.reset_peer_dispatch()
        except Exception as error:  # noqa: BLE001 - retain dependency ownership
            errors.append(error)
            peer_quiesced = False

        post_fence_steps = (
            (
                ("target manager", self._confirmation_manager.reset),
                ("detector", self._detector_reset.refresh),
                ("task actor", self._ports.reset_task_actor),
            )
            if peer_quiesced
            else (("detector", self._detector_reset.refresh),)
        )
        errors.extend(_collect_cleanup_errors(
            (
                *post_fence_steps,
                ("arguments", self._args.refresh),
                ("mission", self._ports.refresh_mission),
            )
        ))
        if errors:
            raise ExceptionGroup("navigation task resource reset failed", errors)

    def _restore_speedup(self) -> None:
        if not self._speedup.restore(log=False):
            raise RuntimeError("SIM_SPEEDUP rollback remains unverified")


class NavigationTaskResetTransaction:
    def __init__(
        self,
        state: NavigationTaskStateReset,
        resources: NavigationTaskResourceReset,
        close_terminal_source: Callable[[], None],
    ) -> None:
        self._state = state
        self._resources = resources
        self._close_terminal_source = close_terminal_source

    def clear(self) -> None:
        try:
            self._close_terminal_source()
        except Exception as error:
            raise ExceptionGroup(
                "navigation task source fence failed",
                [error],
            ) from None
        _run_cleanup_steps(
            "navigation task reset transaction failed",
            (
                ("resources", self._resources.clear),
                ("state", self._state.clear),
            ),
        )


@dataclass(frozen=True)
class AutoMissionResumePorts:
    close_terminal_source: Callable[[], None]
    pause_navigation: Callable[[], None]
    clear_selected_target: Callable[[], None]


class AutoMissionResume:
    """Resume the interrupted search leg without resetting detector identity."""

    def __init__(
        self,
        ports: AutoMissionResumePorts,
        vehicle: IVehicle,
        geo_pointing: GeoPointingPort,
        navigation_task: NavigationTaskState,
        geo_hold: GeoHoldState,
        confirm: ConfirmGateState,
        detections: DetectionSnapshot,
        mission: MissionCatalog,
        pass_tracker: LegacyPassTracker,
        loiter_radius: LoiterRadiusLease,
    ) -> None:
        self._ports = ports
        self._vehicle = vehicle
        self._geo_pointing = geo_pointing
        self._navigation_task = navigation_task
        self._geo_hold = geo_hold
        self._confirm = confirm
        self._detections = detections
        self._mission = mission
        self._pass_tracker = pass_tracker
        self._loiter_radius = loiter_radius

    def run(self) -> None:
        self._ports.close_terminal_source()
        resume_waypoint = self._vehicle.mission_items_next
        if resume_waypoint is not None:
            resume_waypoint = max(resume_waypoint - 1, 0)
        self._ports.pause_navigation()
        self._geo_pointing.stop_geo_tracking()
        self._loiter_radius.restore()
        if self._vehicle.get_mode != FlightMode.AUTO:
            self._vehicle.set_mode(FlightMode.AUTO)
        if resume_waypoint is not None:
            self._vehicle.set_current(resume_waypoint)
        self._navigation_task.peer_navigation = False
        self._navigation_task.peer_approach_distance_m = PEER_APPROACH_DIST
        self._navigation_task.orbit_radius_m = 0.0
        self._navigation_task.orbit_approach_alt_rel_m = None
        self._navigation_task.peer_target_location = None
        self._navigation_task.navigation_target_location = None
        self._confirm.hold_active = False
        self._mission.clear_navigation_task()
        self._pass_tracker.reset()
        self._detections.clear()
        self._geo_hold.last_own_target_geo = None
        self._geo_hold.active = False
        self._geo_hold.target_location = None
        self._geo_hold.acquisition_log_bucket = None
        self._ports.clear_selected_target()


__all__ = [
    "AutoMissionResume",
    "AutoMissionResumePorts",
    "NavigationTaskResetTransaction",
    "NavigationTaskResourcePorts",
    "NavigationTaskResourceReset",
    "NavigationTaskStateReset",
]
