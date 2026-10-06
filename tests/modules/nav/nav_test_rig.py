"""Explicit test-only view of the composed navigation graph.

Production keeps a four-capability application boundary.  Navigation unit
tests use this fixture to address the real focused owner or workflow directly
instead of restoring private methods and scalar aliases on NavController.
"""

from __future__ import annotations

from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.nav_composition import create_nav_application
from navpy.modules.nav.nav_state import NavState
from navpy.modules.vision.detection_coordination import DetectionCoordination
from navpy.modules.vision.detection_coordinator import DetectionCoordinator


def as_detection_coordination(detector) -> DetectionCoordination:
    """Expose one legacy test double through the production capability bundle."""
    if isinstance(detector, DetectionCoordination):
        return detector
    if isinstance(detector, DetectionCoordinator):
        return detector.coordination
    return DetectionCoordination(
        snapshot=detector,
        events=detector,
        tracking_commands=detector,
        tracking_status=detector,
        target_identity=detector,
        geo_pointing=detector,
        zoom=detector,
        mounts=detector,
        simulation=detector,
        reset=detector,
        lifecycle=detector,
        cadence=detector,
    )


class _DetectionTestView:
    """Test-only scalar view over the production atomic snapshot API."""

    def __init__(self, state) -> None:
        object.__setattr__(self, "_state", state)

    @property
    def detected_targets(self):
        return self._state.targets()

    @detected_targets.setter
    def detected_targets(self, targets) -> None:
        selection = self._state.selection()
        self._state.replace_selection(list(targets), selection.primary_target)

    @property
    def primary_target(self):
        return self._state.selection().primary_target

    @primary_target.setter
    def primary_target(self, target) -> None:
        selection = self._state.selection()
        self._state.replace_selection(list(selection.targets), target)

    @property
    def pending_events(self):
        return self._state.events()

    def __getattr__(self, name):
        return getattr(self._state, name)


class _NavPhaseTestView:
    """Test-only state seeding over the production transition API."""

    def __init__(self, state) -> None:
        object.__setattr__(self, "_state", state)

    @property
    def current(self):
        return self._state.current

    @current.setter
    def current(self, phase) -> None:
        self._state.request(phase)

    @property
    def previous(self):
        return self._state.previous

    @previous.setter
    def previous(self, phase) -> None:
        with self._state._lock:
            self._state._previous = phase

    @property
    def oneshot_completed(self):
        return self._state.oneshot_completed

    @oneshot_completed.setter
    def oneshot_completed(self, completed) -> None:
        self._state.oneshot_completed = completed

    @property
    def last_parameter_refresh_s(self):
        return self._state.last_parameter_refresh_s

    @last_parameter_refresh_s.setter
    def last_parameter_refresh_s(self, timestamp_s) -> None:
        self._state.last_parameter_refresh_s = timestamp_s

    def __getattr__(self, name):
        return getattr(self._state, name)


class NavTestRig:
    def __init__(
        self,
        application,
        *,
        vehicle,
        detector,
        navigation,
        args,
        logger,
        approach_kind,
        vision_profile,
        scheduler_cadence,
    ) -> None:
        self.application = application
        self.vehicle = vehicle
        self.detector = detector
        self.navigation = navigation
        self.args = args
        self.logger = logger
        self.approach_kind = approach_kind
        self.vision_profile = vision_profile or {}
        self.scheduler_cadence = scheduler_cadence

        self.loop = application._loop
        self.network = application._network
        self.shutdown = application._shutdown
        self.overrides = application._overrides
        self.clock = self.loop._clock
        self.cycle = self.loop._cycle
        self.sensor = self.cycle._sensor
        self.decision = self.cycle._decision
        self.actions = self.cycle._actions

        self.phase_state = self.decision._phase
        self.phase = _NavPhaseTestView(self.phase_state)
        self.inactive = self.decision._inactive
        self.recovery = self.decision._recovery
        self.nav_decision = self.decision._nav
        self.navigation_failures = self.nav_decision._failures
        self.target_status = self.decision._target_status
        self.mission = self.decision._mission
        self.detection_state = self.cycle._detections
        self.detections = _DetectionTestView(self.detection_state)

        self.navigation_task = self.inactive._navigation_task
        self.confirm = self.inactive._confirm
        self.confirmation_manager = self.inactive._confirmation_manager
        self.loiter_radius = self.inactive._loiter_radius
        self.mission_pass = self.inactive._mission_pass
        self.pass_tracker = self.mission_pass._pass_tracker
        self.status = self.inactive._status

        self.source = self.target_status._presence._source
        self.timing = self.target_status._presence._timing
        self.retry_policy = self.target_status._confirmed._retry
        self.retry_state = self.retry_policy._retry
        self.geo_coordinator = self.target_status._presence._geo_coordinator
        self.track_recovery = self.target_status._confirmed._recovery
        self.identity_reacquisition = self.geo_coordinator._identity
        self.geo_hold = self.target_status._presence._geo_hold
        self.release = self.target_status._confirmed._release
        self.freshness = self.release._freshness
        self.auto_resume = self.target_status._rejection._resume_auto

        self.detect_action = self.actions._actions[NavState.DETECT].__self__
        self.confirmation_action = (
            self.actions._actions[NavState.CONFIRM].__self__
        )
        self.terminal_nav = self.actions._actions[NavState.NAV].__self__
        self.transitions = self.actions._transitions
        self.selector = self.detect_action._selector
        self.navigation_task_action = self.detect_action._navigation_task
        self.peer_notifier = self.detect_action._peer_notifier
        self.peer_geo_acquisition = self.detect_action._peer_geo._acquisition
        self.peer_navigation = self.navigation_task_action._peer_navigation
        self.peer_geo_tracker = self.peer_navigation._approach._geo_tracker
        self.fallback_navigation = self.navigation_task_action._fallback_navigation
        self.self_approach = self.navigation_task_action._self_approach
        self.terminal = self.navigation_task_action._terminal
        self.speedup = self.navigation_task_action._speedup
        self.vehicle_navigation = self.fallback_navigation._commands
        self.approach_planner = self.fallback_navigation._approach_planner

        self.frame_policy = self.confirmation_action._frame_policy
        self.terminal_admission = self.confirmation_action._terminal_admission
        self.recognition = self.confirmation_action._recognition_gate
        self.blocked = self.confirmation_action._blocked
        self.debug = self.confirmation_action._debug
        self.confirmation_geo_hold = self.confirmation_action._geo_hold
        self.review = self.recognition._review

        self.terminal_commands = self.terminal_nav._commands
        self.deadline = self.terminal_nav._record._deadline
        self.nav_peers = self.terminal_nav._peers
        self.nav_transition = self.transitions._nav
        self.zoom = self.nav_transition._entry._zoom

        self.reset = self.shutdown._navigation_task_reset
        self.state_reset = self.reset._state
        self.resource_reset = self.reset._resources


def create_nav_test_rig(
    vehicle,
    detector,
    navigation,
    args,
    logger,
    approach_kind=ApproachKind.OFFSET,
    vision_profile=None,
    scheduler_cadence=None,
) -> NavTestRig:
    application = create_nav_application(
        vehicle,
        as_detection_coordination(detector),
        navigation,
        args,
        logger,
        approach_kind,
        vision_profile,
        scheduler_cadence,
    )
    return NavTestRig(
        application,
        vehicle=vehicle,
        detector=detector,
        navigation=navigation,
        args=args,
        logger=logger,
        approach_kind=approach_kind,
        vision_profile=vision_profile,
        scheduler_cadence=scheduler_cadence,
    )


__all__ = [
    "NavTestRig",
    "as_detection_coordination",
    "create_nav_test_rig",
]
