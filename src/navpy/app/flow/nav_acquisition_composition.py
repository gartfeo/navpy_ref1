"""Composition of the own-detection ORBIT and its acquisition gates (HITL step 25)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.skills.confirm.acquisition_gate_settings import (
    FINAL_APPROACH_GATES_KEY,
    acquisition_gate_settings,
)
from navpy.core.skills.confirm.acquisition_sequence import (
    OPEN_ACQUISITION,
    AcquisitionGate,
    AcquisitionSequence,
    GatedOrbitStart,
    OrbitStart,
)
from navpy.core.skills.confirm.fresh_visual_lock import FreshVisualLock
from navpy.core.skills.confirm.orbit_established import OrbitEstablished
from navpy.core.skills.geo_approach.board_orbit_centre import board_orbit_centre
from navpy.core.skills.geo_approach.self_detected_approach import (
    SelfDetectedApproach,
    SelfDetectedApproachPorts,
)
from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    VehicleApproachOwnership,
)
from navpy.products.delivery.home_dock import HomeDockRules
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.perception.detection_coordination import DetectionCoordination


@dataclass(frozen=True)
class AcquisitionComposition:
    # What the navigation task's start calls: the ORBIT itself, or with gates only the POI mark.
    task_start: OrbitStart
    # Confirm and NAV ask this on each tick's detection.
    gate: AcquisitionGate


def compose_acquisition(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    profile: dict,
    state: NavStateOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation: NavCapabilities,
    home_dock: HomeDockRules,
    approach_kind: ApproachKind,
    logger: ILogger,
) -> AcquisitionComposition:
    settings = acquisition_gate_settings(profile)
    if settings is not None and approach_kind != ApproachKind.ORBIT:
        raise ValueError(f"{FINAL_APPROACH_GATES_KEY} needs the ORBIT approach, not {approach_kind}")
    # With the gates, the circle must be the own one on the own-detected pose: a circle already flown
    # (a peer's) is never kept, or the orbit gate would wait for a command that never comes.
    needs_own_orbit = home_dock.needs_own_orbit if settings is None else (lambda _poi: True)
    self_approach = _self_detected_approach(
        vehicle, detection, state, approach, navigation, needs_own_orbit, approach_kind, logger,
    )
    if settings is None:
        return AcquisitionComposition(self_approach, OPEN_ACQUISITION)
    sequence = AcquisitionSequence(
        FreshVisualLock(
            settings,
            observation.freshness.detection_is_fresh_for_confirm,
            lambda: detection.tracking_status.is_detection_armed,
        ),
        OrbitEstablished(
            settings,
            # The stamped fix, not the cached location: a fix counts only while its boot time
            # advances. A vehicle without one fails the gate closed.
            lambda: getattr(vehicle, "position_sample", None),
            lambda: state.clock.decision_s(),
        ),
        self_approach,
        logger,
    )
    return AcquisitionComposition(GatedOrbitStart(sequence), sequence)


def _self_detected_approach(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    state: NavStateOwnership,
    approach: VehicleApproachOwnership,
    navigation: NavCapabilities,
    needs_own_orbit: Callable[[DetectedObject], bool],
    approach_kind: ApproachKind,
    logger: ILogger,
) -> SelfDetectedApproach:
    return SelfDetectedApproach(
        SelfDetectedApproachPorts(
            final_approach_active=lambda: navigation.final_approach.is_active,
            detector_is_simulation=lambda: detection.simulation.is_simulation,
            ground_location=navigation.legacy_pois.ground_location,
            current_relative=lambda: vehicle.location(True),
            plan_orbit=lambda *pos, **kw: (
                approach.planner.plan_orbit_approach(*pos, **kw)
            ),
            absolute_location=approach.commands.absolute_location,
            save_loiter_radius=lambda: approach.commands.save_loiter_radius(),
            request_guided=lambda: approach.commands.request_guided(),
            loiter_poi=navigation.vehicle_commands.peer_poi_loiter,
            needs_own_orbit=needs_own_orbit,
            board_centre=lambda poi: board_orbit_centre(
                poi, navigation.legacy_pois.geo_ref.camera_point_to_ned,
                vehicle.home_location,
            ),
        ),
        state.navigation_task,
        approach_kind,
        logger,
    )


__all__ = ["AcquisitionComposition", "compose_acquisition"]
