"""Visual-only final-approach pass state machine."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from navpy.core.ports.frame import FinalApproachVisionFrame

# Arm only once the dock is clearly ahead: the frame-local LOS within this
# angle of the nose. From the waiting circle the dock starts about 90 deg
# abeam, so arming on any body_x > 0 let one small turn read as "ahead, then
# behind" and declared a pass 630 m out (.logs/a1-measure-b, uav_1). 60 deg
# keeps the abeam start 30 deg outside the cone, while a closing approach
# (LOS a few deg off the nose in the a1 terminal phase) arms well inside it.
PASS_ARM_MAX_OFF_NOSE_DEG = 60.0
PASS_ARM_MIN_BODY_X = math.cos(math.radians(PASS_ARM_MAX_OFF_NOSE_DEG))

# A gimballed narrow camera cannot look behind, so at the hit the dock leaves
# the image while still centred (projector: behind_cam), never as two aft
# frames. Owner rule 2026-10-13: an armed dock lost while CENTRED counts as
# passed. In a1 the last frames before that loss were 2.0 and 2.3 deg off the
# optical axis (49 and 57 px at fx ~1407 px, ZR10 at 1x); 3 deg covers them
# with margin and is an angle, so zoom does not change it.
PASS_CENTRED_MAX_DEG = 3.0
# A dock leaving through an image edge stays a failure at any zoom. At ZR10
# 1x the short half-FOV is 14.4 deg and the 3 deg limit binds; from ~6.5x the
# whole half-height is within ~3 deg, so centred also requires the inner half
# of the short half-FOV.
PASS_CENTRED_MAX_HALF_FOV_FRACTION = 0.5
# TEMPORARY GUARD (owner, 2026-10-13, "workaround"): a centred loss counts as
# a pass only with the camera fully zoomed out: its FRESH zoom readback, in the
# mount's zoom-command space, at most this multiple of the widest zoom level.
# Zoomed in, a far dock can drop out while centred (finding 1). ZR10's widest
# reading (1.1) maps to command 1.0, its widest level; 1.2 leaves room for
# command-space interpolation there and rejects real zoom (2x and up). Stale or
# unknown zoom fails closed.
PASS_MAX_ZOOM_OVER_WIDEST = 1.2


@dataclass(frozen=True)
class VisualPassState:
    identity: tuple[str, int, int, int] | None = None
    armed: bool = False
    aft_pending: bool = False
    passed: bool = False
    last_frame: FinalApproachVisionFrame | None = None


@dataclass(frozen=True)
class VisualPassPlan:
    suppress_command: bool
    next_state: VisualPassState


def centred_limit_deg(camera_half_fov_deg: float) -> float:
    """Largest off-axis angle that still counts as centred for this optics."""
    return min(
        PASS_CENTRED_MAX_DEG,
        PASS_CENTRED_MAX_HALF_FOV_FRACTION * camera_half_fov_deg,
    )


def is_centred_angles(camera_off_axis_deg: float, camera_half_fov_deg: float) -> bool:
    """Centred needs a known, positive field of view (0 means unknown)."""
    return camera_half_fov_deg > 0.0 and camera_off_axis_deg <= centred_limit_deg(
        camera_half_fov_deg
    )


def is_centred(frame: FinalApproachVisionFrame) -> bool:
    """Frame-local LOS near the optical axis and well inside the image."""
    return is_centred_angles(frame.camera_off_axis_deg, frame.camera_half_fov_deg)


class VisualPassDetector:
    """Arm on a clearly-ahead frame; pass on two aft frames or a centred loss."""

    def __init__(self) -> None:
        self._state = VisualPassState()

    @property
    def passed(self) -> bool:
        return self._state.passed

    @property
    def last_frame(self) -> FinalApproachVisionFrame | None:
        """The last committed frame (evidence for a loss-latched pass)."""
        return self._state.last_frame

    def reset(self) -> None:
        self._state = VisualPassState()

    def plan(self, frame: FinalApproachVisionFrame) -> VisualPassPlan:
        state = self._state
        key = frame.continuity_key
        if state.identity != key:
            state = VisualPassState(identity=key)
        if state.passed:
            return VisualPassPlan(True, state)
        if frame.body_x > 0.0:
            armed = state.armed or frame.body_x >= PASS_ARM_MIN_BODY_X
            return VisualPassPlan(
                False,
                VisualPassState(key, armed, False, False, frame),
            )
        if not state.armed:
            return VisualPassPlan(False, replace(state, last_frame=frame))
        return VisualPassPlan(
            True,
            VisualPassState(key, True, True, state.aft_pending, frame),
        )

    def commit(self, plan: VisualPassPlan) -> None:
        self._state = plan.next_state

    def latch_detection_loss(self, current_half_fov_deg: float | None) -> bool:
        """Latch a pass when the armed dock was lost centred; True if passed.

        ``current_half_fov_deg`` is the camera's half-FOV now. If the optics
        changed since the last frame (zoom/intrinsics), its centring no longer
        tells an edge exit from a hit, so the loss is not eligible until a
        frame with the current optics arrives.
        """
        state = self._state
        if state.passed:
            return True
        frame = state.last_frame
        if not (state.armed and frame is not None and is_centred(frame)):
            return False
        if current_half_fov_deg is None or not math.isclose(
            current_half_fov_deg, frame.camera_half_fov_deg, rel_tol=1e-9
        ):
            return False
        self._state = replace(state, passed=True)
        return True


__all__ = [
    "PASS_ARM_MAX_OFF_NOSE_DEG",
    "PASS_CENTRED_MAX_DEG",
    "PASS_CENTRED_MAX_HALF_FOV_FRACTION",
    "PASS_MAX_ZOOM_OVER_WIDEST",
    "VisualPassDetector",
    "VisualPassPlan",
    "VisualPassState",
    "centred_limit_deg",
    "is_centred",
    "is_centred_angles",
]
