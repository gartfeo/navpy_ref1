"""Image-quality admission for operator confirmation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_frame_policy import extract_confirmation_size
from navpy.modules.nav.confirmation_policy import ConfirmationTimingPolicy
from navpy.modules.nav.confirmation_reporting import (
    ConfirmBlockedReporter,
    ConfirmDebugReporter,
)
from navpy.modules.nav.confirmation_review import ConfirmationReview
from navpy.modules.nav.nav_state import ConfirmOverrideInbox
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id
from navpy.modules.vision.vision_class_profile import get_min_pixels_for_class


@dataclass(frozen=True)
class RecognitionGatePorts:
    vision_profile: dict
    logger: ILogger


class RecognitionGate:
    """Own image-size, zoom-stability, and one-shot override admission."""

    def __init__(
        self,
        ports: RecognitionGatePorts,
        review: ConfirmationReview,
        timing: ConfirmationTimingPolicy,
        overrides: ConfirmOverrideInbox,
        debug: ConfirmDebugReporter,
        blocked: ConfirmBlockedReporter,
    ) -> None:
        self._ports = ports
        self._review = review
        self._timing = timing
        self._overrides = overrides
        self._debug = debug
        self._blocked = blocked

    def request(self, target: DetectedObject) -> None:
        self._timing.refresh_timer_on_reacquire()
        source_size = extract_confirmation_size(target)
        min_pixels = get_min_pixels_for_class(
            self._ports.vision_profile,
            target.classification.class_id,
        )
        pixels_ok = source_size is None or source_size >= min_pixels
        zoom_result = self._timing.active_zoom_result(target)
        zoom_ok = self._timing.active_zoom_stable(zoom_result)
        at_max_zoom = bool(getattr(zoom_result, "at_max_zoom", False))
        best_available = not pixels_ok and at_max_zoom

        if self._consume_override(
            target,
            source_size,
            min_pixels,
            pixels_ok,
            zoom_ok,
            best_available,
        ):
            return
        if best_available:
            target.set_confirmation_degraded(False)
            self._ports.logger.info(
                "CONFIRM gate best available at max zoom for "
                f"T{get_target_task_id(target)} "
                f"(source_sz={source_size:.1f}/{min_pixels:.1f})",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
        elif not (pixels_ok and zoom_ok):
            if self._is_blocked(target, source_size, min_pixels, pixels_ok):
                return
        else:
            target.set_confirmation_degraded(False)
        self._blocked.clear()
        self._review.request_operator_review(target)

    def _consume_override(
        self,
        target: DetectedObject,
        source_size: Optional[float],
        min_pixels: float,
        pixels_ok: bool,
        zoom_ok: bool,
        best_available: bool,
    ) -> bool:
        if best_available or (pixels_ok and zoom_ok):
            return False
        target_id = get_target_task_id(target)
        if not self._overrides.consume(target_id):
            return False
        target.set_confirmation_degraded(True)
        self._ports.logger.warning(
            f"CONFIRM gate override (Ask me anyway) for T{target_id} "
            f"(source_sz={source_size}, min={min_pixels}, zoom_ok={zoom_ok})",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        self._blocked.clear()
        self._review.request_operator_review(target)
        return True

    def _is_blocked(
        self,
        target: DetectedObject,
        source_size: Optional[float],
        min_pixels: float,
        pixels_ok: bool,
    ) -> bool:
        target_id = get_target_task_id(target)
        if self._timing.gate_timed_out():
            if not pixels_ok:
                self._debug.log(
                    f"timeout_pixels_blocked obj={target.identity.obj_id} "
                    f"source_sz={source_size:.0f}/{min_pixels:.0f}"
                )
                self._blocked.emit(
                    target_id,
                    "pixels",
                    f"{source_size:.0f}/{min_pixels:.0f}",
                )
                return True
            self._ports.logger.warning(
                "CONFIRM gate timeout (zoom): sending recognition-sized "
                f"image for T{target_id} (source_sz={source_size})",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
            target.set_confirmation_degraded(False)
            return False
        if not pixels_ok:
            self._debug.log(
                f"source_pixels_insufficient obj={target.identity.obj_id} "
                f"source_sz={source_size:.0f}/{min_pixels:.0f}"
            )
            self._blocked.emit(
                target_id,
                "pixels",
                f"{source_size:.0f}/{min_pixels:.0f}",
            )
        else:
            self._debug.log(f"zoom_not_stable obj={target.identity.obj_id}")
            self._blocked.emit(target_id, "zoom")
        return True


__all__ = ["RecognitionGate", "RecognitionGatePorts"]
