from __future__ import annotations

from typing import Callable, Optional

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.network_abc import ImageSendUnitRunner
from navpy.core.review.confirmation_ports import ConfirmationImageSender
from navpy.core.review.confirmation_image_artifacts import ConfirmationImageArtifacts
from navpy.core.ports.detect_data import DetectedObject


_LOCATION_TYPE_LABELS = {
    "building": "Building",
    "vehicle": "Vehicle",
    "antenna": "Comms",
    "operations_site": "Operations site",
    "bridge": "Bridge",
    "fuel": "Fuel",
    "other": "Other",
}

def poi_class_name(class_id: int, location_type: Optional[str] = None) -> str:
    """Describe a location type or an ID without guessing the model's labels."""
    if location_type:
        return _LOCATION_TYPE_LABELS.get(location_type, location_type.capitalize())
    return f"Detection class {class_id}"


def _nothing() -> None:
    return None


class ConfirmationMedia:
    """Build, send, and persist operator confirmation imagery."""

    def __init__(
        self,
        *,
        sys_id: int,
        network: Callable[[], ConfirmationImageSender | None],
        logger: ILogger,
        thumbnail_builder: Callable[..., Optional[str]],
        artifact_saver: Callable[..., Optional[ConfirmationImageArtifacts]],
    ) -> None:
        self._sys_id = sys_id
        self._network = network
        self._logger = logger
        self._thumbnail_builder = thumbnail_builder
        self._artifact_saver = artifact_saver

    def send(
        self,
        poi: DetectedObject,
        poi_id: int,
        run_unit: ImageSendUnitRunner,
    ) -> bool:
        """Send the confirmation image; False once the round no longer takes it.

        ``run_unit`` runs each wire send inside the round's outbound fence and
        refuses once the round is resolved, so a response resolves between two
        chunks instead of after the whole image, and the rest is not sent.
        Building the thumbnail and saving artifacts run outside the fence.
        """
        if not run_unit(_nothing):  # resolved already: send and save nothing
            return False
        confirmation = poi.confirmation
        classification = poi.classification
        if confirmation.frame is None or confirmation.bbox_cxcywh is None:
            return True
        class_name = poi_class_name(
            classification.class_id,
            classification.location_type,
        )
        image_b64 = self._thumbnail_builder(
            frame=confirmation.frame,
            poi_id=poi_id,
            bbox=confirmation.bbox_cxcywh,
            confidence=classification.confidence,
            class_name=class_name,
            frame_bboxes=confirmation.frame_bboxes,
            degraded=confirmation.degraded,
        )
        if not image_b64:
            return True
        refused = False

        def run_current(unit: Callable[[], None]) -> bool:
            nonlocal refused
            refused = not run_unit(unit)
            return not refused

        try:
            network = self._network()
            if network is None:
                return True
            num_packets = network.send_image(poi_id, image_b64, run_unit=run_current)
            if refused:
                self._logger.info(
                    f"P{poi_id} image send stopped after {num_packets} "
                    f"packets: confirmation resolved.",
                    key="nav_state",
                )
            elif num_packets > 0:
                self._logger.info(
                    f"P{poi_id} image handed to the link ({num_packets} packets).",
                    key="nav_state",
                    dest=LogStatusDest.DRONE,
                )
        finally:
            self._save_artifacts(poi, poi_id, class_name, image_b64)
        return not refused

    def _save_artifacts(
        self,
        poi: DetectedObject,
        poi_id: int,
        class_name: str,
        image_b64: str,
    ) -> None:
        log_path = getattr(self._logger, "log_path", None)
        if not log_path:
            return
        try:
            artifacts = self._artifact_saver(
                log_path=log_path,
                sys_id=self._sys_id,
                poi_id=poi_id,
                source_frame=poi.confirmation.frame,
                sent_image_b64=image_b64,
                bbox_cxcywh=poi.confirmation.bbox_cxcywh,
                class_id=poi.classification.class_id,
                class_name=class_name,
                confidence=poi.classification.confidence,
                frame_bboxes=poi.confirmation.frame_bboxes,
                confirmation_degraded=poi.confirmation.degraded,
            )
            if artifacts is not None:
                self._logger.info(
                    f"P{poi_id} confirmation images saved: "
                    f"{artifacts.source_path.name}, "
                    f"{artifacts.sent_thumbnail_path.name}",
                    key="nav_state",
                )
        except OSError as exc:
            self._logger.warning(
                f"Failed to save confirmation image artifacts for "
                f"P{poi_id}: {exc}",
                key="nav_state",
            )
