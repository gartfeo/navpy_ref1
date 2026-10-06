"""Operator-review hold and confirmation metadata behavior."""

from __future__ import annotations

from navpy.modules.nav.confirmation_local import LocalConfirmationPublisher
from navpy.modules.nav.confirmation_operator_review import OperatorReviewPublisher
from navpy.modules.vision.models.detect_data import DetectedObject


class ConfirmationReview:
    """Issue review holds and metadata without polluting final approach."""

    def __init__(
        self,
        local: LocalConfirmationPublisher,
        operator_review: OperatorReviewPublisher,
    ) -> None:
        self._local = local
        self._operator_review = operator_review

    def confirm_local(self, target: DetectedObject) -> None:
        self._local.publish(target)

    def request_operator_review(self, target: DetectedObject) -> None:
        self._operator_review.publish(target)


__all__ = ["ConfirmationReview"]
