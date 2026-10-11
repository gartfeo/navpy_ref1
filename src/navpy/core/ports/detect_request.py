from __future__ import annotations


class DetectRequest:
    def __init__(
            self,
            force_lock_id: int | None = None,
            force_lock_bbox_cxcywh=None,
    ):
        self.force_lock_id = force_lock_id
        self.force_lock_bbox_cxcywh = force_lock_bbox_cxcywh
