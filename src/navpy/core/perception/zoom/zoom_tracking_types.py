from enum import Enum


class ZoomTrackingState(Enum):
    IDLE = "idle"
    HOLDING = "holding"
    ZOOMING_IN = "zooming_in"
    ZOOMING_OUT = "zooming_out"
    UNSUPPORTED = "unsupported"
