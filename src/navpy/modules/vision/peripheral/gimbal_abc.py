from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.worker_failure import StaticFailureHealth


@dataclass
class GimbalMountSetup:
    """Camera-to-gimbal mounting transform."""

    att: Attitude = field(default_factory=lambda: Attitude(90, 0, 90))
    seq: str = "XYZ"
    degrees: bool = True
    dist: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class GimbalData:
    """Gimbal state and configuration - single class for all gimbal info.

    The mounting transform moved into one ``setup: GimbalMountSetup`` field
    (2026-08 SOLID regroup). Construct with ``setup=GimbalMountSetup(...)``;
    the former ``setup_att``/``setup_seq``/``setup_degrees``/``setup_dist``
    constructor keywords are now read-only properties.
    """

    # Current gimbal attitude
    att: Attitude

    # Camera-to-gimbal transform (mounting)
    setup: GimbalMountSetup = field(default_factory=GimbalMountSetup)

    # Gimbal-to-UAS transform
    g_seq: str = "XYZ"
    degrees: bool = True

    # Stabilization
    roll_stabilize: bool = False
    pitch_stabilize: bool = False

    # Detection range
    max_detect_distance: float = 3000.0

    # Identity
    name: str = "gimbal"

    # Wall-clock receipt time for dynamic attitude telemetry. ``None`` means
    # timing is unproven; fixed gimbals declare timeless state separately.
    timestamp_s: Optional[float] = None

    # Aircraft attitude used when a stabilized world-frame angle was converted
    # into this body-frame readback. Simulator renderers use the pair atomically;
    # real payloads leave it unset and use their frame-associated pose sample.
    reference_aircraft_attitude: Optional[Attitude] = None

    @property
    def args(self) -> "GimbalData":
        """Backward compatibility - geo_ref_calc uses g_data.args.field."""
        return self

    @property
    def g_att(self) -> Attitude:
        """Alias for att (backward compat)."""
        return self.att

    @property
    def setup_att(self) -> Attitude:
        return self.setup.att

    @property
    def setup_seq(self) -> str:
        return self.setup.seq

    @property
    def setup_degrees(self) -> bool:
        return self.setup.degrees

    @property
    def setup_dist(self) -> List[float]:
        return self.setup.dist

    def __str__(self):
        return f"att={self.att}, setup_att={self.setup_att}"


class GimbalAbc(StaticFailureHealth, ABC):
    @abstractmethod
    def get_data(self) -> GimbalData:
        pass

    @abstractmethod
    def set_att(self, att: Attitude):
        pass

    def set_rate(self, yaw_rate: float, pitch_rate: float):
        """Command gimbal angular rates. Default no-op."""
        pass

    def state_is_static(self) -> bool:
        """True only when ``get_data`` is a timeless rigid transform."""
        return False

    def get_frame_state_sample(self):
        """Return one gimbal/zoom readback association for a camera frame.

        The tuple is ``(gimbal_data, zoom_level, zoom_age_s, zoom_sample_id)``.
        Dynamic drivers should override this so all values are read under one
        lock. A non-zoom dynamic gimbal may leave the zoom fields ``None``.
        """
        return self.get_data(), None, None, None

    def set_zoom(self, zoom):
        """Command payload zoom if supported. Default means no hardware zoom path."""
        return None

    def get_zoom_level_sample(self):
        """Return (zoom_level, age_s, sample_id) from one hardware readback."""
        return None

    def supports_zoom_readback(self) -> bool:
        """True if this gimbal provides a LIVE hardware zoom readback.

        Default False: a static/fixed mount has no live zoom feed, so its optics
        come from the camera's own intrinsics. Zoom gimbals override this True so
        a missing/stale readback is treated as *stale* (fail closed), never as a
        cue to report base-zoom intrinsics.
        """
        return False

    def supports_absolute_zoom_control(self) -> bool:
        """Declare support for setting an absolute payload zoom."""
        return False

    def supports_continuous_zoom_control(self) -> bool:
        """Declare support for zoom-in/out/hold rate-style commands."""
        return False

    def zoom_in(self) -> bool:
        """Start incremental zoom in; false means no committed command."""
        return False

    def zoom_out(self) -> bool:
        """Start incremental zoom out; false means no committed command."""
        return False

    def zoom_hold(self) -> bool:
        """Stop incremental zoom; false means no committed command."""
        return False

    def set_motion_mode(self, mode: int) -> None:
        """Set gimbal motion mode (e.g. LOCK, FOLLOW, FPV). Default no-op."""
        pass

    def request_autofocus(self):
        """Request autofocus if supported. Default no-op."""
        pass
