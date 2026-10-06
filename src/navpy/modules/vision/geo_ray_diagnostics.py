from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, TYPE_CHECKING

import numpy as np
import pymap3d

if TYPE_CHECKING:
    from navpy.modules.common.models.attitude import Attitude
    from navpy.modules.common.models.location import Location
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


PixelValidator = Callable[[float, float], bool]


@dataclass(frozen=True)
class GeoRayDiagnostic:
    target_ned: tuple[float, float, float]
    target_uv: tuple[float | None, float | None]
    target_in_frame: bool | None
    optical_axis_ned: tuple[float, float, float]
    plane_hit_ned: tuple[float, float, float] | None
    plane_scale: float | None
    lateral_miss_m: float | None
    projection_status: str
    intersection_status: str

    def cache_key(self, command_pitch: float, command_yaw: float) -> tuple:
        return (
            self.projection_status,
            self.intersection_status,
            _round_tuple(self.target_ned, 1),
            _round_pixel_tuple(self.target_uv, 5.0),
            self.target_in_frame,
            _round_tuple(self.optical_axis_ned, 3),
            _round_tuple(self.plane_hit_ned, 1),
            _round_optional(self.lateral_miss_m, 1.0),
            round(float(command_pitch), 1),
            round(float(command_yaw), 1),
        )

    def format_log(self, mount_name: str, command_pitch: float, command_yaw: float,
                   gimbal_att: "Attitude") -> str:
        return (
            f"GimbalNavigation({mount_name}): GEO_RAY "
            f"proj={self.projection_status} intersect={self.intersection_status} "
            f"uv={_fmt_uv(self.target_uv)} in_frame={self.target_in_frame} "
            f"target_ned={_fmt_vec(self.target_ned)} "
            f"axis_ned={_fmt_vec(self.optical_axis_ned)} "
            f"hit_ned={_fmt_vec(self.plane_hit_ned)} "
            f"miss_h={_fmt_optional(self.lateral_miss_m, 'm')} "
            f"scale={_fmt_optional(self.plane_scale, '')} "
            f"cmd_frame=world cmd=(p={command_pitch:.1f},y={command_yaw:.1f}) "
            f"g_readback=(p={gimbal_att.pitch:.1f},y={gimbal_att.yaw:.1f},r={gimbal_att.roll:.1f})"
        )


def compute_geo_ray_diagnostic(
        *,
        target_loc: "Location",
        uav_loc: "Location",
        uav_att: "Attitude",
        k: np.ndarray,
        g_data: "GimbalData",
        geo_ref: "GeoRefCalc",
        is_valid_pixel: PixelValidator | None = None,
) -> GeoRayDiagnostic:
    target_ned = _as_vec3(pymap3d.geodetic2ned(
        target_loc.lat, target_loc.lng, target_loc.alt,
        uav_loc.lat, uav_loc.lng, uav_loc.alt,
    ))

    target_uv = _pixel_tuple(geo_ref.calc_uv(target_ned, k, g_data, uav_att))
    projection_status, target_in_frame = _classify_projection(target_uv, is_valid_pixel)

    cx = float(k[0][2])
    cy = float(k[1][2])
    optical_axis_ned = _as_vec3(geo_ref.calc_ned(cx, cy, k, g_data, uav_att))
    plane_hit, scale, intersection_status = _intersect_target_down_plane(
        optical_axis_ned, target_ned[2],
    )
    lateral_miss = None
    if plane_hit is not None:
        lateral_miss = float(np.linalg.norm(plane_hit[:2] - target_ned[:2]))

    return GeoRayDiagnostic(
        target_ned=_tuple3(target_ned),
        target_uv=target_uv,
        target_in_frame=target_in_frame,
        optical_axis_ned=_tuple3(optical_axis_ned),
        plane_hit_ned=_tuple3(plane_hit) if plane_hit is not None else None,
        plane_scale=scale,
        lateral_miss_m=lateral_miss,
        projection_status=projection_status,
        intersection_status=intersection_status,
    )


def _classify_projection(
        target_uv: tuple[float | None, float | None],
        is_valid_pixel: PixelValidator | None,
) -> tuple[str, bool | None]:
    u, v = target_uv
    if u is None or v is None:
        return "behind_cam", None
    if is_valid_pixel is None:
        return "projected", None
    in_frame = bool(is_valid_pixel(u, v))
    return ("in_frame" if in_frame else "out_of_fov"), in_frame


def _intersect_target_down_plane(
        optical_axis_ned: np.ndarray,
        target_down_m: float,
) -> tuple[np.ndarray | None, float | None, str]:
    ray_down = float(optical_axis_ned[2])
    if abs(ray_down) < 1e-9:
        return None, None, "parallel_to_target_alt"
    scale = float(target_down_m) / ray_down
    if scale < 0.0:
        return None, scale, "target_alt_behind_axis"
    return optical_axis_ned * scale, scale, "target_alt_hit"


def _as_vec3(value) -> np.ndarray:
    vec = np.asarray(value, dtype=float).reshape(-1)
    if vec.size != 3:
        raise ValueError(f"expected 3-vector, got shape {np.asarray(value).shape}")
    return vec


def _tuple3(value: np.ndarray | None) -> tuple[float, float, float]:
    if value is None:
        raise ValueError("_tuple3 does not accept None")
    return float(value[0]), float(value[1]), float(value[2])


def _pixel_tuple(value) -> tuple[float | None, float | None]:
    u, v = value
    if u is None or v is None:
        return None, None
    return float(u), float(v)


def _round_optional(value: float | None, step: float):
    if value is None:
        return None
    return round(float(value) / step) * step


def _round_tuple(value: tuple[float, float, float] | None, step: float):
    if value is None:
        return None
    return tuple(_round_optional(v, step) for v in value)


def _round_pixel_tuple(value: tuple[float | None, float | None], step: float):
    u, v = value
    if u is None or v is None:
        return None, None
    return _round_optional(u, step), _round_optional(v, step)


def _fmt_optional(value: float | None, suffix: str) -> str:
    if value is None:
        return "N/A"
    return f"{value:.1f}{suffix}"


def _fmt_vec(value: tuple[float, float, float] | None) -> str:
    if value is None:
        return "N/A"
    return f"(n={value[0]:.1f},e={value[1]:.1f},d={value[2]:.1f})"


def _fmt_uv(value: tuple[float | None, float | None]) -> str:
    u, v = value
    if u is None or v is None:
        return "(u=N/A,v=N/A)"
    return f"(u={u:.0f},v={v:.0f})"
