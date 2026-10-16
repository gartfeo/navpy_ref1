"""The catapult rail (--launch rail, the bridge's default): a launcher as its logs measured it.

The rail's numbers are the owner's measurements and stay out of the repository: a RailSpec comes from
the private launch profile (launch_profile.py, tools/tuning/private_data.py).
- At rest the aircraft sits on the carriage at the spec's carriage pitch, wings level, on a straight
  rail of elevation_deg along the origin's heading, height_m above the ground.
- A set delay after the first armed frame the stroke starts (as with the simple launcher). The
  carriage moves along the rail with the measured acceleration (accel_table_g), is arrested
  after travel_m, and the aircraft leaves it with the velocity it has then.
- On the rail the attitude is the carriage's and the body rates are 0; the accelerometer reads
  the true specific force, the rail acceleration plus the reaction to gravity (rail_state), so
  the board's TKOFF_THR_MINACC launch detection sees what the aircraft's IMU sees. The airspeed
  is the relative wind's.
- The physics starts at the release: the plant's initial conditions are the release state
  (RailLaunch.release_pose), and it is held unstepped until then.

A release speed other than the spec's (--rail-release-speed-m-s) scales the push for the same
rail: the acceleration by k^2 and the time by 1/k, k = speed / the measured release speed (an
elastic launcher's energy grows with the square of the speed it gives over a fixed travel).

Not modelled: the pitch-up on the cradle, the separation's nose-down kick and the fading carriage
contact after the arrest. A table from the flight controller's own filtered accelerometer trails
the push by a few milliseconds and peaks lower than the push; the board filters it once more.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional, Sequence

from tools.tuning.hitl_bridge.environment import US_PER_S, Wind
from tools.tuning.hitl_bridge.physics import Origin, PlantState
from tools.tuning.hitl_bridge.plant_options import FT_PER_M, StartPose
from tools.tuning.hitl_bridge.state_fields import STANDARD_GRAVITY_M_S2, ned_to_body


@dataclass(frozen=True)
class RailSpec:
    """A catapult rail as measured.

    accel_table_g: the push, g per accel_table_bin_s bin from the stroke start (the last bin's value is held
    until the arrest); travel_m: the travel to the arrest; release_speed_m_s: the speed at the arrest;
    elevation_deg: the rail's angle above the horizon; carriage_angle_to_rail_deg: the airframe's pitch above
    the rail on the carriage; height_m: the aircraft's height above the ground on the carriage at rest."""
    accel_table_bin_s: float
    accel_table_g: tuple[float, ...]
    travel_m: float
    release_speed_m_s: float
    elevation_deg: float
    carriage_angle_to_rail_deg: float
    height_m: float

    @property
    def carriage_pitch_deg(self) -> float:
        """The aircraft's body pitch at rest on the carriage: the rail's elevation plus the carriage's angle."""
        return self.elevation_deg + self.carriage_angle_to_rail_deg


@dataclass(frozen=True)
class RailStroke:
    """The push along the rail: a constant acceleration in each bin from the stroke start, arrested
    when travel_m is reached, duration_s after the start, at release_speed_m_s.
    bin_starts: (travel, speed) at the start of each bin."""
    bin_s: float
    accel_m_s2: tuple[float, ...]
    bin_starts: tuple[tuple[float, float], ...]
    duration_s: float
    travel_m: float
    release_speed_m_s: float

    def at(self, t_s: float) -> tuple[float, float, float]:
        """Travel along the rail (m), speed (m/s) and acceleration (m/s^2) t_s after the stroke
        start: at rest before it, arrested at the travel with the release speed after it."""
        if t_s <= 0.0:
            return 0.0, 0.0, 0.0
        if t_s >= self.duration_s:
            return self.travel_m, self.release_speed_m_s, 0.0
        index = min(int(t_s / self.bin_s), len(self.accel_m_s2) - 1)
        s0, v0 = self.bin_starts[index]
        a, tau = self.accel_m_s2[index], t_s - index * self.bin_s
        return s0 + v0 * tau + 0.5 * a * tau * tau, v0 + a * tau, a


def _time_to_cover(distance_m: float, speed_m_s: float, accel_m_s2: float) -> float:
    """The positive root of speed*t + accel*t^2/2 = distance (the form without cancellation)."""
    return 2.0 * distance_m / (speed_m_s + math.sqrt(speed_m_s * speed_m_s + 2.0 * accel_m_s2 * distance_m))


def rail_stroke(release_speed_m_s: float, table_g: Sequence[float], bin_s: float, travel_m: float) -> RailStroke:
    """The measured push (table_g in g per bin, its last value held past the table) arrested at
    travel_m, scaled in time and acceleration so it releases at release_speed_m_s."""
    if release_speed_m_s <= 0.0:
        raise ValueError(f"the rail's release speed must be positive, not {release_speed_m_s} m/s")
    accel = [a * STANDARD_GRAVITY_M_S2 for a in table_g]
    bins: list[float] = []
    starts: list[tuple[float, float]] = []
    s = v = 0.0
    while True:
        a = accel[min(len(bins), len(accel) - 1)]
        if len(bins) >= len(accel) and a <= 0.0:
            raise ValueError(f"the acceleration table ends {s:.2f} m along the rail, short of {travel_m} m")
        bins.append(a)
        starts.append((s, v))
        if s + v * bin_s + 0.5 * a * bin_s * bin_s >= travel_m:
            tau = _time_to_cover(travel_m - s, v, a)
            duration_s, speed = (len(bins) - 1) * bin_s + tau, v + a * tau
            break
        s, v = s + v * bin_s + 0.5 * a * bin_s * bin_s, v + a * bin_s
    k = release_speed_m_s / speed
    return RailStroke(bin_s=bin_s / k, accel_m_s2=tuple(a * k * k for a in bins),
                      bin_starts=tuple((s0, v0 * k) for s0, v0 in starts), duration_s=duration_s / k,
                      travel_m=travel_m, release_speed_m_s=release_speed_m_s)


def spec_stroke(spec: RailSpec, release_speed_m_s: Optional[float] = None) -> RailStroke:
    """The spec's push, released at its measured speed or at release_speed_m_s (scaled as rail_stroke does)."""
    speed = spec.release_speed_m_s if release_speed_m_s is None else release_speed_m_s
    return rail_stroke(speed, spec.accel_table_g, spec.accel_table_bin_s, spec.travel_m)


@dataclass(frozen=True)
class RailGeometry:
    """Where the rail is: it starts at the origin with the aircraft height_m above the ground there and
    runs along the origin's heading at elevation_deg; the carriage holds the aircraft at pitch_deg."""
    origin: Origin
    elevation_deg: float
    height_m: float
    pitch_deg: float

    def direction_ned(self) -> tuple[float, float, float]:
        """The unit vector along the rail, north/east/down."""
        el, hdg = math.radians(self.elevation_deg), math.radians(self.origin.heading_deg)
        return math.cos(el) * math.cos(hdg), math.cos(el) * math.sin(hdg), -math.sin(el)


def rail_state(release: PlantState, geometry: RailGeometry, fraction: float, speed_m_s: float,
               accel_m_s2: float, wind: Wind) -> PlantState:
    """The aircraft on the rail, at this fraction of the travel: on the line from the rail's start to the
    plant's release position, in the carriage's attitude (the release state's), not rotating, moving
    along the rail, its accelerometer reading the rail acceleration plus the reaction to gravity in body
    axes. The airspeed is the relative wind's; the indicated one scaled as the plant's at the release."""
    origin = geometry.origin
    start_alt_m = origin.ground_alt_msl_m + geometry.height_m
    u = geometry.direction_ned()
    fx, fy, fz = ned_to_body((accel_m_s2 * u[0], accel_m_s2 * u[1], accel_m_s2 * u[2] - STANDARD_GRAVITY_M_S2),
                             release.roll_rad, release.pitch_rad, release.yaw_rad)
    vn, ve, vd = (speed_m_s * c for c in u)
    wind_n, wind_e = wind.air_velocity_ne_m_s()
    tas = math.sqrt((vn - wind_n) ** 2 + (ve - wind_e) ** 2 + vd ** 2)
    # EAS/TAS of the plant's air at the release; a release at rest (only a test plant) reads its TAS
    eas_per_tas = release.ias_m_s / release.tas_m_s if release.tas_m_s > 0.0 else 1.0
    return replace(release,
                   lat_deg=origin.lat_deg + fraction * (release.lat_deg - origin.lat_deg),
                   lon_deg=origin.lon_deg + fraction * (release.lon_deg - origin.lon_deg),
                   alt_msl_m=start_alt_m + fraction * (release.alt_msl_m - start_alt_m),
                   p_rad_s=0.0, q_rad_s=0.0, r_rad_s=0.0, vn_m_s=vn, ve_m_s=ve, vd_m_s=vd,
                   ias_m_s=tas * eas_per_tas, tas_m_s=tas, fx_m_s2=fx, fy_m_s2=fy, fz_m_s2=fz)


class RailLaunch:
    """physics.Launch for the catapult rail: at rest on the carriage, then pushed along the rail (no
    physics), then released into the plant, which starts there (release_pose)."""

    holds = True

    def __init__(self, geometry: RailGeometry, stroke: RailStroke, delay_s: float) -> None:
        self.geometry = geometry
        self.stroke = stroke
        self.delay_s = delay_s
        self.armed_at_us: Optional[int] = None

    def note(self, board_time_us: int, armed: bool) -> None:
        if armed and self.armed_at_us is None:
            self.armed_at_us = board_time_us

    def reset(self) -> None:
        self.armed_at_us = None

    def stroke_time_s(self, board_time_us: float) -> Optional[float]:
        """Seconds since the stroke started (negative before it); None before the first armed frame."""
        if self.armed_at_us is None:
            return None
        return (board_time_us - self.armed_at_us) / US_PER_S - self.delay_s

    def force_at(self, board_time_us: float) -> float:
        """No force on the plant: the rail moves the aircraft, and the plant starts after it."""
        return 0.0

    def holding(self, board_time_us: float) -> bool:
        t_s = self.stroke_time_s(board_time_us)
        return t_s is None or t_s < self.stroke.duration_s

    def held_state(self, plant_state: PlantState, board_time_us: float, wind: Wind) -> PlantState:
        """At rest on the carriage, then on the rail; from the release on, the plant's own state (its
        initial conditions are the release) until its first step."""
        t_s = self.stroke_time_s(board_time_us)
        if t_s is not None and t_s >= self.stroke.duration_s:
            return plant_state
        travel_m, speed, accel = self.stroke.at(0.0 if t_s is None else t_s)
        return rail_state(plant_state, self.geometry, travel_m / self.stroke.travel_m, speed, accel, wind)

    def release_pose(self) -> StartPose:
        """Where the plant starts: the rail's end, in the carriage's pitch, moving along the rail at the release speed."""
        n, e, d = self.geometry.direction_ned()
        travel, speed = self.stroke.travel_m, self.stroke.release_speed_m_s
        return StartPose(height_agl_ft=(self.geometry.height_m - travel * d) * FT_PER_M,
                         pitch_deg=self.geometry.pitch_deg, on_ground=False, north_m=travel * n, east_m=travel * e,
                         velocity_ned_m_s=(speed * n, speed * e, speed * d))
