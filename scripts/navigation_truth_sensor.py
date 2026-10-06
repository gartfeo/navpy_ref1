"""A perfect line-of-sight sensor: what an ideal camera would have measured.

This stands in for the whole vision chain. It exists so that a miss is
attributable to the navigation law and never to a camera: no intrinsics, no
gimbal transform, no pixel quantisation, no detector, no dropout, no field of
view. If the law misses here, the law is wrong.

WHAT THE LAW IS ALLOWED TO SEE
------------------------------
Simulator truth is used to work out WHERE THE TARGET APPEARS, and is then
discarded. What comes out is a TerminalVisionFrame: two UNIT rays plus the
aircraft's own pitch, roll, airspeed and yaw rate. A unit ray carries direction
and nothing else, so no range, no target position and no altitude can travel
inside it -- the law cannot recover them from what it is handed.

Yaw needs care, because the project rule bans compass yaw from the command path.
Two different things are called yaw here:

  * Truth yaw is needed to know where the target sits relative to the nose. A
    real camera answers that question by looking; a simulator has to compute it.
    It must be SIMULATOR TRUTH and never the ATTITUDE message, whose yaw is the
    compass-derived estimate -- feeding that in would rotate every ray by the
    compass bias and disguise it as a navigation error.
  * The control ray deliberately drops yaw. frame_projection.py:53 builds the
    control frame with `yaw=0.0`, using pitch and roll only, and this module
    matches that convention exactly.

So yaw shapes the geometry the sensor reports, exactly as the world's geometry
shapes what a camera sees, and no yaw-derived quantity reaches the law.
"""

from __future__ import annotations

import math

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation

# Matches every other construction of TerminalProjectionConfig in the tree
# (scripts/vision_static_point_mass_run.py:55, tests/.../vision_nav_camera_loop.py:54).
AIRCRAFT_SEQUENCE = "ZYX"

# WGS-84. The local tangent conversion below is only exact near the reference
# point, which is what it is used for.
EARTH_RADIUS_M = 6378137.0
EARTH_FLATTENING = 1.0 / 298.257223563


def _radii(lat_deg: float, height_m: float = 0.0) -> tuple[float, float]:
    """Meridional and transverse radii of curvature at this latitude.

    Using a single spherical radius instead would bias north/south against
    east/west by roughly half a percent at mid latitudes -- a systematic error
    that would look like a navigation bias.
    """
    sin_lat = math.sin(math.radians(lat_deg))
    eccentricity_sq = EARTH_FLATTENING * (2.0 - EARTH_FLATTENING)
    denominator = 1.0 - eccentricity_sq * sin_lat * sin_lat
    meridional = EARTH_RADIUS_M * (1.0 - eccentricity_sq) / denominator ** 1.5
    transverse = EARTH_RADIUS_M / math.sqrt(denominator)
    # HEIGHT MATTERS. The local scale at altitude h is (M + h) and (N + h), not
    # M and N: a degree subtends more ground the further you are from the
    # centre. Omitting it made every distance read 211 ppm short at the 1345 m
    # altitude these scoring intervals reach, so a requested 3000 m slant range was
    # actually built as 3000.714 m.
    #
    # The error is a fixed FRACTION of whatever is measured, and both
    # directions used the same wrong scale, so it cancelled out of the miss and
    # could not have biased any comparison -- 0.09 mm on the worst arm. Fixed
    # because the harness should measure the range it says it does, not because
    # anything it has reported was wrong.
    return meridional + height_m, transverse + height_m


def target_offset_ned_m(
    *,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    target_lat_deg: float,
    target_lon_deg: float,
    target_alt_m: float,
) -> np.ndarray:
    """Vector from the aircraft to the target, in local NED metres.

    A local tangent plane, not a full geodesic. Over the few kilometres a
    terminal scoring interval covers, the curvature error is far below the metre the
    score is quoted in; over hundreds of kilometres it would not be, which is
    why the radii are evaluated at the AIRCRAFT's latitude rather than assuming
    a sphere.
    """
    meridional, transverse = _radii(lat_deg, alt_m)
    north = math.radians(target_lat_deg - lat_deg) * meridional
    east = math.radians(target_lon_deg - lon_deg) * transverse * math.cos(
        math.radians(lat_deg)
    )
    # NED: down is positive, so a target BELOW the aircraft is a positive down.
    down = alt_m - target_alt_m
    return np.array([north, east, down], dtype=float)


def coordinates_from_offset_ned(
    *,
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    offset_ned_m: np.ndarray,
) -> tuple[float, float, float]:
    """Exact inverse of `target_offset_ned_m`, on the same tangent plane.

    Used once per run to turn a range/bearing placement into the fixed absolute
    target the scoring interval is flown against. It is the inverse of the forward
    conversion and not a second approximation of it, so the coordinates it
    returns reproduce the offset that was asked for.
    """
    meridional, transverse = _radii(lat_deg, alt_m)
    north, east, down = (float(value) for value in offset_ned_m)
    target_lat = lat_deg + math.degrees(north / meridional)
    target_lon = lon_deg + math.degrees(
        east / (transverse * math.cos(math.radians(lat_deg)))
    )
    return target_lat, target_lon, alt_m - down


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("line of sight has no direction")
    return vector / norm


def _rotation(pitch_deg: float, yaw_deg: float, roll_deg: float) -> np.ndarray:
    attitude = Attitude(pitch_deg, yaw_deg, roll_deg)
    return Rotation.from_euler(
        AIRCRAFT_SEQUENCE,
        get_euler_by_sequence(attitude, AIRCRAFT_SEQUENCE),
        degrees=True,
    ).as_matrix()


def build_frame(
    *,
    offset_ned_m: np.ndarray,
    truth_pitch_deg: float,
    truth_roll_deg: float,
    truth_yaw_deg: float,
    airspeed_mps: float,
    yaw_rate_rad_s: float,
    source_timestamp_s: float,
    est_pitch_deg: float | None = None,
    est_roll_deg: float | None = None,
    source_generation: int = 0,
    task_id: int = 1,
    obj_id: int = 1,
    source_name: str = "truth_los",
) -> TerminalVisionFrame:
    """One frame-local line of sight, as an ideal body-fixed camera would see it.

    `offset_ned_m` may be any length; only its direction survives, which is the
    structural reason range cannot leak to the law.

    Two attitudes go in and they are not interchangeable, and the split is not
    just about which numbers the law reads:

      * `truth_*` AIMS the ray. Where the target appears is a fact about the
        world -- the question a camera answers by looking, and one a simulator
        can only answer from truth.
      * `est_*` INTERPRETS it. De-rotating the camera into the command frame,
        and every attitude scalar the law reads, come from what the aircraft
        believes about itself, because that is all a real aircraft has.

    So an attitude estimate error does not move the target, but it does move
    where the law thinks the target is -- which is exactly what it does in
    flight. Where an estimate is not supplied the truth value stands in, which
    is right for an offline check with no estimator but must NOT be relied on
    when flying against SITL: it would quietly hand the law a perfect attitude.
    """
    if airspeed_mps <= 0.0:
        raise ValueError("airspeed must be positive")
    reported_pitch = truth_pitch_deg if est_pitch_deg is None else est_pitch_deg
    reported_roll = truth_roll_deg if est_roll_deg is None else est_roll_deg
    # NED -> body, using the FULL truth attitude. This is the "where is it
    # relative to the nose" question, and yaw is part of that question.
    body_ray = _unit(
        _rotation(truth_pitch_deg, truth_yaw_deg, truth_roll_deg).T
        @ np.asarray(offset_ned_m, dtype=float)
    )
    # body -> control, with yaw ZEROED, matching frame_projection.py:50-53 --
    # and with the REPORTED attitude, matching it there too. The real projector
    # builds this rotation from `observation.aircraft_pitch_deg` and
    # `observation.aircraft_roll_deg`, which are estimates, because an estimate
    # is all a real aircraft has to de-rotate its camera with.
    #
    # Using truth here instead was a hole, and a quiet one. The body ray is
    # aimed by truth, so de-rotating it by truth cancels exactly and an attitude
    # error cannot move the control ray at all -- the frame stays perfect no
    # matter how wrong the estimator is. On a real aircraft the two do not
    # cancel, and the residual is precisely the navigation error an attitude bias
    # causes. A harness that removes it cannot see the thing it was built to
    # measure.
    control_ray = _unit(
        _rotation(reported_pitch, 0.0, reported_roll) @ body_ray
    )
    return TerminalVisionFrame(
        source_name=source_name,
        source_generation=source_generation,
        task_id=task_id,
        obj_id=obj_id,
        source_timestamp_s=source_timestamp_s,
        body_x=float(body_ray[0]),
        body_y=float(body_ray[1]),
        body_z=float(body_ray[2]),
        control_x=float(control_ray[0]),
        control_y=float(control_ray[1]),
        control_z=float(control_ray[2]),
        # The ESTIMATE, not the truth -- see the note above.
        aircraft_roll_deg=float(reported_roll),
        air_speed_mps=float(airspeed_mps),
        aircraft_yaw_rate_rad_s=float(yaw_rate_rad_s),
        aircraft_pitch_deg=float(reported_pitch),
    )


class ClosestApproach:
    """Smallest distance the aircraft ever reached, found between samples too.

    Position arrives at a finite rate, so the sample nearest the target is
    almost never the closest the aircraft actually came. At 35 m/s and 50 Hz the
    aircraft moves 0.7 m per sample, so reading the miss straight off the
    samples would quote a number whose error is a large fraction of the miss
    itself. Each consecutive pair is therefore treated as a straight segment and
    the true minimum on that segment is used.

    Straight-line interpolation is honest here in a way it would not be for a
    slow feed: over one 20 ms step the flight path's curvature is negligible
    compared with the distance being measured.
    """

    def __init__(self) -> None:
        self.miss_m: float | None = None
        self.at_t_s: float | None = None
        # Split because the scalar cannot say which channel missed: all
        # vertical indicts the pitch law, all horizontal the roll law.
        #
        # HORIZONTAL IS ITS OWN MINIMISATION over the north-east track, not
        # the horizontal component at the 3D closest point. Found in review:
        # a dive through [10,0,100] -> [-10,0,0] crosses the target's ground
        # position exactly, yet at the 3D closest point -- pinned near the
        # low end by the altitude term -- the horizontal component reads
        # 9.6 m. A roll law scored that way answers for the pitch law's
        # timing. Vertical stays the component at the 3D closest point: an
        # independent vertical minimum would read near zero whenever the
        # path crosses the target's altitude anywhere.
        self.miss_horizontal_m: float | None = None
        self.miss_vertical_m: float | None = None
        self._previous: tuple[float, np.ndarray] | None = None

    def observe(self, t_s: float, offset_ned_m: np.ndarray) -> None:
        current = (t_s, np.asarray(offset_ned_m, dtype=float))
        if self._previous is not None:
            distance, when, closest = _segment_minimum(self._previous, current)
            horizontal, _, _ = _segment_minimum(
                (self._previous[0], self._previous[1][:2]),
                (t_s, current[1][:2]),
            )
        else:
            distance, when, closest = (
                float(np.linalg.norm(current[1])), t_s, current[1])
            horizontal = float(np.linalg.norm(current[1][:2]))
        if self.miss_m is None or distance < self.miss_m:
            self.miss_m = distance
            self.at_t_s = when
            self.miss_vertical_m = float(abs(closest[2]))
        if self.miss_horizontal_m is None or horizontal < self.miss_horizontal_m:
            self.miss_horizontal_m = horizontal
        self._previous = current


def _segment_minimum(
    start: tuple[float, np.ndarray],
    end: tuple[float, np.ndarray],
) -> tuple[float, float, np.ndarray]:
    """Closest approach to the origin along the segment: distance, when, where."""
    t0, a = start
    t1, b = end
    travel = b - a
    length_sq = float(travel @ travel)
    if length_sq <= 0.0:
        return float(np.linalg.norm(a)), t0, a
    # Fraction along the segment of the foot of the perpendicular, clamped so a
    # target passed before this segment is not reported as a future approach.
    fraction = max(0.0, min(1.0, float(-(a @ travel) / length_sq)))
    closest = a + travel * fraction
    return float(np.linalg.norm(closest)), t0 + (t1 - t0) * fraction, closest
