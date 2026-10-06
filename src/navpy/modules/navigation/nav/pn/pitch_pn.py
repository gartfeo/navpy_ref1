import math

import numpy as np


class PitchPn:
    r"""
    Pitch controller: geometric feedforward + Kp closed-loop.

    Control law:
        pitch_cmd = -los_angle + Kp*(current_pitch - pitch_error)
                    \_________/   \_____________________________/
                    feedforward        closed-loop (like PID)

    The feedforward term points at target based on NED geometry.
    The Kp term provides closed-loop wind rejection (same as PID).
    """

    def __init__(self, kp: float, out_min: float, out_max: float):
        self._kp = kp
        self._out_min = out_min
        self._out_max = out_max

    def reset(self, kp: float, out_min: float, out_max: float) -> None:
        self._kp = kp
        self._out_min = out_min
        self._out_max = out_max

    def calc(self, target_ned: np.ndarray, pitch_error: float, current_pitch: float) -> float:
        """
        Calculate pitch command.

        Args:
            target_ned: Normalized NED direction vector to target
            pitch_error: Vision-based pitch error (deg)
            current_pitch: Current aircraft pitch (deg)

        Returns:
            Commanded pitch (deg, negative = nose down)
        """
        # LOS elevation angle from NED vector (rad)
        n, e, d = target_ned
        horiz = math.sqrt(n * n + e * e)
        los_angle = math.atan2(d, max(horiz, 1e-6))

        # Feedforward: point at target
        pitch_cmd_rad = -los_angle

        # Closed-loop: Kp correction for wind rejection
        if pitch_error is not None:
            pitch_cmd_rad += self._kp * math.radians(-pitch_error + current_pitch)

        pitch_cmd_deg = math.degrees(pitch_cmd_rad)
        return max(self._out_min, min(pitch_cmd_deg, self._out_max))
