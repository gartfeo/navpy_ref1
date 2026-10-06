import math
import numpy as np


class Rotation:
    """Simple replacement for scipy.spatial.transform.Rotation."""

    def __init__(self, matrix: np.ndarray):
        self._matrix = np.asarray(matrix, dtype=float)

    @classmethod
    def from_euler(cls, seq: str, angles, degrees: bool = False):
        """Create rotation from Euler angles.

        The semantics mirror ``scipy.spatial.transform.Rotation.from_euler``:
        sequences with lower case letters are interpreted as intrinsic
        rotations, while upper case denotes extrinsic rotations.
        """

        angles = np.asarray(angles, dtype=float).reshape(-1)
        if degrees:
            angles = np.deg2rad(angles)

        intrinsic = seq.islower()
        axes = seq.lower()
        if intrinsic:
            # intrinsic rotations about body axes are equivalent to
            # extrinsic rotations about the reversed axes in reversed order
            axes = axes[::-1]
            angles = angles[::-1]

        R = np.eye(3)
        for axis, angle in zip(axes.upper(), angles):
            c = math.cos(angle)
            s = math.sin(angle)
            if axis == 'X':
                m = np.array([[1, 0, 0],
                              [0, c, -s],
                              [0, s, c]])
            elif axis == 'Y':
                m = np.array([[c, 0, s],
                              [0, 1, 0],
                              [-s, 0, c]])
            elif axis == 'Z':
                m = np.array([[c, -s, 0],
                              [s, c, 0],
                              [0, 0, 1]])
            else:
                raise ValueError(f"Invalid axis '{axis}'")
            R = R @ m
        return cls(R)

    @classmethod
    def from_matrix(cls, matrix):
        return cls(np.asarray(matrix, dtype=float))

    def as_matrix(self) -> np.ndarray:
        return self._matrix.copy()

    def apply(self, vectors):
        vec = np.asarray(vectors, dtype=float)
        if vec.ndim == 1:
            return self._matrix @ vec
        return (self._matrix @ vec.T).T

    def inv(self):
        return Rotation(self._matrix.T)

    def as_euler(self, seq: str, degrees: bool = False):
        seq = seq.upper()
        m = self._matrix
        if seq == 'ZYX':
            y = math.atan2(-m[2, 0], math.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2))
            x = math.atan2(m[2, 1], m[2, 2])
            z = math.atan2(m[1, 0], m[0, 0])
            angles = [z, y, x]
        elif seq == 'XYZ':
            # β  -------------------------------------------------------------
            beta = math.asin(m[0, 2])
            cb = math.cos(beta)

            # α, γ -----------------------------------------------------------
            if abs(cb) < 1e-8:  # gimbal-lock: |cos β| ≈ 0
                alpha = math.atan2(m[2, 1], m[2, 2])
                gamma = 0.0  # γ is not observable
            else:
                alpha = math.atan2(-m[1, 2], m[2, 2])
                gamma = math.atan2(-m[0, 1], m[0, 0])

            angles = [alpha, beta, gamma]
        elif seq == 'YZX':
            b = math.asin(m[1, 0])
            a = math.atan2(-m[2, 0], m[0, 0])
            c = math.atan2(-m[1, 2], m[1, 1])
            angles = [a, b, c]
        elif seq == 'ZXY':
            b = math.asin(m[2, 1])
            a = math.atan2(-m[0, 1], m[1, 1])
            c = math.atan2(-m[2, 0], m[2, 2])
            angles = [a, b, c]
        elif seq == 'ZYZ':
            b = math.acos(m[2, 2])
            a = math.atan2(m[1, 2], m[0, 2])
            c = math.atan2(m[2, 1], -m[2, 0])
            angles = [a, b, c]
        else:
            raise ValueError(f"Sequence '{seq}' not supported")
        angles = np.array(angles)
        if degrees:
            wrap = 180.0
            angles = np.rad2deg(angles)
        else:
            wrap = math.pi

        angles = (angles + wrap) % (2 * wrap) - wrap
        if abs(angles[-1] + wrap) < 1e-7:
            angles[-1] = wrap
        return angles
