"""Attitude stabilization state for the generic simulated gimbal."""

from __future__ import annotations

import threading
from dataclasses import replace

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.geo.rotation_utils import calculate_euler_angles
from navpy.modules.vision.gimbal_attitude_reader import AircraftAttitudeReader
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.utils.euler_utils import get_att_by_sequence, get_euler_by_sequence


class GimbalStabilizer:
    def __init__(
        self,
        data: GimbalData,
        attitude_reader: AircraftAttitudeReader,
        uas_sequence: str,
    ) -> None:
        self._data = data
        self._initial_attitude = data.att
        self._attitude_reader = attitude_reader
        self._uas_sequence = uas_sequence
        self._lock = threading.Lock()

    def advance(self) -> None:
        aircraft = self._attitude_reader.read()
        if aircraft is None:
            return
        with self._lock:
            data = self._data
        if not data.roll_stabilize and not data.pitch_stabilize:
            return
        roll = aircraft.roll if data.roll_stabilize else 0.0
        pitch = aircraft.pitch if data.pitch_stabilize else 0.0
        aircraft_euler = get_euler_by_sequence(
            Attitude(pitch, 0.0, roll),
            self._uas_sequence,
        )
        gimbal_euler = get_euler_by_sequence(
            self._initial_attitude,
            data.g_seq,
        )
        euler = calculate_euler_angles(
            self._uas_sequence,
            aircraft_euler,
            data.g_seq,
            gimbal_euler,
            degrees=True,
        )
        with self._lock:
            self._data = replace(
                self._data,
                att=get_att_by_sequence(euler, data.g_seq),
            )

    def get_data(self) -> GimbalData:
        with self._lock:
            return self._data

    def set_att(self, attitude: Attitude) -> None:
        with self._lock:
            self._data = replace(
                self._data,
                att=attitude,
                roll_stabilize=False,
                pitch_stabilize=False,
            )


__all__ = ["GimbalStabilizer"]
