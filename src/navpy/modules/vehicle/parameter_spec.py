"""MAVLink parameter write validation."""
from __future__ import annotations

import math
from dataclasses import dataclass

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_PARAM_TYPE_INT8,
    MAV_PARAM_TYPE_INT16,
    MAV_PARAM_TYPE_INT32,
    MAV_PARAM_TYPE_INT64,
    MAV_PARAM_TYPE_REAL32,
    MAV_PARAM_TYPE_UINT8,
    MAV_PARAM_TYPE_UINT16,
    MAV_PARAM_TYPE_UINT32,
    MAV_PARAM_TYPE_UINT64,
)

from navpy.logger.cache_logger import ILogger


INTEGER_PARAMETER_TYPES = frozenset({
    MAV_PARAM_TYPE_INT8,
    MAV_PARAM_TYPE_INT16,
    MAV_PARAM_TYPE_INT32,
    MAV_PARAM_TYPE_INT64,
    MAV_PARAM_TYPE_UINT8,
    MAV_PARAM_TYPE_UINT16,
    MAV_PARAM_TYPE_UINT32,
    MAV_PARAM_TYPE_UINT64,
})


@dataclass(frozen=True)
class ParameterWriteSpec:
    name: str
    value: float
    expected_int: int | None
    wire_type: int
    strict_type_check: bool
    requested_type: int | None


def value_in_int_range(value: int | float, mav_param_type: int) -> bool:
    try:
        integer = int(value)
    except (TypeError, ValueError):
        return False
    bounds = {
        MAV_PARAM_TYPE_INT8: (-(1 << 7), (1 << 7) - 1),
        MAV_PARAM_TYPE_INT16: (-(1 << 15), (1 << 15) - 1),
        MAV_PARAM_TYPE_INT32: (-(1 << 31), (1 << 31) - 1),
        MAV_PARAM_TYPE_INT64: (-(1 << 63), (1 << 63) - 1),
        MAV_PARAM_TYPE_UINT8: (0, (1 << 8) - 1),
        MAV_PARAM_TYPE_UINT16: (0, (1 << 16) - 1),
        MAV_PARAM_TYPE_UINT32: (0, (1 << 32) - 1),
        MAV_PARAM_TYPE_UINT64: (0, (1 << 64) - 1),
    }.get(mav_param_type)
    return True if bounds is None else bounds[0] <= integer <= bounds[1]


def build_parameter_write_spec(
    name: str,
    value: int | float,
    mav_param_type: int | None,
    logger: ILogger,
) -> ParameterWriteSpec | None:
    try:
        numeric_value = float(value)
    except (TypeError, ValueError):
        logger.warning(
            f"set_parameter {name}: non-numeric value {value!r}.", "param"
        )
        return None
    if not math.isfinite(numeric_value):
        logger.warning(
            f"set_parameter {name}: non-finite value {value!r}; "
            "rejecting before send.",
            "param",
        )
        return None
    expected_int = None
    if mav_param_type in INTEGER_PARAMETER_TYPES:
        if numeric_value != int(numeric_value):
            logger.warning(
                f"set_parameter {name}: value {value!r} is not an integer "
                f"but MAV_PARAM_TYPE {mav_param_type} is integral; "
                "rejecting before send.",
                "param",
            )
            return None
        expected_int = int(numeric_value)
        if not value_in_int_range(expected_int, mav_param_type):
            logger.warning(
                f"set_parameter {name}: value {expected_int} out of range "
                f"for MAV_PARAM_TYPE {mav_param_type}; rejecting before send.",
                "param",
            )
            return None
    return ParameterWriteSpec(
        name=name,
        value=float(expected_int) if expected_int is not None else numeric_value,
        expected_int=expected_int,
        wire_type=mav_param_type or MAV_PARAM_TYPE_REAL32,
        strict_type_check=mav_param_type is not None,
        requested_type=mav_param_type,
    )
