"""MAVLink File Transfer Protocol support.

Two parts:

- `_upstream_mavftp.py` and `_upstream_mavftp_op.py` are vendored from
  upstream pymavlink (https://github.com/ArduPilot/pymavlink). They
  carry the upstream `SPDX-License-Identifier: GPL-3.0-or-later`. NavPy
  is currently licensed proprietarily; the project will be re-licensed
  appropriately before redistribution. (Tracked separately.)

- `vehicle_ftp.py` is the NavPy-owned bridge between upstream `MAVFTP`
  and our `VehicleMav` / `MavBus` single-reader architecture.

- `param_pck.py` is the NavPy-owned `@PARAM/param.pck` parser
  (clean-room; not derived from MAVProxy/pymavlink source).
"""
from .param_pck import (
    AP_TYPE_FLOAT,
    AP_TYPE_INT8,
    AP_TYPE_INT16,
    AP_TYPE_INT32,
    AP_TYPE_NONE,
    FLAG_HAS_DEFAULT,
    PCK_MAGIC,
    ParamPck,
    ParamPckError,
    ParamRecord,
    parse_param_pck,
)

__all__ = [
    "AP_TYPE_FLOAT",
    "AP_TYPE_INT8",
    "AP_TYPE_INT16",
    "AP_TYPE_INT32",
    "AP_TYPE_NONE",
    "FLAG_HAS_DEFAULT",
    "PCK_MAGIC",
    "ParamPck",
    "ParamPckError",
    "ParamRecord",
    "parse_param_pck",
]
