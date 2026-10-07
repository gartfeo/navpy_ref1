#!/usr/bin/env python3
"""Recenter a SIYI gimbal and print one machine-readable result."""
import argparse
import json
import time

from navpy.modules.vision.peripheral.siyi.siyi_sdk import SIYISDK


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default="192.168.144.25")
    p.add_argument("--port", type=int, default=37260)
    p.add_argument("--settle", type=float, default=4.0)
    a = p.parse_args()
    sdk = SIYISDK(a.ip, a.port)
    result = {"pre_command_mono_ns": None, "post_command_mono_ns": None}
    try:
        result["connected"] = sdk.connect(maxWaitTime=3, maxRetries=1)
        if not result["connected"]:
            raise RuntimeError("SIYI control connection failed")
        result["attitude_before"] = sdk.getAttitude()
        result["mode"] = sdk.getMotionMode()
        result["pre_command_mono_ns"] = time.monotonic_ns()
        result["command_sent"] = sdk.requestCenterGimbal()
        result["post_command_mono_ns"] = time.monotonic_ns()
        time.sleep(a.settle)
        result["attitude_after"] = sdk.getAttitude()
        result["centering_feedback"] = sdk.getCenteringFeedback()
        result["ok"] = bool(result["command_sent"])
    finally:
        sdk.disconnect()
    print(json.dumps(result, separators=(",", ":")))
    if not result.get("ok"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
