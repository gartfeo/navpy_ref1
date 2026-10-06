"""Read-only telemetry proof that the default-off simulator passes the window."""

import time

from gcs.backend import instance_ports as ip
from pymavlink import mavutil


def observe_disabled(vehicle: int) -> dict:
    # Beyond the prototype's 40..60 second source-time window. This is an
    # experiment acceptance point, not a navigation threshold.
    required_boot_ms = 65000
    deadline = time.monotonic() + 20
    connection = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{ip.companion_port(vehicle)}")
    try:
        while time.monotonic() < deadline:
            message = connection.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
            if (message is not None and message.get_srcSystem() == vehicle
                    and message.time_boot_ms >= required_boot_ms):
                return {"observed_boot_ms": int(message.time_boot_ms)}
        raise TimeoutError("default-off simulator did not pass the measured window")
    finally:
        connection.close()
