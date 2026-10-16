"""HITL boot order: the estimator must start on the bridge's state (Board.start, board.py).

Before its first state the HIL backend shows the aircraft level at the origin (fork libraries/SITL/SIM_HIL.cpp); when
EKF3 aligns on that and the first state then jumps to the aircraft on its rail, EKF3 takes the jump partly as gyro
bias and carries degrees of attitude error for minutes (sign_check2, 2026-10-13: 5 deg of pitch on the rail, 3-5 deg
in flight, which FBWA flew as a steady descent). Booting with the bridge already answering puts the first state ahead
of the EKF's initialisation; the boot's STATUSTEXTs (sent in order) show which came first.
"""
from __future__ import annotations

import re
import time
from typing import Protocol

HIL_FIRST_STATE_TEXT = "HIL: first state received"          # SIM_HIL.cpp report_link_state
EKF_INIT_TEXT = re.compile(r"EKF\d IMU\d+ initialised")       # AP_NavEKF3_core.cpp InitialiseFilterBootstrap
HIL_BOOT_ATTEMPTS = 3          # reboots for a boot whose first state came after the EKF's initialisation
HIL_BOOT_TIMEOUT_S = 60.0      # from the reconnect to the EKF's initialisation text
PUMP_S = 0.5                   # one wait for the boot's messages (board.OPEN_RETRY_S)


class BootingBoard(Protocol):
    messages: list[str]

    def reboot(self) -> None: ...

    def pump(self, timeout: float = 0.05) -> object: ...


def boot_on_bridge(board: BootingBoard) -> int:
    """Reboot with the HITL processes running until a boot's first HIL state came before the EKF's initialisation;
    returns the boots it took. RuntimeError after HIL_BOOT_ATTEMPTS boots that did not."""
    for attempt in range(1, HIL_BOOT_ATTEMPTS + 1):
        seen = len(board.messages)
        board.reboot()
        if state_before_ekf(board, seen):
            return attempt
        print(f"[board] boot {attempt}: the EKF initialised before the first HIL state; rebooting", flush=True)
    raise RuntimeError(f"in {HIL_BOOT_ATTEMPTS} boots the EKF initialised before the first HIL state arrived: "
                       "it would start on the backend's level placeholder, not the bridge's aircraft")


def state_before_ekf(board: BootingBoard, seen: int) -> bool:
    """Wait for the EKF's initialisation text after messages[seen]; True if the first HIL state came first."""
    deadline = time.time() + HIL_BOOT_TIMEOUT_S
    while time.time() < deadline:
        texts = board.messages[seen:]
        ekf = next((i for i, t in enumerate(texts) if EKF_INIT_TEXT.search(t)), None)
        if ekf is not None:
            return any(HIL_FIRST_STATE_TEXT in t for t in texts[:ekf])
        board.pump(PUMP_S)
    raise RuntimeError(f"no EKF initialisation text within {HIL_BOOT_TIMEOUT_S:g} s of the reboot")
