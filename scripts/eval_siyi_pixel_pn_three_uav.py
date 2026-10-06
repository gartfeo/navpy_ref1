"""Three-UAV finite-SIYI-pixel pure-vision navigation certificate."""

from __future__ import annotations

import subprocess
from pathlib import Path

from scripts import eval_direct_pixel_pn as one
from scripts import eval_direct_pixel_pn_three_uav as direct


def main() -> int:
    original_launch = one._launch_child
    original_wait = one._wait_ready

    def launch(*args: Path, **kwargs: object) -> subprocess.Popen[bytes]:
        kwargs["child_script"] = one.SCRIPTS / "siyi_pixel_pn_child.py"
        return original_launch(*args, **kwargs)

    def wait(path: Path, process: subprocess.Popen[bytes], timeout_s: float) -> None:
        return original_wait(path, process, timeout_s, marker="SIYI_PIXEL_READY")

    one._launch_child = launch
    one._wait_ready = wait
    return direct.main()


if __name__ == "__main__":
    raise SystemExit(main())
