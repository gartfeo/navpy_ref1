"""Host load during a HITL run (hitl_guidance.md section 9, step 27 "Simple checks": record CPU/GPU/temperature;
no crash or thermal throttling).

The runner starts it as its own process for the whole run. Each period it samples the host's CPU busy share
(``/proc/stat``, the receiving host: WSL, where navpy, the router and the publisher run) and the GPU through
``nvidia-smi`` (utilisation, temperature and the active clock-throttle reasons). It writes one CSV row per sample and,
on its stop (SIGINT), a JSON summary. ``ok`` needs samples, every GPU sample readable and no thermal or hardware
slowdown in any sample. It reads no run data and sends nothing.

    python -s -m tools.hitl.host_load --out-csv host_load.csv --out-json host_load.json
"""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Optional

READY_LINE = "host load: sampling"
PERIOD_S = 1.0                  # my value: one sample a second (a 10-minute observation holds 600)
NVIDIA_SMI_TIMEOUT_S = 5.0
GPU_QUERY = "utilization.gpu,temperature.gpu,clocks_throttle_reasons.active"
# nvidia-smi's clocks_throttle_reasons bits that mean the GPU slowed for heat or a hardware limit.
SLOWDOWN_BITS = {0x08: "hw_slowdown", 0x20: "sw_thermal_slowdown", 0x40: "hw_thermal_slowdown",
                 0x80: "hw_power_brake_slowdown"}
COLUMNS = ("t_s", "cpu_busy_pct", "gpu_util_pct", "gpu_temp_c", "gpu_throttle_mask", "gpu_error")


@dataclass(frozen=True)
class CpuTimes:
    busy: int
    total: int


@dataclass(frozen=True)
class GpuSample:
    util_pct: Optional[float]
    temp_c: Optional[float]
    throttle_mask: Optional[int]
    error: str = ""


def parse_cpu(text: str) -> CpuTimes:
    """The aggregate ``cpu`` line of /proc/stat: idle and iowait are not busy."""
    fields = next(line for line in text.splitlines() if line.startswith("cpu ")).split()[1:]
    values = [int(value) for value in fields]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return CpuTimes(sum(values) - idle, sum(values))


def busy_pct(before: CpuTimes, after: CpuTimes) -> Optional[float]:
    total = after.total - before.total
    return None if total <= 0 else 100.0 * (after.busy - before.busy) / total


def parse_gpu(text: str) -> GpuSample:
    """One GPU's ``util, temp, 0xMASK`` (the first GPU); an unreadable answer is a sample with its error."""
    line = text.strip().splitlines()[0] if text.strip() else ""
    parts = [part.strip() for part in line.split(",")]
    try:
        return GpuSample(float(parts[0]), float(parts[1]), int(parts[2], 16))
    except (IndexError, ValueError):
        return GpuSample(None, None, None, f"unreadable: {line!r}")


def read_gpu() -> GpuSample:
    try:
        done = subprocess.run(["nvidia-smi", f"--query-gpu={GPU_QUERY}", "--format=csv,noheader,nounits"],
                              capture_output=True, text=True, timeout=NVIDIA_SMI_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return GpuSample(None, None, None, f"{type(exc).__name__}: {exc}")
    return parse_gpu(done.stdout) if done.returncode == 0 else GpuSample(None, None, None, f"exit {done.returncode}")


def slowdowns(mask: Optional[int]) -> list[str]:
    return [] if mask is None else [name for bit, name in SLOWDOWN_BITS.items() if mask & bit]


def summarize(rows: list[dict]) -> dict:
    cpu = [row["cpu_busy_pct"] for row in rows if row["cpu_busy_pct"] is not None]
    util = [row["gpu_util_pct"] for row in rows if row["gpu_util_pct"] is not None]
    temps = [row["gpu_temp_c"] for row in rows if row["gpu_temp_c"] is not None]
    reasons = sorted({name for row in rows for name in slowdowns(row["gpu_throttle_mask"])})
    throttled = sum(1 for row in rows if slowdowns(row["gpu_throttle_mask"]))
    gpu_errors = sum(1 for row in rows if row["gpu_error"])
    return {"samples": len(rows), "period_s": PERIOD_S,
            "cpu_busy_max_pct": max(cpu, default=None), "cpu_busy_mean_pct": sum(cpu) / len(cpu) if cpu else None,
            "gpu_util_max_pct": max(util, default=None), "gpu_temp_max_c": max(temps, default=None),
            "throttled_samples": throttled, "throttle_reasons": reasons, "gpu_errors": gpu_errors,
            "ok": bool(rows) and throttled == 0 and gpu_errors == 0}


def sample(rows: list[dict], writer: csv.DictWriter, t_s: float, cpu: Optional[float], gpu: GpuSample) -> None:
    row = {"t_s": round(t_s, 3), "cpu_busy_pct": cpu, "gpu_util_pct": gpu.util_pct, "gpu_temp_c": gpu.temp_c,
           "gpu_throttle_mask": gpu.throttle_mask, "gpu_error": gpu.error}
    rows.append(row)
    writer.writerow(row)


def run(out_csv: IO[str], read_cpu: Callable[[], CpuTimes], gpu: Callable[[], GpuSample],
        clock: Callable[[], float], sleep: Callable[[float], None]) -> dict:
    """Sample until interrupted; the summary of every sample taken."""
    writer = csv.DictWriter(out_csv, fieldnames=COLUMNS)
    writer.writeheader()
    rows: list[dict] = []
    start, before = clock(), read_cpu()
    print(READY_LINE, flush=True)
    try:
        while True:
            sleep(PERIOD_S)
            after = read_cpu()
            sample(rows, writer, clock() - start, busy_pct(before, after), gpu())
            out_csv.flush()
            before = after
    except KeyboardInterrupt:
        pass
    return summarize(rows)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.host_load", description=__doc__.split("\n")[0])
    parser.add_argument("--out-csv", required=True)
    parser.add_argument("--out-json", required=True)
    args = parser.parse_args(argv)
    with open(args.out_csv, "x", encoding="utf-8", newline="") as out:
        summary = run(out, lambda: parse_cpu(Path("/proc/stat").read_text(encoding="ascii")), read_gpu,
                      time.monotonic, time.sleep)
    Path(args.out_json).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["READY_LINE", "SLOWDOWN_BITS", "busy_pct", "main", "parse_cpu", "parse_gpu", "run", "summarize"]
