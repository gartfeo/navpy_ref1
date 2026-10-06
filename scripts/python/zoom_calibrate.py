"""Measure commanded zoom vs actual readback zoom on a SIYI mount.

Commands each advertised calibration level, waits for the lens to
settle, reads hardware zoom, and prints a table of (commanded -> actual)
pairs. Use the output to inform a stepped-zoom calibration strategy in
PoiZoomTracker (the SIYI drift between commanded and readback — e.g.
commanded 1 produces actual 1.1 — is what makes label-based stepping
thrash).

    python scripts/python/zoom_calibrate.py
    python scripts/python/zoom_calibrate.py --settle 1.5
    python scripts/python/zoom_calibrate.py --levels 1,2,3,4,5,10
    python scripts/python/zoom_calibrate.py --ip 192.168.144.25 --port 37260
"""
from __future__ import annotations

import argparse
import time
from navpy.modules.vision.peripheral.gimbal_siyi import GimbalSiyi
from navpy.modules.vision.vision_profiles import (
    build_gimbal_data,
    get_devices,
    resolve_profile,
)


SIYI_PROFILE_NAME = "siyi_zr10"


class _Log:
    def info(self, *a, **k):
        print("[INFO]", *a)

    def warning(self, *a, **k):
        print("[WARN]", *a)

    def error(self, *a, **k):
        print("[ERR]", *a)

    def debug(self, *a, **k):
        pass


def parse_args():
    p = argparse.ArgumentParser(description="SIYI zoom calibration probe")
    p.add_argument("--ip", default="192.168.144.25",
                   help="SIYI gimbal IP (default: 192.168.144.25)")
    p.add_argument("--port", type=int, default=37260,
                   help="SIYI gimbal UDP port (default: 37260)")
    p.add_argument("--settle", type=float, default=2.5,
                   help="Seconds to wait after each set_zoom before reading "
                        "back the actual level (default: 2.5). Low zoom "
                        "levels need >= 2 s because the SIYI lens slews "
                        "slowly; 1 s leaves the lens mid-travel.")
    p.add_argument("--levels", default=None,
                   help="Comma-separated override for levels to probe (e.g. "
                        "1,2,3,4,5,10). Default: levels from the siyi_zr10 "
                        "vision profile.")
    p.add_argument("--samples", type=int, default=3,
                   help="How many readback samples to take per commanded "
                        "level, spaced 0.25 s apart (default: 3). Last "
                        "sample is the reported 'actual'; all samples are "
                        "printed so you can see slew/settle behaviour.")
    p.add_argument("--start", default=None,
                   help="Pre-position the lens to this commanded level "
                        "before the sweep starts. Use to probe whether "
                        "the commanded->actual mapping depends on prior "
                        "lens state (e.g. --start 10 approaches from the "
                        "top, --start 5 from the middle).")
    p.add_argument("--start-settle", type=float, default=3.0,
                   help="Seconds to wait after --start set_zoom (default: "
                        "3.0). Longer than --settle because a cold-start "
                        "from rest needs more time to reach the commanded "
                        "position.")
    p.add_argument("--reverse", action="store_true",
                   help="Sweep levels in descending order (high -> low) "
                        "instead of the default ascending.")
    p.add_argument("--trials", type=int, default=1,
                   help="Run the full sweep N times back-to-back and "
                        "print per-trial + per-level statistics. Useful "
                        "for checking determinism (default: 1).")
    return p.parse_args()


def get_default_levels(logger) -> list[str]:
    """Pull the SIYI_ZR10 profile's calibration labels."""
    _, profile, _ = resolve_profile(SIYI_PROFILE_NAME, logger)
    devices = get_devices(profile)
    if len(devices) != 1:
        raise ValueError(
            f"Expected exactly one device in profile '{SIYI_PROFILE_NAME}'"
        )
    camera_cfg = devices[0].get("camera", {})
    zooms = camera_cfg.get("intrinsics", {}).get("zooms", {})
    if not zooms:
        raise ValueError(
            f"Profile '{SIYI_PROFILE_NAME}' has no camera.intrinsics.zooms "
            f"entries"
        )
    return sorted(zooms.keys(), key=lambda k: float(k))


def build_siyi_gimbal(args, logger) -> GimbalSiyi:
    _, profile, _ = resolve_profile(SIYI_PROFILE_NAME, logger)
    devices = get_devices(profile)
    gimbal_data = build_gimbal_data(devices[0], 0, None)
    if gimbal_data is None:
        raise ValueError(
            f"Profile '{SIYI_PROFILE_NAME}' does not define gimbal data"
        )
    return GimbalSiyi(gimbal_data, args.ip, args.port, logger)


def probe_level(gimbal: GimbalSiyi, level: str, settle_s: float,
                samples: int, logger: _Log) -> tuple[float | None, list[float]]:
    """Command a level, sample readback, return (final_actual, samples)."""
    try:
        ok = gimbal.set_zoom(level)
    except Exception as e:  # noqa: BLE001 — hardware best-effort
        logger.error(f"set_zoom({level}) raised: {e}")
        return None, []
    if ok is False:
        logger.warning(f"set_zoom({level}) returned False")
        return None, []

    time.sleep(settle_s)

    readings: list[float] = []
    for i in range(max(1, samples)):
        try:
            v = gimbal.get_zoom_level()
        except Exception as e:  # noqa: BLE001 — hardware best-effort
            logger.warning(f"get_zoom_level raised: {e}")
            v = None
        if isinstance(v, (int, float)) and v > 0:
            readings.append(float(v))
        if i < samples - 1:
            time.sleep(0.25)

    if not readings:
        return None, []
    return readings[-1], readings


def main():
    args = parse_args()
    logger = _Log()

    if args.levels:
        levels = [s.strip() for s in args.levels.split(",") if s.strip()]
    else:
        levels = get_default_levels(logger)
    if args.reverse:
        levels = list(reversed(levels))
    logger.info(f"Probing levels: {levels}")
    logger.info(
        f"settle={args.settle}s  samples={args.samples}  trials={args.trials}"
        + (f"  start={args.start} (settle {args.start_settle}s)"
           if args.start else "")
    )

    gimbal = build_siyi_gimbal(args, logger)
    gimbal.start()
    if not gimbal.is_connected():
        raise RuntimeError(
            f"Failed to connect to SIYI gimbal at {args.ip}:{args.port}"
        )
    # Let the 20 Hz attitude/zoom poll populate state.
    time.sleep(1.0)

    # trials[trial_idx] = [(level, actual, samples), ...]
    trials: list[list[tuple[str, float | None, list[float]]]] = []
    try:
        for trial_idx in range(args.trials):
            logger.info(f"=== Trial {trial_idx + 1}/{args.trials} ===")
            if args.start is not None:
                logger.info(f"Pre-position: set_zoom({args.start!r})")
                try:
                    gimbal.set_zoom(args.start)
                except Exception as e:  # noqa: BLE001
                    logger.warning(f"pre-position set_zoom failed: {e}")
                time.sleep(args.start_settle)
                try:
                    z0 = gimbal.get_zoom_level()
                    logger.info(f"  actual after pre-position: {z0}")
                except Exception as e:  # noqa: BLE001
                    logger.warning(f"get_zoom_level after start failed: {e}")

            table: list[tuple[str, float | None, list[float]]] = []
            for lvl in levels:
                logger.info(f"--- set_zoom({lvl!r}) ---")
                actual, samples = probe_level(
                    gimbal, lvl, args.settle, args.samples, logger,
                )
                if actual is None:
                    logger.warning(f"  no readback for level {lvl!r}")
                else:
                    sample_str = " ".join(f"{s:.2f}" for s in samples)
                    logger.info(
                        f"  samples=[{sample_str}]  final={actual:.3f}"
                    )
                table.append((lvl, actual, samples))
            trials.append(table)
    finally:
        # Leave the lens at the lowest level so the rig is in a sane
        # starting state for the next run.
        try:
            if levels:
                lowest = min(levels, key=lambda s: float(s))
                gimbal.set_zoom(lowest)
        except Exception as e:  # noqa: BLE001
            logger.warning(f"return-to-lowest failed: {e}")
        time.sleep(0.5)
        gimbal.stop()

    # Summary per trial
    for idx, table in enumerate(trials):
        print()
        print("=" * 52)
        print(f"  Trial {idx + 1}/{len(trials)}")
        print(f"  {'commanded':>10}   {'actual':>8}   {'delta':>7}")
        print("=" * 52)
        for lvl, actual, _ in table:
            try:
                cmd_f = float(lvl)
            except ValueError:
                cmd_f = None
            if actual is None:
                print(f"  {lvl:>10}   {'  n/a':>8}   {'n/a':>7}")
            else:
                delta = (actual - cmd_f) if cmd_f is not None else float("nan")
                print(f"  {lvl:>10}   {actual:>8.3f}   {delta:+7.3f}")
        print("=" * 52)

    # Determinism summary across trials: for each level, show min/max/spread.
    if len(trials) >= 2:
        print()
        print("=" * 60)
        print("  Determinism (per-level across trials)")
        print(f"  {'level':>6}   {'min':>7}   {'max':>7}   {'spread':>7}   {'mean':>7}")
        print("=" * 60)
        for i, lvl in enumerate(levels):
            vals = [t[i][1] for t in trials if t[i][1] is not None]
            if not vals:
                print(f"  {lvl:>6}   {'n/a':>7}   {'n/a':>7}   {'n/a':>7}   {'n/a':>7}")
                continue
            vmin = min(vals)
            vmax = max(vals)
            spread = vmax - vmin
            mean = sum(vals) / len(vals)
            print(
                f"  {lvl:>6}   {vmin:>7.3f}   {vmax:>7.3f}   "
                f"{spread:>7.3f}   {mean:>7.3f}"
            )
        print("=" * 60)


if __name__ == "__main__":
    main()
