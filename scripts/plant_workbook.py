"""Collect the SITL plant sweeps into one .xlsx, one row per aircraft.

Writes the workbook directly (xlsx is a zip of XML) so this needs no Excel
library installed.

This script IS the definition of every derived column in the sheet. The
workbook holds static values and no formulas, so without this file nothing in
it -- TAS, angle of attack, pitch drift, the velocity-triangle closure -- can be
checked or rebuilt. It reads only the run artefacts under .sitl-runs, and each
row carries the exact directory it came from in its `source` column, so the
whole sheet regenerates from the raw runs and any cell traces back to the
flight that produced it.

Nothing here is read from result.json when it can be recomputed from the raw
per-sample series instead:

  course     circular mean, because bearings wrap at north. One run predates
             that fix and its stored course is wrong wherever the track crossed
             north.
  altitude   integrated backwards from the recorded end altitude, so the
             density correction is evaluated over the same window as the
             airspeed it corrects.
  TAS        reported channel / sigma, because SITL converts TAS to EAS twice.
  AoA        against the AIR path angle, not the ground one.

Run from the repository root:

    python scripts/plant_workbook.py <out.xlsx>

Set PLANT_WORKBOOK_TABS=1 to also emit the derived matrix and notes tabs; by
default only the single flat Data sheet is written, since the rest is a pivot
away.
"""
from __future__ import annotations

import glob
import json
from collections.abc import Callable
from typing import Any
import math
import os
import statistics
import sys
import zipfile

# Launcher default start point. NOT sea level, which is why reported (equivalent)
# airspeed sits ~10% under true airspeed at these hold altitudes.
HOME_MSL_M = 1294.86

RUNS = [
    # (path, tag, description, verdict, hold_s)
    #
    # HOLD DURATION IS PART OF THE IDENTITY OF A RUN. The phugoid -- the slow
    # speed/altitude oscillation -- has a period near 4.44*V/g, about 13.6 s
    # here. A 20 s hold is barely 1.5 periods, so at shallow pitch the aircraft
    # is still swinging when the mean is taken and replicates disagree by over a
    # degree of path angle and tens of degrees of course. At 90 s the same cells
    # repeat to 0.14 deg and a couple of degrees. Do not pool 20 s and 90 s rows
    # when judging reproducibility.
    (".sitl-runs/sitl-scale-20260815-091605", "shallow_thr_90s",
     "Shallow pitch (-5,-10,-15) x 5 throttle, 90 s hold, 3 repeats",
     "PASS all 3 (clock 1.029-1.073)", 90),
    (".sitl-runs/sitl-scale-20260815-091732", "shallow_wind_90s",
     "Shallow pitch (-5,-10,-15) x 5 track-relative wind, 90 s hold, 3 repeats",
     "PASS all 3 (clock 1.021)", 90),
    (".sitl-runs/sitl-scale-20260815-004418", "throttle",
     "Throttle sweep: 8 pitch x 5 throttle, no wind", "PASS (clock 0.977)", 20),
    (".sitl-runs/sitl-scale-20260815-090837", "throttle",
     "Throttle sweep, 3 replicates", "PASS / VOID x2 -- clock does not affect "
     "physics (0.023 deg across a 0.529-vs-1.0 test)", 20),
    (".sitl-runs/sitl-scale-20260815-085101", "roll",
     "Roll sweep: 8 pitch x 5 roll (-20..+20), no wind", "VOID (clock 0.934)", 20),
    (".sitl-runs/sitl-scale-20260815-085856", "roll",
     "Roll sweep, 3 replicates", "PASS x2 / VOID x1", 20),
    (".sitl-runs/sitl-scale-20260815-085132", "wind_rel",
     "Wind with directions RELATIVE to the ~275 deg track: 275=head, 95=tail, "
     "5/185=cross", "PASS (clock 1.0039)", 20),
    (".sitl-runs/sitl-scale-20260815-090143", "wind_rel",
     "Track-relative wind, 3 replicates", "FAIL / VOID x2 on capacity only", 20),
    (".sitl-runs/sitl-scale-20260815-005735", "speed",
     "Speed invariance: same grid at 1x and 10x", "PASS both", 20),
    # The settling control for STEEP pitch. The 90 s grids only reach -15, so
    # until this run the claim "throttle biases pitch tracking" had never been
    # tested at a steep dive with a settled hold. 60 s is ~4 phugoid periods and
    # is the longest that fits: at -40 the aircraft sinks ~24 m/s, so 60 s costs
    # 1435 m and the hold has to start at 1500 m to stay off the floor.
    (".sitl-runs/sitl-scale-20260815-094809", "steep_thr_60s",
     "Steep pitch (-20..-40) x 5 throttle, 60 s hold, 3 repeats",
     "PASS all 3 (clock 1.009-1.084)", 60),
    # RETIRED: .sitl-runs/sitl-scale-20260815-002701
    # Not deleted and not badly executed -- every hold completed and the body-
    # frame numbers are sound. Retired because its wind DIRECTIONS were set in
    # the world frame while heading was never commanded. The track settled near
    # 275 deg, so "8@0 headwind" delivered 6.4-8.0 m/s of CROSSWIND and
    # "8@90 crosswind" delivered a near-pure TAILWIND. The labels describe an
    # experiment that did not happen. Superseded by the track-relative grids.
]


def mean_bearing(values: list[float]) -> float | None:
    """Circular mean. The arithmetic mean puts 359 and 1 at 180."""
    if not values:
        return None
    radians = [math.radians(v) for v in values]
    east = statistics.fmean(math.sin(r) for r in radians)
    north = statistics.fmean(math.cos(r) for r in radians)
    if math.hypot(east, north) < 0.1:
        return None
    return round(math.degrees(math.atan2(east, north)) % 360.0, 2) % 360.0


def tail_mean_alt(series: list[list[float]], end_alt: float) -> float:
    """Mean relative altitude over the SETTLED TAIL, rebuilt from the samples.

    The density correction has to be evaluated over the same window as the
    airspeed it corrects, and every held value here is a second-half mean.
    result.json only offers max_rel_alt_m and end_rel_alt_m, which span the
    WHOLE hold; their midpoint sits well above the tail's. On a 20 s hold the
    gap is ~51 m and hides. On a 60 s steep dive the aircraft loses over a
    kilometre, the gap reaches ~370 m, and the density is wrong by ~4%.

    samples.json carries no altitude column, so height is integrated backwards
    from the recorded end altitude using the recorded sink rate,
    gs*tan(-path angle). Only recorded columns and one recorded endpoint --
    nothing fitted.

    Checked against the zero-wind identity GS = TAS*cos(path angle), which must
    hold exactly, over 385 wings-level calm aircraft:

        full-hold midpoint    mean -0.417  RMS 0.529 m/s, and the bias grows
                              6x with hold length (-0.24 at 20 s, -1.47 at 57 s)
        this tail rebuild     mean -0.124  RMS 0.145 m/s, roughly flat in hold

    The disappearance of the hold-length trend is what identifies it as the
    right window rather than merely the smaller residual.
    """
    alts = [0.0] * len(series)
    alts[-1] = end_alt
    for i in range(len(series) - 1, 0, -1):
        step = (series[i][1] - series[i - 1][1]) / 1000.0
        sink = statistics.fmean(
            s[5] * math.tan(math.radians(-s[7])) for s in (series[i], series[i - 1]))
        alts[i - 1] = alts[i] + sink * step
    return statistics.fmean(alts[len(alts) // 2:])


def tas_of(row: dict[str, Any], mean_alt: float | None) -> float | None:
    """True airspeed from the reported airspeed channel.

    That channel is NOT equivalent airspeed. ArduPilot SITL converts TAS to EAS
    in SIM_Aircraft.cpp (`airspeed = velocity_air_ef.length() / eas2tas`), hands
    the already-converted value to SITL_State::_update_airspeed() -- whose
    parameter is named `true_airspeed` -- and that divides by EAS2TAS a SECOND
    time. The reported number is therefore TAS*sigma, so recovering TAS divides
    by sigma once, not by sqrt(sigma) as an equivalent airspeed would need.

    Measured on the no-wind aircraft: dividing by sigma closes the speed
    triangle at 0.9922 +-0.0026, against 1.0889 +-0.0061 for sqrt(sigma), and
    the sqrt version's leftover tracks altitude at r=0.97 -- the missing density
    factor showing itself.

    Consequence past this sheet: a real pitot reports EAS = TAS*sqrt(sigma), so
    SITL's airspeed reads ~8% BELOW a real sensor at this density altitude. Any
    airspeed-scheduled navigation gain tuned against SITL inherits that offset.
    """
    reported = row.get("held_airspeed_mps")
    if reported is None or mean_alt is None:
        return None
    sigma = (1 - 2.25577e-5 * (HOME_MSL_M + mean_alt)) ** 4.2561
    return round(reported / sigma, 2) if sigma > 0 else None


def aoa_of(row: dict[str, Any], pitch: float | None,
           ground_angle: float | None) -> float | None:
    """Angle of attack, which needs the AIR path angle and not the ground one.

    pitch - ground_path_angle is only the angle of attack in still air. The
    path angle recorded here comes from GLOBAL_POSITION_INT ground velocity
    (scripts/scratch_sitl_uav.py, path_angle = atan2(-down, ground_speed)), so
    in wind it is the GROUND path angle and the subtraction silently adds the
    whole wind-induced track rotation into what is meant to be an aerodynamic
    angle. This is the same air-versus-ground confusion the velocity-triangle
    closure had; it was fixed there and had been left standing here.

    Vertical speed is frame-free because the simulated wind is horizontal, so
    the air path angle follows with nothing fitted:

        Vz        = GS*tan(ground path angle)
        gamma_air = asin(Vz / TAS)
        AoA       = pitch - gamma_air

    Returned only for wings-level rows. With bank, angle of attack needs the
    air-relative velocity resolved into the body frame, and that needs a
    heading, which this harness never records -- ATTITUDE yaw is not sampled and
    GLOBAL_POSITION_INT gives course, not heading. There is no honest value to
    put in those cells, so they stay empty rather than carry a wings-level
    formula applied to a turning aircraft.
    """
    tas, gs = row.get("_tas"), row.get("held_ground_speed_mps")
    if None in (pitch, ground_angle, tas, gs) or not tas:
        return None
    if row.get("cmd_roll_deg"):
        return None
    vertical = gs * math.tan(math.radians(ground_angle))
    ratio = vertical / tas
    if not -1.0 <= ratio <= 1.0:
        return None
    return round(pitch - math.degrees(math.asin(ratio)), 2)


def closure_of(row: dict[str, Any]) -> float | None:
    """Per-row self-check: does this row's own numbers form a valid flight?

    The velocity triangle is not an opinion. Vertical speed is GS*tan(path
    angle), and because the simulated wind is horizontal that same vertical
    speed appears in the air frame, which pins the horizontal air speed at
    sqrt(TAS^2 - Vz^2). The aircraft crabs until its crosswind is cancelled, so
    only sqrt(Va_h^2 - crosswind^2) survives along the course, plus the tailwind:

        GS = sqrt(TAS^2 - (GS*tan(path))^2 - crosswind^2) + tailwind

    Nothing here is fitted. It caught two real convention errors: using
    TAS*cos(path) for the horizontal air speed assumes the air and ground path
    angles are equal, true only in calm, and it missed by -1.9 m/s at 14 m/s
    wind; dropping the crab term missed by -0.8 m/s. Both are gone.

    READ THIS COLUMN RELATIVE TO ITS OWN HOLD FAMILY, NOT AGAINST ZERO. The
    residual is NOT condition-independent, and an earlier claim here that it was
    is withdrawn. It was checked against calm (-0.141), windy (-0.102) and
    banked (-0.120), which do agree -- but not against hold length, which does
    not:

        20 s  mean -0.094   90 s  mean -0.227   corr(hold, closure) = -0.654

    Computing the whole identity PER SAMPLE, before any averaging, leaves that
    split intact (-0.091 vs -0.230), so it is not an artefact of applying a
    nonlinear identity to separately-averaged means. Nor is it the home
    altitude: 1294.86 m is the launcher's own default and the aircraft echo
    "Field Elevation Set: 1295m". Solving for the home that would zero each
    family gives 1267 m for 20 s against 1226/1219 m for 60/90 s, so no single
    home explains both. At MATCHED altitude the 60 s and 90 s families agree
    with each other and both differ from 20 s, which rules out a pure density
    explanation too.

    So an offset of 0.09-0.25 m/s remains -- 0.29% of true airspeed on the 20 s
    rows, 0.68-0.75% on the 60 s and 90 s ones -- it tracks hold length, and its
    cause is not established. Measured ground speed always falls BELOW what the
    triangle predicts, so it is a bias and not scatter.

    What would settle it is recording absolute altitude and NED velocity per
    sample. Then density needs no reconstruction and ground speed needs no
    re-derivation, which removes every candidate cause at once: an imperfect
    altitude integration, ISA differing from the simulator's own density, and
    airspeed arriving on VFR_HUD while position arrives on GLOBAL_POSITION_INT
    at a different cadence, so the two averages cover slightly different spans.

    It is left standing rather than tuned away -- absorbing it into a fitted
    constant would destroy the column's only value, which is being a check
    nothing was fitted to.
    """
    tas, gs, angle = (row.get("_tas"), row.get("held_ground_speed_mps"),
                      row.get("held_path_angle_deg"))
    if None in (tas, gs, angle):
        return None
    cross = row.get("_crosswind") or 0.0
    vertical = gs * math.tan(math.radians(angle))
    inner = tas * tas - vertical * vertical - cross * cross
    if inner <= 0:
        return None
    return round(gs - (math.sqrt(inner) + (row.get("_tailwind") or 0.0)), 3)


def collect() -> list[dict[str, Any]]:
    out = []
    for root, tag, _, _, hold in RUNS:
        for path in sorted(glob.glob(os.path.join(root, "**", "result.json"),
                                     recursive=True)):
            try:
                row = json.load(open(path, encoding="utf-8"))
            except (OSError, ValueError):
                continue
            folder = os.path.dirname(path)
            row["_run"] = tag
            row["_level"] = os.path.basename(os.path.dirname(folder))
            # Re-derived, never trusted from the file.
            try:
                series = json.load(open(os.path.join(folder, "samples.json"),
                                        encoding="utf-8"))
            except (OSError, ValueError):
                series = []
            # Samples taken before a GPS fix are written short. A missing column
            # is absence of data, not a zero, so those rows leave rather than
            # drag a mean toward the origin.
            series = [s for s in series
                      if len(s) > 7 and s[5] is not None and s[7] is not None]
            tail = series[len(series) // 2:] if series else []
            # Hold length is MEASURED, not taken from the launch flag: boot_ms is
            # the autopilot's own clock, so it already runs at aircraft rate and
            # needs no un-scaling by the sim speedup. Hold length is part of the
            # identity of a row -- see the note on RUNS -- so it must be the
            # length that actually ran.
            row["_hold_s"] = (round((series[-1][1] - series[0][1]) / 1000.0)
                              if len(series) > 1 else hold)
            row["_alt_mean"] = (round(tail_mean_alt(series, row["end_rel_alt_m"]), 1)
                                if len(series) > 1
                                and row.get("end_rel_alt_m") is not None else None)
            row["_tas"] = tas_of(row, row["_alt_mean"])
            # Pitch still moving at the END of the window the mean was taken
            # over. A settled hold has none; anything else means the reported
            # mean is a snapshot of a number in motion, and its value depends on
            # when you looked. This column is what shows the 20 s rows to be
            # unsettled and the 90 s rows to be trim.
            row["_drift"] = None
            if len(tail) > 8:
                span = (tail[-1][1] - tail[0][1]) / 1000.0
                if span > 0:
                    times = [(s[1] - tail[0][1]) / 1000.0 for s in tail]
                    pitches = [s[2] for s in tail]
                    mt, mp = statistics.fmean(times), statistics.fmean(pitches)
                    var = sum((t - mt) ** 2 for t in times)
                    if var > 0:
                        row["_drift"] = round(sum(
                            (t - mt) * (p - mp) for t, p in zip(times, pitches)
                        ) / var, 4)
            # Drift alone does NOT prove a row settled. A pitch oscillation
            # symmetric about the window has near-zero net slope while swinging
            # wildly: the worst row here drifts only -0.036 deg/s yet its pitch
            # ranges over 18.5 deg. Spread is the companion test, and a row is
            # trim only when BOTH are small.
            row["_pitch_sd"] = (round(statistics.pstdev(s[2] for s in tail), 3)
                                if len(tail) > 2 else None)
            row["_pitch_range"] = (round(max(s[2] for s in tail)
                                         - min(s[2] for s in tail), 2)
                                   if tail else None)
            # Exact artefact path. The friendly run label is for reading; this is
            # what makes a row traceable back to the flight that produced it, and
            # what lets the whole sheet be regenerated from the raw runs.
            row["_source"] = os.path.relpath(folder).replace("\\", "/")
            course = [s[6] for s in tail if s[6] is not None]
            row["_course"] = mean_bearing(course)
            row["_crossed_north"] = bool(course and max(course) - min(course) > 180)
            pitch, angle = row.get("held_pitch_deg"), row.get("held_path_angle_deg")
            row["_aoa"] = aoa_of(row, pitch, angle)
            cmd = row.get("cmd_pitch_deg")
            row["_pitch_err"] = (round(pitch - cmd, 2)
                                 if None not in (pitch, cmd) else None)
            # A run that never passed --winds left SIM_WIND_SPD alone, so the
            # field is null rather than zero. ArduPilot's default is zero, and
            # the measured ground-speed-to-airspeed ratio in those cells
            # (1.0889) matches the cells where zero was set explicitly (1.0891)
            # to 0.0002 -- the same still air by measurement, not by assumption.
            # Recorded as 0 so the column stays numeric and pivots cleanly.
            row["_wind_speed"] = row.get("wind_speed_mps") or 0.0
            row["_wind_dir"] = row.get("wind_dir_deg") or 0.0
            # Wind resolved onto the aircraft's OWN ground track. SIM_WIND_DIR
            # is the direction the wind comes FROM, so the along-track push is
            # -W*cos(wind_dir - course). This is the quantity that actually
            # moves ground speed: fitting against the scalar wind speed instead
            # explains only R2=0.52 of it, against 0.99 with this.
            speed, course = row["_wind_speed"], row.get("_course")
            if course is None:
                row["_tailwind"] = row["_crosswind"] = None
            else:
                offset = math.radians(row["_wind_dir"]) - math.radians(course)
                row["_tailwind"] = round(-speed * math.cos(offset), 2)
                row["_crosswind"] = round(-speed * math.sin(offset), 2)
            row["_closure"] = closure_of(row)
            out.append(row)
    return out


# INPUTS first, then the measured response, then the run context. One row is
# one flight condition, so the sheet sorts and pivots on the input columns
# without any rearranging.
COLUMNS = [
    # --- commanded: the sweep axes ---
    ("pitch cmd deg", "cmd_pitch_deg", 13),
    ("throttle cmd", "cmd_throttle", 12),
    ("roll cmd deg", "cmd_roll_deg", 12),
    ("wind speed m/s", "_wind_speed", 13),
    ("wind dir deg", "_wind_dir", 12),
    ("sim speed", "sim_speedup", 10),
    # Not cosmetic: 20 s is ~1.5 phugoid periods and does not settle shallow
    # pitch; 90 s does. Reproducibility must be judged within one hold length.
    ("hold s", "_hold_s", 8),
    # Derived, but they belong beside the inputs: these are the components the
    # aircraft actually feels. Fitting ground speed against the scalar wind
    # speed explains R2=0.52 of it; against the tailwind component, R2=0.99.
    ("tailwind m/s", "_tailwind", 12),
    ("crosswind m/s", "_crosswind", 13),
    # --- measured response ---
    ("pitch held deg", "held_pitch_deg", 14),
    ("pitch err deg", "_pitch_err", 13),
    ("roll held deg", "held_roll_deg", 13),
    ("EAS m/s", "held_airspeed_mps", 9),
    ("TAS m/s", "_tas", 9),
    ("ground speed m/s", "held_ground_speed_mps", 16),
    ("course deg", "_course", 11),
    ("path angle deg", "held_path_angle_deg", 14),
    ("AoA deg", "_aoa", 9),
    # THE TRUST COLUMN. Pitch rate still present at the end of the averaging
    # window, in deg per aircraft second. Near zero means the row is a trim
    # point and the held values above are properties of the aircraft. Far from
    # zero means the row is a snapshot of an oscillation that had not finished,
    # and its held values are properties of WHEN THE AVERAGE WAS TAKEN. The 20 s
    # rows average 0.025 and the 90 s rows 0.0003, which is why hold length
    # decides whether two rows may be compared at all.
    ("pitch drift deg/s", "_drift", 17),
    # Drift's companion. Near-zero drift with a large spread is an oscillation
    # centred on the window, not a settled hold -- judge the two together.
    ("pitch sd deg", "_pitch_sd", 12),
    ("pitch range deg", "_pitch_range", 15),
    # THE OTHER TRUST COLUMN. Residual of the velocity triangle this row's own
    # numbers imply, in m/s. The population sits at -0.125 with RMS 0.161, the
    # same in calm, wind and bank. A row far from that is not a flight the
    # recorded values can describe, whatever the held columns look like.
    ("gs closure m/s", "_closure", 15),
    # --- context: what the flight actually did ---
    # Mean height over the averaged tail, rebuilt from the samples. This is the
    # altitude the TAS column was density-corrected at, and it is deliberately
    # NOT the midpoint of the two columns beside it.
    ("alt settled m", "_alt_mean", 13),
    ("alt start m", "max_rel_alt_m", 11), ("alt end m", "end_rel_alt_m", 11),
    ("hold end", "hold_end", 10),
    ("clock ratio", "clock_ratio", 11), ("att Hz", "attitude_hz", 8),
    ("sysid", "sysid", 7), ("run", "_run", 10), ("level", "_level", 12),
    ("crossed north", "_crossed_north", 13),
    # Provenance. Every row names the exact directory it came from, so any cell
    # can be traced to its result.json and samples.json and the sheet can be
    # rebuilt from the raw runs.
    ("source", "_source", 62),
]


def matrix(rows: list[dict[str, Any]], row_key: str, col_key: str, metric: str,
           col_label: Callable[[Any], str]) -> list[list[Any]]:
    """One sweep axis down the rows, the other across the columns."""
    cols = sorted({r.get(col_key) for r in rows if r.get(col_key) is not None})
    grid = [[""] + [col_label(c) for c in cols]]
    for key in sorted({r.get(row_key) for r in rows
                       if r.get(row_key) is not None}, reverse=True):
        line = [key]
        for col in cols:
            hit = [r.get(metric) for r in rows
                   if r.get(row_key) == key and r.get(col_key) == col]
            line.append(hit[0] if hit and hit[0] is not None else "")
        grid.append(line)
    return grid


def escape(text: object) -> str:
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def column_name(index: int) -> str:
    name = ""
    while index >= 0:
        name = chr(ord("A") + index % 26) + name
        index = index // 26 - 1
    return name


def sheet_xml(grid: list[list[Any]], widths: list[int] | None = None,
              bold_first_row: bool = True) -> str:
    body = []
    for r, line in enumerate(grid, start=1):
        cells = []
        for c, value in enumerate(line):
            ref = f"{column_name(c)}{r}"
            style = ' s="1"' if (bold_first_row and r == 1) else ""
            # `False` must land here too: an empty cell, not an inline string
            # holding nothing. bool is checked before the numeric branch below,
            # otherwise True/False would be written as 1/0.
            if value is None or value == "" or value is False:
                cells.append(f'<c r="{ref}"{style}/>')
            elif value is True:
                cells.append(f'<c r="{ref}"{style} t="inlineStr">'
                             '<is><t>yes</t></is></c>')
            elif isinstance(value, (int, float)):
                cells.append(f'<c r="{ref}"{style}><v>{value}</v></c>')
            else:
                cells.append(f'<c r="{ref}"{style} t="inlineStr"><is>'
                             f'<t xml:space="preserve">{escape(value)}</t></is></c>')
        body.append(f'<row r="{r}">{"".join(cells)}</row>')
    cols = ""
    if widths:
        cols = "<cols>" + "".join(
            f'<col min="{i}" max="{i}" width="{w}" customWidth="1"/>'
            for i, w in enumerate(widths, start=1)) + "</cols>"
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        '<sheetViews><sheetView workbookViewId="0">'
        '<pane ySplit="1" topLeftCell="A2" activePane="bottomLeft" state="frozen"/>'
        '</sheetView></sheetViews>'
        f'{cols}<sheetData>{"".join(body)}</sheetData></worksheet>'
    )


def write(path: str,
          sheets: list[tuple[str, list[list[Any]], list[int]]]) -> None:
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
        overrides = "".join(
            f'<Override PartName="/xl/worksheets/sheet{i}.xml" ContentType='
            '"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
            for i in range(1, len(sheets) + 1))
        zf.writestr("[Content_Types].xml",
                    '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                    '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                    '<Default Extension="xml" ContentType="application/xml"/>'
                    '<Override PartName="/xl/workbook.xml" ContentType='
                    '"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
                    '<Override PartName="/xl/styles.xml" ContentType='
                    '"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>'
                    f'{overrides}</Types>')
        zf.writestr("_rels/.rels",
                    '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                    '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/'
                    'officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
                    '</Relationships>')
        tabs = "".join(
            f'<sheet name="{escape(name)}" sheetId="{i}" r:id="rId{i}"/>'
            for i, (name, _, _) in enumerate(sheets, start=1))
        zf.writestr("xl/workbook.xml",
                    '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                    '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
                    'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
                    f'<sheets>{tabs}</sheets></workbook>')
        rels = "".join(
            f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/'
            f'officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{i}.xml"/>'
            for i in range(1, len(sheets) + 1))
        zf.writestr("xl/_rels/workbook.xml.rels",
                    '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                    f'{rels}<Relationship Id="rId{len(sheets)+1}" Type='
                    '"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" '
                    'Target="styles.xml"/></Relationships>')
        zf.writestr("xl/styles.xml",
                    '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                    '<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
                    '<fonts count="2"><font><sz val="11"/><name val="Calibri"/></font>'
                    '<font><b/><sz val="11"/><name val="Calibri"/></font></fonts>'
                    '<fills count="1"><fill><patternFill patternType="none"/></fill></fills>'
                    '<borders count="1"><border/></borders>'
                    '<cellStyleXfs count="1"><xf/></cellStyleXfs>'
                    '<cellXfs count="2"><xf xfId="0"/>'
                    '<xf xfId="0" fontId="1" applyFont="1"/></cellXfs></styleSheet>')
        for i, (_, grid, widths) in enumerate(sheets, start=1):
            zf.writestr(f"xl/worksheets/sheet{i}.xml", sheet_xml(grid, widths))


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "plant-baseline.xlsx"
    rows = collect()
    flown = [r for r in rows if r.get("held_pitch_deg") is not None]
    throttle = [r for r in flown if r["_run"] == "throttle"]
    wind = [r for r in flown if r["_run"] == "wind"]
    speed = [r for r in flown if r["_run"] == "speed"]

    readme = [
        ["NavPy SITL plant baseline"], [],
        ["Every row is ONE aircraft: a commanded (pitch, throttle, wind) in, a "
         "settled response out."],
        ["Held values are means over the settled tail (second half) of a 20 s "
         "hold in AIRCRAFT time."], [],
        ["run", "what it is", "verdict"],
    ]
    readme += [[tag, what, verdict] for _, tag, what, verdict, _h in RUNS]
    readme += [
        [],
        ["Columns that need explaining"],
        ["Data tab", "one row per flight condition. Columns left to right: the "
                     "five COMMANDED inputs, then the measured response, then "
                     "run context."],
        ["pitch/throttle/wind/sim speed",
         "the four sweep axes. sim speed is read back from the aircraft, not "
         "the value requested."],
        ["wind speed 0", "a run that did not pass --winds left SIM_WIND_SPD "
                         "alone. Recorded as 0: the ground-speed ratio in those "
                         "cells (1.0889) matches the explicitly-zero cells "
                         "(1.0891), so it is still air by measurement."],
        ["clock ratio", "sim seconds delivered per requested sim second. Below "
                        "1.0 means the host could not keep up in WALL time; "
                        "under lockstep the physics is unaffected, which the "
                        "Speed 1x vs 10x tab demonstrates."],
        ["EAS m/s", "airspeed as the autopilot reports it (equivalent airspeed)"],
        ["TAS m/s", f"true airspeed, ISA density at the hold altitude, home "
                    f"{HOME_MSL_M} m MSL. EAS understates TAS ~10% here."],
        ["course deg", "circular mean of the ground track. Re-derived from the "
                       "per-sample series for EVERY run, because a plain mean "
                       "reports 359 and 1 as 180."],
        ["AoA deg", "held pitch minus ground path angle"],
        ["path angle", "climb/dive angle over the GROUND, from GPS velocity. "
                       "This is what wind moves; pitch and airspeed do not."],
        [],
        ["Known limits"],
        ["blank ground-frame cells",
         "the 80 rows of run 'speed' have no ground speed / course / path angle "
         "/ AoA. That run flew before those outputs were recorded, so the "
         "cells are genuinely absent, not zero. Its pitch, roll and airspeed "
         "are complete, which is all the 1x-vs-10x comparison needs."],
        ["sample rate", "~4 Hz in aircraft time. Fine for steady state, coarse "
                        "for transients/settling."],
        ["airspeed scale", "ground speed exceeds TAS*cos(path angle) by a "
                           "constant 1.0889 +-0.0061 across 19-39 m/s. Constant "
                           "over a 2x speed range means a scale factor, not "
                           "wind. Cancels in every delta."],
        ["heading", "not recorded, so absolute wind cannot be reconstructed. "
                    "Only differences between wind cells are valid."],
        ["wind direction", "heading is uncommanded, so each cell met the wind at "
                           "its own relative bearing. Wind SPEED effects are "
                           "solid; wind DIRECTION comparisons are weaker."],
    ]

    # ONE sheet: every row a flight condition, inputs on the left, units in the
    # header. The matrix views and the notes page were separate tabs; they are
    # derivable from this table with a pivot, so the table is the deliverable.
    sheets = [("Data",
               [[label for label, _, _ in COLUMNS]]
               + [[r.get(key) for _, key, _ in COLUMNS] for r in rows],
               [w for _, _, w in COLUMNS])]
    if os.environ.get("PLANT_WORKBOOK_TABS"):
        sheets.append(("README", readme, [18, 58, 42]))

    for title, data, col_key, label in () if not os.environ.get(
        "PLANT_WORKBOOK_TABS"
    ) else (
        ("Throttle sweep", throttle, "cmd_throttle", lambda c: f"thr {c:g}"),
        ("Wind sweep", wind, "wind_speed_mps", None),
    ):
        grid = []
        if title == "Wind sweep":
            for r in data:
                r["_cell"] = (f"{r['wind_speed_mps']:g}@{r['wind_dir_deg']:g}"
                              if r.get("wind_speed_mps") else "no wind")
            col_key, label = "_cell", lambda c: c
        for metric, name in (("held_pitch_deg", "HELD PITCH deg"),
                             ("_pitch_err", "PITCH ERROR deg (held - commanded)"),
                             ("held_airspeed_mps", "EAS m/s"),
                             ("_tas", "TAS m/s"),
                             ("held_ground_speed_mps", "GROUND SPEED m/s"),
                             ("held_path_angle_deg", "PATH ANGLE deg"),
                             ("_course", "COURSE deg"),
                             ("_aoa", "ANGLE OF ATTACK deg")):
            grid.append([name])
            grid += matrix(data, "cmd_pitch_deg", col_key, metric, label)
            grid.append([])
        sheets.append((title, grid, [16] + [12] * 6))

    # 1x against 10x, same commanded cell, so the two columns are one flight
    # condition measured twice at different wall rates.
    grid = [["Same grid flown at 1x and 10x. If these columns agree, 10x is "
             "trustworthy and 6.7x cheaper."], []]
    levels = sorted({r["_level"] for r in speed})
    for metric, name in (("held_pitch_deg", "HELD PITCH deg"),
                         ("held_airspeed_mps", "EAS m/s")):
        grid.append([name])
        header = ["cmd pitch"] + levels + ["difference"]
        grid.append(header)
        for pitch in sorted({r["cmd_pitch_deg"] for r in speed}, reverse=True):
            line = [pitch]
            for level in levels:
                hit = [r.get(metric) for r in speed
                       if r["cmd_pitch_deg"] == pitch and r["_level"] == level
                       and not r.get("wind_speed_mps")]
                line.append(hit[0] if hit else "")
            pair = [v for v in line[1:] if isinstance(v, (int, float))]
            line.append(round(max(pair) - min(pair), 3) if len(pair) > 1 else "")
            grid.append(line)
        grid.append([])
    if os.environ.get("PLANT_WORKBOOK_TABS"):
        sheets.append(("Speed 1x vs 10x", grid, [14, 16, 16, 12]))

    write(out, sheets)
    print(f"wrote {out}")
    print(f"  aircraft: {len(rows)} ({len(flown)} with a held result)")
    for _, tag, _, _, _h in RUNS:
        print(f"    {tag}: {sum(1 for r in rows if r['_run'] == tag)}")
    print(f"  tabs: {', '.join(n for n, _, _ in sheets)}")


if __name__ == "__main__":
    main()
