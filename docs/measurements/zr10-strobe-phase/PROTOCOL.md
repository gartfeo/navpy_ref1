# ZR10 strobe phase bench — runbook

Closes layer (a): where, relative to affine-mapped PTS, does the camera
respond to light — and is that offset stable? Uses the EXISTING rig and
acquisition (`../zr10-latency-20260830/measure_zr10_latency.py`); this
directory adds only argument wrappers and analysis.

**What the result is.** The effective optical-response offset relative
to the session's fitted PTS→publication map. Not an intrinsic camera
constant: GPIO stamps the electrical command, and the map's intercept is
publication-anchored. Assumption stated up front: a direct-driven LED's
electrical-to-optical delay is sub-microsecond, negligible against the
millisecond scale measured here. If the driver is not a plain
transistor/LED chain, bound its delay with a scope before trusting the
absolute number (offset DELTAS between conditions do not need this).

## Physical setup — do all of this before any run

1. **Re-aim.** The camera currently points at the air unit. Point it so
   the LED sits in frame. Recenter helper: `recenter_siyi.py` in the
   08-30 directory.
2. **Re-derive the ROI.** The old `[800,290,140,130]` is void. Grab a
   frame, find the LED, box it with margin. **ROI height ≤ 40 px**: the
   sensor is likely rolling-shutter (~37 µs/row), so a 130-row ROI
   would smear the phase boundary by ~5 ms.
3. **Scene brightness: moderate-to-dim, NOT bright.** No SIYI exposure
   command exists, so shutter is whatever AE picks. A bright scene
   shortens the integration window E; the informative straddle pulses
   exist only while E + pulse ≳ period. Dimmer scene → longer E → more
   splits. (This reverses earlier advice — deliberate.)
4. **No saturation.** With a dim scene the LED is relatively bright.
   Check a lit frame: the ROI red channel must stay below 250. If
   clipped, reduce LED current or add a diffuser — a clipped ROI
   destroys the score split the estimator reads.
5. **LED spot small relative to the frame**, so it does not drive AE
   itself, but large enough inside the ROI that the p98 score responds
   (aim for the spot covering ≳ 10% of ROI pixels).

## Modulation check (before committing to a 400-event run)

Run ~30 events at `--on-time 0.003`, then run the ANALYZER on the
pilot and read the `classes` histogram — modulation alone is not
enough: near E + w ≈ P a bright LED can still produce too few splits
AND too few misses for either estimator. Project the counts to the
full run (splits_pilot × events_full / events_pilot must clear the
declared minimums for at least one estimator); if neither projects
clear, adjust scene brightness (dimmer → longer E → splits) or pulse
width. Lit frames must score ≳ 12 above the neighboring baseline
(`roi_score`); too weak → escalate on-time to 0.005, then 0.010, and
stop at the first width that clears.

## Runs

| run | command | purpose |
|---|---|---|
| S1 | `./run_strobe_phase.sh s1 out_<cond>.jsonl <roi>` per condition | Is the offset stable across: cold boot, stream restart, zoom 1x vs 10x, two bitrates? One file per condition, conditions in recorded order. Short pulses like S2 — the estimator has no long-pulse mode. |
| S2 | `./run_strobe_phase.sh s2 out_s2.jsonl <roi>` | Absolute offset + integration width. ~400 short strobes, uniform phase. |
| S3 | `./run_strobe_phase.sh s3 out_s3_<block>.jsonl <roi>` | Row dependence. **Alternating blocks: top, bottom, top** (move the LED or re-aim between blocks, re-derive ROI each time). Two single sessions cannot separate row effect from session drift — the A-B-A design can. |

Record per session, in a note next to the files: firmware version, zoom
level, resolution, bitrate, ROI, LED position (top/bottom), condition
order, and the SHA-256 of each raw JSONL before transfer.

## Analysis

```bash
python3 analyze_strobe_phase.py out_s2.jsonl --report out_s2_summary.json
python3 analyze_strobe_phase.py boot=a.json restart=b.json --compare --report s1_compare.json
```

The analyzer FAILS CLOSED: exit code 1 unless a valid primary estimate
exists. The verdict block lists why — `unexpected_lit` (retune
`--prior-offset-ms` and record the value used), unstable blockwise
alpha, disagreeing split estimators, split-vs-band mismatch, an
incoherent miss band (below), or an unstable prior-sensitivity scan.
The scan (±20 ms, predeclared steps) never picks the best-looking
prior; it demands the crossing hold still across every admissible one.

The analyzer refuses (`InsufficientData`) rather than reporting a
crossing from too few pulses: ≥ 25 splits for the crossing, or ≥ 15
misses + 30 lit for the band. If S2 lands in the miss-band regime
(shutter too short for splits), the band CENTER is the
threshold-invariant deliverable and the run is still decisive — do not
re-run chasing splits without recording why.

**`miss band incoherent` is a session-validity failure, not an
analysis knob.**
The model admits one dead-time interval per frame period, so every
miss must fall inside the single band those misses define. The band is
fitted as the widest contiguous miss run, so one always comes back:
when misses are scattered noise the widest accidental cluster still
looks like a confident band. The verdict therefore requires zero
misses outside it and zero lit samples inside — no tolerance, because
a fraction here would stand in for a classification-error model nobody
has measured. When this fires, the miss population cannot be explained
as one stationary dead-time band — some of those misses may still be
genuine dead time, but the model behind the estimate does not hold, and
the gate alone does not name the cause. Check LED beam-cone alignment
first (a panned camera leaves the cone and deposits collapse ~10x,
which is what produced the two rejected sessions of 2026-09-01), then a
response that changed mid-session (`half_session_drift_ms`, AE state).
Do not raise `--lit-threshold` to make it pass. An incoherent band still appears in the summary with its
edges, center and counts — that is forensic evidence for the re-run,
never a number to report.

`linear_split_diagnostic_ms` in the summary is NOT a result. On clean
data it must AGREE with `split_crossing.offset_ms` (that holds under
any shared monotone response — verified in the unit tests); a gap
flags EITHER a response differing between adjacent frames (AGC step,
clipping, asymmetric pulse) OR asymmetric phase sampling / prior
truncation, and disqualifies the session either way — it never offers
a second number.

The canonical offset is reported WRAPPED modulo the session's frame
period (`canonical.offset_wrapped_ms` + `frame_period_ms`). Wrapping
alone would alias a between-condition shift of exactly one period
(~40 ms) to zero, so `--compare` restores the integer-period branch
from each session's `coarse_latency_ms` (median event-to-publication
latency of the lit frame): `delta_ms` is absolute, with
`circular_delta_ms` and `branch_periods` shown separately. A nonzero
branch is a finding, not an error. Keep `--prior-offset-ms` FIXED
across every condition of a comparison; retuning it between sessions
is allowed only when a condition genuinely moves the pipeline delay,
and the branch restoration is what keeps that honest.

The prior-sensitivity scan probes PRIOR-ASSIGNMENT sensitivity only.
`affine.block_map_divergence_ms` (verdict-bounded at 1 ms) constrains
ONLY the PTS-to-publication map — exposure timing can drift against
PTS while that map stays perfect, so a valid verdict does NOT exclude
within-session optical-offset drift. `half_session_drift_ms` (circular
gap between first-half and second-half estimates, null when either
half is too thin) is the diagnostic: treat a value beyond ~1.5 ms as a
suspect session and re-run.

## What decides what

- S1 offsets agree within a few ms → affine-mapped PTS is a calibrated
  capture-time estimator; the offset calibrates out once.
- S1 offsets jump between conditions → PTS is not usable as a capture
  estimator without per-session calibration, whatever its precision.
- S3 top-vs-bottom delta at millisecond scale → row-dependent capture
  time is a real term in the guidance error chain (the target does not
  sit at a fixed row in flight); goes in the ledger with the number.
