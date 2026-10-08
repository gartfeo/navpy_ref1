# ZR10 strobe phase bench — first execution, 2026-09-01/02

Executed remotely (Claude driving `ssh pf@orin1`), owner assisting with
scene lighting and LED aim. Analyzer: `analyze_strobe_phase.py` at commit
`8d0f08e0f`, prior `--prior-offset-ms 99.8` FIXED across every session.
Claims below were adversarially reviewed (Codex round 99); the review
demoted two miss-band sessions and several first-draft claims — the
retractions are kept visible in "What this bench did NOT establish".

## Trustworthy sessions (split-crossing, verdict green, internally consistent)

| Session | Regime | Pulse | ROI row (y) | Split offset M (ms) | CI95 |
|---|---|---|---|---|---|
| `s2_final` | dark | 3 ms | 386 | −75.886 | [−76.067, −75.520] |
| `s1_restart2` | dark | 3 ms | 383 | −75.993 | [−76.162, −75.916] |
| `s2_dim` | dim lamp | 3 ms | 372 | −75.840 | [−75.905, −75.473] |
| `s2_bright2` | bright lamp | 10 ms | 471 | −70.876 | [−71.336, −70.459] |

Two further sessions returned green verdicts from the MISS-BAND path and
are EXCLUDED as internally inconsistent: `s1_dim_restart` (305 of 317
misses fall outside the band they set) and `s1_zoom_bright` (493 of
618). The zoom condition therefore has NO trustworthy result. Those two
verdicts were green when this bench ran; the coherence gate added
afterwards now rejects both mechanically, and their stored reports carry
the failure.

## Findings

**1. The exposure-boundary phase M is measured absolutely.** Mid-frame
rows, dark regime: M ≈ −75.9 ms relative to the affine-mapped PTS plus
the fixed prior, CI ~0.5 ms, reproduced across three sessions and two
lighting regimes at nearby rows. M = c + (E+P)/2 — it becomes a
capture-time constant c only once the exposure width E is resolved,
which this bench did not do for the split-regime sessions (no misses →
E unobservable there). Layer (a) is PARTIALLY closed: the phase anchor
exists; the E term is the remaining unknown.

**2. Repeatability across stream restarts: ≤0.107 ms combined.** The
valid dark pair differs by 0.107 ms (`compare_restart.json`, branch 0 —
the integer-period branch restoration ran on real data). The two ROIs
sit 3 rows apart, and the provisional row slope alone would predict
0.13 ms, so this is a COMPOUND restart-plus-3-row repeatability bound,
not an isolated restart effect. A bright-regime repeat pair agreed to
0.08 ms (one member invalid at block divergence 1.100 ms — informative
only).

**3. Rolling shutter, provisional: ~43.1 µs/row.** Three dark-era row
points: y=386 → −75.886 (valid), y=128 → −87.019 (invalid: 3 saturated
frames), y=436 → −73.843 (invalid: 256 saturated frames). The slope from
the two extremes predicts the third to 0.11 ms; full-frame extrapolation
~31 ms. Two of the three points failed the saturation gate, and
saturation can distort the crossing, so this is a PROVISIONAL slope —
strong enough to demand a valid same-regime row ladder, not strong
enough to calibrate against. If it holds, capture-time correction is
row-dependent and the guidance-facing consequence is first-order.

**4. Cross-regime shifts are exposure-width confounded.** An AE state
flip between two dim-regime sessions moved the canonical offset by
−13.29 ms. That is consistent with a ~26.6 ms exposure-width reduction
IF c stayed fixed — but the bench did not independently measure the
width change (the flipped session's own width figure comes from the
defective miss-band path), so this is a consistency statement, not a
confirmation. Same caveat for dark-vs-dim: the 0.046 ms agreement at
rows 14 apart actually sits in TENSION with the provisional row slope
(which predicts 0.60 ms) unless E also shifted — another reason the
row ladder and an E measurement must be same-regime.

**5. Analyzer defect, found by this bench and now fixed.** The miss-band
path returned a VALID verdict on sessions whose own class counts
contradict their band. The band is fitted as the largest contiguous run
of miss phases, so one always comes back — when the misses are scattered
noise, the widest accidental cluster still looks like a confident band,
and the reported miss count has no relationship to it.

The gate is topological, not statistical. One dead-time interval per
period admits exactly ONE circular miss run, so a coherent band contains
every miss; `misses_outside_band != 0` fails the verdict, with no
tolerance (a fraction here would be a tuned number standing in for a
classification-error model nobody has measured). Measured separation:
clean synthetic 0 outside of 328, `s1_zoom_bright` 493 of 618,
`s1_dim_restart` 305 of 317. The uniform-phase figure
`(band_end − band_start) / P` is retained as a report diagnostic only,
since it assumes an even phase distribution the gate does not need.

The rule, stated exactly: an incoherent band cannot become the canonical
estimate, an admissible prior-scan point, the half-session drift, or the
cross-regime contrast, and no failed verdict enters a comparison. It is
NOT "a rejected session publishes no number" - a session rejected for
some other reason can still carry a valid split canonical. Here the two
rejected sessions have no split, so they publish no offset at all. Their
band and its center stay in the stored report as forensic evidence.

The circle is also no longer the population's to choose. Phases are
wrapped on the frame-grid clock (median mapped-PTS step over every
frame), which the analyzer passes in; deriving it from the classified
pulses let the population being judged move the clock that judges it.

Confirmed not to move any published number. Regenerating all nine stored
reports changed only the two miss-band sessions: both verdicts flipped
to invalid, both gained the coherence fields and lost their canonical
and prior scan, and `s1_zoom_bright`'s band geometry moved about a
nanosecond (its center by 0.61 ns) when phases went onto one circle.
The other seven reports are byte-identical, the four trusted split
measurements among them — they carry too few misses for a band at all.
The split-crossing path is unaffected. Raw fixture committed:
`sessions/s1_dim_restart.jsonl`.

## What this bench did NOT establish (first-draft claims retracted in review)

- "Layer (a) is no longer an estimate" — overstated; M is measured, c
  is not until E is resolved.
- "The offset is a camera constant" — the observable is M; constancy of
  c needs E constant or measured.
- "The zoom session is internally consistent" — false; it fails the same
  band-coherence check as the defect fixture (493 of its 618 misses lie
  outside the band they set).
- The dark-vs-dim 0.046 ms as "scene/AE invariance" — confounded by row
  and E.

## Operational mechanisms (why 14 pilots preceded the first valid run)

- **Recenter engages a servo that chases the drifting gyro yaw** (~2°/90 s
  readback drift): after any recenter/angle command the camera pans
  ~40 px/90 s and the LED walks out of a 60-px ROI in ~90 s. Fix:
  `--no-recenter` (also disables the periodic mid-run recenter).
- **The idle gimbal still wanders** (~7 px/min) and dark-scene AE state
  wanders on minutes scale, so a probe-then-measure gap of even minutes
  invalidates the derived ROI. Fix: `aim_and_run.py` locates the LED and
  execs the measurement in the same process (seconds apart), ROI 400x40
  centered on the live centroid; `wait_and_run.py` adds a
  centroid-stability gate before starting.
- **AE regime ladder is the SNR dial.** Dark room: gain maxed, 3 ms
  deposit ~167, LED core clips (saturation gate trips). Bright room:
  deposit ~8, sub-threshold. Working points found: dim lamp + 3 ms pulse,
  or bright lamp + 10 ms pulse (deposit ~30, no clipping).
- **LED beam-cone alignment dominates flux** (bare 5 mm LED, narrow cone,
  camera at close range): gimbal pan moves the lens out of the cone and
  deposits collapse 10x. The LED must be aimed at the lens; alignment is
  the first thing to check when deposits sag — and misalignment is
  exactly what manufactured the false-valid miss-band sessions.
- **Follow mode re-levels pitch**, so gimbal pitch offsets do not hold —
  row placement via pitch is unreliable at this close range (arm
  parallax). Zoom moves the LED row (magnification about frame center)
  but destroys the measurement itself (AE crush + clipping trade-off).
- A killed SSH mid-run left a partial JSONL; a retry APPENDED a second
  session into the same file (`run_start: 2`) and the analyzer choked.
  Delete partial output files before rerunning.

## Files

- `sessions/*.jsonl` — raw JSONL for the two anchor sessions
  (`s2_final`, `s2_bright2`) and the defect fixture (`s1_dim_restart`).
- `sessions/*_report.json` — analyzer reports for all nine sessions
  referenced above (including the invalid row-line sessions).
- `sessions/compare_restart.json` — the dark restart contrast.
- `aim_and_run.py`, `wait_and_run.py` — the Jetson-side wrappers that
  made sessions repeatable (copied from `~/bench/` on orin1).
- Raw JSONL not committed here remains on the Jetson in `~/bench/`;
  SHA-256 of every session named above:

```
008c144339848d2a6822e90874c759373fcc20b489b1b81792c0c44699a59d01  s2_final.jsonl
2a1c6e89ee2804a39f17d7eb53762417b28949857ffa102b4fb9e07b6130e1d7  s1_restart2.jsonl
ab2827721fe643ed8ecc50a3ebca070f60daaaddc21f788de9c1ec0fa39a4621  s2_dim.jsonl
f3005c4dfad4f5ad6322b4afd111ae7ee8ae78b845e5d61078e77aa6ffbcf455  s2_bright2.jsonl
424001699c579b71e3627838aec0713f8d77e251f6b4ed76a9d42226721b7eaa  s1_zoom_bright.jsonl
ec59a245bb5d8b3e837452e577b49b0ea3321b0180966f25e2009f2921f28b57  s1_dim_restart.jsonl
c9d6b531f2a1bf1904e98bed844904ef20724886a2011593643b495f73a4bf1a  s3_rowB.jsonl
5482b09a6d854d1e3bbef5de085633b288be31bc0b21b0ef92b83123586cc050  s3_topA.jsonl
5d84d7ab13f56c38bb43b16df7f1cca945fecf31d236cecfeb0ad520ed3d8f7c  s3_topA2b.jsonl
```

## Open items

- ~~Analyzer miss-band consistency gate~~ — DONE (ledger 67); both
  miss-band sessions are now regression fixtures. The same change put
  every phase on one caller-supplied circle, the frame-grid period
  (previously each sample wrapped by its own `local_period_ns` while the
  band was fitted on a pooled median of those).
- Valid same-regime row ladder (rolling-shutter slope with green
  verdicts end to end) and an E (exposure width) measurement in the same
  regime — together these convert M into c.
- Cold-boot condition — needs a hands-on power cycle.
- Zoom condition — needs an attenuation approach that survives zoom AE.
- The 08-30 `CAPTURE_ESTIMATED` label stands until c (not M) is
  resolved and the owner decides how row and E enter the production
  capture-time model.
