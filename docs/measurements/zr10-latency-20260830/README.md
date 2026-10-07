# ZR10 optical-event latency — provenance

Measured 2026-08-30 by the owner on the real bird's companion (Jetson Orin,
JetPack 6.2 / L4T R36.5.0, kernel 5.15.185-tegra) against a SIYI ZR10 over
its RTSP endpoint. Copied into the repo unmodified from `~/zr10_latency/`.

`REPORT.md` is the owner's write-up. This file records where the data came
from, and — in "What it settles" — corrects two conclusions that overreach
what the data support.

## Files

| File | Bytes | What it is |
|---|---:|---|
| `REPORT.md` | 5,049 | Owner's report |
| `zr10_latency_recentered_20260830.jsonl` | 10,583,861 | Accepted raw log, 40,688 events |
| `zr10_latency_recentered_20260830_summary.json` | 3,624 | Machine-readable analysis |
| `measure_zr10_latency.py` | 15,705 | Acquisition |
| `analyze_zr10_latency.py` | 4,568 | Analysis |
| `recenter_siyi.py` | 1,385 | Gimbal recenter helper |

Raw log SHA-256 (stated by the owner before transfer, re-verified after):

```
03c8a1548df0e14dd8acf6a125d7b5633893029ae9b88ab6794ef6c0421c3317
```

Earlier calibration and failed runs stay on the Jetson as failure evidence
and are deliberately not copied here — they are not part of the accepted
dataset and must not be pooled with it.

## What it settles

Read `REPORT.md` first, then this section — **it corrects two claims that
`REPORT.md` and the first version of this file both made too strongly.**
The corrections came from a Codex review and were confirmed by independent
recomputation from the raw log; the two computations agree to the digit on
the fitted skew.

### Established

- Event-to-publication latency for this pipeline, 300/300 matched:
  min 24.886, median 99.841, p95 117.676, p99 121.540, max 154.339 ms.
  Median and p95 are robust to the detection threshold (raising it from 80
  to 110 moves the median to 100.469 ms); the 24.886 ms minimum is not.
- **PTS and `CLOCK_MONOTONIC` are strongly affine, not wandering.** A single
  least-squares fit over all 7,226 frames gives
  `mono ~= 1.2026908 * pts + offset` — PTS advances at 0.8315x wall, because
  the camera stamps a nominal 33.333 ms step while frames land at ~40 ms.
  Independent 500-frame block fits span 1.20260 to 1.20279, stable to
  ~160 ppm after startup. Residual against publication time, excluding the
  three startup frames and the outer 0.5%: -6.3 to +16.8 ms, sd ~3.8 ms.
- Per-event, not by differencing medians: **88.947 ms median, sd 14.541**
  elapses between the GPIO write and the first RTP packet observable at the
  `rtpjitterbuffer` sink. Everything downstream of that probe is ~9.5 ms
  median with small spread — first-to-last packet 0.000, last packet to
  access unit 0.333, access unit to decoder 4.826, decoder to publication
  4.335. That segment also carries essentially all the variance (sd 14.541
  against a total sd of 14.190, correlation 0.857).
- Transport is clean: zero missing RTP sequence numbers, zero duplicate RTP
  timestamps, zero duplicate PTS. The jitter is not packet loss.

### Retracted — layer (a) is NOT closed

`REPORT.md` line 22 states that no timestamp tracks exposure on a stable
`CLOCK_MONOTONIC` axis, citing a 48.682 s clock divergence and a 48.009 s
LED-event offset spread. **Both figures are the uncorrected rate ratio**,
not evidence of wander; the affine fit above removes them.

The deeper problem is that this experiment cannot answer the question at
all. Matching an LED edge to the first frame classified `led_on`
(`analyze_zr10_latency.py:27-36`) does not locate that frame's exposure: it
includes an unknown wait of up to one frame period. Under the affine map,
295 of 300 LED offsets fall inside a **single 40.09 ms mapped frame
interval**, and within it they are statistically indistinguishable from
uniform (KS D 0.0552 against a 5% critical value of 0.0791). Their sd is
13.23 ms where uniform-over-one-frame-period predicts 11.57 ms.

That is the signature of a stable exposure clock plus random stimulus phase
plus a fixed pipeline delay. The data are equally consistent with
"affine-mapped PTS *is* exposure, up to a fixed offset". Fitting PTS to
publication cannot separate the two hypotheses.

The defensible statement is therefore:

> The ZR10 exposes no *documented native* exposure timestamp on
> `CLOCK_MONOTONIC`. Its PTS/RTP clock is strongly affine to local time in
> this run and is a credible calibrated capture-time estimator, with
> uncertainty not yet bounded below frame-phase scale (~40 ms).

### Retracted — the ~89 ms is not "inside the camera"

The first version of this file attributed the 88.947 ms to sensor, ISP,
encoder and pre-network buffering. That is wrong at both ends. The segment
*starts* at an asynchronous LED edge, so it contains the wait for the next
qualifying exposure. It *ends* at a probe placed after kernel TCP receive
and RTSP interleaved demultiplexing (`REPORT.md` lines 70-71), so it also
contains camera network scheduling, Ethernet, TCP, host kernel receipt and
demux. Zero missing sequence numbers does not exclude TCP delay — TCP can
deliver every byte late.

`REPORT.md` lines 108-118 already say these contributions cannot be
determined. The earlier claim here contradicted its own source document.

The correct label: **~89 ms median from GPIO completion to the first RTP
packet observed after host TCP/RTSP processing**, un-decomposed.

## Consequence for the design

Capture time is carried as an **estimate with declared uncertainty**.
`scripts/los_replay/records.py` already encodes this — `CAPTURE_MEASURED`
versus `CAPTURE_ESTIMATED`/`CAPTURE_PUBLICATION` — and a real ZR10 session
may only ever be one of the latter two, because the camera publishes no
exposure stamp we can point at.

But the reason is weaker and the outlook better than first written: PTS is
not disproven as a capture-time source, it is *uncalibrated*. Before any
real-bird run, the open question is whether the affine map holds across
stream restarts, power cycles and temperature, and how large the residual
is once frame phase is removed.

Numbers a candidate must tolerate on this camera today:

- ~100 ms median glass-to-publication, ~±14 ms sd, with a tail to 154 ms
  driven by a handful of events rather than the body of the distribution
- publication cadence median 41.815 ms, p95 46.974 ms, max 87.795 ms,
  against a constant 33.333 ms RTP step — the camera's clock claims 30 fps
  while frames land slower and unevenly
- frame-phase uncertainty of ~40 ms on any exposure instant inferred from
  an asynchronous external event

Lag hurts a rate estimator through the LOS curvature it misses, which is
what the shootout scenarios measure. None of this decides the terminal law;
it bounds what a future SITL- or bird-fed harness run may assume.

## What would settle the open question

A phase-controlled short strobe — and, **correcting this file's first
version**, no photodiode is needed for the phase localization itself.
The acquisition already logs a continuous `roi_score` for every frame
(`measure_zr10_latency.py:288`), so a short pulse swept in phase locates
the integration handoff from the score data alone: the phase where two
adjacent frames respond equally is invariant under any single monotone
sensor response. The design, runbook and analysis live in
`../zr10-strobe-phase/`.

What the strobe bench still does NOT close: GPIO stamps the electrical
command, not the emitted light, so its result is an *effective
optical-response offset* under the stated assumption that a
direct-driven LED's electrical-to-optical delay is sub-microsecond. A
photodiode (or a scoped LED-current-to-light check) is the remaining
way to convert that into an unconditional exposure timestamp.
Separately, NIC or kernel packet timestamps alongside the existing
jitterbuffer probe would bound the non-camera share of the ~89 ms.
