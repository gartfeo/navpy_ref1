# SIYI ZR10 optical-event latency measurement

Date: 2026-08-30 (Asia/Yerevan)

## Result

The accepted run contains 300 pseudorandom LED-on events and 300 independently
matched first-visible frames.

| Metric | min | median | p95 | p99 | max | spread |
|---|---:|---:|---:|---:|---:|---:|
| LED GPIO post-write to first published frame showing LED | 24.886 ms | 99.841 ms | 117.676 ms | 121.540 ms | 154.339 ms | 129.453 ms |
| Last RTP packet to decoder output | 4.187 ms | 5.164 ms | 6.022 ms | 12.754 ms | 14.335 ms | 10.148 ms |
| Decoder output to publication | 3.803 ms | 4.335 ms | 4.491 ms | 4.558 ms | 4.650 ms | 0.847 ms |

The latency tail is therefore p95 117.676 ms, p99 121.540 ms, and maximum
154.339 ms. These are not exposure timestamps; they are measured event-to-first-
visible-publication delays.

## Exposure timestamp conclusion

**No available timestamp tracks exposure on a stable CLOCK_MONOTONIC time axis.**

- PTS and RTP timestamp are the same sender clock up to scale/offset. Across the
  run, CLOCK_MONOTONIC publication time advanced 289.513 s while PTS advanced
  only 240.831 s. The clocks diverged by 48.682 s. Their LED-event offset spread
  was 48.009 s, so neither can be an exposure timestamp without an unavailable
  and time-varying clock mapping.
- First RTP packet arrival offset from LED-on spread over 96.913 ms.
- Last RTP packet arrival offset spread over 130.428 ms.
- Decoder-output offset spread over 129.626 ms.
- Publication latency spread over 129.453 ms.

Receipt, decoder, and publication times are useful local pipeline timestamps,
but their offset to the optical event wanders. A mean delay must not be treated
as exposure time.

## Cadence and continuity

- 7,226 decoded/published frames and 7,241 completed RTP frames were logged.
- Publication interval: median 41.815 ms, p95 46.974 ms, p99 53.724 ms,
  minimum 5.293 ms, maximum 87.795 ms.
- 23 publication gaps exceeded 62.5 ms; 32 burst intervals were below 20 ms.
- RTP timestamp step was normally 3,000 ticks = 33.333 ms, despite the slower
  wall-time cadence. There was one step above 4,500 ticks (maximum 5,103 ticks,
  56.700 ms).
- Missing RTP sequence numbers: 0.
- Consecutive duplicate RTP timestamps: 0. Consecutive duplicate PTS: 0.

The difference between RTP-frame and published-frame counts includes pipeline
startup/shutdown and cannot by itself identify decoder drops. Exact camera-side
dropped exposures cannot be determined.

## Acquisition settings

The RTSP endpoint advertised H.265 even though its path ends in `.264`.

```text
rtspsrc location=rtsp://192.168.144.25:8554/main.264
  protocols=tcp latency=0 buffer-mode=none drop-on-latency=true
  do-retransmission=false
! rtph265depay
! nvv4l2decoder disable-dpb=true enable-max-performance=true
  num-extra-surfaces=0
! nvvidconv
! video/x-raw,format=BGRx
! appsink emit-signals=true sync=false max-buffers=1 drop=true
```

RTP arrival was stamped at the `rtpjitterbuffer` sink, after kernel TCP receive
and RTSP interleaved demultiplexing but before GStreamer jitter buffering. JSONL
writing ran on a separate queue-backed thread to avoid blocking packet/decode
callbacks. GPIO used Linux character-device ABI ioctls directly on
`/dev/gpiochip0`, line 144; pre-write and post-write CLOCK_MONOTONIC stamps are
preserved for every change.

The calibrated gimbal still drifted slowly in LOCK mode, so it was recentered
while the LED was off at startup and before events 50, 100, 150, 200, and 250.
Every recenter reported positive feedback and was allowed four seconds to
settle before the next pseudorandom delay. No gimbal command occurred during an
LED pulse.

## Software

- Linux kernel 5.15.185-tegra
- Python 3.10.12
- GStreamer 1.20.3
- NVIDIA `nvv4l2decoder` plugin 1.14.0
- PyGObject/python3-gi 3.42.1
- GLib 2.71.3
- NumPy 1.21.5
- libgpiod 1.6.3

## Integrity and files

- Raw accepted log: `zr10_latency_recentered_20260830.jsonl`
- Raw events: 40,688; file size: 10,583,861 bytes
- SHA-256: `03c8a1548df0e14dd8acf6a125d7b5633893029ae9b88ab6794ef6c0421c3317`
- Machine-readable analysis: `zr10_latency_recentered_20260830_summary.json`
- Acquisition script: `measure_zr10_latency.py`
- Analysis script: `analyze_zr10_latency.py`
- Recenter helper: `recenter_siyi.py`

Earlier logs are retained as calibration/failure evidence but are not combined
with the accepted dataset.

## Cannot determine

- The actual moment of exposure for any frame: no camera-side exposure stamp or
  valid sender-to-CLOCK_MONOTONIC clock mapping is exposed.
- Camera sensor readout, ISP, encoder, and pre-network buffering contributions:
  the earliest available receive observation is already after those stages.
- Kernel socket receipt time separately from RTSP/TCP demultiplexing time.
- Exact camera-side dropped exposures or whether every encoded timestamp denotes
  a genuine sensor exposure.
- Whether the residual 24.886–154.339 ms variation arises in sensor scheduling,
  ISP, encoding, TCP delivery, or a combination before the first observable RTP
  packet.
