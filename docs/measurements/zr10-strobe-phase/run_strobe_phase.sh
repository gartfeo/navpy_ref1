#!/usr/bin/env bash
# Strobe phase bench - thin wrapper over the EXISTING acquisition.
# No new acquisition code: measure_zr10_latency.py already takes
# --on-time, --interval-min/max, --roi-* and logs per-frame roi_score.
#
# Usage:  ./run_strobe_phase.sh s2 <out.jsonl> <roi_x> <roi_y> <roi_w> <roi_h> [extra args...]
#         ./run_strobe_phase.sh s1 <out.jsonl> <roi...>   # per condition, rerun per condition
#         ./run_strobe_phase.sh s3 <out.jsonl> <roi...>   # per LED-position block
#
# Read PROTOCOL.md first. The ROI must be re-derived per aim; the old
# [800,290,140,130] is void. Keep ROI HEIGHT <= 40 px (rolling-shutter
# smear ~37 us/row). Escalate --on-time 0.003 -> 0.005 -> 0.010 only if
# the score modulation is too weak (see PROTOCOL.md check).
set -euo pipefail

MEASURE="$(dirname "$0")/../zr10-latency-20260830/measure_zr10_latency.py"
MODE="${1:?mode: s1 | s2 | s3}"
OUT="${2:?output jsonl path}"
ROI_X="${3:?roi x}"; ROI_Y="${4:?roi y}"; ROI_W="${5:?roi w}"; ROI_H="${6:?roi h}"
shift 6

if [ "$ROI_H" -gt 40 ]; then
  echo "ROI height $ROI_H > 40 px: rolling-shutter smear (~37 us/row)" >&2
  echo "voids the phase boundary. Re-derive a shorter ROI (PROTOCOL.md)." >&2
  exit 2
fi

ROI_ARGS=(--roi-x "$ROI_X" --roi-y "$ROI_Y" --roi-w "$ROI_W" --roi-h "$ROI_H")

case "$MODE" in
  s1)
    # Stability per condition (cold boot / stream restart / zoom 1x vs
    # 10x / two bitrates). SHORT pulses, same as S2: the estimator has
    # no long-pulse mode - a 180 ms pulse spans four frames and every
    # event classifies unexpected_lit (review finding). Fewer events
    # than S2, still above the declared estimator minimums.
    exec python3 "$MEASURE" --output "$OUT" "${ROI_ARGS[@]}" \
      --events 250 --on-time 0.003 --interval-min 0.31 --interval-max 0.73 \
      "$@"
    ;;
  s2)
    # Absolute offset: short strobe, uniform phase over the ~40.09 ms
    # period. Intervals are drawn uniformly over a span that is NOT a
    # multiple of the frame period, so phase coverage is uniform.
    exec python3 "$MEASURE" --output "$OUT" "${ROI_ARGS[@]}" \
      --events 400 --on-time 0.003 --interval-min 0.31 --interval-max 0.73 \
      "$@"
    ;;
  s3)
    # Rolling shutter: same as S2. Run once per LED-position BLOCK in
    # an alternating top-bottom-top design (PROTOCOL.md) - two single
    # sessions cannot attribute an offset delta to row position.
    exec python3 "$MEASURE" --output "$OUT" "${ROI_ARGS[@]}" \
      --events 400 --on-time 0.003 --interval-min 0.31 --interval-max 0.73 \
      "$@"
    ;;
  *)
    echo "unknown mode: $MODE (want s1 | s2 | s3)" >&2
    exit 2
    ;;
esac
