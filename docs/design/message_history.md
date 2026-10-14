# MAVLink message history (design v1, todo 31)

## Problem
In 10x SITL every NavPy process freezes all threads for 90-220 ms at
~26 s, ~66 s, ~148 s, ~233 s of process age. A freeze inside a final
approach ages the last receipt past the 100 ms wall limit (1.0 s aircraft),
so the approach fails with `NAV_FAIL: source_command_stale` (todo 31).

## Root cause (measured, `.logs/gc-probe-1`)
Each freeze is a CPython full (gen2) garbage collection, matched 1:1 in time
and length. Its cost is ~0.6 us per tracked object. The heap grows because
`MessageStore` (`plugins/vehicle/ardupilot/message_store.py:14,43-45,66-69`)
keeps a 1024-sample deque for every MAVLink type it receives, about 4.4
tracked objects per sample. 40 Hz types fill at ~27 s, 20 Hz at ~51 s, 10 Hz
at ~100 s: the growing pause pattern. Gen2 frees almost nothing: it is live
history.

Only six types are ever read from history:
- `wait_after`: PARAM_VALUE, MISSION_COUNT, MISSION_ITEM(_INT), HOME_POSITION,
  TERRAIN_REPORT (`parameter_reader.py:66`, `parameter_writer.py:72`,
  `mission_download.py:55,90`, `position_service.py:76,126`).
- `recent`: ATTITUDE, at most `ATTITUDE_HISTORY_COUNT=64`
  (`pose_telemetry.py:170`, `real_frame_association.py:45`).
Everything else needs only the latest sample.

## Fix
`MessageStore` takes an explicit history policy: message type -> depth.
- The vehicle composition declares it next to its consumers: the response
  types keep 1024 (today's capacity and overrun detection; TERRAIN_REPORT is
  ~2 Hz in the demo), ATTITUDE keeps `ATTITUDE_HISTORY_COUNT` (64), the
  consumer's own constant, moved to `core/ports/pose_stream_rate.py` so
  the plugin can import it; the two cannot drift.
- Validation is against the policy, for every requested type; unknown
  incoming types still publish (latest only). The policy is copied at
  construction; eviction compares against each deque's own depth; all reads
  and writes stay under the one `rt.Condition`.
- Other types keep only the latest sample (`latest`, `cursor` unchanged).
- `wait_after` or `recent` on a type with no declared history raises
  `ValueError` at the call, so a new consumer must declare its depth instead
  of silently missing responses.
- Expected (estimate, not measured): the heap stays near its startup size
  (~85k objects) and full collections become rare; one at that size measured
  36-53 ms at startup.

`gc.freeze()` is deferred: it would also pin later-unreachable cycles. Assess
any remaining pauses after this fix.

## Tests
- History kept only for declared types, at the declared depth; an undeclared
  high-rate type keeps no history; `wait_after`/`recent` on an undeclared type
  raises.
- Existing `wait_after` overrun and response tests unchanged.
- Guard: every `wait_after`/`recent` type used in `plugins/vehicle/ardupilot`
  is in the composition's policy.
- SITL: 3-UAV runs at 10x lasting past 300 s of process age (the last
  measured pause was at 233 s), with a duration-only GC log (temporary, not
  committed, no heap census). Pass: no full collection after startup except
  ones under the 100 ms wall limit, the attitude-CSV gaps match them, and no
  `source_command_stale`.

## Not in scope
- ATTITUDE at 64 samples spans 1.6 s at 40 Hz but only ~0.2 s at a 320 Hz
  stream (`pose_streams.py:63`, capture ceiling 0.5 s in
  `real_frame_association_gate.py:78`). Pre-existing; logged as a todo.
A smaller simultaneous pause in all processes (50-76 ms, host-wide) seen once
per run; log it if it recurs.

## Result (2026-10-13)
SITL `.logs/msghist-1` (34 s, ended by a pre-existing approach failure): heap flat at ~90k objects, one full collection per UAV at ~20 s of 40-48 ms (was 92-106 ms at ~26 s, 171k objects), no `source_command_stale`. The 300 s check was not reached.
