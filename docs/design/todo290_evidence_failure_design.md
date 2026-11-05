# Todo 290: recovery from a failed evidence write (design draft)

Status: draft for the owner, 2026-11-05. No runtime code changed. Scope: todo 290 only;
the overall recovery design (todo 292, component failures, operator reset) is drafted
separately.

## Today

- Final-approach evidence (compact/debug rows, DOCKED rows) is written off the command
  thread by one FIFO worker (`core/approach/navigation_postprocess_dispatcher.py`,
  `logs/navigation_stream_worker.py`). The command path never waits on it.
- The first failure stops the worker for good: it refuses every later job. The failure is
  logged with `NAVIGATION_EVIDENCE_FAILURE`, latched, and raised by the runtime health check
  (`app/runtime_composition.py` `check_runtime_health` -> `navigation.raise_if_failed`).
  That ends NavPy, mid-flight if need be.
- RESET waits up to 2 s for evidence already owned by the worker
  (`vision_nav/command_postprocess.py` `drain`, called from `command_reset.py`).

## Failure kinds

1. **A write raises** (disk full, a permissions error, a bug in a row formatter). It is fast
   and may happen again on every write.
2. **Backlog overflow**: writes are slower than commands (slow disk, CPU load); the queue
   passes 2000 jobs and the worker fails itself.
3. **A write hangs** (a stuck file system). Python cannot kill the thread. The RESET drain
   then times out every cycle, so the next approach cannot start cleanly.

## Options

| | What happens | For | Against |
|---|---|---|---|
| A. Fail the approach, run on | NAV_FAIL with a new reason `evidence_write_failed` -> normal RESET; a fresh writer for the next approach | Closed and clear; the eval and GCS see why | Loses a dock that would have worked; the command path itself was fine |
| B. Retry the write | Retry the failed job a bounded number of times, then A or C | Rides through a short hiccup | FIFO order breaks; a bug or a full disk never succeeds; the hang case is not covered |
| C. Continue, evidence marked incomplete | The approach flies on; the writer restarts; the run carries an "evidence incomplete" mark | No dock lost, because evidence is not on the command path | A docked approach without full evidence; the eval must fail it |
| D. Keep the fail-stop (today) | NavPy exits | Honest evals | Against the 2026-11-05 ruling: it strands the UAV |

For every option: the operator gets one STATUSTEXT per failure, the marker stays in the
navigation log, and every eval gate fails a run that logged it. Today only the ideal
three-UAV gate reads it. The single-vehicle, delivery and certified-lockstep gates would
need it too. Bench children (`direct_pixel_pn_child.py`, `siyi_pixel_pn_child.py`) can keep
failing fast; they fly one scored approach and have no NAV loop.

The hang case needs its own answer under A and C: stop waiting for the stuck writer (mark its
evidence lost) and start a new writer on new files, or keep refusing evidence until it comes
back while the flight goes on.

## Prototype (option A, not merged)

The patch is `todo290_prototype.diff` in the side-session notes folder. Its parts:

- `take_failure` hands the failure over once and retires the failed writer.
- The NAV cycle polls `take_failure` first, then fails NAV with `evidence_write_failed`
  (index 11, with a GCS label).
- The health check no longer raises.
- The eval gates fail on the marker.

Its unit tests and the frontend tests and build passed. In the wider run (guards, nav,
navigation, scripts, lockstep) 5784 tests passed and 1 failed: the SOLID guard, because
`compose_nav_application` grew to 155 lines (limit 150). Codex review round 1 found two
material problems, both open:

- **Hang or overflow blocks recovery.** Retiring the writer joins it on the NAV thread for up
  to 2 s before the approach is failed. A stuck writer repeats that every cycle, and the RESET
  drain cannot finish. Fix: fail the approach first, then clean up the writer off the NAV
  thread.
- **A raising logger loses the operator warning.** The failure is consumed before reporting,
  so the warning is never retried. Fix: report each part on its own, and keep a warning that
  is still pending.

## Questions for the owner

1. Should a failed evidence write end the approach (A), or only mark the evidence incomplete
   while the approach flies on (C)? My pick is C: evidence is not on the command path.
2. Retry at all (B)? If yes, how many times, or for how long?
3. If a write hangs, may NavPy drop that writer and lose the rest of that approach's
   evidence, so the next approach starts clean?
4. Operator: is one STATUSTEXT per failure plus a GCS reason enough, or should the GCS show a
   standing "evidence incomplete" mark for the flight?
5. Evals: may any logged evidence failure fail the run on every gate?
6. Bench children: keep the fail-stop there?
