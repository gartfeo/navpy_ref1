# Todo 290: recovery from a failed evidence write (design draft)

Status: draft, owner rulings 2026-11-06 recorded below. No runtime code changed. Scope: todo 290 only;
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

## Why a write fails

No run has shown one: no navigation log under the main checkout's `.logs` carries
`NAVIGATION_EVIDENCE_FAILURE`. The fail-stop is a guard, not a fix for a seen fault. By the
code, the possible causes are:

- the handler raises: an OS error on the CSV (disk full, a file held by another process such
  as an antivirus scan or an open editor on Windows) or a bug in a row formatter;
- the backlog passes 2000 jobs (`NAVIGATION_STREAM_MAX_PENDING`, 2 s of rows at the 50 Hz x
  10 lockstep worst case): a slow disk or CPU load;
- the writer thread cannot start;
- a write blocks (a stuck file system).

Every failure must therefore keep its exception, traceback, job kind and file, so the cause
can be found later.

## Owner rulings (2026-11-06)

1. **Option C.** The approach flies on; the evidence is marked incomplete; NavPy keeps
   running. A failure must still be recorded and investigated: ideally it never happens.
2. **Retry**, with the numbers below.
3. **A hung write is skipped**; writing goes on with the rest.
4. **Reporting goes to the logs, not the GCS.** In dev mode the warning is a log warning; no
   STATUSTEXT and no GCS reason.
5. **The app never fails on it**: log and continue.

## Proposed numbers

All of this runs on the writer thread; the command path never waits.

| Case | Behaviour |
|---|---|
| A write raises | Retry the same job in place 3 times, after 50, 100 and 200 ms (0.35 s in all, under the 2 s RESET wait). Then skip that job, count it, and go on with the next. FIFO order holds because the retry happens before the next job. |
| Backlog full (2000 jobs) | Do not fail the writer: drop the new row, count it, keep writing the queue. |
| A write hangs | A job running longer than 2 s (the RESET wait) marks the writer stuck. A new writer starts on new files (same name, suffix `_part2`, `_part3`); the stuck thread is left behind, since Python cannot kill it. At most 3 replacements per NavPy run; after that evidence stops for the run, and NavPy flies on. |
| Logging | The first failure of each approach is logged in full (exception, traceback, job kind, file). Later ones are counted only. At the approach end one line gives the counts: retried, skipped, dropped, writer replaced. |

## Remaining questions

1. **Evals.** NavPy logs and continues, but a run with missing rows cannot be scored
   honestly. My proposal: the eval reports such a run as "evidence incomplete, not scored",
   never as a pass, on every gate. Is that right?
2. **Outside dev mode.** NavPy has no dev or field mode setting today. Should field use also
   stay logs-only, or show something in the GCS later?
3. **Bench children** (`direct_pixel_pn_child.py`, `siyi_pixel_pn_child.py`,
   `siyi_geo_track_child.py`, `scratch_direct_geo_child.py`). These are not the app: each is a
   test script that flies one scored approach on the bench or in SITL, and its only output
   is that evidence. They stop at the first failed write today. My proposal: keep that, since
   a bench run with missing rows has nothing to score. Agree?
