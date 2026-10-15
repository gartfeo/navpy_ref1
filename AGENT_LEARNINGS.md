# Agent learnings

## Lockstep baton tests (step 10f-2, 2026-10-15)

- **Prove each test with a mutation.** Two first drafts passed with the rule
  broken: a stale-bid test (the peer never reported FREE), and an image test
  (distinct task ids hid a shared reassembler). Codex later found two more:
  an ACK test where the acceptance cancelled the timer, and a coalescing test
  without a repeated id. Next time, break the product rule, confirm the test
  fails, then restore the file.
- **Import-time locks stay on the wall runtime.** Class attributes made with
  rt.Lock() at import (MsgMetaProvider._lock, MsgRegistry._registry_lock)
  and module singletons (full_param_cache) are not rebound by a test's
  RuntimeBinding. Patch them with monkeypatch after binding; it also restores
  them.
- **Product code may swallow harness errors.** remove_vehicle ignores any
  exception from forget_vehicle. When a call passes through an except-pass,
  spy on it once to confirm it really ran on the baton.
- **Run codex exec with stdin closed.** In a background shell it waits on
  "Reading additional input from stdin..." until the timeout. Add < /dev/null.
- **Keep scratch scripts out of shared temp folders.** A stray dis.py in the
  temp folder shadowed the stdlib; use a task-specific directory.

## Sim A/B probes (20 ms spike residual, 2026-10-15)

- **Env vars do not reach the NavPy child of the demo eval.** An
  env-var-gated A/B arm silently ran the baseline code and gave a false
  negative. Hard-code the arm in a temp edit, and prove the arm is live (a
  dump file appears, or a metric moves) before reading its result.
- **Run-dir names cannot be reused.** A second eval with an old --run-dir
  fails with "WSL input step failed" (its WSL root exists). Use a new name.
- **Split the rate before guessing.** Logging ATTITUDE yaw, body rates and
  the dock NED vector per rendered frame splits the inertial LOS rate into
  azimuth, yaw-vs-gyro and pixel terms; that named the cause in one run.
- **Check the frame cadence before an A/B.** The sim poll loop waits with
  Event.wait, which on Windows follows the system timer tick (~15.6 ms)
  unless the process has 1 ms resolution; at 10x speedup that tripled the
  frame gaps between two sessions on the same code. Compare per-UAV poll
  counts (source-time detector_pose CSV) with the baseline run first.

## Gate split (step 10g-1, 2026-10-15)

- **Check a design's premises against later commits.** The fleet design
  asked for a strict xfail on todo 46. The EKF launch gate (61bdb67) had
  already made the slice dock, so the xfail was dropped. Before building a
  workaround the design names, grep the log for its todo.
- **Relaxing a check: do not drop evidence rows.** First fix for "a miss
  hides other gates" filtered the miss row out; Codex found it also hid a
  cadence gap before that row. Relax the refusal (a flag), keep the data.

## Polling-schedule replay (step 10g-2, 2026-10-15)

- **Grep every reader before widening a record.** Turning on operator
  evidence appends fields to `received`; the slice's confirmation timing
  unpacked it as a pair and would have crashed on the first evidence run.
- **A replay needs both ends anchored.** Rounds 1-2 of Codex found that
  dropping the first or the last passes still passed: anchor the start
  (a journaled loop start) and the end (the stream's unsubscribe), and
  account for every record in between.
- **Pick mutations that change behaviour on the tested inputs.** `<=` to
  `<` on "queued at start" was equivalent (the next branch gave the same
  answer) and survived; a stronger mutation showed the test was fine.
- **Wiring needs its own test.** Mutating the slice gate's call to the
  replay survived all replay tests; each integration point got a test.
- **Worktree files are CRLF here (core.autocrlf).** Byte-level mutation
  scripts must normalise line endings before matching.
- **Build gate fixtures from the real producer.** The replay tests and the
  slice fixtures run the real confirmation workflow on the baton and then
  change one record, instead of hand-writing a schedule the gate would only
  echo.

## Lockstep runs beside wall stacks (owner-plan scenario, 2026-10-15)

- **A "relay closed" with no firmware line can be another chat's kill.**
  The SITL test failed at frame 3722 with the journal byte-equal to a
  passing run up to the fault. Cause: a slot-0 wall eval stopped and
  relaunched, and gcs_stop's cleanup pkills `--sysid 1..3` across WSL,
  which matches lockstep SITLs on any slot (todo 51). SIGTERM ends the
  child silently; KILL, SEGV and HUP make the launcher print a line.
  Before a lockstep run, check `gcs_launch.py --list` and ask any slot-0..N
  wall user for a quiet window; after a fault, compare the WSL eval-root
  creation times with the run window.

## Fleet journal gates (step 10g-3, 2026-10-15)

- **Read these notes before the first edit.** The CRLF note above was
  already here and was missed. Check each file: tracked files are CRLF in
  the worktree, files new from the Write tool are LF.
- **Delete a runner's result file before each run and check its exit
  code.** A broken edit gave the mutation runner a syntax error, and the
  last run's "39/39 killed" file stayed in place and looked current.
- **Take fixture values from the producer, not from memory.** The fixture's
  ACK ref types were 6/7; NavPy writes `MsgType` values 130/140 and routes
  ACKs by them. The gate passed a chain NavPy would ignore. Import the
  producer's constants and match every field the consumer routes on.
- **Read the measurement the design names.** The fleet reused the slice's
  response-send reader (SITL admissions), but the design names the backend's
  emissions; under a backlog the two differ.

## Wall poll cadence and the Windows timer (2026-10-15)

- **A held timeBeginPeriod is not an honoured one.** Windows 11 dropped
  the 1 ms request of windowless eval processes once the screen locked;
  the API still returned 0. Measure the actual wait (perf_counter around
  Event.wait) instead of trusting the call; the fix is the
  ProcessPowerThrottling opt-out in windows_timer_resolution.py.
- **time.monotonic ticks at 15.6 ms on Windows.** Work times measured
  with it read 0 or 16 ms; use perf_counter for sub-tick diagnostics.
- **Run tests/guards before the full suite.** The SOLID 300-line module
  cap failed only after a 10-minute full run.
