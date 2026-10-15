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

## Gate split (step 10g-1, 2026-10-15)

- **Check a design's premises against later commits.** The fleet design
  asked for a strict xfail on todo 46. The EKF launch gate (61bdb67) had
  already made the slice dock, so the xfail was dropped. Before building a
  workaround the design names, grep the log for its todo.
- **Relaxing a check: do not drop evidence rows.** First fix for "a miss
  hides other gates" filtered the miss row out; Codex found it also hid a
  cadence gap before that row. Relax the refusal (a flag), keep the data.
