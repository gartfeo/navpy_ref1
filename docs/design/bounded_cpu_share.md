# Bounded CPU share for wall runs

Status: approved by the owner as written (2026-10-28), including
best-effort background affinity; target `main`. Replaces the quiet-PC rule
withdrawn in todo 65 (`8a0b880e`).

## Problem

A wall (real-time, 10x) 3-UAV run fails when other work loads the PC
(todo 65). The quiet-PC rule hid this by making every chat wait. The owner
asked instead for a bounded share: a demo or wall run reserves a measured
slice of cores for a limited time, and everything else keeps running on the
other cores at lower priority.

Measured 2026-10-27 (`.logs/cpu-share/m1`, probe in `m1-probe`): one
certified wall eval ran beside two other chats' 3-UAV stacks (a wall stack
and a lockstep matrix), so 9 NavPys and 9 SITLs ran at once. Every vehicle
failed (observation speedup 5.5-6.3x outside 8-12x, no `DOCKED`).

**The SITLs kept 10x; NavPy fell behind.** Over each vehicle's final
approach (`report.json` cadence episodes) the GCS receiver saw the SITL's
ATTITUDE clock advance at 10.1-10.3x, but NavPy saw 6.5-7.1x. NavPy
accepted the same ATTITUDE samples a median 0.67-0.87 s and at most 1.6 s
later than the GCS (`m1/source-time/pose_cadence_attitude_arrival_*`). Over
the whole flight both saw 10.2x. So the delay lies between NavPy's
companion link and its pose cache, at NavPy's busiest phase. The marker
records accepted pose-cache updates, not socket ingress, so reader backlog,
scheduling and link buffering are not yet told apart (todo 66 measures
ingress).

Likely cause, to be confirmed by the acceptance runs: each NavPy needs about
one core (GIL-bound) and receives about 2000 packets/s (todo 66). All 9
NavPys raise themselves to ABOVE_NORMAL (`windows_process_qos.py`), so the
eval's NavPys compete as equals with six other NavPys, on any core,
including E-cores that take about 1.8x as long per frame (todo 65).

## What one run needs (m1)

While the eval's NavPys ran (40 s, 21 samples of about 1.9 s):

| Process (one 3-UAV wall run) | avg | p95 |
|---|---|---|
| NavPy, each of 3 (Windows) | 0.97 core | 1.27 core |
| GCS backend (Windows) | 0.38 | 0.52 |
| Run total, Windows | 3.3 | 4.2 (max 4.5) |
| Whole host | 10.7 of 20 | 11.8 |
| SITL, each of 3 (WSL) | 0.5 vCPU | 0.75 vCPU |

The CPU is a Core Ultra 7 255HX: 8 P-cores (logical CPUs 0, 1, 6-9, 18,
19) and 12 E-cores, without SMT. One short run is thin evidence: the share
size below is a candidate that the acceptance runs confirm or change.

## Design

**Share record.** `~/.gcs/cpu_shares.json`, next to the instance registry,
guarded by a named mutex (`cpu_share.py`; a holder that dies hands it over
at once, so no stale lock is left). Every transaction, reads included, takes
it: on Windows an open reader makes the atomic replace fail. A transaction
holds it for milliseconds, so each waits for it at most the registry's lock
timeout (10 s). A claim that cannot get it, or finds the record unreadable
or unwritable, is refused; a release that cannot leaves the CPUs held until
the owner exits (fail closed); a background read that cannot avoids
nothing. Nothing waits for a share. Each entry holds an id, the
owner (pid and exact creation time, compared strictly, unlike the
registry's lenient `pid_matches`), a label, its logical CPUs, and
`expires_at`. An entry holds its CPUs while its owner is alive with the
same creation time and has not released it; a dead owner's entry is
dropped by the next transaction. Background work avoids an entry only
before its `expires_at`. A claim prunes, picks CPUs disjoint from the held
entries, and writes, all under the lock. There is no renewal.

**Size.** `SHARE_CPUS = 4` P-cores (P-cores found with
`GetSystemCpuSetInformation`; on a non-hybrid CPU, any CPUs): one per NavPy
plus the backend. At most `MAX_SHARES = 2`, which covers all 8 P-cores and
leaves the E-cores to everyone else.

**Who claims.** The stack's GCS backend, at startup, when `gcs_launch.py
--cpu-share-s N` asks for it. The backend lives as long as the stack, so
the share ends when the stack stops; its NavPys die with it
(`KILL_ON_JOB_CLOSE` job held by the backend, `navpy_process_instance.py`).
A holder needs that job around every NavPy process. The backend therefore
starts NavPy suspended (`CREATE_SUSPENDED`), assigns it to the job, and
only then resumes it, the order Microsoft documents for
`AssignProcessToJobObject`, so the venv launcher cannot start the real
interpreter outside the job. Today a failed job creation or assignment only
logs a warning and the NavPy runs on (`navpy_process_launch.py:221-244`).
In a holder, the backend first releases the share as at expiry: on a
failed creation before it spawns; on a failed assignment it first
terminates the still-suspended child. Then it starts that NavPy as
background.
Two callers pass the flag: the wall branch of the GCS demo eval
(`--timeout-s` plus 300 s setup margin) and `delivery_demo.py`, the hand
demo (`--cpu-share-min`, default 30, at most 120). Lockstep runs never
claim, because load changes only their wall time.

**Scheduling states.** Set explicitly, whatever the parent's state:

| Stack | Backend and its NavPys |
|---|---|
| Share granted | backend ABOVE_NORMAL on the share's CPUs; NavPys raise to ABOVE_NORMAL |
| Share refused (none free, record busy or unusable) | background, and the backend log says so |
| No `--cpu-share-s` (`AAS_GCS.bat`, live tests) | background |
| Share expired | background |

A refused claim never waits. Its run is todo-65 evidence, not a reason to
wait.

**Holder.** Before it spawns NavPy, the backend sets ABOVE_NORMAL and the
share's CPUs on itself, level with its NavPys: at NORMAL their threads
starved its reader, which fell 3-5 s behind the SITL in acceptance runs C1
and Q1 (not in the runs without a share). Every NavPy inherits the affinity
(checked through the venv launcher, a new process group and a grandchild);
ABOVE_NORMAL is not inherited, so each NavPy starts NORMAL and raises
itself.

**Time limit (confirmed release).** At `expires_at` a one-shot timer in the
backend demotes the stack, then releases the entry: it sets BELOW_NORMAL
and all CPUs on itself and, through each NavPy's job, on every process in
that job (`JOB_OBJECT_LIMIT_PRIORITY_CLASS` and `JOB_OBJECT_LIMIT_AFFINITY`;
checked on a running venv-launched child: both the launcher and the real
interpreter are in the job and took the limits). NavPy spawn and this
demotion share one lock, and a NavPy spawned later inherits background.
Until the release is written, the CPUs stay held, so no new holder gets
them early; a claim that finds no free CPUs is refused at once. The run
itself keeps going. A backend that shuts down before `expires_at` gives the
share up the same way. The expiry timer is a host thread, so only the wall
lifespan starts it, never the lockstep backend.

**Everyone else (background).** `cpu_share_background.apply_background()` sets
BELOW_NORMAL, which children inherit, and limits affinity to the CPUs
outside the valid shares. At each avoided share's `expires_at` a one-shot
timer recomputes the process's own mask from the shares valid then.
Priority is the main guarantee: holder threads outrank all background
work, apart from Windows' short priority boosts. Affinity is best effort: children keep the mask they started
with until they exit, and a job that started before a share keeps all
CPUs. Callers:

- the `pytest_configure` hook in `tests/conftest.py`
- the lockstep evals and the lockstep matrix
- `swarm_run.py --eval`
- `gcs_launch.py` without `--cpu-share-s`, so `AAS_GCS.bat` and live tests
  run as background

**NavPy priority.** `raise_process_qos` raises to ABOVE_NORMAL only from
NORMAL. A NavPy that inherited BELOW_NORMAL keeps it. It still opts out of
EcoQoS.

**Visibility.** `python scripts\cpu_share.py --list` reads the record
without changing it.

**Limits (no polling).** A share releases early only when its owner exits,
and background work learns of that only at `expires_at`. A process outside
the project (Claude, Chrome, Codex) keeps NORMAL. WSL is not touched: the
SITLs held 10x in m1, and every acceptance run checks the SITL clock again
at the GCS. Every Windows call is a no-op off win32.

## Acceptance

At 10x on this PC, with the other chats' work running as usual. Every run
records the probe (host CPU per process, WSL CPU), the SITL clock and
NavPy's extra ATTITUDE delay over the GCS's for each vehicle's whole final
approach, the background load's CPU during that approach, and the eval
verdict.

Runs, 3 of each, interleaved, beside a background load that I start (one
lockstep matrix and a pytest run):

- A: eval and load from a clean `origin/main` worktree (today)
- B: eval from `origin/main` (NORMAL, NavPys ABOVE_NORMAL, no share), load
  from this branch (background): priority only
- C: eval and load from this branch: priority and share

C passes when every run passes the eval and the background load keeps
using CPU during each final approach and finishes. A and B tell which part
does the work; a passing A is inconclusive, not a failure of C. Then two
share-holding wall evals run at once from two worktrees (disjoint P-cores,
both pass). A long-lived background child that spans a share's expiry and
a later claim shows the best-effort affinity case. After todo 66, C runs
again and the share size is re-measured.

Unit tests cover: claim, strict owner identity, expiry, dead owner, no free
share, busy and abandoned mutex, disjoint CPUs, P/E selection (fake kernel),
every row of the scheduling table, release only after demotion (a late
timer keeps the CPUs held), a NavPy spawned during and after expiry, job
creation and assignment failure (the suspended child is terminated before
it runs, then started as background), background mask recompute, a real
child inheriting affinity and BELOW_NORMAL, a real suspended venv-launched
child whose interpreter is inside the job after resume, and a real job
demoting that running child.

## Risks

- Timing-sensitive unit tests run at BELOW_NORMAL and may lose to NORMAL
  desktop work. Re-run such a failure on its own before calling it a
  regression (as today).
- If C still fails, the next suspect is NavPy's own per-packet cost: see
  todo 66.
- The suspended start changes every NavPy launch, not only holders'. It
  also closes the same gap for `KILL_ON_JOB_CLOSE` today.

## Todo 66 (packet-stream cut)

Count NavPy's inbound packets at the companion link, before dispatch, by
(source system, component, message type), per wall second and per sim
second. `run_swarm.sh` already blocks peer autopilot and camera traffic on
the router path (`BlockSrcCompOut = 1,100`), so most of the traffic is
probably the SITL's own `SR0_*` streams at 10x
(`eval_gcs_demo_sitl_root.py:69-76`). A stream group sends several
messages, so the established cut is per message: set the unneeded groups to
0 and request each message NavPy reads with `MAV_CMD_SET_MESSAGE_INTERVAL`,
as `pose_streams.py` already does for the 40 Hz pose. Audit every consumer
first (`pose_telemetry.py`, `flight_telemetry.py`, `position_service.py`,
`preflight_health.py`, `power_gps_telemetry.py`, mission and parameter
protocols). Measure the cut on its own, after the share: reader lag, NavPy
CPU, and the eval verdict. The session "Find and fix NavPy telemetry lag in
NAV" (worktree `nav-reader-lag-3a2c04`) is tracing this reader now; agree
who makes the cut before editing.
