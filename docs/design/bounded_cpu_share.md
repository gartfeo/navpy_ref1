# Bounded CPU share for wall runs

Status: approved by the owner as written (2026-10-28), including
best-effort background affinity; target `main`. Replaces the quiet-PC rule
withdrawn in todo 65 (`8a0b880e`). Built and measured 2026-10-28 (Results).
The hand demo's larger share (2026-10-31) is a later change to that design,
approved by the owner on 2026-11-01.

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
guarded by a named mutex (`cpu_share.py`; `Global\`, so a scheduled task
in another session shares it; a holder that dies hands it over at once, so
no stale lock is left). Every transaction, reads included, takes
it: on Windows an open reader makes the atomic replace fail. A transaction
holds it for milliseconds, so each waits for it at most the registry's lock
timeout (10 s). A claim that cannot get it, or finds the record unreadable
or unwritable, is refused; a release that cannot leaves the CPUs held until
the owner exits (fail closed); a background read that cannot avoids
nothing. A mutex that cannot be created makes the record unusable the same
way. Nothing waits for a share. Each entry holds an id, the
owner (pid and exact creation time), a label, its logical CPUs, and
`expires_at`. An entry holds its CPUs until it is released or its owner is
known dead: Windows knows no process with the pid, the process has exited,
or the pid now has another creation time. An owner that cannot be read
keeps its entry (fail closed). The next transaction drops a dead owner's
entry. Background work avoids an entry only
before its `expires_at`. A claim prunes, picks CPUs disjoint from the held
entries, and writes, all under the lock. There is no renewal.

**Size.** `SHARE_CPUS = 4` P-cores (P-cores found with
`GetSystemCpuSetInformation`; on a non-hybrid CPU, any CPUs): one per NavPy
plus the backend. At most `MAX_SHARES = 2`, which covers all 8 P-cores and
leaves the E-cores to everyone else. The hand demo instead claims every free
P-core, at least 4 (`--cpu-share-all-cores`; "Hand demo" under Results): 8 on
this PC, or the 4 left beside a holding eval. While a demo holds all 8, a wall
eval's claim is refused and that eval runs as background.

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
background. When the demotion fails, the share stays held and that NavPy
does not start: it would run outside any job.
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
| Background priority refused by Windows | backend keeps its class (state `normal`, an ERROR in its log); NavPys still start BELOW_NORMAL |

A refused claim never waits. Its run is todo-65 evidence, not a reason to
wait. A granted share that can be neither applied nor backed out to
background stops the backend at startup: it might run ABOVE_NORMAL on every
CPU with nothing to bound it. Once the backend has exited its entry no
longer holds CPUs, and the next transaction drops it.

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
EcoQoS. A backend without a share starts each NavPy with the
`BELOW_NORMAL_PRIORITY_CLASS` creation flag, so its NavPys never raise
themselves, even when the backend's own background priority was refused.

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

Unit tests cover: claim, owner identity (a reused pid is dead, an
unreadable one is kept), expiry, dead owner, no free
share, busy, abandoned and unavailable mutex, its `Global\` name, disjoint
CPUs, P/E selection (fake kernel),
every row of the scheduling table, release only after demotion (a late
timer keeps the CPUs held), a NavPy spawned during and after expiry, job
creation and assignment failure (the suspended child is terminated before
it runs, then started as background; with a failed demotion nothing
starts), a refused background priority, an inherited `GCS_CPU_SHARE_S` or
`GCS_CPU_SHARE_ALL_CORES` without its flag, every free P-core (alone, beside
a holder, too few), background mask recompute, a real
child inheriting affinity and BELOW_NORMAL, a real suspended venv-launched
child whose interpreter is inside the job after resume, a real job
demoting that running child from ABOVE_NORMAL, and a real child of a NORMAL
backend starting BELOW_NORMAL by its creation flag.

## Results (2026-10-28)

Runs in `.logs/cpu-share` (`acc/`, `t66/`), at 10x beside one lockstep
matrix and a pytest subset, with the other chats' work running as usual
(in A1 the pytest stopped at collection: Application Control blocked a
numpy DLL, so A1 ran beside the matrix only):

| Runs | Pass | NavPy's extra ATTITUDE delay, p95 per UAV |
|---|---|---|
| A (main) | 0 of 3 | 71-226 ms |
| B (priority only) | 3 of 3 | 38-197 ms |
| C (priority and share) | 2 of 3 | 4-61 ms |
| C after todo 66 (C5-C7) | 3 of 3 | 2-12 ms |

- Priority keeps the load from blocking the run; the share also removes
  NavPy's reader lag. The C3 miss (UAV 2, SNAP without `DOCKED`) had no
  lasting backlog (p95 61 ms, max 77 ms) but slow frames (todo 65's load
  sensitivity); a share of the miss from that lag is not ruled out.
- The load kept running during every final approach (matrix 0.3-0.9 core
  in C2-C6; 0.05 in C7, between cells, with about 16 cores idle) and
  finished. In C2 one matrix cell failed: another chat's 20 NORMAL burners
  (a checkout without this change) starved this branch's BELOW_NORMAL
  lockstep cell past its 30 s exit budget (`STEP_DEADLINE_US`). Until every
  checkout has this change, older NORMAL work outranks background work.
- Expiry and a later claim (own record, `acc/expiry`): the background parent widened
  its mask at expiry, its child kept its start mask, neither learned of the
  later claim, and both stayed BELOW_NORMAL, as designed.
- **Two wall runs at once stay unreliable on this PC.** With two shares,
  5 of 14 stacks passed (6 pairs before todo 66, 1 after); with the second
  run refused and background (`MAX_SHARES 1` emulated, `ref1`, `ref2`), the
  holder passed 1 of 2. The failing stacks kept 10x with little lag, but
  their observation coverage fell to 0.19-0.39: during the overlap 17-19 of
  20 cores were busy and the clock fell to about 85% of nominal (about 160%
  when light), consistent with the CPU's power limit but not proven to
  be it. A share bounds cores, not that, so `MAX_SHARES` stays 2 and this
  is todo 65's open load sensitivity. The
  probe's processes and WSL's guest (vmmemWSL, about 3 cores) explain only
  about 10 of those cores; WSL's own Windows processes used under 0.5 core
  and DPCs 2%, so the rest is not yet attributed.
- **Size stays 4.** In final approach a stack used 2.2-2.5 cores on Windows
  (NavPy 0.65-0.8 each, backend 0.1-0.25), leaving room for NavPy's peaks
  (1.27 core p95 in m1). Four is a safe size, not shown to be the
  smallest. Todo 66 saved an estimated 0.02 core per NavPy (the cut
  packets times m1's cost per packet), not a measured one.

### Hand demo: every free P-core (2026-10-31)

The 2026-10-29 hand demo lost UAV 3 at 606 m (`NAV_FAIL: source_command_stale`,
10:06:54.550; `.logs\navpy-demo\delivery-demo\20261029-094230`). The receipt
limit tripped: the frame's pose was received at 54.444, and the limit at 10x is
100 ms. The aircraft lost no time: lines fitted to each UAV's earliest arrivals
over 60 s either side give 10.27x before and after and meet within 1 ms at the
trip. SITL's pacing (`sync_frame_time`) repays a stall of up to 100 ms and loses
the rest for good, so any stall in SITL was at most about 100 ms and repaid. A
short slowing within that cannot be excluded: these CSVs time reader acceptance.
The other 300 ms or more at the trip was readers. All three NavPys and the
backend fell behind together and caught up (peak over the line 588 ms on UAV 3,
161-192 ms on the others, GCS 675-685 ms; 426 ms on UAV 3 at the trip). A window
that ends inside the GCS's backlog shows a low rate for it (7.4x over 53-55 s).
Frame wall time was about 3x its CPU time and the poll's log time was 0 ms, so
they waited for CPU or the GIL, not for disk. The trip was correct: the UAV flew
on telemetry about 4 s of aircraft time old. The limit stays as it is. How it
behaves if the clock itself slows for longer was not reached by this failure, and
stays with todo 54 and design a3.

Replays of that demo with a probe, at 10x with the page open, under one outside
burst at NORMAL priority (8 `git status` and 3 WMI loops, all 20 CPUs 99-100%,
240 s; `.logs/stale-receipt/`, phases in `flights.md`):

| Flight (real approaches) | CPUs | Stale | Largest pose gap | Poll gap p99.9 | Poll gaps >= 50 ms |
|---|---|---|---|---|---|
| A, previous code | 4 | 2 of 3 | 114 ms | 24-25 ms | 13 |
| D, this code, forced to 4 | 4 | 0 of 2 | 198 ms | 13-15 ms | 14 |
| B, previous code, widened by hand | 8 | 0 of 3 | 63 ms | 7 ms | 5 |
| C, this code as shipped | 8 | 0 of 3 | 65 ms | 7-9 ms | 6 |

D had a third UAV whose request timed out unapproved (the driver's timing), so
2 approaches. Its 198 ms stall hit all three NavPys but fell outside an approach.

In two more flights with the same burst, the NavPys moved at run time, NavPy lag
p99 was 42-109 ms on 4 CPUs, 18-38 ms on 8 and 18-42 ms on all 20; HIGH priority
on 4 CPUs was 24-81 ms, so priority does not replace the CPUs. Likely mechanism,
not traced thread by thread: confined to 4 CPUs, the stack waits behind threads
it cannot outrank there (WSL's vCPU threads run at Windows priority 15), though
other CPUs may be free for it to use on a wider share.

Limits of this evidence:

- 2 of 5 against 0 of 6 approaches is a small sample (Fisher p about 0.18). The
  tail difference is the stronger evidence, and it repeated in every comparison.
- The trips in A came with poll stalls of 40-67 ms and no pose gap of 100 ms: the
  100 ms covers the whole path from pose receipt to command, not one stall. So 8
  CPUs make a trip less likely and do not rule it out. Their remaining 53-65 ms
  stalls (about 2 per UAV per 240 s) and rarer quiet spikes of 90-136 ms stay
  (todo 154).
- The 2026-10-29 run had no probe, so what took the CPUs that day is not known
  (todo 155). The mechanism is reproduced, not that day's load.
- Evals stay on 4 CPUs (todo 153): an 8-CPU eval would refuse the owner's demo.

## Risks

- Timing-sensitive unit tests run at BELOW_NORMAL and may lose to NORMAL
  desktop work. Re-run such a failure on its own before calling it a
  regression (as today).
- C can still miss with no lasting reader backlog (C3), most likely from
  frame cost under host load (todo 65); packets cost about 0.1 core per
  NavPy (todo 66), and a part from the remaining lag is not ruled out.
- The suspended start changes every NavPy launch, not only holders'. It
  also closes the same gap for `KILL_ON_JOB_CLOSE` today.

## Todo 66 (packet-stream cut)

Done 2026-10-28 (todo 66 in `architecture_todo.md`). A temporary tally in
the router counted 149 packets per sim second per NavPy, by (system,
component, type). `SR0_EXTRA1` and `SR0_RAW_SENS` carried 28 of them and
NavPy reads none of their messages, so both are 0 now: 123 per sim second,
ATTITUDE 40.0 from NavPy's own request (tallies in `t66/tally-Q3` and
`t66/tally-Q4`). Groups that mix read and unread messages stay; cutting
those needs a request per message. That request is sent once, without an
acknowledgement; since the cut a lost one leaves NavPy without ATTITUDE
(todo 151).
