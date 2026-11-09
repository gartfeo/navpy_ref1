# NavPy recovery: never stop, try to recover (draft r4, 2026-11-05)

Status: draft for the owner. Codex reviewed r1-r4. Its r4 points (bounded
watchdog write, fence synchronization contract) are written in below but not
re-reviewed (one-round process). Nothing
implemented. Covers todos 290-292 and the AUTOTUNE seen after a restart.

Owner rulings (2026-11-05):

- NavPy must not stop or strand a UAV in any situation; it tries to recover.
  The old fail-stops existed so a buggy situation would not go on.
- A failure in the middle of an approach recovers and continues "smart"
  (for example lost frames): approaching stays approaching, searching stays
  searching.
- An operator reset after a finished run resets everything, as if starting
  from scratch. The mode before and after that reset is AUTO.
- Sim only: flight-mode switch position 6 becomes AUTO.
- Persistent NAV cycle errors: 3 s limit (time base open, see Q1).
- 2026-11-06: keep the dock through recovery (Q2). The operator reset resets
  NavPy only; SITL is not restarted (Q4). Most outages are short (a failed
  send, a lost message), so recovery from them must be prompt. Restart
  semantics need research into established practice first (Q3).
- 2026-11-06 (second answer): timers in aircraft time (Q1). Restart model
  agreed: a part restarts fresh inside NavPy with limited tries and back-off,
  escalating to a supervised restart of the whole NavPy process (Q3). A
  flying UAV must never be stranded. A short failure must not trigger an
  action on every failed tick; react only when it is meaningful. A critical
  failure (nothing working) goes to a failsafe-like recovery, possibly RTL.
  After a short failure, search continues where it stopped (Q5). The
  existing GCS stop/start of NavPy is the operator reset; no new GCS action
  (level 5 below is dropped).
- 2026-11-06 (third answer), supersedes level 2 and 4's mode targets below:
  a lasting failure first switches to a hold, then back to AUTO to continue
  where the UAV was once NavPy is healthy again. A critical failure goes to
  RTL. Proposed details, to confirm: the hold is LOITER (the 2026-10-07 hold
  ruling; CIRCLE drifts with the wind), `NAV_HOLD_RECOVERY_S` = 30 s of
  aircraft time in the hold without recovery escalates to RTL, the dock is
  kept as the task through the hold, and an autopilot Lua watchdog on
  NavPy's heartbeat applies the same hold-then-RTL when NavPy is dead or hung.
- 2026-11-06 (fourth answer): follow ArduPilot's own failsafe pattern; LOITER
  hold, 30 s, RTL and the dock kept are confirmed; the watchdog is our own
  module on the autopilot.

## ArduPlane's pattern, which NavPy follows

From the ArduPlane source in WSL (`~/ardupilot`, 4c23c0a1ff):

- Two stages (`ArduPlane/Parameters.cpp:447-484`). A *short* failsafe after
  `FS_SHORT_TIMEOUT` (1.5 s) holds the aircraft (CIRCLE, or no change in
  AUTO/GUIDED/LOITER). A *long* failsafe after `FS_LONG_TIMEOUT` (5 s
  default) takes the final action, typically RTL.
- When the short failsafe clears, ArduPlane restores the saved mode, but
  only if the current mode is still the one the failsafe set
  (`events.cpp:218-228`). A long failsafe is not undone automatically.
- Re-entering AUTO calls `mission.start_or_resume()` (`mode_auto.cpp`);
  with `MIS_RESTART` = 0 (default, set nowhere in this repo) the mission
  resumes at its current item, so LOITER -> AUTO continues where the UAV
  was. The live check reads `MIS_RESTART` back to confirm it.
- ArduPlane's own GCS failsafe (`FS_GCS_ENABL`) watches the GCS heartbeat,
  not a companion's, so it can't detect a dead NavPy; hence our own module.

NavPy's stages, mapped onto it:

| Stage | Trigger (aircraft time) | Action | Undo |
|---|---|---|---|
| Short failure | failed ticks under `NAV_HOLD_TRIGGER_S` | retry quietly, keep state and dock | - |
| Hold | `NAV_HOLD_TRIGGER_S` = 1.5 s with no good tick (ArduPlane's short timeout) | fence commands, LOITER, keep the dock as the task | when healthy again, and only if the mode is still NavPy's LOITER: AUTO, resume where it was, then the normal flow re-approaches the kept dock |
| Critical | `NAV_HOLD_RECOVERY_S` = 30 s in the hold without recovery, or LOITER not confirmed | RTL | not undone automatically (as ArduPlane's long failsafe); the operator decides |
| NavPy dead or hung | the Lua module sees no NavPy heartbeat for `NAV_HOLD_TRIGGER_S` while in NavPy's GUIDED | the same hold, then RTL after `NAV_HOLD_RECOVERY_S` | AUTO when NavPy's heartbeat returns, as above |

The Lua module is a script in `scripts/lua/`, on stock firmware (the
2026-10-07 ruling), deployed like the others. It acts only in GUIDED, or in
the LOITER it set itself, so pilot and failsafe modes always win.

## What happens today (code read, not run)

| Failure | Today | Where |
|---|---|---|
| NAV cycle raises (not OSError) | logged, told once, active approach failed `internal_error`; the next cycle's decide goes NAV_FAIL -> RESET | `core/flow/navigation_loop.py` `_recover`; `app/flow/nav_runtime_composition.py` `_fail_active_approach` |
| NAV cycle raises every tick | if it raises in sensing (before decide, `navigation_loop.py:123-128`) decide never runs and nothing reaches RESET; the UAV keeps its last mode and target (todo 292). Errors after decide still let decide run | same |
| NAV cycle raises OSError | logged and retried; a raising logger there ends the loop | `navigation_loop.py:198` |
| NAV timing, period or deadline wait raises | outside the cycle's handler; latched by the thread wrapper, loop ends, health check ends NavPy (todo 291's OverflowError lands here) | `navigation_loop.py:183-216` |
| Command work raises Exception | handled inside the running worker (`navigation_command_worker.py:147`); an error escaping the pass restarts the worker after one period (`:100-106`) | `core/approach/navigation_command_worker.py` |
| Command worker dies (BaseException) or hangs | nothing notices: the command session's health checks only postprocessing | `navigation_command_session.py:126` |
| Held or new command too old (1 s source age) | hold cleared, episode failed; the NAV exit still needs a working cycle | `vision_nav/command_hold.py`, `command_freshness.py`, `command_executor.py` |
| Evidence write fails | `NavigationPostprocessFailure` -> health check -> NavPy ends (todo 290) | `navigation_postprocess_dispatcher.py:89`; `app/runtime_resources.py:223` |
| Camera, gimbal, detector, publisher, coordinator, network fails | first failure latched for good; health check ends NavPy | `core/kernel/failure_health.py`; `app/runtime_composition.py:244` |
| Heartbeat send raises | the heartbeat thread dies silently; NavPy keeps running with no companion heartbeat | `plugins/vehicle/ardupilot/heartbeat_runtime.py:102` |
| NavPy process exits in sim | GCS restarts it only for auto-managed, active sys_ids | `gcs/backend/navpy_sim_runtime.py:184` |
| NavPy process exits on the aircraft | nothing restarts it | - |

Mode on the normal paths: RESET entry only clears state
(`navigation_transition_handler.py:99`). AUTO comes on RESET -> DETECT through
`restart_search`, which sets AUTO and restarts the mission at item 1
(`products/delivery/flow.py:62`). The delivery resume path
(`AutoMissionResume.run`, `navigation_task_reset.py:242`) sets AUTO after
several fallible calls and ignores the result. A pilot mode outside
GUIDED/AUTO/LOITER, or disarmed, makes decide request ONHOLD after clearing
the task (the clearing can raise first), and NavPy never changes that mode
(`navigation_decision.py:79-90`).

GCS "Restart mission" (`routes/control_launch_workflow.py:154-179`) sets the
mission to item 0, arms if needed, requests AUTO and reports "restarted"
without checking the mode. It does not clear NavPy, swarm or sim state.

## AUTOTUNE after a restart

Seen in UAV 1's ArduPilot log `~/ardupilot/1/logs/00000008.BIN` (owner run,
21:39): `MODE 8 (AUTOTUNE), Rsn 1 (RC)` with RCIN C5 = 1800 us, then `MODE 10
(AUTO), Rsn 2 (GCS)` 0.7 s later. SITL's built-in RC holds channel 5 at
1800 us (`AP_RCProtocol_UDP.cpp`); our params use channel 5 as the mode switch
with position 6 = AUTOTUNE (`scripts/eval_gcs_demo_sitl_root.py:55-57`; the
GCS stack's values live in the WSL template eeproms). Codex confirmed the
mechanism from upstream source; the event that made ArduPlane re-read the
switch at 21:39 is not proven.

Fix: sim UAVs only, FLTMODE6 = 10 (AUTO) and INITIAL_MODE = 10, written before
boot (eval root defaults, and the GCS sim stack's launcher-owned instances)
and read back after connect. The connect-time write is only a backstop. The
real aircraft's switch layout is not touched. The GCS restart confirms AUTO
from the heartbeat and reports failure instead of "restarted".

## Design

### Principles

- **One owner of navigation transitions.** Recovery decisions run on the NAV
  loop thread, outside the cycle's error boundary, never on a second thread
  that mutates the same state. A separate watchdog only detects a hung loop
  and escalates (below).
- **Flight handover first, cleanup second.** "Control handed to the mission
  (AUTO confirmed from the heartbeat)" is recorded separately from "reset
  cleanup complete". Cleanup is retried; handover never waits for it.
- **Pilot and autopilot failsafe keep authority; ownership is explicit.**
  NavPy holds an ownership record per approach episode. It is acquired when
  NavPy requests GUIDED for that episode and a fresh heartbeat (newer than the
  request) confirms GUIDED. It is revoked by any heartbeat showing a mode NavPy
  did not request (pilot, GCS, failsafe), by disarm, or when the episode ends
  with AUTO confirmed. Automatic recovery requests AUTO only while ownership
  is held, and rechecks a fresh heartbeat before every request and retry.
  Unknown ownership (no fresh heartbeat, lost link) means no automatic mode
  change. During search the UAV is already in the mission's AUTO and recovery
  changes no mode. The AUTO-always rule applies only to the operator's sim
  reset.
- **Recovery is a latched state outside the cycle.** A `NavRecovery` record
  owned by the NAV loop, checked before sensing each period. While it is
  pending, the period runs only recovery steps: no sensing, decide or action,
  and command generation stays fenced, so normal work can't reopen what
  recovery is closing. Each cleanup step is idempotent and retried; the latch
  clears only when handover and cleanup are both done.
- **Existing approach bounds stay.** The 1 s command freshness and hold
  expiry are not extended to fit a recovery timer.
- **Pure vision constraint holds.** Recovery may read mode, armed state and
  mission state. It ends the final-approach command generation before any
  mission recovery, never feeds the approach law, and never routes an active
  approach through `recovery.py`'s altitude branch.

### Levels

1. **Continue in state.** A failed cycle or a lost frame is logged, reported
   once, and NavPy keeps its state; approach and search continue. The
   approach's own checks (freshness, lost POI) still end an approach.
   A cycle error no longer fails the approach by itself.
2. **NAV handover.** If no cycle succeeds for `NAV_CYCLE_ERROR_HANDOVER_S`
   (3 s), or the approach episode ended (for example hold expiry) and the
   cycle cannot run its exit, the loop sets the recovery latch: fence command
   generation, request AUTO if ownership is held, wait for a fresh AUTO
   heartbeat up to `AUTO_CONFIRM_S`, retry the request each period, then retry
   the RESET cleanup each period until it succeeds. The search mission
   continues. AUTO not confirmed within `AUTO_CONFIRM_S` is reported to the
   operator and goes to level 4; the latch keeps retrying meanwhile.
3. **Component restart, only where proven safe.** A component restarts
   in-process only after it is verified quiet, with a new generation that
   rejects old frames and callbacks (camera capture, detector, heartbeat
   worker are the candidates). Others are not restarted in-process.
4. **Escalation.** Triggers: a component that can't be quiesced, a hung NAV
   loop or command worker (a watchdog sees no progress counter advance), or
   AUTO not confirmed in level 2. The watchdog is a separate thread with two
   powers only, both safe without the NAV loop:
   - **Close the send fence.** Every guided or attitude command carries the
     command generation it was formed in. The vehicle plugin checks it at
     the final send boundary, inside `MavTransport`'s send lock
     (`plugins/vehicle/ardupilot/mav_transport.py`), right before the MAVLink
     write. A closed fence or an old generation drops the send and logs it,
     including a command that was already inside `execute_work`
     (`navigation_command_worker.py:133-151`) and the GUIDED mode request in
     `guided_navigation.py`. The watchdog closes the fence by advancing the
     generation under the fence's own small lock, never a worker's lock, so a
     hung worker can't block it. Contract: a write is admitted only while
     holding the send lock and, inside it, the fence lock, with the fence
     open and the generation current. Closing takes the fence lock only. A
     write admitted before closing is "in flight" (at most one, because the
     send lock serializes writes); no write is admitted after closing.
   - **Request AUTO** under the ownership rule, retried until a fresh AUTO
     heartbeat or ownership is revoked. `MavTransport.try_call` bounds only
     taking the lock, not the write, so the watchdog hands each AUTO write to
     a short-lived sender and waits for it with a deadline. A sender that
     doesn't return is reported and left behind; the watchdog's own loop keeps
     running, reporting and retrying. If the send lock is held by a hung send,
     nothing else can send either; this is reported too.

   It never touches NAV state. With the fence closed, ArduPlane keeps flying
   to its last GUIDED target and loiters there, so the UAV is held, not lost.
   NavPy exits for a fresh start only when all hold: commands fenced,
   ownership released (AUTO confirmed, or the pilot/failsafe took over), and a
   supervisor is known to restart it (the GCS in sim for auto-managed
   companions; systemd on the aircraft once it exists). Otherwise it stays
   alive, fenced, retrying and reporting. A failed handover never ends in an
   exit while NavPy owns GUIDED.
5. **Operator sim reset.** A distinct GCS action, not "Restart mission":
   stop participants, clear NavPy, swarm, GCS task and map state, reload
   mission and config, set mission item and AUTO, then confirm AUTO and
   readiness. Whether SITL also restarts (pose and time back to the start)
   is Q4.

### Evals stay honest

Each recovery is counted through a path independent of the failing writer,
and the evaluator sets the verdict: clean, recovered, or invalid evidence. A
clean-run gate rejects unexpected recoveries; a fault-injection eval passes
only when its expected recovery happens. Todo 290: an evidence-only failure
keeps flying and marks the evidence incomplete.

## Questions for the owner

1. Time base for the 3 s (and `AUTO_CONFIRM_S`): aircraft time or wall time?
   My suggestion: aircraft time (ATTITUDE boot time), because the existing 1 s
   freshness bound is aircraft time, so both scale the same with sim speedup;
   at speedup 10, a 3 s wall limit would let the UAV fly 30 s of sim time on
   stale state. Fallback when no fresh aircraft time arrives: wall time, since
   a lost link must not stop the timer.
2. During an approach with persistent errors: keep the same POI only while
   its episode and inputs are still valid, otherwise hand over to the search
   mission (Codex's and my suggestion)?
3. Component restart: in-process only for camera capture, detector and
   heartbeat; everything else escalates to a supervised restart. Agree?
4. Operator sim reset: does "from scratch" include restarting SITL (UAVs back
   at home, sim time reset), or only NavPy, swarm and GCS state?
5. After automatic recovery (not the operator reset), the search mission
   resumes where it was interrupted. Today `restart_search` restarts it at
   item 1. Which do you want?
6. Should the aircraft get a supervisor (systemd) now, or later with HITL?

## Implementation plan (2026-11-06, for owner approval)

Order matters: each step lands on its own commit with its tests, and steps
1-2 can merge on their own. Todo 291 (branch `claude/sharp-mirzakhani-1a4e62`)
merges before step 4, which builds on its deadline fix.

1. **AUTOTUNE (sim only).** Eval roots: `FLTMODE6` = 10 and `INITIAL_MODE` =
   10 in `SITL_DEFAULT_OVERRIDES` (`scripts/eval_gcs_demo_sitl_root.py`).
   GCS sim stack: on connect to a launcher-owned sim instance, write and save
   both params if they differ, then read them back. Saving updates the eeprom,
   so the next boot starts right. GCS "Restart mission" confirms AUTO from the
   heartbeat, retries a bounded number of times, and reports failure instead
   of "restarted". Tests: param list and scoping (sim, owned instance only);
   restart route with a mode that never confirms.
2. **Send fence.** Command generations and a fence checked inside
   `MavTransport`'s send lock at every guided/attitude send and NavPy's own
   GUIDED request. Tests: a command admitted before closing goes out, none
   after; an old generation is dropped; closing never takes a worker lock.
3. **Ownership record.** Acquired on a fresh GUIDED heartbeat after NavPy's
   request, revoked by any other mode, disarm, or confirmed handover; unknown
   when the heartbeat is stale. Tests: each acquire/revoke path; no automatic
   mode request without ownership.
4. **NAV recovery stages in the loop.** Aircraft-time health tracker
   (`NAV_HOLD_TRIGGER_S` 1.5 s, `NAV_HOLD_RECOVERY_S` 30 s, wall-time fallback
   when no aircraft time arrives), recovery latch checked before sensing,
   timing code inside the loop's guard. Short: retry and keep state (the
   per-error `_fail_active_approach` is removed). Hold: fence, LOITER if owned,
   keep the dock. Healthy again and still in NavPy's LOITER: AUTO, resume, and
   the normal flow re-approaches the kept dock. 30 s or LOITER not confirmed:
   RTL. Tests: one failed tick changes nothing; 1.5 s of failures gives LOITER
   once (not per tick); recovery gives AUTO and the same dock; 30 s gives RTL;
   a pilot mode is never touched; the approach command path gains no new
   inputs (Pure Vision Approach Constraint).
   Owner ruling 2026-11-07, hold model "keep running, blocked": in the hold
   the full cycle keeps running behind the closed fence, so health is seen;
   1.5 s of good cycles is healthy. While holding, NAV does not end the
   approach for the hold's LOITER and search is not restarted. Without
   ownership (search in the mission's AUTO, a pilot mode) the hold only
   fences, there is no RTL, and the fence reopens when healthy. Known gap:
   GUIDED set implicitly by a peer goto records no ownership request, so a
   hold then only fences and ArduPlane loiters at the last GUIDED target.
   Fixes after the Codex code review of steps 1–4: recovery's clock only moves
   forward and adds aircraft time per step, else wall time, so aircraft time
   freezing after it moved still lets the timers expire
   (`nav_recovery_clock.py`); the command worker states the fence generation
   from taking a work item to sending it, so a close and reopen meanwhile drops
   it; critical sends RTL once a fresh heartbeat shows the mode still owned
   (entered while the link was stale); after the hold ends the
   decision asks for GUIDED again instead of reading the resume's AUTO as
   GUIDED lost, so the kept dock is approached; an unowned GUIDED request
   lapses when another mode or a disarm shows, or when NavPy stops waiting.
5. **Component restart and escalation.** Camera capture, detector and
   heartbeat worker restart fresh with a new generation, limited tries and
   back-off. Any other latched failure, or a part that can't be quiesced,
   escalates: hold, then exit only when commands are fenced, ownership is
   released and a supervisor exists (GCS-managed sim); otherwise NavPy stays
   up, fenced and reporting. The health check no longer ends NavPy by itself.
   Tests per component, plus the escalation exit rule.
   Step 5a as built: the first failure the runtime health check raises
   closes the fence and asks NAV to escalate (`RuntimeEscalation`,
   `NavEscalation`): if NavPy owns the mode it requests AUTO; AUTO not
   confirmed in 3 s gives RTL. NAV releases the UAV once the mode is not
   NavPy's (AUTO handed over, the pilot's) or RTL shows. Only then, and
   only with `--supervised` (passed by the GCS to the auto-managed sim
   companions it restarts on exit; the lockstep launcher restarts nothing
   and never passes it), NavPy exits for a fresh start. Otherwise it stays
   up, fenced and reporting, until the operator stops it.
   Fixes after the Codex review of the step 1–4 fixes and step 5a: only a
   hold that ends in NavPy's own AUTO (handed over) resumes the approach; a
   mode the operator took in the hold or the resume stands, and the approach
   ends as before. The resume is a latch the decision takes, not a one-period
   pulse, so a cycle that fails before deciding does not lose it; a new hold
   voids it, and it applies only while AUTO still shows (an operator mode
   selected after the resume stands). NavPy's GUIDED request is recorded at
   the heartbeat count and mode read at the request boundary, just before
   the send (`RecoveryHold.guided_requested` reads the link), so a mode
   change after the send lapses it and one before it does not; a GUIDED
   request formed behind the fence (dropped) is not recorded. An unowned escalation releases only on a
   fresh heartbeat and once NavPy's pending GUIDED request is settled or has
   not shown within 3 s (`MODE_CONFIRM_S`). The main thread latches the fence
   closed (`MavCommandFence.latch_closed`), so NAV's resume racing the
   escalation cannot reopen it.
   Steps 5b-5c as built: restarts share one budget
   (`core/kernel/restart_budget.py`): three tries, the first at once (short
   outages recover promptly), then after 1 s and 4 s; an instance that ran
   30 s without failing, counted from its successful start, earns every try
   back. Times are aircraft time with the wall-time fallback, as for NAV
   recovery (`AircraftTimeClock` in `nav_recovery_clock.py`, owner ruling
   Q1). The health check (main thread)
   drives them, and only a part that cannot be restarted raises, which is
   step 5a's escalation. Step 5b: each mount's detector sits behind a stable
   proxy (`app/perception/restarting_detector.py`); a failed instance must
   stop completely (its stop returns True: threads and model cleanup), and is
   replaced by a fresh one from the same
   factory (the real detector owns the camera capture, so this restarts the
   capture too; the integration-timing input goes again to the same mount).
   An instance that does not stop escalates at once and stays the owned one,
   so the shutdown retries its cleanup; a replacement that fails to start and
   does not stop becomes the owned one for the same reason. Each instance is
   a source generation: the proxy stamps it on every object the instance
   returns (`DetectionIdentity.source_generation`), the coordinator's task ids
   key on it (`PoiIdentity`), and tracking and zoom commands for a task seen
   by a replaced instance are refused, since the new tracker's local ids
   start again. The restart replays no tracking, geo or zoom command: object
   ids were the old instance's and its event lease closes, so the approach's own checks (closed source, lost POI,
   freshness) end an approach as for lost frames, and search detects again.
   The camera row reads not ready while a restart is pending. Step 5c: a
   failed swarm heartbeat generation is restarted fresh instead of its
   failure being kept for good (`SwarmHeartbeatRuntime.raise_if_failed`);
   after the owner's stop, or for a BaseException, the failure still stands.
   Restarts, and NAV recovery's stage changes, are reported to the operator
   as status texts. The companion
   heartbeat to the autopilot (`plugins/vehicle/ardupilot/heartbeat_runtime.py`)
   no longer dies on a failed send: the next period is the retry, the first
   failure and the recovery are logged, and sends that keep failing are the
   Lua watchdog's case (step 7), since the autopilot then sees no NavPy.
6. **In-process watchdog.** A thread that watches loop and command-worker
   progress counters; on a hang it closes the fence and requests LOITER under
   the ownership rule through a bounded sender. Tests: a hung cycle and a hung
   worker each give the fence and the hold; the watchdog never touches NAV
   state.
   As built (`core/flow/nav_watchdog.py`), after the Codex review of the
   first version, which ran a second hold machine and handed it back:
   - One stage machine. NAV recovery (`NavRecovery`) is the only hold:
     one stage, one mode request, one hold deadline. The NAV loop steps it
     each period; while the NAV loop itself is stuck the watchdog thread
     steps the same object in its place, one step at a time
     (`SharedRecoveryStep`, `nav_recovery_step.py`; the watchdog waits for
     its lock at most 1 s, and a step never blocks). Nothing is handed over
     when the hang ends.
   - Detection. The NAV loop's cycle and recovery step, and each whole
     command-worker pass (`command_pass`, deadline to next wait), mark
     themselves busy on a progress beacon (`core/kernel/progress_beacon.py`).
     A pass busy for `NAV_HANG_S` (1.5 s) beyond the waits it declared
     bounded (`bounded_wait`: every reply wait of the vehicle link, so a
     parameter exchange's 3 x 2 s) is stuck. The hang is measured in aircraft
     time, like the flight timers it leads to (owner ruling Q1, and on Codex
     round-2 finding 7): at a sim speed-up it trips at the same aircraft time,
     not that many times later. Wall time is only the backstop while aircraft
     time is frozen, no new ATTITUDE time for `AIRCRAFT_TIME_STALE_S` (0.5 s),
     counted from the last sample, so a stalled link or a frozen SITL still
     trips (`HangClock`, `nav_recovery_clock.py`). The declared waits are
     wall-time bounds and count on the same clock, so at speed-up k a wait of
     W s allows only W/k s of wall time.
   - While stuck: the watchdog holds the fence closed
     (`MavCommandFence.hold_closed`, one lock with reopening), so no reopen
     opens it, even one recovery decided before the hang was seen; recovery
     then stays in its hold. `WatchdogSignal.hung` fails recovery's period
     and holds at once (the hang already outlasted the trigger). A hand-over
     to AUTO that completes while NavPy is unhealthy keeps the fence closed
     and resumes the approach once healthy. A healthy hold does not resume
     while its LOITER request is unconfirmed. A hold that only now owns the
     mode (NavPy's GUIDED sent just before a hang, shown after the hold
     began) asks for LOITER then. Recovery marks itself holding before it
     requests any mode, so the cycle never sees its LOITER as the pilot's.
   - Mode writes (`core/flow/nav_mode_sender.py`). Recovery never writes a
     mode itself: it submits to the mode sender's thread, so a stalled write
     cannot stall recovery or the watchdog. The write waits for the link's
     send lock at most `MODE_SEND_LOCK_TIMEOUT_S` (0.5 s), and a guard under
     that lock, right before the bytes go out, cancels it unless the
     heartbeat still shows the mode recovery decided from (a pilot mode
     shown meanwhile stands). Only a write that passes the guard is NavPy's
     request, recorded at the heartbeat count it read; an unsent one records
     nothing, so the pilot's later choice of that mode is never taken for
     NavPy's. Re-submitted after `MODE_RETRY_S`.
   - Steps take their time sample under the step lock, so the NAV loop's
     and the watchdog's steps see aircraft time in order.
   - Started with the NAV application, stopped before the loop's shutdown
     join. It never touches NAV state (task, phase, decision).
7. **Autopilot Lua watchdog** (`scripts/lua/nav_watchdog.lua`). It receives
   NavPy's HEARTBEAT (`mavlink:register_rx_msgid`). After 1.5 s without one
   in GUIDED, it sets LOITER; after 30 s more it sets RTL; when the heartbeat
   returns and the mode is still its LOITER, it sets AUTO. It does nothing in
   any other mode, runs its loop under `pcall`, and reports its state, so
   NavPy and the GCS can show the operator when it isn't running. Params
   under the AAS table. Tests: the lua5.3 harness, as for `chute_deploy.lua`,
   including the 100-local limit and the instruction hook. Redeployed to WSL
   `~/ardupilot/{1,2,3}/scripts` before the SITL check.

   As built (2026-11-08):
   - NavPy's heartbeat is a HEARTBEAT from this aircraft's system id,
     component `AAS_NWD_COMP` (191, `MAV_COMP_ID_ONBOARD_COMPUTER`), type
     `MAV_TYPE_ONBOARD_CONTROLLER`, with a good CRC: ArduPilot also hands
     scripts frames whose CRC failed (`GCS_Common.cpp`, bad-CRC framing).
     The GCS (system 255) and the autopilot (component 1) never count.
   - "NavPy's GUIDED" is a GUIDED, armed, during which a NavPy heartbeat was
     fresh. A GUIDED entered while NavPy was dead, or never alive, is left
     alone.
   - Params, in the script's own table (key 89, prefix `AAS_NWD_`; step 9
     found that sharing `aas_params.lua`'s key 88 lets one script's
     `add_table` re-blank the other's params, so the watchdog read no
     `AAS_NWD_ENABLE` and stayed disabled in SITL): `AAS_NWD_ENABLE` (1),
     `AAS_NWD_TMO` (1.5 s, `NAV_HOLD_TRIGGER_S`), `AAS_NWD_RTL` (30 s,
     `NAV_HOLD_RECOVERY_S`), `AAS_NWD_COMP` (191). Times are the
     autopilot's own clock (aircraft time); a heartbeat is timed when the
     autopilot received it (`receive_chan`'s timestamp), so one read late
     from the queue is not fresh, and only one received after the hold
     began ends it.
   - Timeout: fixed, `AAS_NWD_TMO` of aircraft time after the last
     heartbeat received (Demo's ruling on Codex round 2). A cadence
     estimate (eabb6503) was dropped: it could not tell a cadence from a
     duplicate burst (which collapsed it) or an outage (which became the
     period, unbounded), and it never timed out after a single heartbeat.
   - Speed-up, fixed at its root in NavPy (`heartbeat_pacing.py`): the
     companion heartbeat's period is aircraft time; its wall wait is the
     period divided by `SIM_SPEEDUP`, read from the vehicle's parameter
     cache (absent on real hardware, so 1). Each PARAM_VALUE updates the
     cache, so it follows a run-time change (NAV's speed-up lease, an
     operator's param set); the update wakes the wait, the heartbeat goes
     out at once and the next period runs at the new rate, so no gap is
     paced at the old one. A SITL running below its `SIM_SPEEDUP` only brings
     heartbeats closer in aircraft time. It is the one period conversion
     outside `SchedulerCadence` (named in `test_frozen_inventories.py`): the
     heartbeat starts with the vehicle, before any cadence, and must not take
     the cadence's process-wide timer and interpreter settings. Lockstep runs its SITL at speed-up 1 on a
     virtual clock, which is aircraft time already. Pacing by the aircraft
     time NavPy receives was not chosen: a stalled downlink would stop the
     heartbeat, and the autopilot would take over from a NavPy that still
     commands over the uplink and holds by itself.
   - Rate (owner ruling 2026-11-08, "4 Hz, 1.5 s"): the companion heartbeat
     is 4 Hz (`HEARTBEAT_HZ`, `vehicle_mav.py`) against `AAS_NWD_TMO` 1.5 s,
     so a hold needs 1.5 s without one, six beats in a row lost; five lost
     do not hold. Radio cost: ArduPilot forwards the companion's broadcast
     HEARTBEAT to its other links, the GCS telemetry radio among them; at 21
     bytes a frame (MAVLink 2, unsigned), 3 more a second is about +63 B/s
     per UAV on the downlink. The GCS backend's own heartbeat (also a
     `VehicleMav`, `MAV_TYPE_GCS`) stays 1 Hz (`GCS_HEARTBEAT_HZ`): no
     watchdog reads it, and it is uplink on every radio. Checked for things
     keyed to the old 1 Hz: the companion's "heartbeat timeout" warning
     (autopilot silent) ran once per send, so it is now held to once a
     second of wall time (`HEARTBEAT_TIMEOUT_REPORT_S`), as before; the GCS
     companion liveness (`companion_status`, 5 s ok / 15 s down windows) and
     `MavBusHeartbeats` keep only the newest receipt time, so a faster
     heartbeat only keeps them fresher; mode confirmation counts the
     autopilot's heartbeats, not the companion's; no frozen-inventory guard
     names the rate.
   - Stages: WATCH, HOLD (its LOITER), RTL (not undone, as ArduPlane's long
     failsafe). Leaving its LOITER or RTL for any other mode, or disarming,
     stands it down. A refused mode change is retried each tick and reported
     once; a refused AUTO keeps the hold and its RTL deadline.
   - Mode writes, a known limit: each is preceded, as the last binding
     call, by a read that the mode is still the one it decided from (GUIDED
     for the hold, its LOITER for AUTO or RTL); a test checks the read is
     the binding called right before every write. Bindings take the
     scheduler semaphore one call at a time and ArduPilot's scripting has no
     compare-and-set, so a pilot's mode change processed by the main loop
     between that read and `set_mode` is still overwritten. This is the
     smallest window a stock-firmware script allows.
   - Payload length is checked (at most HEARTBEAT's 9 bytes) before the
     CRC, so corrupt frames claiming long payloads cannot run the script
     past `SCR_VM_I_COUNT`, which `pcall` cannot catch.
   - State report: NAMED_VALUE_FLOAT `NAVWD` once a second (-1 off, 0
     watching, 1 hold, 2 RTL), sent even after an error in a step; status
     texts on each change. The loop's step runs under `pcall`.
   - `mavlink:init(queue, ids)`: the order `lua_bindings.cpp` reads (the
     docs.lua comment has it reversed). The receive queue is the scripting
     engine's, shared by all scripts; no other script here receives MAVLink.
   - Not covered: NavPy's own hold. If NavPy set LOITER (its in-process
     hold) and then dies, the mode is LOITER the script did not set, which
     it cannot tell from a pilot's LOITER, so the UAV loiters until the
     operator acts.
   - Eval roots (`eval_gcs_demo_sitl_root.py`) deploy every
     `scripts/lua/*.lua`, so evals now run with the watchdog.
8. **Evals honest.** Each stage change is counted and logged apart from the
   evidence writer; eval verdicts are clean, recovered or invalid.

   As built (2026-11-08):
   - Recovery journal: every recovery report goes through
     `ILogger.recovery(message, source)`. It is counted and appended to
     `uav_<id>_recovery.jsonl` (`core/kernel/logging/recovery_journal.py`)
     first, then logged as before (a warning with its status text to the
     vehicle). The journal line is written and flushed in the caller's
     thread, apart from the evidence writer (the navigation CSV streams) and
     from the log worker's queue and its de-duplication. Sources: `nav` (NAV
     recovery stages), `watchdog` (a stuck loop and its end), `vision`
     (detector restarts), `swarm` (swarm heartbeat restarts), `heartbeat`
     (a failed companion heartbeat send and its recovery). The file opens in
     append mode with an `open` line, so a restarted NavPy adds to the same
     journal. A failed open or write is reported once with
     `RECOVERY_JOURNAL_FAILURE` in the NavPy log.
   - Verdict (`scripts/eval_gcs_demo_recovery.py`), set by the evaluator,
     per vehicle and per run, in `report.json` (`verdict`, and each
     vehicle's `recoveries`):
     - `invalid`: the evidence is missing or incomplete: a missing file
       (the recovery journal included), the evidence-writer or
       recovery-journal failure marker, unreadable final-approach evidence or
       journal, or a lockstep analysis that raised. Not scored, never a pass.
     - `failed`: a product gate failed, or a run without scheduled faults
       recovered (the clean-run gate, "unexpected recovery in a clean run").
     - `recovered`: every gate passed, and NavPy recovered or the run's
       scheduled faults bit and recovered (their gates,
       `eval_gcs_demo_fleet_faults`, already require that). A pass only
       where faults were scheduled, that is a lockstep grid with faults.
     - `clean`: every gate passed and nothing recovered.
     The exit status is unchanged: 0 only for a pass (clean, or recovered
     with scheduled faults), else 1; the verdict tells invalid from failed.
   - Codex review of step 8, fixed:
     1. Recording never blocks recovery: `record` only counts and appends
        to a bounded buffer (`RECOVERY_JOURNAL_BACKLOG`, 256) under a lock
        no I/O holds; a writer thread of its own writes and flushes. A full
        buffer drops the entry and counts it.
     2. Each process writes `open` first and, when its logger closes,
        `close` with how many entries it recorded and dropped. The eval reads
        the journal per process: every process after the first is a recovery
        ("NavPy restarted", whether its predecessor crashed or stopped); a last
        process without `close`, a count that disagrees with the lines, or any
        drop makes the evidence invalid.
     3. The autopilot's NavPy watchdog: NavPy subscribes to its own
        autopilot's NAMED_VALUE_FLOAT `NAVWD` and "NavPy watchdog: ..."
        status texts (`app/autopilot_watchdog_report.py`); a stage change
        into or out of hold or RTL, and each action text, is a recovery
        report (source `autopilot`). A NavPy alive while its heartbeat does
        not reach the autopilot, or restarted during the hold, sees them. Not
        covered: a NavPy that hears neither while the Lua watchdog acts and
        is never restarted; its run then fails or is invalid on its own
        evidence (no `close` line, or the mission gates).
     4. The run-level evidence errors (the backend log, the lockstep journal
        and NavPy lines, the slice and delivery evidence) carry named
        prefixes (`RUN_EVIDENCE_ERRORS`), and make the run invalid.
     5. A journal or NavPy log that exists but cannot be read is invalid
        evidence, not an aborted analysis.
   - Codex round 2 on step 8, fixed: the logger closes the journal before
     its worker stops, so a failure writing the close line is still logged
     (and the run invalid); only reading and parsing the backend or delivery
     evidence is invalid, while readable evidence that breaks an assignment
     rule is a product failure (`global assignment rule broken`, `delivery:`);
     a close line whose counts are not non-negative integers is invalid.
   - To check in SITL (step 9): every NavPy in a wall and a lockstep eval
     closes its logger at the end, so its journal ends with `close`;
     otherwise every run would read as invalid.
9. **Checks.** Targeted tests with `--testmon` and by name for Lua and
   configs; `tests/gcs` and `npm test`/`npm run build` if GCS UI text changes.
   live-tester on its own slot, every stack stopped afterwards:
   (a) a sim restart stays in AUTO, never AUTOTUNE; (b) an injected cycle
   fault mid-search and mid-approach gives LOITER, then AUTO resuming at the
   current item, then the kept dock; (c) a fault held past 30 s gives RTL;
   (d) a killed NavPy gives the Lua LOITER, then AUTO after the GCS restarts
   it; (e) `MIS_RESTART` read back as 0. One Codex review round on the code.

   Fault injection for (b) and (c), SITL only (`navpy/sim/cycle_fault.py`):
   `--sim-cycle-fault-file PATH` wraps NAV's cycle at composition (without
   the flag the cycle is not wrapped). While PATH exists each NAV cycle
   fails: `hang` in it blocks the cycle (a stuck NAV loop, for the step-6
   watchdog); anything else raises (a failed cycle, for NAV recovery).
   Deleting it heals; remove it before stopping NavPy. Its default is the
   environment variable `NAVPY_SIM_CYCLE_FAULT_FILE`, which reaches a NavPy
   the GCS starts; `{sys_id}` in the path gives each UAV its own file.
   Refused with an error
   on a non-simulated autopilot and by the lockstep host. Arming it and each
   mode change are recovery reports (source `sim_fault`), so an eval run with
   it fails the clean-run gate, never scores clean.
10. **Todos.** Done entries for 292 (and 291/290 from their sessions); new
    todos: systemd supervisor on the aircraft (with HITL), an upstream
    ArduPilot companion-heartbeat failsafe, and the unproven trigger of the
    21:39 mode-switch re-read. From the step 9 SITL checks (2026-11-09),
    not fixed here:
    - a false "NavPy NAV loop stuck" on all three NavPys at once, for about
      1.7 s, with no fault injected (likely a host stall; the in-process
      watchdog's `NAV_HANG_S` counts it). A UAV owning GUIDED would have
      held for it;
    - with `AAS_NAV_ONESHOT` = 1, a failed dock (DOCK_FAILED) disarms in
      simulation, and the SITL UAV fell (`Hit ground at 9.4 m/s`); seen when
      the held approach still failed stale before the step 9 fix.

   Step 9 fix, re-approach after a hold (owner ruling 2026-11-09, "New
   approach"; `core/flow/nav_held_approach.py`): the command worker's held
   command goes stale (1.0 s) before the hold triggers (1.5 s), latching the
   episode's failure and command liveness, which the resumed approach read
   as `NAV_FAIL: source_command_stale`. Now the NAV decision ends the
   in-flight episode at its first decision while recovery holds
   (`NavExitCleanup.end_episode`: dock-contact episode, source admission,
   navigation reset; tracking, zoom, speedup and task stay) and drops what
   it latched: not DOCK_FAILED, no one-shot finish. After the hold, if
   recovery resumed in AUTO (or GUIDED was not reached before the hold, or
   shows), it starts a fresh episode to the kept dock through NAV entry;
   a mode the operator selected stands and ends the approach.

   Codex review of the step 9 fixes, fixed:
   1. The default delivery hub transit also asks for GUIDED through
      `request_guided_owned`; the guard test scans all of `src/navpy`.
   2. A fresh episode starts only on recovery's explicit resume in AUTO. A
      hold that ends without it (the operator's mode, or a hold NavPy did
      not own the mode for) ends the approach: no GUIDED over that mode.
   3. The fresh episode's NAV entry clears the held marker only once it
      succeeds; one that raises is retried at the next decision, unless the
      operator selected a mode meanwhile.
   4. The held episode's dock contact ends as aborted for the hold
      (`HELD_REASON`, `FinalApproachDockContact.disarm_held`), never from
      the terminal latched while fenced, so the sim record never shows it
      as missed.

   Codex round 2 on the step 9 fixes, fixed with one resume grant instead of
   more flags (`RecoveryHold.resumed`):
   1. Recovery issues the grant when the hold ends in NavPy's own AUTO and
      voids it in any new hold or when a fresh heartbeat shows a mode other
      than AUTO or GUIDED. The decision uses it only when a held episode
      exists, and consumes it only once the fresh episode's NAV entry
      succeeded; a stale grant with nothing held is discarded.
   2. Abandonment keeps the held marker until NAV exits
      (`NavTransition.exit` clears it), so a failed cleanup abandons again
      next cycle instead of acquiring GUIDED over the operator's mode.
   3. One hold budget per incident (`nav_recovery_incident.py`; Demo's
      review of 114dde16): the RTL budget is the time spent holding (HOLD
      and RESUMING) summed across an incident's holds, in aircraft time. An
      incident ends when NavPy has run healthy in NORMAL for
      `NAV_INCIDENT_CLEAR_S` (30 s, the hold budget). A single hold reaches
      RTL at 30 s as before; two short unrelated stalls (the 1.7 s host
      stall) cost only their own hold time, so no early RTL. A failed fresh
      entry fails its cycle, which recovery counts. Bound: each hold adds at
      least `NAV_HOLD_TRIGGER_S` before it can resume, so an entry that
      always raises reaches RTL after at most budget / trigger = 20 holds,
      within 2 x `NAV_HOLD_RECOVERY_S` = 60 s of aircraft time after the
      first hold.
   4. A fresh entry that raises rolls back what it completed first: the
      partial episode ends as held (`HeldApproach.start_fresh_episode`), so
      the retry arms again only after a matching end.
