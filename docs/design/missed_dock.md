# Missed final approach ends as "dock failed" (v3.1, 2026-10-18, Codex- and owner-approved)

Owner ruling 2026-10-18 (todo 74): a missed final approach is a failed dock. The UAV must not RESET, restart its
mission or search. It ends with the same behaviour as DOCKED but reports "dock failed" (status, log, GCS); the cause
is investigated later from the logs. The approach is over once it stops closing (it passed the closest point):
contact means docked, no contact means dock failed. Builds on `docked_state.md` and `physical_docking.md`.
v2 answered Codex v1 (rework). v3 answers Codex v2 (rework): it drops the late-contact window (findings 3-6), makes a
NAV_FAIL end commands (2), narrows the scope (7), and puts the stop-point reading to the owner (1). v3.1 applies the round-3 fixes 8-11
(Codex: approve after them).

## Today (evidence: `.logs/2026-10-18/155544/uav_2_navigation.log`, worktree amazing-mestorf-6b8cc3)
- UAV 2 lost the dock at 6.0 m with LOS (12.9, -7.2) deg (line 204), logged `NAV_FAIL: no_detection` 17 ms later,
  and the next log row is 0.7 m with LOS 171 deg (dock aft). SNAP 3d=0.7 m. Then `ONHOLD: WAYPOINT 1` and a second
  approach that docked (0.16 m).
- Mechanism: a NAV_FAIL latches FAILED (`episode_latch.py`), which wins over a later contact; `NavStateDecision`
  requests RESET (`navigation_decision.py:158`); NAV exit disarms the sensor; RESET restarts AUTO at waypoint 1.

## What vision can and cannot say
Vision has no range or range rate (Pure Vision), so it cannot prove the closest point was passed. An aft dock
(`missed_dock`) is the only visual sign of a pass. A loss (`no_detection`) or a command failure only says the
approach can no longer be flown; it may come before the pass (owner question 1). Nothing here reads range, GPS or
truth to time the end.

## Rules
1. **Scope (simulation, this step).** A vision final-approach episode armed with the sim contact sensor, whose GUIDED
   session was observed
   (`final_approach_navigation_active`) and which ends by a NAV_FAIL (the NAV decision's RESET request) ends as a dock:
   contact kept → DOCKED (today), otherwise DOCK FAILED with the NAV_FAIL reason. Everything else keeps today's flow:
   GUIDED not accepted (→ DETECT), operator exits (disarm, manual mode, GUIDED departure: the inactive guard first),
   builds without the vision runtime, and the real build (no sensor, so no episode is armed today). The real build
   follows when a real capture signal exists; it never docks today either.
2. **The end is immediate.** NAV exit redirects that RESET to DOCKED, as it already does for a contact. No waiting
   state, no new timer.
3. **A NAV_FAIL ends commands, atomically.** `mark_failed` takes the reason; the episode latch records FAILED with
   it (an earlier MISSED stays the terminal, reason `missed_dock`; first latch wins), and FAILED of an armed episode
   joins `commands_ended`, so `terminal_result()` suppresses any frame admitted after it. Unarmed (the real build) it
   does not: RESET retires the work as today. Under the same runtime lock held
   and queued work is retired (as `retire_after_docking`). Today RESET does that a tick later.
4. **Outcome is decided once, at NAV exit.** Cleanup snapshots the episode terminal (identity, terminal, reason;
   `FinalApproachDockOutcome`, its own NAV-exit port)
   before `navigation_reset` clears the latch, into `NavCleanupResult.dock_terminal`, next to the kept contact
   (`own_dock_contact`). Contact kept → DOCKED; FAILED/MISSED terminal → DOCK FAILED.
   First latch still wins (`physical_docking.md` unchanged): a contact after the NAV_FAIL is not counted. Hit or miss
   stays the sensor's. Lock order stays contact lock → runtime lock; nothing takes them in reverse.
5. **Behaviour is DOCKED's.** The DOCKED entry, hold, one-shot force-disarm (SITL only, `nav_oneshot_completion.py`)
   and short-circuit (final), cyclic climb
   then AUTO restart (the sim restart test, as for DOCKED), swarm BUSY. `NavState` gains no value. As built, no
   `dock_outcome` field: nothing reads it, so NAV exit reports the outcome once (status line and row) instead.

## Reporting
- Status text (GCS feed) and log, once the exit succeeded: `STATE DOCK_FAILED: reason=no_detection dock=0 task=1`.
  DOCKED keeps `STATE DOCKED: contact 3d=…`. The SNAP line before it is a command-sampled diagnostic, not the
  closest physical distance.
- Evidence row `DOCK_FAILED` (vehicle, task, dock, episode, reason). As built it is written at NAV exit, after
  `navigation_reset` drained the episode's command evidence, so it follows every command row and the SNAP row.
- The delivery eval: a role whose episode has a `DOCK_FAILED` row is reported "dock failed: <reason>" and fails;
  it is matched to its `DOCK_ARMED` by identity, not by command rows (a failure may come before any command).
  Rejected: duplicate, stale-episode or wrong vehicle/task/dock rows, a `DOCKED` and a `DOCK_FAILED` for one episode,
  and any command row after the terminal row.

## Forced miss for the check
A sim-only net radius override `NAVPY_SIM_DOCK_NET_RADIUS_M` (default 0.5, owner-set): read by the sim composition,
logged at start, passed by the eval through its environment allow-list (`eval_gcs_demo_config.py`) and recorded in
its report. `0` makes every pass a miss.

## Not in this step
- Counting a contact that comes after a NAV_FAIL (owner question 2).
- Handing a failed dock's task to a peer (the slot stays BUSY). Logged as a todo.
- A GCS vehicle-card badge (the `docked_state.md` follow-up). The delivery product move (`delivery_product.md`).

## Steps
| Step | Change | Gate |
|---|---|---|
| m1 | FAILED with reason ends commands; NAV exit redirect and outcome; status line and row; eval parse/report/conflict; radius override | Unit: each NAV_FAIL reason in scope → DOCKED state, outcome FAILED + reason, no AUTO restart, no new confirmation, status line once; out-of-scope endings → today; a frame admitted after the NAV_FAIL and before exit → suppressed, no command row; held and queued work retired; contact before the NAV_FAIL → DOCKED; contact after it → still FAILED; real build → FAILED; one-shot: disarmed and final; cyclic: climb then AUTO, outcome cleared; outcome survives the RESET clear; cleanup error → today's raise; terminal snapshot survives the reset; MISSED then NAV_FAIL keeps `missed_dock`; repeated NAV_FAIL; contact/failure/exit interleavings; Pure Vision guard; eval: forced-miss report, zero-command failure, duplicate/stale/wrong-identity/conflicting rows, command after terminal; full suite |
| m2 | SITL | One delivery wall eval (quiet PC) green. One forced-miss run (radius 0): every UAV logs `STATE DOCK_FAILED`, no `ONHOLD: WAYPOINT 1` or new confirmation after it, disarmed in one-shot |

## Owner questions
1. **Stop point.** Vision cannot measure closing. Recommended: every NAV_FAIL of a started approach (aft, loss,
   stale or rejected command) is the end, reported as dock failed with its reason, so the UAV never returns to search.
   Alternative: only an aft dock and a loss end it this way, and software failures keep RESET.
2. **Late contact.** A contact after the NAV_FAIL (e.g. lost at 6 m, then hits the net) still counts as failed, as
   today. Counting it needs a bounded evidence window after exit; Codex found that adds threads, clocks and races,
   so it is left out (recommended). Accept?
3. **Forced miss** by the sim-only radius override (recommended), or another way?
4. **Risk:** today a miss gets a second try that often docks (the evidence UAV docked on its second approach). After
   this a miss is final, so wall runs will show more red. That follows the ruling; confirm.

Owner (2026-10-18): approved v3.1 with the recommended answers: every NAV_FAIL of a started approach is a failed
dock; a late contact is not counted; forced miss by the sim radius override; more red wall runs accepted.
