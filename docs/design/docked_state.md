# DOCKED final state (v5, 2026-10-17, owner-approved)

Owner (2026-10-17): "the final state should be docked". v5 (owner, after the SITL check): decide it from the contact
sensor's closest pass (`DOCKED` row `closest_3d_m`), not the SNAP line. SITL: contact 0.05 / 0.07 / 0.01 m, SNAP
0.0 / 1.9 / 4.4 m, because SNAP is sampled only at command rows and the contact latch stops the commands.
Owner (2026-10-17, on v3): cyclic test → after dock climb, then AUTO; single cycle, like a real dock → disarm so the
net is not on the motor, and DOCKED is final. Builds on `physical_docking.md` (contact sensor, episode latch). Simulator only. v2 answers Codex v1 findings 1-8; v3 answers round-3 findings 1-4 and rounds 4-5 cleanup ownership (DOCKED only replaces RESET); v4 applies the owner's answers; v4.1 the round-7 latch lifecycle (Codex: approve after it; re-checked at code review).

## Today
- Contact sensor → episode latch → nav loop requests RESET (`final_approach_nav_completed`).
- NAV exit (`nav_transition.py`) runs cleanup (contact disarm, source close, navigation reset → SNAP) and reports SNAP.
  One-shot force-disarms SITL only when `final_approach_nav_completed`, then redirects to ONHOLD.
- After that: disarm → `ONHOLD: DISARMED`; normal mode RESET → DETECT → AUTO restart; with no POI the
  confirmation dispatch requests DETECT; `publish_approaching(False)` makes the swarm slot FREE again.
- So no state says "docked", and a docked UAV can take a new task or fly the mission again.

## Decision source
The sim contact sensor (`physical_docking.md`) checks every telemetry segment against the true own dock and publishes
one event only when the closest pass, plus its bow margin, is inside the 0.5 m net. The episode latch accepts it only
for the armed (vehicle, task, dock, episode). SNAP stays a diagnostic.

## Entry
Add `NavState.DOCKED = 6`. Only NAV exit enters it, on the navigation loop thread, after the cleanup fence, and only
when the requested state is RESET (the NAV decision's own endings: contact pass, visual pass, `NAV_FAIL`). An exit by
the inactive guard (disarm, manual mode) or GUIDED departure keeps today's flow: fail-closed, and the operator wins.
The exit hook gets `requested` from the handler.
- **Decision:** `FinalApproachDockContact` keeps the latched event of the armed episode; latch and keep run under one
  lock, so NAV exit sees both or neither. `disarm()` returns it before the reset and `NavCleanupResult.own_dock_contact`
  carries it. DOCKED when it is set. A refused latch, another episode or a new arm leaves it unset. The real build has
  no sensor, so it never docks. Core imports nothing from sim.
- **Precedence:** docked wins over every other NAV ending (contact pass, visual pass, `NAV_FAIL`, loss). The redirect
  is DOCKED. In one-shot it also runs `OneShotCompletion` (force-disarm) even when `final_approach_nav_completed` is
  false, and sets `oneshot_completed`. A cleanup error keeps today's behavior (raise, no DOCKED).
- **Pure Vision:** classified after commands are fenced; never read by frames, the law or the command path. Guard tests
  keep contact values to their readers (now with `nav_transition`) and `own_dock_contact` off the command path.

## In DOCKED
DOCKED is always entered and committed (status text, swarm BUSY). What follows depends on the mode (`is_oneshot`):

| Mode | After DOCKED |
|---|---|
| Single cycle (one-shot; how a real dock behaves) | Force-disarm on entry (`OneShotCompletion`, today's path), so the net is not on the motor. DOCKED is final for the process: one short-circuit at the top of `NavigationDecision.decide()` returns before every other decision (inactive guard, recovery, NAV, confirmation dispatch), so the disarm never turns it into `ONHOLD: DISARMED` |
| Cyclic (sim restart test) | The next decision leaves DOCKED. The inactive guard runs first (operator wins). Then climb, then AUTO: `RECOVERY` if altitude < `min_alt - ALT_HYST`, else DETECT; RECOVERY climbs to `min_alt + ALT_HYST`, then DETECT. The DOCKED/RECOVERY → DETECT pair hook restarts the AUTO mission. Today `RecoveryAction` skips the climb whenever a final approach algorithm is active (`recovery.py:51`, always true in vision builds), so the DOCKED → RECOVERY pair hook sets a nav-thread latch `post_dock_climb`, and the RECOVERY exit hook clears it on any exit (DETECT, or an interruption: disarm, manual mode, waypoint hold). While it is set, RECOVERY uses the existing altitude branch until `min_alt + ALT_HYST`. Any other RECOVERY keeps today's behavior. The altitude read is after the approach ended, never in its command path (Pure Vision holds; GPS/altitude ok outside the approach, owner 2026-10-07) |

- **Entry cleanup, exactly once:** DOCKED replaces a RESET entry, so it takes over its cleanup owner. Cyclic: the DOCKED
  entry runs the RESET transaction (`NavigationTaskResetTransaction.clear`: confirmation, peer task, DDH, retries).
  One-shot: `OneShotCompletion.run` already ran it, so the entry skips it. Detector stop and navigation pause always run.
- **Action while DOCKED:** navigation pause (no commands). No mode request.
- **Swarm:** the per-cycle publish becomes one call, `publish_participation(approaching, docked)`, applied under the
  actor lock in one update, so NAV → DOCKED has no FREE window. Docked makes the slot report BUSY, reject offers with
  "docked", and drop a WAITING task with its token, so a late ACK finds no match. Cyclic clears it when DOCKED ends.
  `is_approaching` keeps its meaning. Wire format unchanged.
- **Real build:** never enters DOCKED (no sim port). A real net-capture signal is later work; it will be final, like
  one-shot.

## GCS
Contact already sends STATUSTEXT `DOCKED: task=… dock=… episode=… 3d=…` (contact evidence). The state entry sends a
different one: `STATE DOCKED: contact 3d=0.07m dock=… task=…`. The GCS shows both in its status-text feed.
No new UI in this step; a vehicle-card badge is a follow-up.

## Boundary note for e6 (no eval edits here)
Score docking from the `DOCKED` row `closest_3d_m`, as the state does. The SNAP row prints 1 decimal and misses the
crossing segment.

## Steps
| Step | Change | Gate |
|---|---|---|
| s1 | State, maps, short-circuit, entry cleanup, NAV exit redirect from the kept contact, swarm docked flag, cyclic climb | own-dock contact → DOCKED whatever the SNAP; no contact (close SNAP), refused latch, stale episode, re-arm → old flow; contact after `NAV_FAIL` through the real dispatcher; one-shot disarm with completed false; one-shot: disarm sent, DOCKED held in every mode, disarmed, with no POI and with a pending peer task; cyclic: DOCKED committed once, then RECOVERY (low, climbs even with final approach active, latch cleared on every RECOVERY exit; interruption then re-arm/mode restore and a new task do not climb) or DETECT (high) with AUTO restart, inactive guard first; a min-alt RECOVERY without the latch keeps today's behavior; offer rejected while DOCKED; real build never DOCKED; Pure Vision guard; sequencing: offer arriving during entry cleanup, late ACK after docked, delayed/rejected one-shot disarm keeps DOCKED, NAV → DOCKED has no FREE slot state; reset clear runs once on every path (normal and one-shot × pass, `NAV_FAIL`); disarmed, manual-mode and GUIDED-departure NAV exits with a contact keep today's flow; no mode request in DOCKED; full suite |
| s2 | SIM check | One `swarm_run.py --eval` 3-UAV one-shot run: each UAV with a `DOCKED` contact row ends in DOCKED, disarmed. One cyclic SITL run: DOCKED, climb, AUTO restart |
| s3 | `physical_docking.md`: the contact sensor stays the only success source; the state follows it | Done with s1 |
