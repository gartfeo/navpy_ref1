# Delivery 3-UAV eval: the owner's scenario (v2, approved 2026-10-13)

**Status.** Approved by the owner on 2026-10-13 ("go and fix"). Q1: the other chat ("Deterministic Run") is still
active, so its eval work lands first and e1 builds on it; e2, e2b and e3 start meanwhile. Q2 and Q3 as recommended.
**Side by side (owner, 2026-10-13).** This scenario is a second eval beside the other chat's rectangle eval, which
keeps its docks on waypoints for the determinism work. That rectangle (1.7 x 0.8 km, lines 187 m apart) is too small
for real search. e1 landed as the other chat's step 1 (`fa9ea47`, mission upload with exact readback); e3 is
`37cb1e5`. Line numbers are on main `779694e`. UNVERIFIED marks claims not
checked by a test or a run. Codex reviewed v1 ("rework", 8 findings); v2 applies all 8 (see the end).

## Goal
The 3-UAV GCS eval (`scripts/eval_gcs_navigation_demo.py`) tests the delivery product as the owner defines it:
- the operator draws one delivery area, and the GCS planner splits it into three search routes;
- the UAVs search and find delivery docks;
- a UAV that finds two docks keeps one and offers the other; another UAV wins it;
- a UAV that finds nothing flies to its home dock (the DDH in its mission) and parks there;
- every approach waits for one operator approval, given after the UAV sees the dock;
- a never-seen home dock means the UAV keeps orbiting. A mechanism test covers it: the orbit holds, with no
  confirmation and no final-approach command.

The current eval does not test this. It puts the docks on the owner's NAV waypoint ordinals 2, 3 and 6
(`eval_gcs_demo_scenario.json:19`), and it never tests search, the home dock or the no-dock case.

## The scenario
Two delivery docks, d1 and d2, both on the middle line of UAV A's route, with d2 150 m after d1. At about 260 m
the view band is some 340 m deep, so both should be in view together and A should offer d2 as soon as it picks d1.
This is an assumption until the e2b SITL slice shows it.

| UAV | Sees | Does |
|---|---|---|
| A | d1, then d2 while confirming d1 | Approval, approach d1; offers d2 |
| B or C (the winner) | nothing on its route | Bids, wins d2 and stops its search; approval, approach d2 |
| The other one | nothing on its route | End of route, home-dock orbit, sees its home dock, approval, approach |

Either of B and C may win d2: the swarm picks by reported ETA, not by zone (`task_assignment_planner.py:38`). The
other one goes home. The checks are by role (offerer, winner, home), never by sysid.

Two docks, not three: a UAV flies one approach per flight. With a third dock on B's route, C would win d2 and nothing
would go home.

## Inputs: one fixture made by the GCS's own planner
- There is no Python planner. `planner.js:1-3` is the only copy (its Python source is gone).
- A Node script runs `generatePlan` (`planner.js:253-300`) and `autoAssignDeliveryHubs` (`deliveryHubAssignment.js`)
  on the fixed inputs: polygon, launch point, three home docks, altitude, spacing, d1 and d2.
- It writes `scripts/eval_delivery_scenario.json` with the inputs, the resolved settings (catalog, preset, spacing
  override, each zone's altitude, since the planner adds a step per zone, `planner.js:275`), the three routes, each
  UAV's home dock and the expected outcome per role.
- It follows the existing Python→Node test pattern.
- A vitest test reruns the script and compares byte for byte, so a planner change shows up as a diff. The eval reads
  only the JSON and needs no Node at run time.
- The eval uploads the three missions through `POST /api/missions/upload` (`routes/missions.py:144`) and checks the
  readback exactly. Today it relies on template eeprom missions instead.
- Home-dock coordinates are chosen to encode exactly at 1e-7°. The planner truncates them to int (todo 26).
- **As built (e2):** `scripts/eval_delivery_scenario_gen.py` and `scripts/eval_delivery_scenario.json`. The area is
  5.3 x 3 km, centred due south of the launch point, because the GCS passes the launch point as the approach point
  and the planner aims the lines at it. Result: 3 equal routes of 3 north-south lines, 500 m apart, at 260, 240 and
  220 m above home, each with its own home dock 1.6 km north of the area. Points move at most 1.5 encoding quanta.
  The fixture records the runtime settings the oracle assumes: detection from track row 1, one-shot, search zoom 1.

## Sim docks: one shared world
- Today each NavPy builds its own dock list from its own mission waypoints (`navigation_poi_args.py:42-56`,
  `poi_provider.py:194-196`).
- It also adds docks where it expects them: at a peer task's estimated position (`peer_approach_start.py:31-35`) and at
  the hub (`fallback_mission_navigation.py:63-68`).
- In a shared world those added docks would be ghosts, offset by the estimate error. A UAV could then see two docks
  and offer one.
- **New:** `sim/world/fleet_dock_world.py`, loaded from a file named by `NAVPY_SIM_DOCKS`.
  - The eval writes that file (d1, d2 and the three home docks) into the run dir. All three NavPy processes load it,
    so dock ids match across UAVs.
  - This world ignores `set_sim_poi`.
  - It is picked at startup (`sim_render_composition.py:75`) as a second world implementation, not a shim. Without
    the variable, today's world is unchanged.
- Docks have stable ids, an absolute altitude from the terrain at their position, and no refresh. A missing or bad
  file stops the run before launch.
- World truth stays in the sensor simulator and the scoring. The command path's yaw-free boundary is unchanged
  (`frame_projection.py:49`).
- In the sim, offers carry the truth position (`peer_poi_notification.py:43`), so the winner's 1 m join holds.
- This is sim only. The one product change is e5 (a home dock must never be offered).

## Geometry (UNVERIFIED arithmetic; an offline test proves it)
- The ZR10 sees the 3.5 × 2.5 m dock out to about 829 m (8 px plus the 1.1 margin; `vision_class_profile.py:15,168-173`).
- That is the detection limit. Tracking needs a full 8 px box, so it holds only to about 754 m
  (`finite_poi_projector.py:112`).
- The camera mount is -14°. At 240 m above ground in level flight, a dock is in view 451-794 m ahead and about
  356 m left to 328 m right.
- **At the planner's 363 m preset almost nothing is seen** (a band of 682-745 m ahead), and at 403 m nothing at all.
  The fixture therefore flies at 240 m (owner question 2).
- Every other UAV's route stays at least 980 m from d1 and d2 (829 m plus a 150 m margin). Home docks lie outside
  the area, at least 980 m from every route.
- An offline oracle samples every route every 10 m. It is distance only: a dock farther than the 829 m zoom-1
  limit (plus a 150 m margin) from every point a route flies after its detection waypoint cannot be seen from that
  route. Slant range is never shorter than ground distance, so terrain and bank do not weaken this negative proof.
- It cannot prove that A sees and offers d2. A changes course once it picks d1 (confirmation orbit, approach). So a
  1-UAV SITL slice proves it first (e2b).

## What the eval checks
**Per UAV** (against the fixture's expected block):
- **Offerer (A):** detection, then review image, then approval, then selection, then commands, then SNAP, all for
  d1; exactly one logical offer, for d2 (protocol retransmissions allowed).
- **Winner:** its bid, the APPLIED assignment (the response correlation stays, `eval_gcs_demo_assignment_audit.py:146`),
  "Peer navigation started" after the assignment (`eval_gcs_demo_gates.py:170-180`), the mode change, and its search
  ends; then one approval and SNAP bound to d2.
- **Home:** "Default delivery hub navigation:" for its own home dock, then evidence it reached the orbit, then the
  dock seen there; no offer and no assignment; one approval; SNAP bound to its home dock.

**Fleet:**
- Exactly 3 approvals, one per UAV.
- Every approach comes after its approval: the live check stays (`eval_gcs_demo_mission.py:154-159`), and a new
  offline check is added.
- Exactly one logical assignment, and no offer within one dock diagonal of a home dock (todo 29).
- Only world dock ids are detected.
- Review image quality and the 1 m joins stay.
- Synthetic-log tests cover the pass case and each failure: a missing offer, two winners, an approach before
  approval, a home-dock offer, and the winner still searching.

**Unchanged:** vision-only with the ZR10 mount, roll quality, command bounds, cadence, and the speedup audit. The SNAP
truth limit was replaced by DOCKED evidence (see Remaining plan).

The roles in `workflow_errors` change from owner/peer to offerer, winner and home (`gates.py:183-213`). The tests
change on purpose, and each commit message says why. No test is weakened.

## Why the eval is red today
1. **Main fails at setup** ("owner mission has no NAV waypoint ordinal 7"). A fix by mission upload exists only as
   uncommitted work in another chat's worktree (`eval-canonical-mission`, session `e5537386`), together with todos
   26-31 (owner question 1).
2. **A home dock was offered as a task** (runs `step1-wall-a/b`). UAV 2's two-row route ended early, so it reached the
   hub, saw the hub dock and offered it to UAV 1. UAV 1 dropped its own dock, saw it again and asked a second time,
   which the eval refuses (`eval_gcs_demo_mission.py:146-151`).
   - The new geometry avoids this: equal routes and home docks out of view.
   - The new gate catches it. The product fix (a home dock must never become a swarm task, todo 29) stays a separate
     task.
3. **Both peers fail right after the approach starts** (runs `demo-rect-a/b`): `NAV_FAIL: source_command_stale`
   0.3-1 s in.
   - The freshness limit is 1.0 sim s, which is 0.1 wall s at 10× (`command_freshness.py:12,40-47`). The logs show a
     103 ms gap.
   - There are two clocks to check: producer freshness and receipt freshness (`command_freshness.py:29`). Host jitter
     is likely but UNVERIFIED. First measure both from the two runs' source-time CSVs. Lower the eval speedup only if
     the cause is host jitter (owner question 3); the freshness rule stays.
4. **The owner lost its track while waiting for approval and went RTL** (`demo-rect-a`, UAV 1 log lines 1274 and
   4149-4172). The cause is UNVERIFIED. It needs a mechanism test before a fix, because A takes the same path here.
5. **The 0.5 m SNAP truth limit** is a known blocker: the navigation eval misses by 1.3-4.3 m at 10× (todo 22). The
   limit stays as it is, so e9 needs its own diagnosis and fix before green is possible.

## Commits (each with tests; pushed ff-only after both suites, guards and a Codex review)
| # | What | Size |
|---|---|---|
| e1 | Mission upload and readback, plus todos 26-31 (taken over from, or landed by, the other chat) | S |
| e2 | Node fixture generator, the committed fixture, the byte test, the Python visibility oracle | M |
| e3 | `FleetDockWorld`, the env var and its wiring. Tests: same ids across loads, `set_sim_poi` ignored, the projector sees world docks, missing-file refusal, the layer guard | M |
| e2b | 1-UAV SITL slice: A alone sees d1, then d2, and offers d2 (no peer, so the offer goes unanswered). Proves the geometry before the 3-UAV wiring | S |
| e4 | The resolver and eval wiring for the new fixture, in new modules (`eval_gcs_demo_scenario.py` is at 299 lines and `eval_gcs_demo_mission.py` at 297, against the 300 limit) | M |
| e5 | Product fix for todo 29: a home dock is never offered to the swarm. Mechanism test, plus the never-seen-home orbit test | S |
| e6 | Gates, analysis, assignment audit and POI binding for the new roles; the synthetic-log tests | M |
| e7 | The stale-command cause, after measuring both clocks | S-M |
| e8 | The track loss during approval: root cause plus a mechanism test | S-M |
| e9 | The SNAP miss above 0.5 m (todo 22): root cause and fix | M-L |
| e10 | Two wall runs, then a live check by the live-tester agent; the report is committed unedited | S |

## Remaining plan (2026-10-16, v2, Codex and owner approved)
**Where it stands.** On main: e1 (`fa9ea47`), e2 (`2bc1289`), e3 (`37cb1e5`). Later work changed three items:
- e7 (stale commands): bounded attitude refresh (`1313d58`), reader batching (`099a690`) and synced swarm clocks
  (`58af9de`) address the known causes. This is not proven; the runs re-check it.
- e8 (track loss during approval): fixed 2026-10-13 (todo 31, runs `self-orbit-c`, `-d`).
- e9 (SNAP 0.5 m): docking is now decided by the sim contact sensor (`3a401df`, `physical_docking.md`), so the SNAP
  limit above no longer gates. **Valid DOCKED evidence:** a DOCKED row matched to its own DOCK_ARMED episode
  (`eval_gcs_demo_docked.py`), naming the role's expected world dock id (d1, d2 or that UAV's home dock), and
  after that dock's approval.

Todo 29 (a home dock offered as a task) is still open.

**Known blockers.** Todo 31's `no_detection` and `missed_dock`, and pitch divergence, still occur in recent wall
runs. Other chats work on them (sparse observations, LOS-rate spikes). e10 starts after their fixes land. Any
failure that remains in e10 is a blocker owned by this plan.

One step at a time, each with tests, a Codex review, and an ff push to main:
1. **e2b, 1-UAV slice.** A flies its fixture route alone. Done when, in one wall run:
   - A selects d1 (stable world id) and asks for its approval;
   - during that approval orbit, it offers d2 (by world id; the offer goes unanswered).
   If d2 is not seen, move d2 in the generator and re-run the oracle.
2. **e5, product fix for todo 29.**
   - Rule: during the home-dock fallback, a detection is the UAV's own home dock only if it lies within the dock
     association radius of that home dock's position. Using that position is allowed: it is the DDH exemption.
   - The home dock is never offered. Any other unassigned dock seen then is handled per ruling 4.
   - Mechanism tests:
     - the home dock is not offered;
     - with an unassigned delivery dock in view on the way home, the UAV asks for it (ruling 4);
     - a never-seen home dock keeps the orbit, with no confirmation and no command;
     - the winner passing d1 neither locks d1 nor offers it again (ruling 5, the Risks item).
3. **e4, eval wiring.** `eval_gcs_navigation_demo.py --scenario delivery` uploads the fixture missions, writes
   `NAVPY_SIM_DOCKS` and resolves roles. It lives in new modules, because the mission and scenario modules are at
   their 300-line limit. The rectangle eval stays as it is.
4. **e6, role gates.** Offerer, winner and home checks as in "What the eval checks", with the DOCKED evidence above in
   place of SNAP. Synthetic-log tests cover the pass case and each failure listed there, plus a DOCKED row for the
   wrong dock id.
5. **e10, green runs.**
   - Two 3-UAV wall runs, in a quiet window agreed with the Deterministic Run chat.
   - Then a live check of the GCS view by the live-tester agent; the report is committed unedited.
   - Each failure becomes its own root-caused bug, fixed before re-running.
   - Done when both runs pass every gate, with all three UAVs DOCKED at their expected docks in sim. That is not
     physical docking.

Not in this plan: a lockstep version of this scenario (after step 11, if the owner wants it); todo 54.

**Owner rulings (2026-10-16).**
4. A UAV on its way home that sees a delivery dock nobody took treats it as a dock found in its own zone: it asks
   for confirmation and, if approved, approaches it instead of going home.
5. A UAV with an assigned dock goes to that dock only. It does not lock or offer any other dock on the way.

e5 builds both, each with a mechanism test.

**e2b as built (2026-10-16, sim only).**
- `scripts/eval_delivery_slice.py` flies the offerer's zone alone. It uses the wall eval's launch path: the isolated
  launcher, mission upload with readback, and `targ_wps=0`. It writes the fixture's `docks` block as the run's
  `NAVPY_SIM_DOCKS` file and never approves. Before launch it checks that the fixture's `runtime` block equals the
  applied settings.
- The offer message carries no obj_id, so the first offer of a task now logs
  `Task offered: task_id=<t> obj_id=<o> at (<lat>, <lon>) sent=<result>` (`task_rebroadcast.py`). This is log text
  only. A detection's world id is `docks[obj_id]`.
- `scripts/eval_delivery_slice_gates.py` checks the full log after the stack stops. It passes when:
  - the first selection is d1, with its self-detect orbit directly before it;
  - `CONFIRMING` follows inside that orbit;
  - d2's first offer is sent inside that orbit, which runs from the d1 selection to the first answer, abort, new
    orbit or new selection, so before or after `CONFIRMING`;
  - nothing answers: no approval and no assignment traffic.
- Codex reviewed the design ("ship with changes", 6 points), and all were applied: the obj_id link in place of a
  position match, first offer vs. rebroadcast, orbit evidence, fixture runtime check, full-log "unanswered", and other
  offers kept as diagnostics.
- Run `e2b-1` (`.logs/e2b-1`):
  - d1 selected (obj_id 0) at 17:12:21.711, after its orbit command at .693;
  - d2 offered (obj_id 1, at d2's file position, sent) at .833;
  - `CONFIRMING` at .950, and the window ended at the 30 s confirm wait;
  - no answer.
- The gate first opened the window at `CONFIRMING`, so it failed this run by 117 ms (`slice_report.json`). The plan's
  criterion is "during that approval orbit", and the geometry note expects the offer "as soon as it picks d1". So
  the window now starts at the d1 selection, and the same log passes (`slice_report_regated.json`). Codex judged
  this a correction, not a goalpost move.
- Before the selection every frame holds one POI (`pois=1`): d1 (P0) just before it, and from 17:11:55, near launch,
  another POI labelled P3 that was never selected or offered. Frames with two POIs (`pois=2`) run from 17:12:21.790 to
  .865, inside the orbit. These are detection debug lines, not a per-frame identity check. The run does not show
  whether d2 stays in view later in the orbit, and the offer is sent only once.
- Codex's result review was "ship with changes" (bind the orbit to the selection, no offer before the selection, no
  geometry overclaim). All three were applied, and the re-review said "ship".

**e5 as built (2026-10-17, unit level).**
- `DockRoles` (`src/navpy/core/skills/search/dock_roles.py`) feeds `PoiSelector`, which every offer path uses
  (DETECT, CONFIRM, NAV).
- A detection is the home dock, or the assigned task's dock, when its position lies within
  `DOCK_ASSOCIATION_RADIUS_M` (30 m, the reacquire gate). The position is the one an offer would carry: the
  projected position, else sim truth, else the ground estimate.
- The home dock is never offered. On the way home, delivery docks come before the home dock, and a delivery dock
  taken there gets its own waiting circle (Codex round 1).
- While a peer task is pending or flown, nothing is offered and only a detection at the task's position is taken.
- The limits are todo 56: hardware estimate error, other UAVs' home docks, no de-duplication by position.

**e4 as built (2026-10-17, wiring only).**
- `eval_gcs_navigation_demo.py --scenario delivery` loads the fixture (`scripts/eval_delivery_fixture.py`): three
  slots in zone order, the offerer's zone as owner, no POI ordinals, gate profile `delivery` (wall only, no lockstep).
- Before launch it checks the fixture's `runtime` block for every slot, refuses a run dir that already holds NavPy
  logs, and writes the `docks` block as the run's `NAVPY_SIM_DOCKS` file; the env reaches every NavPy process.
- The mission runner is unchanged: with no ordinals it approves requests until every UAV has a SNAP.
- `scripts/eval_delivery_roles.py` names the roles from a run's logs: the offerer by zone, the winner as the other
  UAV with a peer task, home as the other UAV with a home-dock task and no peer task. Either peer may win. Nothing
  calls it yet: e6's analysis does.
- The analysis refuses the `delivery` profile until e6 adds its gates, so a delivery run ends with that refusal.

## Risks
- The winner flies past d1 on its way to d2 (150 m apart). Peer acquisition stays in DETECT, and `DetectAction` and
  `ConfirmationAction` still select and offer other docks there, so two things can go wrong: it may lock d1 instead
  of d2, and it may offer d1 as a new task (the swarm does not de-duplicate by position). e5 adds a mechanism test
  for each (and a product fix if one fails); the 3-UAV run checks both.
- Wall-time timing can change who wins the auction. The geometry keeps B nearest by a wide margin, and the check is by
  role.
- A planner change forces the fixture to be regenerated. That is intended.
- The open approach failures (Known blockers in the Remaining plan) are the largest unknown.
- Simulation shows neither physical docking nor cargo receipt.

## Owner questions (recommended answer in bold)
1. **The other chat's eval work** (uncommitted on `eval-canonical-mission`). **I take it over into e1 if that chat is
   finished.** Otherwise it lands first and I build on it.
2. **The planner's 363 m preset cannot see docks with the ZR10.** **The fixture flies at 240 m, and the product preset
   is logged as a todo.**
3. **10× leaves about 100 ms of wall time per command.** **If the gaps prove it is host jitter, lower the eval
   speedup.**

## Codex review of v1 (rework) and how v2 answers it
1. Offering d2 can't be proven from straight routes: added the e2b SITL slice.
2. The winner is chosen by ETA, not zone: either non-A UAV may win, checked by role, with search-end evidence.
3. Home-dock dispatch is not orbit arrival, and the home-dock offer bug was hidden by geometry: orbit-arrival evidence,
   the e5 product fix, and the never-seen-home test.
4. The shared world is sound, and offers use truth in the sim: the estimated-position risk is removed; ids, altitude
   and missing-file rules are specified.
5. Geometry corrected (451-794 m, asymmetric sides, detection vs tracking).
6. The fixture records the resolved settings and per-zone altitudes.
7. Gates are completed: APPLIED correlation, full approval-to-command identity, negative cases, retransmissions.
8. Known blockers are no longer optional: both freshness clocks (e7) and the SNAP miss (e9).
   Not taken: keeping the old waypoint eval beside the new one. The owner chose to replace it, and the gates are
   shared, so nothing is lost.
