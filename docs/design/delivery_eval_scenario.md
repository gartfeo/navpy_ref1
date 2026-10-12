# Delivery 3-UAV eval: the owner's scenario (v2, approved 2026-10-13)

**Status.** Approved by the owner on 2026-10-13 ("go and fix"). Q1: the other chat ("Deterministic Run") is still
active, so its eval work lands first and e1 builds on it; e2, e2b and e3 start meanwhile. Q2 and Q3 as recommended. Line numbers are on main `779694e`. UNVERIFIED marks claims not
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

The current eval does not test this. It puts the docks on the owner's waypoints 3, 4 and 7
(`eval_gcs_demo_scenario.json:9`), and it never tests search, the home dock or the no-dock case.

## The scenario
Two delivery docks, d1 and d2, both on one straight leg of UAV A's route, with d2 400-500 m after d1.

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
- An offline oracle samples every route every 10 m, with the calibrated projection, the terrain and a bank-angle
  band. It shows who **cannot** see which dock: it treats any point within 829 m of a turn as "maybe seen".
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

**Unchanged:** vision-only with the ZR10 mount, roll quality, command bounds, cadence, the SNAP truth limit of 0.5 m
(`eval_gcs_demo_limits.py:97`), and the speedup audit.

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

## Risks
- Wall-time timing can change who wins the auction. The geometry keeps B nearest by a wide margin, and the check is by
  role.
- A planner change forces the fixture to be regenerated. That is intended.
- e9 (the SNAP miss) is the largest unknown and may take longer than the rest.
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
