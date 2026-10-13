# Architecture follow-ups (TODO, logged 2026-10-08)

These tasks were suggested during the mission-module design review. None is started; we
return to them later. Paths are under `src/navpy/` (step-1 layout) unless noted. The design is in
`mission_modules.md` and `architecture_split.md` (same folder).

## Bugs and safety
1. **Altitude floor: off by default, independent of the approach law.**
   - Today `core/flow/recovery.py:49-51` skips altitude whenever the vision law is set, while the legacy laws use 150 m (`scripts/lua/aas_params.lua:16`).
   - The owner wants it off by default, with modules adding their own checks.
   - Also: the legacy RECOVERY climb sends attitude outside GUIDED (`core/flow/recovery.py:93-98`).
2. **Operator LOITER becomes a hold.**
   - Remove LOITER from `ACTIVE_MODES` (`core/flow/navigation_decision.py:42`). NavPy never sets LOITER mode.
   - Update the pinned test (`tests/modules/nav/test_nav_controller.py:508-515`) per the owner ruling.
3. **Airspeed.**
   - Without a real airspeed sensor, the vision law must not use airspeed: VFR_HUD airspeed can be GPS-synthesized (`plugins/vehicle/ardupilot/flight_telemetry.py:41-43`).
   - Detect a real sensor (ARSPD_* params, SYS_STATUS bits).
4. **Dead peers stall the auction.**
   - `core/swarm/task_actor_slots.py` never removes a peer, and the first assignment needs every peer (`core/swarm/task_auction_queries.py:35-50`).
   - Add heartbeat-timeout removal, and assign among reachable peers.
5. **Unknown task_type drops the whole message.**
   - Three decoders raise on unknown kinds (`core/link/task_*_msg.py`).
   - Fix the dialect text too: task_type 1-4 is documented as SMALL..HEAVY, but the code uses DOCK=1.
6. **Pure-vision guard gap.**
   - `plugins/laws/vision_pn/pitch_law.py` is missing from `tests/guards/test_frozen_inventories.py:18-35`.
   - Derive the scanned files from the law's imports.
7. **NavArgs ONHOLD refresh may never run** (`core/flow/navigation_decision.py:243-252`). If so, GCS param edits apply only at task reset. Prove it with a test, then fix.

10. **Late-deny window** (found by the flow-runner review, 2026-10-08; UNVERIFIED in flight).
    - On a late operator deny in NAV, `_teardown` clears the active POI before it publishes DETECT (`core/skills/confirm/poi_rejection_exit.py:98-101`).
    - In that gap the event-pump thread sees NAV with no POI. It marks NAV_FAIL, which is an eval-fatal marker (`core/approach/final_approach_source_event_handler.py:57-66`).
    - The latch then sends DETECT to RESET, so AUTO restarts at item 1 instead of resuming the leg.
11. **Stale failure latch.**
    - The final-approach failure latch is consumed in any state that reaches the NAV decision step (`core/flow/navigation_decision.py:139-152`), with no task id.
    - So a latch left over from an ended approach can send a later state to RESET.
12. **AUTO writes can override an operator mode change.** The restart and resume paths read the mode, then write AUTO directly, with no fence (`core/flow/recovery.py:89-91`, `core/flow/navigation_task_reset.py:249-250`).
13. **Coasted tracks may reach the law as measurements** (UNVERIFIED). Trackers stamp `time.time()` (`plugins/tracking/kalman/multi_object_tracker.py:124`), coast ticks republish (`core/perception/pipeline/real_tracking.py:176-178`), and the law differentiates bearings by capture time (`plugins/laws/vision_pn/lateral_rate.py:113-131`).
18. **Real runs may get a sim gimbal.** `use_sim_gimbal=True` is hardcoded (`app/perception/vision_profile_composition.py:48`), even with `--detector-type real`. Only `scripts/python/run_detector.py:81` builds the real SIYI gimbal. Check the intent first.
19. **Dead radio code.** The sim radio client can't be reached (`app/network_factory.py:23-28` never passes `simulation`), and the eByte radio code has no callers.

## Checks
8. **Delivery SITL eval.**
   - Run it and report whether the circle happens under the vision law. Its `Self-detect orbit:` gate matches the owner's flow: circle until confirmed, then approach.
   - Report only; delivery is paused.

## Vendor seams (so a contractor's module plugs in; 2026-10-08)
14. **Approach law.**
    - Add one `FinalApproachLaw` Protocol and a law-agnostic plan type; PN evidence moves into `VisionNavLaw`.
    - Select it through one registry. Today 5 places hardcode it (`app/approach/navigation_composition.py:120,206-210`, `navigation_law_builders.py:31-37`, `nav_law_factory.py:36-56`, `app/runtime_composition.py:131`).
    - Harnesses take a law factory.
15. **Detection.**
    - One backend registry; today 6 places check names (`core/kernel/config/detector_backend.py:16-24`, `core/perception/pipeline/real_detector_models.py:134-144`).
    - A class catalog instead of dock-only.
    - A contract test and a labelled replay kit.
16. **Tracking.**
    - One port over tracker, identity, lock and bridge (`core/perception/pipeline/real_tracking_batch.py:115-146`).
    - Pass capture time.
    - Run the tracker in the sim; today `sim/camera/finite_poi_projector.py:146` uses truth ids.
17. **Vendor code runs out of process**, and receives only the vision frame. That boundary is the pure-vision guard, because import scans can't check a binary.

## Found after step 1 (2026-10-12; all predate step 1)
20. **Done 2026-10-13 (step 2 of `deterministic_demo_eval.md`): the harness takes the firmware's names from `scripts/simtime_firmware_contract.py`, pinned by a test against the binary.** **Simtime evals can't run.** The harness and the lockstep firmware disagree on names.
    - The harness exports `NAVPY_NAVIGATION_MODE`, `_START_US` and `_COUNT`, and its peer listens on `navpy-navigation.sock` (`scripts/simtime_step_artifacts.py:52-58`).
    - The firmware reads `NAVPY_GUIDANCE_*` and connects to `navpy-guidance.sock` (`ArduPlane/NavPyGuidanceStep.cpp:88-104,126` in `/home/gart/navpy-simtime-step-handshake`).
    - Rebuild the firmware with the new names, or switch the harness back.
21. **Done 2026-10-13 (step 2).** **`scripts/eval_simtime_navigation.py` fails when run directly** ("No module named 'scripts'", from `scripts/eval_navigation_vehicle_config.py:12`) unless `PYTHONPATH` includes the repo root.
22. **The navigation eval misses its 0.5 m SNAP gate at speedup 10** (`scripts/eval_navigation_cases.py`). Lateral misses are 1.3-4.3 m, on `2c887d9` and `cc430e8` alike. Investigate, or set the gate from a measured baseline.
23. **4 `test_board_detector` tests fail** (IndexError at `tools/cam/calibration/board_detector.py:34`), since `9db0a26`.
24. **`tools/cam/calibration/camer_calibration3.py:39` `_collect_detections` has no annotations.** The SOLID re-pin to `e9b1591` stopped reporting it.
25. **`.sitl-runs/` is not in `.gitignore`**, so eval runs leave untracked files.

## Found while fixing the demo eval (2026-10-12; predate the fix)
26. **Mission upload truncates coordinates.** The planner encodes lat/lon as `int(degrees * 1e7)` (`src/gcs/backend/planner/mission_items.py:66-67,91-92,138-139,168-169,187-188`), so a download and re-upload moves some points 1e-7° (about 1 cm) south or west. Use `round()`. The demo eval's scenario stores only values that encode exactly.
27. **NavPy and the GCS number NAV waypoints differently.** NavPy counts the appended default-delivery-hub row (`src/navpy/plugins/vehicle/ardupilot/nav_waypoints.py:15-22`); the GCS parser drops it (`src/gcs/backend/mission_validator.py:106-111`). A POI ordinal past the track names the hub in NavPy and nothing in the GCS. The demo eval rejects such ordinals; check every other path that sets `AAS_TARG_WPS`.
28. **Slot 0 runs on the shared SITL templates** (`~/ardupilot/{1,2,3}`), so its param writes and mission uploads change what every other slot clones. Between 06:42Z and 08:52Z on 2026-10-12 the template missions changed from 7/5/5 to 6/2/6 track rows, which broke the demo eval. The eval now uploads its own missions; step 3 of `docs/design/deterministic_demo_eval.md` gives each run a fresh root.
29. **A UAV at the delivery hub advertises the hub as a new task.** In sim, the hub fallback places a dock at the hub (`src/navpy/core/flow/fallback_mission_navigation.py:64`) so the UAV can approach it. The UAV then detects that dock and offers it to the swarm; the owner flies it first and asks again to confirm its own POI. With the 6/2/6 missions, UAV 2 reaches the hub early, so the demo eval fails with "vehicle requested more than one confirmation round" (`.logs/step1-wall-a`, `-b`). Owner's intent: a UAV that finds no delivery dock flies to the home dock (DDH), acquires it by vision, asks confirmation and parks if confirmed; the home dock is never a swarm task. Behavior after a rejected home dock is undecided.
30. **The planner balances search paths only at whole-strip boundaries** (`assignStripsEqualDistance`, `src/gcs/frontend/src/utils/trackGenerator.js:86`). Requirement: every UAV gets an equal search path. On a diamond-shaped area with 7 strips (224/673/1121/1427/1123/674/225 m), today's demo missions got 2635/1430/2636 m tracks, so UAV 2 finished early and homed (see 29).
31. **Peer final approach fails with a stale vision source right after confirmation.** In the rectangle demo scenario both peers, assigned at their first waypoint, confirm their dock and then fail with `NAV_FAIL: source_command_stale` (`src/navpy/core/approach/final_approach_command_dispatch.py:105-114`) 0.3-1 s later, 575-615 m from the dock. The last detection lines show about 110 ms with no update before the failure. They then fall back to the hub (see 29) and ask a second confirmation, which aborts the eval. The owner also loses its track 2 s into its own confirmation ("return to search (lost 2.0s)"). Identical in two runs (`.logs/demo-rect-a`, `-b`).
   - **Owner track loss fixed 2026-10-13.** Two causes: under the vision law the own POI was not circled during review (it kept flying AUTO), and the sim SIYI gimbal stepped on wall slots and fell up to 7x behind the 10x SITL. Now it circles the camera's ground estimate and the gimbal steps in aircraft time; runs `.logs/self-orbit-c`, `-d` keep the dock through a 20 s wall review.
   - **Still open:** after approval all three final approaches fail near the dock (`source_command_stale`; `no_detection` at 4-11 m), and the retry asks a second confirmation, which aborts the eval.
   - **2026-10-13, after a2/a6/a7:** runs `.logs/fa-fixes-b`, `-c` hit at 0.0, 0.7 and 0.5 m; the remaining failures are 34 and 36.
32. **One vehicle's `AAS_*` writes can lose every echo.** In demo eval run `.logs/step4-b` (slot 1, sysids 4-6) vehicle 4 echoed its 12 `PARAM_SET`s in 66 ms, while vehicle 5 echoed none (12 x 2 s timeouts, `set_parameter` in `src/navpy/plugins/vehicle/ardupilot/vehicle_public_parameters.py:52`) right after a successful MAVFTP download of the same 1500 params. The run stopped at setup. Seen once in five step 3-4 runs; the next run (slot 0) wrote all three in about 70 ms. Suspects: the slot's mavlink-router routing (its console logged "Sniffer sysid 255" for endpoints 6 and 4 only) or a one-off SITL stall. Lockstep (steps 7-10) removes the router from the eval path. Seen again 2026-10-13 (`.logs/self-orbit-b`, vehicle 1, slot 0, no writes echoed); the retry passed.
33. **Other sim parts still step on wall slots.** The generic `GimbalSim` (`src/navpy/sim/gimbal/gimbal_sim.py`) and sim detector polling (`src/navpy/sim/detection/sim_detector_execution.py`) run once per wall slot with no catch-up, so under CPU load they skip or lag aircraft time, as the SIYI plant did before it moved to aircraft time (`src/navpy/sim/gimbal/aircraft_step_clock.py`). Lockstep steps 8-9 of `deterministic_demo_eval.md` cover the eval; wall runs still need it.
34. **One SITL slowing down backs up its commands.** In `.logs/fa-fixes-b` UAV 4's aircraft clock ran at about 6x for 2 wall-seconds while its peers held 10x. Aircraft pitch then lagged the command by up to 7 s of aircraft time (1.3 s when healthy), the UAV arrived 12 m high, and the dock left the frame at 13 m (`NAV_FAIL: no_detection`). NavPy's command rate stayed at 14 per aircraft-second. Not yet known: uplink, downlink or SITL stall. a3 (receipt age in aircraft time, `host_link_stall`) and lockstep cover it; measure with the a1 capture if it repeats. **Update 2026-10-13 (`.logs/dock-d`):** the backlog is in NavPy's telemetry reader thread, not in SITL: the GCS saw 10x while NavPy's reader fell 5-12 s of aircraft time behind, because callbacks run synchronously on it (`src/navpy/plugins/vehicle/ardupilot/inbound_router.py:133-135`). The armed dock contact sensor's per-sample geodetic maths tipped it over; it now filters in a flat frame first (about 6 us per far sample, the exact test only within reach of the net). See item 38.
35. **A far, centred detector dropout at the widest zoom still counts as a visual pass** (`src/navpy/core/approach/vision_nav/visual_pass.py`). The zoom guard (`PASS_MAX_ZOOM_OVER_WIDEST`) is a temporary workaround; the cure is a tracker that keeps the dock through missed frames (track + auto-zoom design). Not seen in five SITL approaches. **Resolved** by physical_docking.md d2: the centred-loss pass and the zoom guard are removed.
36. **The centred-loss pass misses some real hits.** In `.logs/fa-fixes-c` UAV 6 hit at 0.5 m, but at sub-metre range the last frame jumped to 10 deg off axis (miss over range), so the pass was refused and it reported `NAV_FAIL: no_detection` and asked a second confirmation. The completion rule needs a design that does not depend on the angle of the very last frame. **Resolved** by physical_docking.md d2: completion is dock contact only.
37. **The RuntimeTime lint is a first net, not a proof.** `tests/guards/runtime_time_lint.py` resolves names statically. Ten Codex review rounds of step 8 kept finding further edge forms: annotation and alias shapes for identity-hashed sets, and clock aliases through other binding forms. None was found in product code. The documented limits: unannotated sets of objects, reflection and exec, and lexical-scope shadowing. The lockstep thread audit and per-tick journal (step 9) are the backstop. A runtime tripwire would close more of this: a lockstep participant fails on any host clock, thread start or entropy call.
38. **Heavy work runs on the MAVLink reader thread.** `InboundRouter.ingest` dispatches every subscription callback synchronously (`src/navpy/plugins/vehicle/ardupilot/inbound_router.py:133-135`), so any slow callback delays every later message, including attitude and position for navigation. Seen in `.logs/dock-d` (item 34). Callbacks should only copy the message and hand it to their own worker (a bounded queue or a latest-value slot), and a reader-lag measurement (receipt vs boot time) should flag a backlog. Applies on main generally, not only to the contact sensor.
