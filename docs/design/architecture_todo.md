# Architecture follow-ups (TODO, logged 2026-10-08)

These tasks were suggested during the mission-module design review. None is started; we
return to them later. Paths are under `src/navpy/modules/` unless noted. The design is in
`mission_modules.md` and `architecture_split.md` (same folder).

## Bugs and safety
1. **Altitude floor: off by default, independent of the approach law.**
   - Today `nav/recovery.py:49-51` skips altitude whenever the vision law is set, while the legacy laws use 150 m (`scripts/lua/aas_params.lua:16`).
   - The owner wants it off by default, with modules adding their own checks.
   - Also: the legacy RECOVERY climb sends attitude outside GUIDED (`nav/recovery.py:93-98`).
2. **Operator LOITER becomes a hold.**
   - Remove LOITER from `ACTIVE_MODES` (`nav/navigation_decision.py:42`). NavPy never sets LOITER mode.
   - Update the pinned test (`tests/modules/nav/test_nav_controller.py:508-515`) per the owner ruling.
3. **Airspeed.**
   - Without a real airspeed sensor, the vision law must not use airspeed: VFR_HUD airspeed can be GPS-synthesized (`vehicle/flight_telemetry.py:41-43`).
   - Detect a real sensor (ARSPD_* params, SYS_STATUS bits).
4. **Dead peers stall the auction.**
   - `swarm/task_actor_slots.py` never removes a peer, and the first assignment needs every peer (`swarm/task_auction_queries.py:35-50`).
   - Add heartbeat-timeout removal, and assign among reachable peers.
5. **Unknown task_type drops the whole message.**
   - Three decoders raise on unknown kinds (`comm/messages/task_*_msg.py`).
   - Fix the dialect text too: task_type 1-4 is documented as SMALL..HEAVY, but the code uses DOCK=1.
6. **Pure-vision guard gap.**
   - `navigation/nav/vision_nav/pitch_law.py` is missing from `tests/guards/test_frozen_inventories.py:18-35`.
   - Derive the scanned files from the law's imports.
7. **NavArgs ONHOLD refresh may never run** (`nav/navigation_decision.py:243-252`). If so, GCS param edits apply only at task reset. Prove it with a test, then fix.

10. **Late-deny window** (found by the flow-runner review, 2026-10-08; UNVERIFIED in flight).
    - On a late operator deny in NAV, `_teardown` clears the active POI before it publishes DETECT (`nav/poi_rejection_exit.py:98-101`).
    - In that gap the event-pump thread sees NAV with no POI. It marks NAV_FAIL, which is an eval-fatal marker (`nav/final_approach_source_event_handler.py:57-66`).
    - The latch then sends DETECT to RESET, so AUTO restarts at item 1 instead of resuming the leg.
11. **Stale failure latch.**
    - The final-approach failure latch is consumed in any state that reaches the NAV decision step (`nav/navigation_decision.py:139-152`), with no task id.
    - So a latch left over from an ended approach can send a later state to RESET.
12. **AUTO writes can override an operator mode change.** The restart and resume paths read the mode, then write AUTO directly, with no fence (`nav/recovery.py:89-91`, `nav/navigation_task_reset.py:249-250`).
13. **Coasted tracks may reach the law as measurements** (UNVERIFIED). Trackers stamp `time.time()` (`vision/multi_object_tracker.py:124`), coast ticks republish (`vision/real_tracking.py:176-178`), and the law differentiates bearings by capture time (`navigation/nav/vision_nav/lateral_rate.py:113-131`).
18. **Real runs may get a sim gimbal.** `use_sim_gimbal=True` is hardcoded (`vision/vision_profile_composition.py:48`), even with `--detector-type real`. Only `scripts/python/run_detector.py:81` builds the real SIYI gimbal. Check the intent first.
19. **Dead radio code.** The sim radio client can't be reached (`comm/network_factory.py:23-28` never passes `simulation`), and the eByte radio code has no callers.

## Checks
8. **Delivery SITL eval.**
   - Run it and report whether the circle happens under the vision law. Its `Self-detect orbit:` gate matches the owner's flow: circle until confirmed, then approach.
   - Report only; delivery is paused.
9. **Re-pin the SOLID guard** (`tests/guards/solid_architecture_guard.py:16`, the PINNED_BASE commit is missing), so the size limits run during the split. Owner decision pending.

## Vendor seams (so a contractor's module plugs in; 2026-10-08)
14. **Approach law.**
    - Add one `FinalApproachLaw` Protocol and a law-agnostic plan type; PN evidence moves into `VisionNavLaw`.
    - Select it through one registry. Today 5 places hardcode it (`navigation/navigation_composition.py:120,206-210`, `navigation_law_builders.py:31-37`, `nav_law_factory.py:36-56`, `runtime_composition.py:131`).
    - Harnesses take a law factory.
15. **Detection.**
    - One backend registry; today 6 places check names (`args/detector_backend.py:16-24`, `vision/real_detector_models.py:134-144`).
    - Move `Detection` out of `yolo_detector.py`.
    - A class catalog instead of dock-only.
    - A contract test and a labelled replay kit.
16. **Tracking.**
    - One port over tracker, identity, lock and bridge (`vision/real_tracking_batch.py:115-146`).
    - Pass capture time.
    - Run the tracker in the sim; today `vision/sim/finite_poi_projector.py:146` uses truth ids.
17. **Vendor code runs out of process**, and receives only the vision frame. That boundary is the pure-vision guard, because import scans can't check a binary.
