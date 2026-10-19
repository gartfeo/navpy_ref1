# Delivery as a product (design v5, 2026-10-18)

**Status:** v5.1, approved by the owner on 2026-10-19 (Q6 split, Q7 lockstep plus three wall runs). v4 was approved
by the owner on 2026-10-12 (commit `779694e`; its review history, the dropped c1 pins and the v1-v3 changes are there). v5 re-audited v4 against main `168d84d`, which adds the
delivery eval work (e2b, e5, e4, e6, e10), the DOCKED final state (`81b0263`) and the todo 65 fixes. v5.1 re-bases
v5 on main `77abb63`, which adds the failed-dock end (todo 74, `feb2b19`, `docs/design/missed_dock.md`) and the
approved deterministic sim plan (`docs/design/deterministic_dev_sim.md`). Main has since moved to `78ed04c`
(deterministic sim step 2: lockstep sim files, lockstep scripts and docs only); no cited code line changed. v5 keeps
every approved v4 decision; "Changes from v4" lists each change with its reason.
- Owner ruling 2026-10-12: "Core and swarm should not know about missions we have, so SOLID principles should be kept."
- Owner ruling 2026-10-12: safety work comes after the demo; ArduPlane's own safety is enough until then. Today's
  holds stay in core. This step adds no safety work.
- Owner order 2026-10-16: finish delivery (done on main), move the logic into products (this doc), then plan pickup.
- Owner ruling 2026-10-18 (todo 74): a missed final approach is a failed dock and ends like DOCKED; it never goes
  back to search.
- Line numbers are on main `77abb63`. Paths are under `src/navpy/` unless they start with `tests/`, `scripts/`,
  `docs/` or `src/gcs/`. App files are in `app/flow/` unless a path says otherwise. Built: c2-c9 are on main (status under "Commits").
- Here a "product" is a package under `products/` (delivery, later pickup). Other designs' "product code" means all
  shipped runtime code.

Short forms:

| Short form | File |
|---|---|
| `nd` | `core/flow/navigation_decision.py` |
| `rec` | `core/flow/recovery.py` |
| `psd` | `core/flow/poi_status_decision.py` |
| `cat` | `core/flow/mission_catalog.py` |
| `fbn` | `core/flow/fallback_mission_navigation.py` |
| `ntr` | `core/flow/navigation_task_reset.py` |
| `nth` | `core/flow/navigation_transition_handler.py` |
| `nx` | `core/flow/nav_transition.py` |
| `nl` | `core/flow/navigation_loop.py` |
| `rel` | `core/skills/confirm/confirmed_poi_release.py` |
| `rej` | `core/skills/confirm/poi_rejection_exit.py` |
| `trk` | `core/skills/confirm/track_recovery.py` |
| `nta` | `core/skills/search/navigation_task_action.py` |
| `da` | `core/skills/search/detect_action.py` |
| `dr` | `core/skills/search/dock_roles.py` |
| `sel` | `core/skills/search/poi_selection.py` |
| `sda` | `core/skills/geo_approach/self_detected_approach.py` |
| `ntc` | `app/flow/nav_task_composition.py` |
| `nrc` | `app/flow/nav_runtime_composition.py` |
| `rig` | `tests/modules/nav/nav_test_rig.py` |
| `tnc` | `tests/modules/nav/test_nav_controller.py` |

## Changes from v4
Each change is forced by code added since `f744062`, or by a v4 statement that is no longer true.

1. **e5's home-dock rules move to delivery; the assigned-task rule stays in core (new commit c8a).** `DockRoles`
   (`dr`, e5) holds three delivery rules: the home dock (the DDH) is never offered (`dr:44-47`), on the way home a
   delivery dock comes first (`dr:63-67`, owner ruling 4), and a delivery dock taken on the way home gets its own
   circle (`dr:55-61`). Its docstring says "DDH" and "delivery dock" (`dr:3-7`). It also holds one swarm rule: a UAV
   with an assigned task offers nothing and takes only a detection at the task's position (`dr:45-46,49-53`, owner
   ruling 5). The swarm rule stays in core; the three delivery rules move (see "Dock roles"). Q6.
2. **The DOCKED state and the failed-dock end stay in core; the restart goes through `return_to_search`.** DOCKED
   (`core/flow/nav_state.py:22-23`) is the end state of an approach whose own dock reported contact, or (sim,
   `missed_dock.md`) of an armed approach that ended by a NAV_FAIL: "dock failed". The dock is a target, not a
   product (glossary), so the state, the contact latch, the failed-dock end, the post-dock climb (`rec:117-130`),
   the swarm "docked = busy" flag and the one-shot DOCKED hold (`nd:239-242`) are platform. The failed-dock end
   reads no product state: only the requested state, the GUIDED session, the cleanup errors, the contact and the
   episode terminal (`nx:69-80`). Only the AUTO restart is delivery's: it is wired at three pairs, RESET, RECOVERY
   and DOCKED → DETECT (`nth:67-69`), not two. c6 renames all three.
   One coupling: core reads the flow's RESET request at NAV exit as "the approach ended" and alone picks DOCKED or
   RESET (`nx:69-89`). Delivery maps FAILED and PASSED to RESET (`nd:160,204`), so its behaviour does not change.
3. **c3 keeps one `is_oneshot` read in `NavigationDecision`.** v4's c3 deletes `args` from `NavigationDecision`. The
   DOCKED hold now reads `self._args.is_oneshot` (`nd:239`), so c3 replaces `args` with a port
   `NavigationDecisionPorts.is_oneshot`, wired by app to `lambda: args.is_oneshot`. Same read, same point.
4. **The one-shot finish is called at two points.** `nx` calls `OneShotCompletion.run` for a one-shot exit that
   ends DOCKED, by contact or as a failed dock (`nx:85`, under `if docked or failed:` at `:81`), and for a completed
   one-shot exit (`nx:91`; v4: `nx:59`). c7 replaces both with `finish_sortie()`. `OneShotCompletion` still moves
   unchanged, teardown included (v4's decision), so a one-shot failed dock runs delivery's finish, as a contact dock
   does (`missed_dock.md` rule 5). New: core relies on that teardown. `nth._enter_docked` skips the RESET teardown
   for one-shot because the finish already ran it (`nth:129-137`), for both kinds of DOCKED entry. c7 writes this into
   `finish_sortie`'s docstring. The contact path is guarded by `test_docked_state.py:224` and `:503`; the failed path
   has no reset-once test yet, so c7 adds one (see c7).
5. **Two new core gates are swarm rules and stay.** The search-waypoint hold no longer applies once a peer task is
   assigned (`nd:40,81,96-102`), and `DetectAction` neither offers nor takes own detections before the detect
   waypoint (`da:37-43`, `core/flow/mission_navigation.py:47-52`). Neither names a product. v4's "`flow_task_active`
   stays the last operand" is now "keeps its place, before `peer_task_assigned`" (`nd:80-81`).
6. **`DetectAction` asks for work at two points.** The new hold calls `handle_new_poi(None)` (`da:41`) as well as the
   original call (`da:52`). c8's `choose_work` replaces both.
7. **App composition has no line headroom.** On `77abb63` `compose_navigation_task_workflows` is 149 of 150 lines
   and `compose_nav_application` 150 of 150 (v4: 143 and 124). v4's c7 and c8 would push both over. v5 puts the
   moved delivery wiring in one new app file, `app/flow/nav_delivery_composition.py`; the stage composers call its
   helpers. As built (c9): 147, 143 and `create_nav_application` 138 (136 + c4's metadata read), all within 150;
   c7's helper call freed seven lines, and c8a and c8 added four lines of helper calls. Each stage then only swaps arguments; c6's rename at `nrc:125` is one line for one line, and nothing
   else may grow `compose_nav_application` before c7 frees about ten lines. Measured at each commit by the SOLID
   guard.
8. **Ten product call positions, not seven.** c8a adds three: `offer_allowed`, `order_work` and `needs_own_orbit`.
   The arming latch (with pickup) routes all ten.
9. **Two eval gates: lockstep for identity, the delivery wall eval for delivery-only paths.** v4's "GCS demo eval"
   and "eval smoke run" become two runs. (a) The certified 3-UAV lockstep run
   (`scripts/eval_gcs_navigation_demo.py --lockstep`), which the approved deterministic sim plan makes the route for
   simulated development (`deterministic_dev_sim.md:10`). It is deterministic under load, takes about 75 s
   (`:95`), flies one-shot (`scripts/eval_gcs_demo_config.py:146`) with hub markers in its missions, and docks every
   UAV. (b) `scripts/eval_gcs_navigation_demo.py --scenario delivery` on the wall clock, because the delivery
   scenario has no lockstep run yet (`scripts/eval_gcs_demo_cli.py:160-161`): an interim exception to
   `deterministic_dev_sim.md:148`, on a quiet PC only (owner, 2026-10-18). The wall eval reaches the hub: the home UAV
   must log "Default delivery hub navigation:" (`scripts/eval_delivery_roles.py:22`,
   `scripts/eval_delivery_checks.py:181-183`). Q7 sets which commits get which run.
10. **More words for the guard.** New hits since v4: `dr:3-7,14,56,64` (moved in c8a), `sda:52`,
    `core/flow/nav_constants.py:19`, `core/approach/vision_nav/command_hold.py:29,32` and
    `core/kernel/logging/pose_cadence_streams.py:27` (was `pose_cadence_debug.py:38`). `command_hold.py:32` names the
    design file `final_approach_delivery_fix`, so it gets an exact allow row; the rest are reworded.
11. **Corrections.** "Six NavStates" is now seven. The swarm eval markers moved to
    `scripts/eval_gcs_demo_role_gates.py:16,19` and `scripts/eval_gcs_demo_fleet_gates.py:56`. All counts and line
    numbers are re-measured (see "Guards and tests").

12. **The failed-dock end (`feb2b19`) adds test harnesses and markers.** `tests/modules/nav/test_missed_dock.py`
    builds `NavTransition`, `TransitionPorts` and `OneShotCompletion` directly, so c6 and c7 list its edits. The
    status line "STATE DOCK_FAILED:" and the `DOCK_FAILED` evidence row join the byte-identical markers.

What did not change: the step-1 data (`manifest.tsv`, `final_violations.tsv`, `splits.json`, `exceptions.tsv`,
`test_step1_manifest.py` pins) is byte-identical to `f744062`. The vision walk still gives 162 roots, 0 violations,
0 stale rows, and `scripts/step1_move.py --check` still prints "no leftovers" (both run on `77abb63`).

## Goal
- **Where delivery's choices live.** All of delivery's on-board choices live in `products/delivery` as plain code:
  what happens after each result; the order of work; the hub; which docks may be offered or taken first on the way
  home; restarting and resuming the search; what a finished one-shot sortie does.
- **What core keeps.** The loop, the safety holds, the sortie-done latch, the DOCKED state, the transition hooks, the
  swarm rules and every skill. Core never imports `products/`.
- **Core and swarm know no product.** No name, docstring, log text or rule in `core/` (swarm and link included) or
  `plugins/` belongs to delivery or any other product. A guard test keeps it so.
- **Behaviour.** Identical: the same calls, in the same order, with the same log lines. The GCS and Lua do not
  change, and nothing in this step is a CHANGE.
- **Arming.** Choosing the product at arming is specified here and built with pickup.

## The picture
```
products/delivery/                 delivery's choices; imports core only
  flow.py                          DeliveryFlow.decide(), restart_search(), interrupted_leg()
  poi_status_decision.py           POI review: give way to a peer task, fly once confirmed,
                                   resume the AUTO search when closed or aborted
  work_source.py                   DeliveryWorkSource: peer task, own POI, then hub
  home_dock.py                     HomeDockRules: never offer the home dock, other docks first
                                   on the way home, their own circle
  mission_catalog.py               hub record; picks the hub among the mission markers
  fallback_mission_navigation.py   flight to the hub
  nav_oneshot_completion.py        what a finished one-shot sortie does
core/      loop, safety holds, sortie-done latch, DOCKED, transition hooks, swarm rules, skills
plugins/   vehicle/ardupilot decodes mission markers without interpreting them
app/       builds core and delivery (nav_delivery_composition.py), wires them, reads mission metadata
           (with pickup: product_choice.py latches the product at arming)
sim/       unchanged
```

## How it works
### Each tick
- Core's loop is unchanged (`nl:123-129`): sense or clear, decide, act. `act()` runs every tick.
- Core's decision stage (`NavigationDecision.decide`, kept in `nd`) first keeps a one-shot DOCKED final
  (`nd:239-242`), then reads the mode and next waypoint and runs the safety holds in today's order (`nd:243-248`):
  the guard, then recovery. Only when none acts does it call one plain function: delivery's `decide(mode)`.
- `decide(mode)` runs the rest of today's order (`nd:249-253`): the approach, then the POI review.

| Core reports | Delivery does (today's line) |
|---|---|
| Approach abandoned (GUIDED lost or not accepted) | DETECT (`nd:171,186`) |
| Approach passed | Logs "RESET: PASSED POI", then RESET (`nd:203-204`) |
| Approach failed | Clears the POI, then requests RESET (`nd:159-160`). Core's NAV exit keeps it as RESET (real build) or, for an armed sim episode, turns it into DOCKED with "dock failed" (`nx:69-89`) |
| POI confirmed and ready | NAV (`rel:45`) |
| Review closed (rejected, timed out, dwell or re-acquire limit) | Resumes AUTO, then logs "REJECTED: resuming AUTO mission" (`rej:103-110`) |
| Geo hold aborted | Logs "GEO-HOLD abort: … resuming AUTO mission", then resumes AUTO (`trk:74-80`) |
| Peer task pending while not in NAV | Hands off its own POI, then DETECT (`psd:51-56`) |
| RESET, RECOVERY or DOCKED goes to DETECT | Restarts AUTO at item 1 (`rec:96-98`, wired at `nth:67-69`) |
| AUTO resume | Picks the interrupted leg, one item back (`ntr:243-245`) |
| DETECT with no active POI (two call points, `da:41,52`) | Picks a peer task, then its own POI, then the hub (`nta:54-92`) |
| A detection could be offered | Never the home dock (`dr:47`) |
| Detections are ordered for taking | On the way home, other docks before the home dock (`dr:63-67`) |
| A taken POI meets an existing orbit | A dock taken on the way home gets its own circle (`dr:55-61`, used at `sda:50-54`) |
| One-shot NAV exit (pass, docked or dock failed) | Its finish: a sim disarm or a real-autopilot warning, then task teardown (`core/flow/nav_oneshot_completion.py:32-56`, called at `nx:85,91`) |
| New mission markers | Decides which marker is the hub (`plugins/vehicle/ardupilot/mission_metadata.py:110-112`) |

Core still picks these states:
- **ONHOLD** when disarmed, in a manual mode, or before the search waypoint without a peer task (`nd:83-102`);
- **ONHOLD after a one-shot finish** when neither a contact nor a failed dock ends the approach: core redirects
  (`nx:90-95`) and holds (`rec:80-84,132-137`);
- **DOCKED** at a NAV exit of a RESET request with own-dock contact, or with an armed sim episode's FAILED or MISSED
  terminal, reported as "STATE DOCK_FAILED" and a `DOCK_FAILED` row (`nx:68-102,105-122,138-142`); a one-shot DOCKED
  stays final (`nd:239-242`); a cyclic DOCKED climbs, then DETECT (`rec:117-130`);
- **RECOVERY** at the altitude floor;
- **DETECT** after RESET or RECOVERY;
- **CONFIRM** while a review is open;
- **DETECT** when a review or a geo hold is torn down.

### The calls core makes into delivery
App gives core plain functions. Each is called exactly where today's code runs, so the order of calls does not change.

| Core call (position kept) | Wired to | Target looked up | Commit |
|---|---|---|---|
| `NavigationDecision(flow)` (`nd:249-253`), after the guard and recovery | `DeliveryFlow.decide` | at build | c6 |
| `DetectAction(choose_work)` (`da:41,52`) | `DeliveryWorkSource.handle_new_poi` | at build | c8 |
| `InactiveGuardPorts.flow_task_active` (`nd:80`) | `lambda: hub.default_delivery_hub_active` | at call | c4 |
| `clear_flow_task`: state-reset step "mission" (`ntr:101`) and resume (`ntr:260`) | `lambda: hub.clear_navigation_task()` | at call | c4 |
| `TransitionPorts.return_to_search` (renamed from `restart_auto_mission`), the three pair hooks (`nth:23,67-69`) | `lambda: restart_search(vehicle)` | at call | c6 |
| `AutoMissionResumePorts.resume_waypoint` (`ntr:243-245`) | `interrupted_leg` | at build | c6 |
| `NavTransition(finish_sortie)` (`nx:85,91`) | `OneShotCompletion.run` | at build | c7 |
| `PoiSelector` `offer_allowed(poi)` (`sel:34-35`) | `HomeDockRules.offer_allowed` | at build | c8a |
| `PoiSelector` `order_work(pois)` (`sel:64`) | `HomeDockRules.order` | at build | c8a |
| `SelfDetectedApproachPorts.needs_own_orbit` (renamed from `leaves_home_orbit`, `sda:30,50`) | `HomeDockRules.needs_own_orbit` | at build | c8a |
| `NavigationTaskResourcePorts.refresh_mission` (`ntr:168`, exists today) | App's `refresh_mission_metadata`: reads the plugin, gives the band to the planner and the hub to delivery | at call | c4 |

- **`hub`** is delivery's `MissionCatalog`. The first ten calls are product calls; the refresh is app code.
- **Names.** Core's side says what core needs ("a task the flow owns is active", "return to search", "may this be
  offered"), never what delivery does with it. Only app's wiring names delivery.
- **Recovery inputs.** `RecoveryPorts` gains `min_alt` and `is_oneshot`, wired by app to `lambda: args.min_alt` and
  `lambda: args.is_oneshot`. Recovery no longer holds `NavArgs` (`rec:10,38,45,66,73,79,82,127,133`). The same params
  are read at the same points, so nothing changes now; with pickup, app can give each product its own floor.
- **Calls looked up when they run.** Tests replace `auto_resume.run`, `plan_orbit_approach`, `clear_navigation_task`
  and the band on the objects themselves (`tnc:1959,2000,5378-5382,5436-5444,5472-5475,6932`). These calls look up
  their target when they run, so those tests keep checking what they check today.
- **Seven calls bound at build, on purpose:** `flow`, `choose_work`, `resume_waypoint`, `finish_sortie`,
  `offer_allowed`, `order_work`, `needs_own_orbit`. No test patches any of them (AST scan on `77abb63`). The rig
  reaches the flow and the work source through `__self__`.

### Dock roles (new in v5, c8a)
e5's `DockRoles` (`dr`) is split by owner:

| Rule | Owner | After c8a |
|---|---|---|
| A UAV with an assigned or pending peer task offers nothing (`dr:45-46`) | swarm (ruling 5) | core `TaskFocus.may_offer` |
| It takes only a detection within `DOCK_ASSOCIATION_RADIUS_M` of the task's position (`dr:49-53`) | swarm (ruling 5) | core `TaskFocus.may_take` |
| The position an offer would carry (`dr:75-84`) | swarm | core `TaskFocus.dock_location` |
| The home dock is never offered (`dr:47,69-73`) | delivery (todo 29) | `HomeDockRules.offer_allowed` |
| On the way home, other docks before the home dock (`dr:63-67`) | delivery (ruling 4) | `HomeDockRules.order` |
| A dock taken on the way home gets its own circle (`dr:55-61`) | delivery (ruling 4) | `HomeDockRules.needs_own_orbit` |

- **Core.** `git mv dr core/skills/search/task_focus.py`; class `TaskFocus` with ports `assigned`, `assigned_dock`,
  `detector_is_simulation`, `ground_location`. `PoiSelector` (`sel:23-35`) takes `focus: TaskFocus`,
  `offer_allowed: Callable[[DetectedObject], bool]` and `order_work: Callable[[list[DetectedObject]], list[DetectedObject]]`.
  Its `_may_offer` becomes `peer_available() and focus.may_offer(poi) and offer_allowed(poi)`, so a product cannot
  skip the swarm rule. `may_take` stays `focus.may_take`. The radius stays in `core/flow/nav_constants.py:15-20`,
  with neutral wording.
- **Delivery.** New `products/delivery/home_dock.py`, `HomeDockRules(focus, home_dock, homing)`:
  `is_home_dock(poi)` uses `focus.dock_location`; `offer_allowed` = not the home dock; `order` sorts home docks last
  while homing; `needs_own_orbit` = homing, no assigned task, and not the home dock.
- **Same behaviour.** `may_offer` today is `not assigned and not home`; after the split it is
  `focus.may_offer and offer_allowed`, the same boolean, evaluated in the same order of reads. `order` and the orbit
  rule are moved bodies.
- **Wiring.** `ntc:254-274` (`_dock_roles`) moves to `app/flow/nav_delivery_composition.py`. `homing` and
  `home_dock` read delivery's catalog (`ntc:264-265`).
- **Tests.** The controller-level tests in `tests/modules/nav/test_dock_roles.py` (`:102-329`) do not change. The
  four direct unit tests (`:350-382`, built through `_roles` at `:337-346`) build `TaskFocus` and `HomeDockRules`.
  Each assertion moves to the object that owns its rule, with the same inputs and the same expected boolean:
  `is_home_dock` to `HomeDockRules`, `may_take` to `TaskFocus`. The combined "may this be offered" check runs through
  a `PoiSelector` built with both, so the composition is tested, not re-implemented in the test. No compatibility
  facade. More than a construction change, so it is listed with Q6.

### Mission metadata: who decodes it, who interprets it
Unchanged from v4 (every cited line holds on `77abb63`).
- **The plugin decodes, without giving meaning** (`plugins/vehicle/ardupilot/mission_metadata.py`): the search
  pattern (`:113-114,125`), the corridor/scan band (`:47-79,116-122,127`) and every DO_SET_ROI_LOCATION marker, as
  `MissionMarker(code, location, location_type)` (`:105-112`). From c9 on, it no longer asks "is this the hub?" (`:110`).
- **Core holds the neutral record** in `core/ports/mission_marker.py`. Plugins and products may not import each other
  (`scripts/step1_manifest.py:240,245-248`).
- **Delivery interprets the markers** in `products/delivery/mission_catalog.py`: `HUB_MARKER_CODE = 3` is the wire
  code (`plugins/vehicle/ardupilot/mission_encoding.py:52`; a test checks they are equal); `update(markers)` keeps the
  last hub marker, as the plugin's loop does today.
- **App connects the two** (`scripts/step1_manifest.py:236`).
- **Unchanged:** the codec `mission_encoding.py` (the GCS imports it: `src/gcs/backend/planner/mission_items.py:15`,
  `src/gcs/backend/planner/waypoint_builder.py:16`, and `src/gcs/backend/mission_validator.py:9-12` through the
  re-export). `read_default_delivery_hub` (`:33-44`) has no user in `src`; c9 deletes it.

### Choosing the product at arming (specified now, built with pickup)
Unchanged from v4, except that the record holds ten calls.
- **Setting.** `AAS_MOD`, an integer; `0` means delivery and is the default. A Lua AAS param at index 20 in
  `scripts/lua/aas_params.lua` (`:3`), with its line in `src/gcs/backend/aas_params.py:17-37`
  (`tests/scripts/test_aas_params_lua.py:30-34` requires both). Not in `NavArgs`, which re-reads at each task reset
  (`ntr:167`, `core/kernel/config/nav_args.py:79-92`).
- **The read never waits.** At the arm edge app calls `vehicle.get_parameter("AAS_MOD", timeout=0.0, quiet=True)`:
  a cached value comes back at once (`plugins/vehicle/ardupilot/parameter_reader.py:42-43`); every PARAM_VALUE from the
  own autopilot updates the cache (`plugins/vehicle/ardupilot/inbound_router.py:129`); a miss sends one request and returns None
  (`parameter_reader.py:56-73`, `message_store.py:152-154`), not asking again for 10 s (`parameter_reader.py:16,44-45,85`).
  App fills the cache once at start-up (`nav_args.py:40`).
- **Arm edge, in core.** `NavigationCycle.run` reads `vehicle_armed()` once per tick (`nl:124`). It gains one port,
  `armed_changed(armed)`, called before sense and `decide`, on the first tick and on each change.
- **Latch, in app** (`app/flow/product_choice.py`): `choose_product(value)` is the only product check in `src/navpy`
  (`None` or `0` → delivery, anything else → none). `Sortie.armed_changed(armed)`: disarmed → `ready = False`, calls
  stay on the last product so the disarm teardown (`nd:83-85`) runs through it; armed after disarmed → read, latch,
  `ready = True`, status "SORTIE: delivery (AAS_MOD=0)"; unknown value → `ready = False` with a status naming it;
  armed on the first tick → latch as on an arm edge (Q4b). `ProductCalls` holds the ten product calls. Start state:
  delivery's record, `ready = False`.
- **Hold, in core.** `InactiveGuardPorts.sortie_ready()`, checked right after the disarm block (`nd:83-89`) and
  before the mode block (`nd:90`): if False, clear any task, report "ONHOLD: SORTIE NOT READY", request ONHOLD.
- **Reaching the chosen product.** Each of the ten positions is wired to `lambda …: sortie.calls.<name>(…)`. Nothing
  registers into `sortie`; neither core nor a product sees it (Q4e). All products are built at start-up
  (`app/runtime_composition.py:213`; the loop runs while disarmed, `nl:123-129`).
- **Re-arm** reads the value again; edits while armed are ignored. Not after a one-shot dock: DOCKED is final for
  the process (`docked_state.md:42`), and `decide` returns at `nd:239-242` before the disarm block, so that block's
  teardown and `oneshot_completed` clear (`nd:83-86`) never run; the task teardown already ran at DOCKED entry
  (`nth:129-137`). A re-arm then latches a product but starts no sortie. If pickup needs a new sortie after a one-shot
  dock without a restart, its design adds a generic reset at the arm edge, in core, with a lifecycle test (dock or
  failed dock → disarm → product choice → re-arm → new task). **Forced arm** (`docs/design/flow_runner.md:192`,
  `nrc:188`, `src/gcs/backend/routes/control_vehicle_commands.py:74-90`) counts as an ordinary arm edge (Q4c);
  `roadmap.md:107`'s forced-arm hold waits for the safety work. UNVERIFIED: whether a forced arm is visible to NavPy.
- **Behaviour changes, all with pickup:** CHANGE 1, an unknown `AAS_MOD` holds the sortie (Q4a). No restart hold
  (Q4b). A missing value flies delivery (Q4d).
- **Guard test, with pickup:** only `app/flow/product_choice.py` names `AAS_MOD` or compares product values.
- **Why not now:** the Lua param needs the GCS line, and the GCS is untouched here; with one product the latch is dead
  code. It lands as the first commit of pickup's platform work (roadmap R22, `roadmap.md:55`), with `npm test` and
  `npm run build` (`roadmap.md:108`). Gate: the GCS writes `AAS_MOD`, NavPy's cache sees it, the next arm edge
  latches it. UNVERIFIED: a one-tick "ONHOLD: SORTIE NOT READY" when the loop and guard read the armed bit on
  opposite sides of the arm.

## What moves
| Today | After | Commit |
|---|---|---|
| The approach and review half of `NavigationDecision.decide` (`nd:249-253`) and its outcome mapping | `products/delivery/flow.py` `DeliveryFlow`; the DOCKED hold, guard and recovery stay in core | c6 (c3 first deletes the dead refresh, `nd:255-264`) |
| Next state requested after the approach (`nd:160,171,186,203-204`); core's NAV exit may still turn a RESET into DOCKED (`nx:69-89`) | `DeliveryFlow.decide` maps `ApproachOutcome` | c5 writes it in core; c6 moves it |
| `psd` (peer yield, review dispatch) | `products/delivery/poi_status_decision.py` (`git mv`) | c6 |
| Confirmed means fly (`rel:45`) | The review requests NAV on `ReleaseOutcome.READY` | c5, c6 |
| Resume and its log after a closed review (`rej:103-110`) | The review | c5, c6 |
| Resume and its log after a geo abort (`trk:74-80`) | The review, right after the abort returns | c5, c6 |
| Restart AUTO at item 1 (`rec:96-98`) | `restart_search(vehicle)`, in the same three pair hooks | c6 |
| Rewind to the interrupted leg (`ntr:244-245`) | `interrupted_leg(next_waypoint)`, at the same statement | c6 |
| One-shot finish (`core/flow/nav_oneshot_completion.py`) | `products/delivery/nav_oneshot_completion.py` (`git mv`), still called at `nx:85,91` | c7 |
| Home-dock rules (`dr:44-47,55-73`) | `products/delivery/home_dock.py` | c8a |
| Work order (`nta:54-61,88-92`) | `products/delivery/work_source.py` | c8 |
| `cat` | `products/delivery/mission_catalog.py` (`git mv`) | c8 (c4 first removes its reader and band) |
| `fbn` | `products/delivery/fallback_mission_navigation.py` (`git mv`) | c8 (c4 first removes its orbit sizing) |
| Hub-marker choice in the plugin (`mission_metadata.py:33-44,110-112`) | `MissionCatalog.update(markers)` | c9 (Q3) |

**Generic parts split out of delivery (they stay in core or app):**
- **Orbit sizing and scan band** (`fbn:93-105`, `cat:18,24`) go to `ApproachPlanner.plan_orbit_approach` and its
  attribute `scan_altitude_rel`. The peer and self-detect approaches use them too (`ntc:150-155,205-210`).
- **Metadata read** (`cat:20-24`) goes to app. The first read stays the first step after `compose_state_ownership`
  (`app/flow/nav_composition.py:64-67`), before any other composition and before the "Approach strategy:" log
  (`app/flow/nav_foundation_composition.py:172`). Each task reset reads again through `ntr:168`.
- **The assigned-task rule** of e5 (`TaskFocus`, c8a).
- **Plugin.** It decodes every mission marker without interpreting it (c9).

**Core edits (types, return values and ports only):**
- `NavStateDecision.advance` returns `ApproachOutcome` and makes no requests; no `status`. On a failure it returns
  FAILED without clearing the POI (`nd:159`); FAILED means "the task failed", in any phase (`nd:147`).
- `ConfirmedPoiRelease.apply` returns `ReleaseOutcome`. `PoiRejectionExit.apply` returns True when the review is
  closed, and no longer resumes. `TrackRecovery`'s abort stops at the DETECT request (`trk:73`).
- `flow_task_active` and `clear_flow_task` replace the catalog in `nd` and `ntr`.
- `AutoMissionResumePorts.resume_waypoint` replaces the inline rewind.
- `RecoveryAction` loses `restart_auto_mission` and `NavArgs`; `RecoveryPorts` loses `set_mode` and `restart_mission`
  (`rec:25-26`) and gains `min_alt` and `is_oneshot`.
- `TransitionPorts.restart_auto_mission` is renamed `return_to_search` (three pair hooks).
- `NavTransition` takes a `finish_sortie` function instead of the one-shot object (two call points).
- `NavigationDecision` keeps its ports (`vehicle_mode`, `next_waypoint`, and `is_oneshot` from c3), the DOCKED hold,
  the guard and recovery, and takes `flow: Callable[[FlightMode], None]` instead of `nav` and `poi_status`
  (dependencies 8 → 5 after c3, c4 and c6: ports, phase, inactive, recovery, flow). `NavigationCycle` does not change. `DetectAction(choose_work)` takes a plain
  function.
- `NavStateDecision` loses `status` and the confirmation manager (dependencies 8 → 6).
- `NavigationTaskAction` keeps `start` and gains `try_start` (`nta:62-87`); dependencies 12 → 10.
- `PoiSelector` takes `TaskFocus`, `offer_allowed` and `order_work` instead of `DockRoles` (c8a).

**Stays in core, and why:**
- **Swarm rules:** the peer-task mechanism (`pending`, `hand_off`), the en-route gate (`nta:62-67`), the own-POI
  admission gates (`nta:68-80`), `start`, the assigned-task focus (ruling 5, `TaskFocus`), the peer-task exemption
  from the waypoint hold (`nd:96-102`), the own-search hold before the detect waypoint (`da:37-43`), and "docked is
  busy" in the task slots. The same for every product.
- **Completion classification** (`NavStateDecision`, `MissionPassPolicy`): it is on the vision path.
- **`rec`:** the floor, the RESET/RECOVERY → DETECT requests, the one-shot hold and the post-dock climb.
- **The sortie-done and DOCKED mechanism:** the latch `NavPhaseState.oneshot_completed` and its clear at disarm
  (`nd:86`); the ONHOLD redirect (`nx:90-95`); the hold (`rec:80-84,132-137`); the DOCKED redirect and status lines
  (`nx:68-102,124-135`), the one-shot DOCKED hold (`nd:239-242`) and the DOCKED entry teardown (`nth:129-137`);
  `AAS_NAV_ONESHOT` in `NavArgs`. Roadmap R19 lists the sortie-done latch as platform (`roadmap.md:52`;
  `docs/design/architecture_split.md:171`).
- **The failed-dock end** (`missed_dock.md`): the reasoned episode latch and its command end
  (`core/approach/vision_nav/episode_latch.py`, `core/approach/vision_nav/runtime.py:194-201`), the NAV-exit terminal
  snapshot (`core/approach/nav_exit_cleanup.py`, `core/approach/navigation_dock_outcome.py`,
  `core/approach/vision_nav/dock_outcome.py`), the RESET → DOCKED redirect and the `DOCK_FAILED` row and status
  (`nx:68-102,105-122,138-142`). It reads no product state.
- **Acquisition gates before NAV** (for example HITL step 25's visual lock, `docs/design/hitl_final_approach.md:508`):
  a core skill result, reported as `ReleaseOutcome.WAIT`, never a rule inside `DeliveryFlow`, so pickup gets it too.
- **The geo-hold mechanism and its abort teardown;** only the resume and its log line move.
- **The `AutoMissionResume` teardown and its AUTO write;** delivery only chooses when and to which waypoint.
- **The search-waypoint gate** (`AAS_NAV_LAST_WP`, `nd:96-102`).
- **Bid eligibility** (lowest flight time wins). Pickup's role bids come with pickup (R23).
- **The dock as a target:** the detector class, its size preset, the `DOCK` task kind, contact sensing and DOCKED. See Q5.
- **The wire codec** (`plugins/vehicle/ardupilot/mission_encoding.py`), allowed by name in the guard.

## Core and swarm know no product
v4's five audits (see `779694e`) plus v5's re-scan of `168d84d` and v5.1's of `77abb63` (no new hit).

| Item | Today | Fix | Commit |
|---|---|---|---|
| Safety holds would sit inside the flow | `nd:239-248` | Core's `NavigationDecision` runs the DOCKED hold, the guard and recovery, then the flow | c6 |
| Core clears the POI on a failure | `nd:159` | Delivery clears, then requests RESET (same order) | c5 |
| Recovery reads the floor and one-shot from `NavArgs` | `rec:66,73,79,82,127,133` | `RecoveryPorts.min_alt`, `is_oneshot` | c6 |
| Hook named after delivery's restart | `nth:23,67-69` | `return_to_search` | c6 |
| Core-side names say "product" | v3's ports | `flow_task_active`, `clear_flow_task`, `sortie_ready` | c4 (`sortie_ready` with pickup) |
| Home-dock rules in a core skill | `dr` | `HomeDockRules` in delivery; `TaskFocus` in core | c8a |
| Port named after delivery's homing | `sda:30,50` (`leaves_home_orbit`) | `needs_own_orbit` | c8a |
| Docstrings, comments and help text that name delivery or the hub, or say core makes a choice that moves | `rel:1,16`, `rej:30` (c5); `rec:1`, `core/skills/acquire/orbit_geometry.py:13` (c6); `core/flow/nav_oneshot_completion.py:20`, `core/kernel/config/nav_args.py:71` (c7); `dr:1-15`, `sda:52`, `core/flow/nav_constants.py:15-19` (c8a); `nta:24,56`, `core/skills/acquire/peer_offset.py:1,78`, `peer_orbit_plan.py:1`, `core/swarm/peer_task_priority.py:1-7`, `core/flow/mission_navigation.py:1`, `core/flow/nav_task_availability.py:1-7` (c8); `plugins/vehicle/ardupilot/mission_metadata.py:83,90` and its import `:15`, unused once c9 removes `:33-44,110-112` (c9) | Neutral wording | c5-c9 |
| "delivery" as a plain noun | `core/kernel/logging/pose_cadence_streams.py:27`, `core/perception/pipeline/real_tracking.py:1`, `core/swarm/nav_network_shutdown.py:53`, `plugins/vehicle/ardupilot/pose_streams.py:5`, `core/approach/vision_nav/command_hold.py:29` | "hand-off" or "transport" | c8 |

None of these changes behaviour, a log line the evals read, the wire, the GCS or Lua. The `-nos` help text is the
only user-visible text that changes.

**Guard** (`tests/guards/test_core_knows_no_product.py`, new in c8, about 115 lines, modelled on
`tests/guards/test_oracle_stays_in_the_bench.py`; no src code):
- Scans every `.py` and `.json` under `src/navpy/core` and `src/navpy/plugins`, except the vendored `_mavftp`.
- Identifiers (split on `_` and camelCase), strings and comments; whole words, any case.
- Refuses: delivery, deliveries, ddh, hub, hubs, pickup, pick-up, catcher, medicine, survey, spraying, surveillance,
  patrol. Each new product adds its words.
- Exact allow rows (path, word, count, reason): `mission_encoding.py` (the codec the GCS imports); until c9,
  `mission_metadata.py`; `core/approach/vision_nav/command_hold.py:32` (names the design file
  `final_approach_delivery_fix`). A missing or stale row fails.
- A synthetic self-test covers snake case, camel case, acronyms, strings, comments, an allow-row escape, and words that
  must pass ("deliver", "delivered", `AAS_DEL_*`).
- It checks words, not meaning. Each commit's Codex review checks that no product rule hides behind a neutral name.

**Not product knowledge (kept, with reasons):** `AAS_DEL_*` param names (AGENTS.md keeps names); the sortie-done
latch and `AAS_NAV_ONESHOT` (R19); the seven NavStates, DOCKED included, and the transition hooks (a generic cycle,
but core reads a RESET request from NAV as "the approach ended" and alone picks DOCKED or RESET, `nx:69-89`); the
failed-dock end and its "STATE DOCK_FAILED" line; the search-waypoint gate, pass classification, AUTO resume
teardown, peer-task mechanism, busy rule, bidding and the swarm log lines the evals read; the disclaimer required by
AGENTS.md; the review's site-type labels (`core/review/confirmation_media.py:12-26`).

**The dock (Q5).** The glossary makes the dock a general-purpose target, so the name is not product knowledge. Core
supports one target class, in four places: the class id and its catalog check
(`core/perception/profile/vision_class_profile.py:20,66-70`); the size preset (`:119-121`,
`vision_profile_loader.py:47`); the swarm task kind `DOCK` (`core/link/types.py:28-48`); three fall-backs to class 0
(`core/skills/confirm/confirmation_action.py:98`, `core/skills/acquire/peer_geo.py:214`, `peer_offset.py:67`).
Swarm assignments already carry the class on the wire (`core/link/task_assignment_msg.py:61,76`).

## New types
| Name | File | Why |
|---|---|---|
| `ApproachOutcome` (NONE, WAITING, ABANDONED, PASSED, FAILED) | `nd` | Core reports the approach result |
| `ReleaseOutcome` (WAIT, ABORTED, READY) | `rel` | "keep confirming", "geo hold aborted", "fly" (`rel:41,43,45`) |
| `DeliveryWorkSource` | `products/delivery/work_source.py` | Delivery's order of work |
| `HomeDockRules` | `products/delivery/home_dock.py` | Delivery's home-dock rules (c8a) |
| `MissionMarker` (code, location, location_type) | `core/ports/mission_marker.py` | Plugin → delivery without imports (c9) |

- **Renamed and moved, not new.** `DeliveryFlow` is `NavigationDecision`'s second half. `TaskFocus` is `DockRoles`
  minus the home-dock rules. `OneShotCompletion` moves unchanged. `restart_search` and `interrupted_leg` are plain
  functions.
- **None of these.** No Protocols, registries or plugin slots.
- **With pickup only.** `ProductCalls` and `Sortie` (in app).

## Simulation-only parts
Each is kept exactly as it runs today; only delivery's parts move. No simulation machinery goes into `products/`.

| Part | This step | Later |
|---|---|---|
| Mid-flight refresh | Live part stays in the core reset (`ntr:167-168`); its mission half calls app's read. The dead ONHOLD part is deleted (Q1). | With the arming latch, sim runs only (CHANGE). |
| Restart at item 1 | Delivery's `restart_search` in the same three pair hooks (`nth:67-69`). The DOCKED → DETECT restart is the cyclic sim test (`docs/design/docked_state.md:43`). | After Q2, sim runs only (CHANGE). |
| SIM_SPEEDUP apply and restore | Untouched (`nta:96`, `core/approach/nav_exit_cleanup.py:93`, `ntr:145,174-176`). | Deleted with the wall sim route (`deterministic_dev_sim.md` step 14); lockstep refuses `SIM_SPEEDUP` other than 1. |
| Hub sim POI | Moves with the hub route (c8), behind the same ports (`fbn:24-25,63-68`). | Moves to `sim/`. |
| One-shot sim disarm | Moves with delivery's finish (c7), behind `OneShotCompletionPorts.disarm`, gated by `is_simulated_autopilot` (`core/flow/nav_oneshot_completion.py:13-16,36-40`). It also runs for a one-shot failed dock (`missed_dock.md` rule 5). | Only the disarm moves to `sim/`. |
| Failed-dock end (`missed_dock.md`) | Stays in core; an episode is armed only with the sim contact sensor (`app/perception/sim_detector_factory.py:66-69`); the net-radius override is sim-only (`sim/world/dock_contact_sensor.py:45,53-61`). | The real build, when a real capture signal exists. |

## Commits
**Gate for every commit:**
- `pytest tests` and `pytest tests/gcs -o addopts=""`.
- The guards: layers, unresolved imports, vision walk, fresh imports, registry census and SOLID.
- `scripts/step1_move.py --check` ("no leftovers").
- A Codex review.
- An ff-only push to main, then the same checks on main before the next commit.
- From c3 on, the lockstep identity run (Q7): the certified 3-UAV run on the commit gives the same journal digest
  and byte-equal navigation logs as on its parent (`input_manifest.json` excluded: it hashes sources). If
  `deterministic_dev_sim.md`'s development gate (a seed sweep) is in force by then, it runs as well.
- The delivery wall eval where Q7 says, on a quiet PC agreed with any session running tests or SITL.

Build each commit in a temp worktree. **Before c2:** re-record the test baseline (UNVERIFIED now), then two lockstep
checks, each `scripts/eval_gcs_navigation_demo.py --lockstep --seed 1` with the default `hw` profile and the same
firmware lock (a firmware change moves the digest, `deterministic_dev_sim.md:127`):
- **Repeatable:** two runs on the c2 parent. The compared set is the journal digest plus every log file that is
  byte-equal between these two runs; `input_manifest.json` is excluded (it hashes sources,
  `scripts/simtime_navigation_identity.py:11-18`). Different digests are a determinism bug: log it in
  `architecture_todo.md`, stop, and the owner picks the replacement gate. Nothing falls back on its own.
- **A move keeps it:** a throw-away probe on the c2 parent moves one runtime module c7 moves
  (`core/flow/nav_oneshot_completion.py`) to `products/delivery/` (package files and imports only; no guards) and
  runs once; the compared set must match. If it differs, find what depends on a module path before c3; a
  re-baseline needs that mechanism and the owner.
- **Done 2026-10-19 on `869d5a4`:** both runs and the probe gave `fbda9e41…` (step 2's `hw` seed 1 digest) and
  passed; the compared set is 61 files. The probe's three `participant_*_host.log` differ only in the checkout
  folder it ran from, so every later run starts from the same worktree. Runs in `.logs/delivery_move_ref/base_869d5a4`.
- **Built 2026-10-19:** c2-c9 are on main (c2 `1b6faf9`, c3 `75fc2c8`, c4 `eb424b0`, c5 `ee26954`, c6 `420ab6b`,
  c7 `ef45e61`, c8a `d3b011a`, c8 `6953798`, c9 `27d0575`). Every lockstep run from c3 to c9 kept `fbda9e41…` and
  the 61-file compared set, and every gate showed only todo 78's nine failures. Wall runs, in
  `.logs/delivery_move_ref`: the forced miss on c7 passed (each UAV DOCK_FAILED, no return to search, one disarm
  each); the normal run on c8 passed; the normal run on c9 docked all three, the home UAV on the hub route
  ("Default delivery hub navigation:") to its home dock, but failed UAV 1's roll-step gate on its last command
  before contact (63.9 deg at a logged range of 0.9 m). That failure is todo 80, cause not yet traced; the move
  changed no law, tracker or gimbal code (only comments there), and the lockstep runs stayed byte-equal. The identity reference is now c9's lockstep run
  (`c9_27d0575/lockstep`, source hashes in its `input_manifest.json`); no `.py` file changed after c9.

The same rule holds on every later commit: a changed compared set stops the plan until the cause is found. The
parent's run is redone whenever main moves. The wall references are the green and forced-miss delivery runs of
todo 74 on `feb2b19` (copied to `.logs/delivery_move_ref/green` and `/miss` in the main checkout); main's
`src/navpy` differs from `feb2b19` only in lockstep-only files under `sim/lockstep/`.

**Path edit** means only an import, attribute, constructor or patch path changes. No assertion and no expected value
changes.

| Id | What | Size |
|---|---|---|
| c0 | v4 doc (done, `779694e`); v5 doc with the owner's answers | S |
| c1 | Dropped (owner, 2026-10-12) | – |
| c2 | Rename the planned layer `missions` to `products` (script, data, docs) | S |
| c3 | Delete the dead ONHOLD refresh; `is_oneshot` port in `NavigationDecision` | S |
| c4 | Core stops reading the hub record; the planner gets orbit sizing and the band; app reads the metadata | M |
| c5 | Core reports results; the choices are written once, still in core | M |
| c6 | Delivery's flow, review, restart and rewind move; vision root; `nav_delivery_composition.py` | M |
| c7 | Delivery's one-shot finish moves | S |
| c8a | Home-dock rules move; `TaskFocus` stays | S |
| c8 | Work source; the hub record and route move; docs; guard | M |
| c9 | Neutral mission markers (Q3) | S |

**c2: layer rename.** As v4 (data and docs are byte-identical on `77abb63`): `scripts/step1_manifest.py:90,226,247`;
`docs/design/step1/manifest.tsv` (7 lines), `final_violations.tsv:14,16,17,19,21`, `splits.json:28`;
`docs/design/step1_separation.md:19,50,69,92,177`, `roadmap.md:39,45`, `architecture_split.md:20,43,57,168,170,171`,
`flow_runner.md:292`; `tests/scripts/test_step1_manifest.py:151,164,175,216,252-254` and new `allowed()` cases
(core→products, plugins→products, products→sim, sim→products, all False).

**c3: dead refresh (Q1).**
- `nd`: delete the call at `:253`, the method at `:255-264` and `NavigationDecisionPorts.clock_s` (`:211`). Replace
  `args` (`:225,234`) with `NavigationDecisionPorts.is_oneshot`, read at `:239`; remove `mission` (`:226,235`) only in
  c4. `NavArgs` import (`nd:9`) goes.
- `core/flow/nav_state.py`: remove `last_parameter_refresh_s` (`:31,42,47,70-78,96`).
- Constructor: `app/flow/nav_decision_composition.py:94,101` (`:102`, the catalog, goes in c4), plus
  `is_oneshot=lambda: args.is_oneshot`.
- Tests: rig path edits as v4 (`self.mission` view); the rig's `last_parameter_refresh_s` property
  (`tests/modules/nav/nav_test_rig.py:102-108`) goes with the field; `previous` stays (the DOCKED hold reads it). The
  direct harness `_decision` in `tests/modules/nav/test_docked_state.py:325-348` passes `is_oneshot` as a port
  instead of `clock_s` and `args` (constructor edit; its assertions at `:351-367` stay).

**c4: core stops reading the hub record.** As v4, with current lines:
- `core/skills/acquire/approach_planner.py` (unchanged since v4): constructor gains `approach_kind` and
  `scan_altitude_rel` (6 → 8); `plan_orbit_approach` with the body of `fbn:93-105`; docstrings `:1,29`.
- `cat`: as v4 (`:6-7,13-14,18,20-24`), plus `update(hub, hub_type)`.
- `nd:33-36,57,67,80`: `flow_task_active` replaces the catalog, in its place before `peer_task_assigned`.
  `NavigationDecision` loses `mission` (`nd:226,235`); the `MissionCatalog` import (`nd:12`) goes.
- `ntr:65,74,101,207-211,226,237,260`: `clear_flow_task`. `fbn`: remove `approach_kind` (`:38,46`) and
  `plan_orbit_approach`; `:69` calls the planner.
- App: `app/flow/nav_composition.py`, right after `:67` add `metadata = read_mission_metadata(vehicle)`; pass it to
  `:68-74` and the band to `:75-85`; drop `:101`. `nav_foundation_composition.py:106-107,145-152`. `ntc`: drop `:64,82`;
  `:150-155,205-210` call `approach.planner.plan_orbit_approach`. `nav_decision_composition.py:60-74` (drop `:69`)
  and `:102`.
  `nav_reset_composition.py` as v4 (`:45,53,69-73,80`, unchanged).
- Tests (path edits): 16 band lines (tnc 15 + `tests/modules/nav/test_dock_roles.py:95`); 9 orbit lines
  (`tnc:5207,5252,5379,5436,5463,5475,6247,6278,6306`); `tnc:6932`; rig `self.mission` after `:192`.
- Guards: delete `exceptions.tsv:7`; pin `test_step1_manifest.py:107` to `{"layer": 27, "vision": 15}`.
- Done when the gate is green, `test_mission_metadata.py` is untouched, and `tnc:5378-5382,5436-5444` still take effect.

**c5: core reports results.** As v4; `rel`, `rej`, `trk` and `psd` lines are unchanged on `77abb63`.
- `nd`: `NavStateDecision` (`:137-204`) returns `ApproachOutcome`; `_mark_passed` only sets `completed` (`:202`); the
  failure path returns FAILED before `:159`; `status` removed.
- `rel:41,43,45`; `rej` (remove `_resume` `:103-110`, `resume_auto` `:38,44`, import `:14`); `trk` (abort ends at
  `:73`; remove `:74-80`, `resume_auto` `:51,60`, import `:17`).
- `NavigationDecision.decide` (`nd:249-250`) maps the outcomes; on FAILED it clears the POI, then requests RESET.
  PASSED and FAILED keep requesting RESET, which core's NAV exit reads as "the approach ended" (`nx:69-89`), so the
  failed-dock end is unchanged.
  `psd.dispatch` returns None and gains `resume_auto` and `logger` (7 → 9), handling results as v4.
- Constructors: `nav_decision_composition.py:75-103`, `nav_poi_status_composition.py:47-87`,
  `nav_track_recovery_composition.py:36-48`. Rig: `self.poi_status._resume_auto`. Docstrings `rel:1,16`, `rej:30`.
- `tests/modules/nav/test_final_approach_docked_flow.py` calls `NavStateDecision.advance` directly (`:96-128`) and
  asserts the phase, the "RESET: PASSED POI" status and the POI clear (`:165-168,219-222`), which c5 moves to the
  consumer. Its `decide()` helper (`:127-128`) now runs the real consumer: `NavigationDecision`'s outcome mapping in
  c5, `DeliveryFlow.decide` from c6 on. It returns `outcome is not ApproachOutcome.NONE`, which is exactly what
  `advance()`'s True means today (`nd:140-198`: every True path is WAITING, ABANDONED, PASSED or FAILED). Every
  assertion stays.

**c6: delivery's flow, review, restart and rewind move.**
- New `products/__init__.py`, `products/delivery/__init__.py`, `products/delivery/flow.py` (`DeliveryFlow`:
  `nd:249-253` with c5's mapping; `restart_search(vehicle)` = `rec:96-98`; `interrupted_leg` = `ntr:244-245`).
- `git mv psd products/delivery/poi_status_decision.py`.
- `nd`: `NavigationDecision` keeps `:237-248` (DOCKED hold, mode and waypoint reads, guard, recovery) and calls
  `self._flow(mode)` where `:249-253` were.
- `rec`: remove `:96-98`, `RecoveryPorts.set_mode` and `restart_mission` (`:25-26`), the `FlightMode` import (`:16`),
  `NavArgs` (`:10,38,45`); reads at `:66,73,79,82,127,133` go through `min_alt()` and `is_oneshot()`. Docstring `:1`.
- `nth`: rename `restart_auto_mission` to `return_to_search` (`:23,67-69`). `core/skills/acquire/orbit_geometry.py:13`.
- `ntr:243-245` becomes `self._ports.resume_waypoint(self._vehicle.mission_items_next)`.
- App: new `app/flow/nav_delivery_composition.py` builds `DeliveryFlow`; `nav_decision_composition.py` passes
  `flow=delivery_flow.decide`; `nrc:125` wires `return_to_search=lambda: restart_search(vehicle)` (one line for one:
  `compose_nav_application` is at 150 of 150); `nav_reset_composition.py:69-73` `resume_waypoint=interrupted_leg`;
  `nav_poi_status_composition.py`, `nav_composition_types.py:54,61,184,191`.
- Tests: the 129 `decision.decide()` calls are unchanged. Rig path edits through `decision._flow.__self__`.
  `tests/modules/nav/test_docked_state.py:265-282` builds `RecoveryPorts` and `RecoveryAction` without `set_mode`,
  `restart_mission` and `args`, with `min_alt` and `is_oneshot` ports (constructor edit). Its `_decision` passes a
  `flow` Mock instead of `nav` and `poi_status`; `test_one_shot_docked_runs_no_other_decision` (`:351-358`) checks
  that owner with `mock_calls == []`, because `method_calls` does not record a call of the Mock itself and would pass
  even if the flow ran. The rename reaches its transition harnesses too: the `TransitionPorts` keywords at
  `:207,472` and the assertions on that port at `:245,253` (path edits; same expected calls), plus the docstrings
  at `tnc:8791,8896,8906`. The same in `tests/modules/nav/test_missed_dock.py`: the `TransitionPorts` keyword at
  `:443` and the assertions on that port at `:462,478,484,487`.
- Data and vision root: as v4 (`splits.json` flagged part, `manifest.tsv:515` → `split`, `:539` final home,
  `tests/guards/step1_vision_walk.py:20` gains `"products/"`, `test_step1_manifest.py:92-93` 12 → 13 and 13 → 14, a
  new root test, the comment at `tests/guards/test_step1_vision_walk.py:34-35`). Re-run the end-state vision
  simulation on the c6 parent first (Risk 11): `feb2b19` changed `nx` (a vision-path row, `manifest.tsv:514`),
  `core/approach/nav_exit_cleanup.py` and `vision_nav/episode_latch.py`.
- Done when the gate is green, the walk gives 0 violations and 0 stale rows with `products/delivery/flow.py` a root,
  the restart (`tnc:6715,8798,8903,8913`) and rewind (`tnc:8271-8537`) tests pass unchanged, and the DOCKED and
  failed-dock tests (`tests/modules/nav/test_docked_state.py`, `test_missed_dock.py`) pass with only the listed edits.

**c7: delivery's one-shot finish moves.**
- `git mv core/flow/nav_oneshot_completion.py products/delivery/nav_oneshot_completion.py` (body unchanged).
- `nx`: `oneshot` (`:39,48`) becomes `finish_sortie: Callable[[], tuple[Exception, ...]]`; `:85` and `:91` call it,
  still before the error check (`:97`) and the DOCKED or DOCK_FAILED report (`:99-102`); import `:15` goes. The
  comment at `:82` ("disarm off the net") names the sortie finish instead. Docstring: "`finish_sortie` ends with the
  task teardown; `nth._enter_docked` relies on it for a one-shot DOCKED entry, contact or dock failed."
- App: a helper in `nav_delivery_composition.py` builds `OneShotCompletion` (was `nrc:107-117`). It is called inside
  `compose_nav_application`, which passes today's live closure `is_stopping=lambda: loop is not None and
  loop.is_stopping()` (`nrc:69,111-112,194`): `loop` is assigned later, so the closure must look it up when it runs.
  `nrc` passes the finish's `run`; the call is shorter than the 11 lines it replaces.
- Tests (path and constructor edits): `tests/modules/nav/test_nav_transition.py:13-14,111`,
  `tests/modules/nav/test_docked_state.py:146,425-426,464`, `tests/modules/nav/test_final_approach_docked_flow.py:243`,
  `tests/modules/nav/test_missed_dock.py:393` (import) and `:229-232,436-440` (pass `completion.run` as
  `finish_sortie`).
- New test (the failed path has no reset-once test): in `test_missed_dock.py`, the `_chain` harness with one reset
  Mock shared by `OneShotCompletion` and the handler asserts `reset.clear` runs exactly once, for one-shot and for
  cyclic. It does not record into `events`, so the pinned lists at `:464-465` stay.
- Help text `core/kernel/config/nav_args.py:71`. Data: note on `manifest.tsv:504`. Docs: `missed_dock.md:16,45` cite
  the RESET request and the one-shot file that c5 and c7 move; c7 updates them.
- Done when the gate is green and `tnc:7316-7726` (including `:7368-7372`), `test_nav_transition.py:118-186`,
  `test_docked_state.py` and `test_missed_dock.py` (including `:271-275` and the event order at `:464-465`) pass
  with only the listed edits.

**c8a: home-dock rules move** (see "Dock roles").
- `git mv dr core/skills/search/task_focus.py` (`TaskFocus`: `may_offer`, `may_take`, `dock_location`, `assigned`).
- New `products/delivery/home_dock.py` (`HomeDockRules`).
- `sel:8,23-35,53,64,71,76-78`: `TaskFocus`, `offer_allowed`, `order_work`. `sda:30,50-54`: `needs_own_orbit`, comment
  without "delivery". `core/flow/nav_constants.py:15-19`: neutral wording.
- App: `ntc:18,198,215,221-226,254-274` → `nav_delivery_composition.py`; `ntc` passes the built rules.
- Tests: `tests/modules/nav/test_dock_roles.py:23` import, `:337-346` construction, and the four unit tests
  (`:350-382`) re-targeted as in "Dock roles" (Q6).
- Done when the gate is green and `test_dock_roles.py` passes with no assertion change.

**c8: work source; the hub record and route move; docs; guard.**
- `nta`: `handle_new_poi` becomes `try_start(poi)` (`:62-87`); remove `peer_task` and `fallback_navigation` (12 → 10)
  and imports `:11,14`.
- `da`: `choose_work: Callable[[Optional[DetectedObject]], None]` replaces `navigation_task` (`:22,30`) at both calls
  (`:41,52`); import `:7` goes.
- New `products/delivery/work_source.py` (`nta:54-61,88-92` in order, calling `try_start`). `git mv cat` and `fbn`.
- App: `ntc:19-22,227-243`; `nav_composition_types.py:38-39,151-158`; `nav_foundation_composition.py:26`;
  `nav_delivery_composition.py` builds `DeliveryWorkSource`; `nrc:136-163` passes its `handle_new_poi`.
- Tests (path edits): 10 `handle_new_poi` calls (`tnc:1030,5848,8180,8650`,
  `tests/modules/nav/test_reask_after_timeout.py:375,385,399,416,432,531`); rig `:186-197`.
- Docs: `src/navpy/AGENTS.md` layer list; `.claude/agents/architect.md:22-27`, `explorer.md:24`, then
  `python scripts/gen_codex_agents.py`; the docstrings and nouns in the table above; manifest notes 484 and 519.
- Guard: `tests/guards/test_core_knows_no_product.py`.
- Done when the gate and guard are green, the DDH (`tnc:7727-7806`) and peer-priority (`tnc:7807-7940`) tests pass,
  and the swarm markers (`scripts/eval_gcs_demo_role_gates.py:16,19`, `scripts/eval_gcs_demo_fleet_gates.py:56`) are
  unchanged.

**c9: neutral mission markers (Q3).** As v4 (all cited lines unchanged): `core/ports/mission_marker.py`;
`mission_metadata.py:15,29-30,33-44,83,90,105-114`; `MissionCatalog.update(markers)`; consumers (after c4): the
start-up read in `app/flow/nav_composition.py` and `refresh_mission_metadata` in `nav_reset_composition.py`; `tests/modules/nav/test_mission_metadata.py:6,30-75,81-94,188-198`
read the hub through `MissionCatalog().update(meta.markers)`; a test that `HUB_MARKER_CODE` equals the plugin
constant; the guard's `mission_metadata.py` row goes; note on `manifest.tsv:486`. Done when the gate is green, the
delivery wall eval passes with the home UAV's "Default delivery hub navigation:" at its home dock, and the eval
identity is re-baselined (`scripts/simtime_navigation_identity.py:11-18`).

## Guards and tests
- **Layers:** `products/` becomes a real layer in c2. Refused: core→products, plugins→products, products→plugins,
  products→sim, sim→products. One exception row is removed (c4), none added.
- **Vision walk:** 162 roots, 0 violations, 0 stale rows on `77abb63`. v4's end-state simulation (163 roots) ran on
  `f744062`. As built, the end-state simulation was not re-run as a separate step before c6; c6's gate walk
  (`tests/guards/test_step1_vision_walk.py`) stood in for it: 0 violations, 0 stale rows, `products/delivery/flow.py`
  a root, on every commit from c6 on. Without the delivery-flow root, `exceptions.tsv:43` (`trk` → `geo_ref_calc`)
  goes stale.
- **SOLID** (guard rules, `tests/guards/solid_architecture_guard.py:18-23,949-953`; "After" values are projections):

  | Item | Limit | Now (`77abb63`) | After (commit) |
  |---|---|---|---|
  | `ApproachPlanner` (lines, methods, deps) | 200, 15, 12 | 154, 5, 6 | ~172, 6, 8 (c4) |
  | `approach_planner.py` | 300 | 181 | ~199 (c4) |
  | `nav_foundation_composition.py` | 300 | 265 | ~269 (c4) |
  | `create_nav_application` | 150 | 136 | ~142 (c4, c6-c8a) |
  | `compose_navigation_task_workflows` | 150 | 149 | ≤149 (c8a moves `_dock_roles` out; c8 swaps args) |
  | `compose_nav_application` | 150 | 150 | ≤150 (c6 renames one line at `nrc:125`; c7 replaces the 11-line block `nrc:107-117` with a shorter call; c8 swaps args) |
  | `track_recovery.py` | 300 | 292 | ~283 (c5) |
  | `navigation_task_reset.py` | 300 | 277 | ~279 (c4, c6) |
  | Review / `NavigationTaskAction` / `NavTransition` deps | 12 | 7 / 12 / 7 | 9 (c5) / 10 (c8) / 7 (c7) |
  | `NavigationDecision` / `DeliveryFlow` / `NavStateDecision` / `RecoveryAction` deps | 12 | 8 / – / 8 / 6 | 5 (c6) / 5 (c6) / 6 (c5) / 5 (c6) |
  | `DetectAction` / `PoiSelector` deps | 12 | 7 / 4 | 7 (c8) / 6 (c8a) |

  `approach_planner.py` and `navigation_task_reset.py` come under the module limit when c4 and c6 first edit them
  (material against the pinned base). Moved files count as new against the pinned base, so every definition must be fully annotated
  (`solid_architecture_guard.py:333-342,493-549`).
- **Pure vision:** delivery reads only the mode, the next waypoint, the mission state, core's results and a
  detection's offer position (outside the final approach). The hub flight uses GPS outside the final approach, which
  the owner allows. The arming latch reads the armed bit and a param; neither feeds a command. The vision walk covers
  the delivery flow.
- **Test edits:** about 68 path and constructor edits (c3-c8a; 8 of them in `test_missed_dock.py`), three data
  pins, the owner's rename (c2), the docked-flow harness routed through the real consumer (c5), one `method_calls` →
  `mock_calls` check (c6), one new failed-path reset-once test (c7), c8a's 4 re-targeted unit tests (Q6) and c9's 6
  tests (Q3). No expected value changes, nothing is skipped, and no check
  becomes weaker.
- **Eval markers, byte-identical, in `uav_*_navigation.log`:** every marker in
  `scripts/eval_gcs_demo_gates.py:140-161` (in order: "confirmed by ground station." < "INIT: NAV MODE" <
  "RESET: PASSED POI" < "SNAP(VISION-NAV", plus "start_tracking obj_id=" and "Sending confirm request for P");
  "Self-detect orbit:", "CONFIRMING: P", "Task offered:", "Peer navigation started", "Task N assigned by owner M",
  "resuming AUTO mission", "Default delivery hub navigation:", "STATE DOCKED:" (`nx:131`) and
  "STATE DOCK_FAILED: reason=" (`nx:118`); `SIM_SPEEDUP=` order (`scripts/eval_gcs_demo_audit.py:112-175`); the fatal
  markers (`scripts/eval_ideal_three_uav_vehicle.py:32-41`). In `uav_*_navigation_debug.csv`, the `DOCK_ARMED`,
  `DOCKED` and `DOCK_FAILED` event rows, whose fields and order the evals check
  (`scripts/eval_gcs_demo_evidence.py:221,234,242-251`, `scripts/eval_gcs_demo_docked.py:97-110`); c7 keeps the row
  write (`nx:113`) after the cleanup and after `finish_sortie`.
- **Evidence level:** unit tests, the lockstep identity run and the delivery wall eval (SITL, sim only; not physical
  docking). No GCS or Lua
  change, so no npm run, live-tester run or Lua deploy.

## What pickup reuses
- **Skill results:** `ApproachOutcome`, `ReleaseOutcome`, the "review closed" flag, `presence.update` meaning "aborted".
- **Entry points:** `try_start` and `start`, `DetectAction(choose_work)`, `NavigationDecision(flow)`, `PoiSelector`'s
  `offer_allowed` and `order_work`, `needs_own_orbit`.
- **The ten product call positions**, each with pickup's own function (for example RTL as its finish).
- **Mission data:** `plan_orbit_approach` and the band, app's metadata read and the neutral markers.
- **Platform:** DOCKED and the failed-dock end (or a later "caught" contact state, if pickup needs one), `TaskFocus`,
  the layer rule and the arming specification. A one-shot failed dock runs the product's finish, as a contact dock
  does.
- **Without a core edit:** its own `decide(mode)` behind core's holds; its own work order and offer rules; its own
  floor and one-shot setting through app.
- **For pickup's design, with the owner:** a missed-pass retry through NAV → CONFIRM is still possible mechanically
  (only the RESET and DOCKED entries clear the task, `nth:62,129-137`; the CONFIRM entry only resets the gate and
  starts the actor, `nth:60,106-108`), but in sim it skips the failed-dock end, which todo 74 made the rule. Whether
  that rule binds every product is pickup's question, not this step's.
- **Still a core edit, each made generic when pickup adds it:** a second detector class (Q5); an optional CATCH task
  kind; the arming port and hold; role bids (R23); a pickup-only state if its design needs one; a line zone in the
  mission codec (`mission_encoding.py:11-13`).

## Not in this step
- GCS screens; building the arming latch, the Lua param and its GCS line.
- Routing `is_oneshot` (`nrc:119`) through the chosen product.
- The sim step: the speedup lease, the disarm, a sim-only restart.
- The approach-law contract for contractors; the C3 vehicle port (`manifest.tsv:486`).
- Per-class target rules (Q5) and role bids.
- Pickup-step text outside core: the floor default 150 (`nav_args.py:10`, `scripts/lua/aas_params.lua:16`), the Lua
  comment `aas_params.lua:20`, the GCS label "Disarm after attempt" (`src/gcs/frontend/src/components/settings/paramSections.js:84`),
  the GCS `Literal["dock"]` (`src/gcs/backend/routes/vision_profiles.py:51`) and the dialect text in `navlink.xml`
  (untracked; UNVERIFIED).
- The operator fence on AUTO writes (todo 12); dock-role estimate limits (todo 56).
- The latent `TypeError` at `fbn:55`, and the unused imports already in `nd` (`:6,8,11`).
- Drafts owned by other chats that still say `missions/` (`docs/design/architecture_final.md:18`), and
  `docs/design/hitl_final_approach.md:94,106`, which cite the restart and one-shot files c6 and c7 move; their owners
  update them.
- Re-offering a failed dock's task to a peer (todo 75): swarm or product, when the owner rules.

## Owner questions (recommended default in bold)
Q6 and Q7 were answered on 2026-10-19 as recommended. Q1-Q5 were answered on v4 (2026-10-12) and stand: Q1 delete the dead refresh (yes); Q2 no change now; Q3 take the hub
choice out of the plugin (yes); Q4 arming as specified, built with pickup; Q5 a second target class with pickup.

6. **e5's dock rules: where does each go?** **Recommended: the assigned-task focus (ruling 5) stays in core as a
   swarm rule; the home-dock rules (todo 29, ruling 4) move to delivery.** Four unit tests move each assertion to the
   object that owns the rule, with the same inputs and expected values. Alternative: move all of `DockRoles` to delivery; then pickup must copy ruling 5.
7. **Which runtime checks gate which commit?** **Recommended: the lockstep identity run (about 75 s, load-tolerant)
   on every commit from c3 to c9, and three quiet-PC delivery wall runs: a forced-miss run after c7 (the moved finish
   runs on a failed dock), and a normal run after c8 (with c8a) and after c9 (the hub and home-dock paths, which
   lockstep cannot fly yet).** The todo 74 green and forced-miss runs on `feb2b19` are the wall references. The
   forced-miss run passes when every UAV logs "STATE DOCK_FAILED", none logs "ONHOLD: WAYPOINT 1" or a new
   confirmation after it, and each disarms; the eval's own verdict is red by design. Alternative: v5's wall runs
   after c6, c7, c8 and c9 and no lockstep (four quiet-PC runs, no runtime check for c3-c5).

## Risks
1. **Statement order.** Every moved request, log and resume keeps its position; the moved-code tests and the wall
   eval cover it.
2. **Late lookup.** Only the seven bindings named above are fixed at build, and nothing patches them.
3. **`presence.update` returning True means "aborted" only implicitly** (`trk:98,119,190,233,261,272`): a docstring.
4. **SOLID headroom.** On `77abb63` `compose_nav_application` was at 150/150 and `compose_navigation_task_workflows`
   at 149/150. As built (c9) they are 143/150 and 147/150, and `create_nav_application` 138/150.
   `nav_delivery_composition.py` takes the moved delivery wiring and the stage composers call its helpers; if a stage
   still grows, it gets a named helper, never a looser limit.
5. **Test edits.** About 68 path and constructor edits, one new test, three data pins, the c5 harness, c8a's 4 and c9's 6 tests;
   reviewers confirm none drops a check.
6. **Baseline and checkout.** The test baseline is UNVERIFIED; build in a temp worktree.
7. **Finish contract.** Core's DOCKED entry relies on `finish_sortie` ending with the task teardown (`nth:129-137`),
   for a contact and for a failed dock. It is a docstring plus tests, not a type: `test_docked_state.py:224,503`
   (contact) and c7's new failed-path test.
8. **Arming (later).** A one-tick hold across the arm edge; cache freshness after a GCS write is UNVERIFIED.
9. **The decision splits in two (c6).** The DOCKED hold, guard and recovery stay in core and run as today; 129
   unchanged `decision.decide()` tests and the DOCKED and one-shot tests cover it.
10. **Guard words.** A banned word could hit a future generic use; the fix is a better word or a named allow row.
11. **Vision walk.** The v4 end-state simulation predates DOCKED and the failed-dock end. It was not re-run as a
    separate step; c6's gate walk gave 0 violations and 0 stale rows, so c6's data edits stood.
12. **Wall eval timing.** Each wall run needs a quiet PC (todo 65); loaded runs are not a gate. If
    `deterministic_dev_sim.md` step 14 removes the wall route before c9, c9 waits until the delivery scenario runs in
    lockstep, or the owner keeps the wall route for it.
13. **Lockstep identity for a pure move: settled 2026-10-19.** Two runs and a probe move on `869d5a4` gave
    `fbda9e41…`, and so did each commit's own run from c3 to c9 (see Commits). Each commit's own run stays the gate;
    one passing probe does not prove a later move keeps the digest.
14. **HITL edits the same files.** `hitl_final_approach.md` steps 22, 23 and 25 change `sda`, `approach_planner.py`,
    `peer_orbit_plan.py` and `rel` (`:505-508`). Whichever lands second rebases, re-measures the `ApproachPlanner`
    rows and re-checks the cited lines.

## Review handling
v4's review history (five audits, Codex on v1, v3 and v4, the checker on v2) is in `779694e`.

Codex on v5, round 1 ("ship with changes"; each point checked against the code):
1. c3, c5 and c6 missed direct DOCKED test harnesses (`test_docked_state.py:265-282,325-348`,
   `test_final_approach_docked_flow.py:96-128`). **Holds.** c3, c5 and c6 now name them; the docked-flow harness runs
   the real outcome consumer, so its phase, status and POI-clear assertions keep checking behaviour. Also found while
   checking: `method_calls` would let a called `flow` Mock pass, so c6 uses `mock_calls`.
2. c8a's "construction change only" did not fit the split interfaces. **Holds.** Each assertion moves to the owning
   object; the combined offer check runs through `PoiSelector`; no facade.
3. c7 left the live `is_stopping` closure unspecified. **Holds** (`nrc:69,110-111,193`). The helper is called inside
   `compose_nav_application` with today's closure.

Codex on v5, round 2 ("ship with changes"; the first attempt could not read the repository, so the doc and code were
inlined): the round-1 fixes are sound and weaken no test. One point:
1. c6 missed the `TransitionPorts` harnesses in `test_docked_state.py:207,245,253,472`. **Holds.** Added as path
   edits. Not a material design change, so no third round.

v5.1 re-audit on `77abb63` (seven citation checkers, three impact readers and a critic; each point checked against the
code before it was applied):
- `feb2b19` moved every `nx` citation, made the first one-shot finish run for a failed dock too, added
  `test_missed_dock.py` harnesses to c6 and c7, put `compose_nav_application` at 150/150 and added the
  "STATE DOCK_FAILED" marker. All applied above; the failed-dock end is classed as platform, and c7 gains a
  failed-path reset-once test.
- The approved deterministic sim plan makes lockstep the route for simulated checks, so Q7 now proposes a lockstep
  identity run per commit and three wall runs.
- Older slips fixed: `NavigationDecision` ends with 5 dependencies, not 4; the work order is `nta:54-61,88-92`;
  `nta:96` is the speedup apply; c3 and c4 split `nav_decision_composition.py:101-102` correctly; the reset-once test
  names; guard hits `dr:14` and `nav_constants.py:19`.
- Rejected after checking: moving `orbit_geometry.py:13` to `:5` (`:5` is the AGENTS.md disclaimer, kept on
  purpose); re-citing c9's consumers to today's code (c4 moves the first read into `nav_composition.py`); `nth:62` →
  `:60` in the pickup line (`:62` is the RESET entry a CONFIRM retry avoids; both are now cited).

Codex on v5.1, round 1 ("ship with changes"; repository reads failed, so it used the inlined doc and code):
1. The arming spec implied a re-arm starts a new sortie after a one-shot dock. **Holds** (`nd:239-242` returns
   before the disarm block). Not a code bug: `docked_state.md:42` makes a one-shot DOCKED final for the process. The
   arming section now says so, and leaves a re-arm reset to pickup's design with a lifecycle test.
2. Two runs on the parent prove repeatability, not that a move keeps the digest, and a mismatch silently dropped
   lockstep. **Holds.** Before c2 there is now a repeatability check and a probe move, with the compared set fixed;
   a mismatch is a determinism bug that stops the plan for an owner decision.

Codex on v5.1, round 2 (the changed text and code inlined): no material problems; **approve**.
