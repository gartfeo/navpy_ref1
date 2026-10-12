# Delivery as a product (design v4, 2026-10-12)

**Status:** v4 approved by the owner on 2026-10-12, with the recommended answers to Q1-Q5 and Q4 as revised for
"safety after the demo". The owner dropped c1 (pin tests: "current delivery tests not needed"). Order: commit this
doc (c0), replace the 3-UAV eval with the owner's delivery scenario (search the area, offer extra docks, go home
when none is found) and make it green, then c2-c9. G0 runs after the move.
- Codex reviewed v1 ("rework") and v3 ("ship with changes"; changes applied). A checker reviewed v2 ("rework"); v3 answers it.
- v4 applies the owner's ruling on v3: "Core and swarm should not know about missions we have, so SOLID principles
  should be kept." Five audits of core, swarm, link, perception, kernel and plugins, each checked by a second
  reviewer, found what v3 still left behind (see "Core and swarm know no product"). Codex reviewed v4: "ship with
  changes"; all seven applied (see "Review handling").
- Owner ruling 2026-10-12: safety work comes after the demo; ArduPlane's own safety is enough until then. This step
  adds no safety work. The restart and forced-arm holds of roadmap R22 move to after the demo.
- This design is based on main `f744062`. Nothing is built yet.
- Paths are under `src/navpy/` unless they start with `tests/`, `scripts/`, `docs/` or `src/gcs/`. App files are in `app/flow/` unless a path says otherwise.

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
| `rig` | `tests/modules/nav/nav_test_rig.py` |
| `tnc` | `tests/modules/nav/test_nav_controller.py` |

**Changes from v3** (the owner's ruling)
1. **Today's holds stay in core.** Core's decision stage (`NavigationDecision`, which stays in core) runs today's
   holds (disarm, manual mode, search waypoint, altitude floor) unchanged, then calls the product's `decide(mode)`.
   A product cannot skip them (c6). No new safety work.
2. **Core never says "product".** Core's ports get neutral names: `flow_task_active`, `clear_flow_task`,
   `return_to_search` (was `restart_auto_mission`) and, with pickup, `sortie_ready`.
3. **One more delivery choice leaves core:** clearing the POI after a failed approach (`nd:151`, c5). Recovery also
   stops reading the floor and the one-shot flag itself; app passes them in, so each product can later have its own
   floor without a core edit (c6).
4. **A guard test keeps it so** (c8): product words are refused in `core/` and `plugins/`.
5. **Delivery wording leaves core's docstrings and help text** (c5-c9).
6. **One new question (Q5):** when core learns a second target class. The dock is a target name, not a product word;
   pickup flies to a line.

**Changes from v1** (v2 answered Codex on v1; v3 answers the checker on v2)
1. **(v2) Delivery's choices leave core and the plugin.** Four choices move: the restart at item 1 (`rec:89-91`), the rewind (`ntr:243-245`), the geo-abort resume (`trk:74-80`) and the choice of hub marker (`plugins/vehicle/ardupilot/mission_metadata.py:110-112`). Each is still called at the same point as today.
2. **(v3) The one-shot finish moves too, in a new commit c7.** This is what a finished one-shot sortie does. The sortie-done latch, the redirect and the hold stay in core, because they are platform.
3. **(v3) Choosing the product at arming is now exact:**
   - core detects the arm edge in its loop;
   - one app start-up file holds the latch and the only product check;
   - core reaches the chosen product through the same call positions;
   - the read never waits;
   - a sortie with no chosen product holds.

   It is still built with pickup. The restart-in-flight and forced-arm holds of roadmap R22 come with the safety work after the demo.
4. **(v2, v3) Every commit is complete on its own.** v3 also removes the dead `approach_kind` parameter in c4 and updates manifest row 539 in c6.
5. **(v2, v3) Orbit sizing moves once, in c4, and each call looks up its target when it runs.** v3 names the four calls that are bound at build time on purpose.
6. **(v3) The scan band is read as a local in `create_nav_application` and passed to the planner.** No stale copy is kept, and the planner's docstrings are updated.
7. **(v3) Two pins are fixed.** c1(a) allows the teardown refresh on the tick that enters ONHOLD. c1(f) adds the identity re-bind abort (`trk:190`).
8. **(v3) Docs.** `missions/` is renamed in the roadmap, architecture_split and flow_runner docs (c2). One core docstring and one citation are fixed.
9. **(v2, v3) The vision root is concrete.** A re-run of the simulation that includes the one-shot move gives 163 roots, 0 violations and 0 stale rows. `scripts/step1_move.py --check` prints "no leftovers" on `f744062`.

## Goal
- **Where delivery's choices live.** All of delivery's on-board choices live in `products/delivery` as plain code:
  - what happens after each result;
  - the order of work;
  - the hub;
  - restarting and resuming the search;
  - what a finished one-shot sortie does.
- **What core keeps.** Core keeps the loop, the safety holds, the sortie-done latch, the transition hooks and every skill. Core never imports `products/`.
- **Core and swarm know no product.** At the end of this step, no name, docstring, log text or rule in `core/`
  (swarm and link included) belongs to delivery or to any other product. Core offers results and plain call
  positions; app plugs the product in. A guard test keeps it so.
- **Behaviour.** Behaviour stays identical: the same calls, in the same order, with the same log lines. The GCS and Lua do not change, and nothing in this step is a CHANGE.
- **Arming.** Choosing the product at arming is specified here and built with pickup. Its two CHANGEs are owner questions (Q4).

## The picture
```
products/delivery/                 delivery's choices; imports core only
  flow.py                          DeliveryFlow.decide(), restart_search(), interrupted_leg()
  poi_status_decision.py           POI review: give way to a peer task, fly once confirmed,
                                   resume the AUTO search when closed or aborted
  work_source.py                   DeliveryWorkSource: peer task, own POI, then hub
  mission_catalog.py               hub record; picks the hub among the mission markers
  fallback_mission_navigation.py   flight to the hub
  nav_oneshot_completion.py        what a finished one-shot sortie does
core/      loop, safety holds, sortie-done latch, transition hooks, skills
plugins/   vehicle/ardupilot decodes mission markers without interpreting them
app/       builds core and delivery, wires them, reads mission metadata
           (with pickup: product_choice.py latches the product at arming)
sim/       unchanged
```

## How it works
### Each tick
- Core's loop is unchanged (`nl:124-130`): sense or clear, decide, act. `act()` runs every tick, as today.
- Core's decision stage (`NavigationDecision.decide`, kept in `nd`) reads the mode and next waypoint and runs the
  safety holds in today's order (`nd:231-236`): the guard, then recovery. Only when neither acts does it call one plain
  function: delivery's `decide(mode)`.
- `decide(mode)` runs the rest of today's order (`nd:237-241`): the approach, then the POI review.
- Core does the work and reports what happened. Delivery then chooses what comes next.

| Core reports | Delivery does (today's line) |
|---|---|
| Approach abandoned (GUIDED lost or not accepted) | DETECT (`nd:163,178`) |
| Approach passed | Logs "RESET: PASSED POI", then RESET (`nd:195-196`) |
| Approach failed | Clears the POI, then RESET (`nd:151-152`) |
| POI confirmed and ready | NAV (`rel:45`) |
| Review closed (rejected, timed out, dwell or re-acquire limit reached) | Resumes AUTO, then logs "REJECTED: resuming AUTO mission" (`rej:103-110`) |
| Geo hold aborted | Logs "GEO-HOLD abort: … resuming AUTO mission", then resumes AUTO (`trk:74-80`) |
| Peer task pending while not in NAV | Hands off its own POI, then DETECT (`psd:51-56`) |
| RESET or RECOVERY goes to DETECT | Restarts AUTO at item 1 (`rec:89-91`) |
| AUTO resume | Picks the interrupted leg, one item back (`ntr:243-245`) |
| DETECT with no active POI | Picks a peer task, then its own POI, then the hub (`nta:54-92`) |
| One-shot NAV exit after a pass | Its finish: a sim disarm or a real-autopilot warning, then task teardown (`core/flow/nav_oneshot_completion.py:32-56`, called at `nx:58-59`) |
| New mission markers | Decides which marker is the hub (`mission_metadata.py:110-112`) |

Core still picks these states:
- **ONHOLD** when disarmed, in a manual mode, or before the search waypoint (`nd:75-94`);
- **ONHOLD after a one-shot finish:** core redirects (`nx:58-63`) and holds (`rec:73-77,110-115`);
- **RECOVERY** at the altitude floor;
- **DETECT** after RESET or RECOVERY;
- **CONFIRM** while a review is open;
- **DETECT** when a review or a geo hold is torn down.

### The calls core makes into delivery
App gives core plain functions. Each one is called exactly where today's code runs, so the order of calls does not change.

| Core call (position kept) | Wired to | Target looked up |
|---|---|---|
| `NavigationDecision(flow)` (`nd:237-241`), called after the guard and recovery | `DeliveryFlow.decide` | at build |
| `DetectAction(choose_work)` (`core/skills/search/detect_action.py:41`) | `DeliveryWorkSource.handle_new_poi` | at build |
| `InactiveGuardPorts.flow_task_active` (`nd:73`, still the last operand) | `lambda: hub.default_delivery_hub_active` | at call |
| `clear_flow_task`: state-reset step "mission" (`ntr:101`) and resume (`ntr:260`) | `lambda: hub.clear_navigation_task()` | at call |
| `TransitionPorts.return_to_search` (renamed from `restart_auto_mission`), the RESET/RECOVERY → DETECT pair hook (`nth:23,61-62`) | `lambda: restart_search(vehicle)` | at call |
| `AutoMissionResumePorts.resume_waypoint` (`ntr:243-245`) | `interrupted_leg` | at build |
| `NavTransition(finish_sortie)` (`nx:59`) | `OneShotCompletion.run` | at build |
| `NavigationTaskResourcePorts.refresh_mission` (`ntr:168`, exists today) | App's `refresh_mission_metadata`: reads the plugin, gives the band to the planner and the hub to delivery | at call |

- **`hub`** is delivery's `MissionCatalog`. The first seven calls are product calls. The eighth, the refresh, is app code that feeds data to the planner and to delivery.
- **Names.** Core's side of each call says what core needs ("a task the flow owns is active", "return to search"),
  never what delivery does with it. Only app's wiring names delivery.
- **Recovery inputs.** `RecoveryPorts` gains `min_alt` and `is_oneshot`, wired by app to `lambda: args.min_alt` and
  `lambda: args.is_oneshot`. Recovery no longer holds `NavArgs` (`rec:38,44,59,66,72,75,111`). The same params are read
  at the same points, so nothing changes now; with pickup, app can give each product its own floor (owner ruling:
  floor per product) without a core edit.
- **Calls looked up when they run.** Tests replace `auto_resume.run`, `plan_orbit_approach`, `clear_navigation_task` and the band on the objects themselves (`tnc:1790,1831,5181-5185,5242-5254,6708`). Because these calls look up their target when they run, those tests keep checking what they check today.
- **Four calls bound at build, on purpose:** `flow`, `choose_work`, `resume_waypoint` and `finish_sortie`.
  - No test patches any of them (grep).
  - The rig reaches the flow and the work source through `__self__` (`decision._flow.__self__`, and rig `:186` after c8).

### Mission metadata: who decodes it, who interprets it
- **The plugin decodes, without giving meaning** (`plugins/vehicle/ardupilot/mission_metadata.py`). It decodes:
  - the search pattern (`:113-114,125`);
  - the corridor/scan band (`:47-79,116-122,127`);
  - every DO_SET_ROI_LOCATION marker, as `MissionMarker(code, location, location_type)` (`:105-112`).

  From c9 on, it no longer asks "is this the hub?" (`:110`).
- **Core holds the neutral record** in `core/ports/mission_marker.py`. Plugins may import `core/kernel` and `core/ports`; products may import any part of core (`scripts/step1_manifest.py:240,245-248`). Neither may import the other.
- **Delivery interprets the markers** in `products/delivery/mission_catalog.py`:
  - `HUB_MARKER_CODE = 3` is the wire code (`plugins/vehicle/ardupilot/mission_encoding.py:52`). Products cannot import plugins, so a test checks that the two constants are equal.
  - `update(markers)` keeps the last hub marker, as the plugin's loop does today (`:105-112`).
- **App connects the two.** It reads the plugin, gives the band to the planner and the markers to delivery. App may import both (`scripts/step1_manifest.py:236`).
- **Unchanged.**
  - The codec `mission_encoding.py` stays as it is, because the GCS imports it (`src/gcs/backend/planner/mission_items.py:15`, `src/gcs/backend/planner/waypoint_builder.py:16`). `src/gcs/backend/mission_validator.py:8-11` gets the same names through waypoint_builder's re-export.
  - `read_default_delivery_hub` (`:33-44`) has no user in `src`; c9 deletes it.
  - Until c9, app passes the plugin's hub fields straight to `MissionCatalog.update(hub, hub_type)`.

### Choosing the product at arming (specified now, built with pickup)
- **Setting.** `AAS_MOD` is an integer; `0` means delivery and is the default.
  - It is a Lua AAS param at the next free index, 20, in `scripts/lua/aas_params.lua`. Indices 1-19 are in use and the table holds 30 (`:3`).
  - It also needs its line in the GCS map (`src/gcs/backend/aas_params.py:17-37`). `tests/scripts/test_aas_params_lua.py:30-34` fails unless both have it.
  - It is not in `NavArgs`. NavArgs re-reads every param at each task reset (`ntr:167`, `core/kernel/config/nav_args.py:79-92`), and the product must not change mid-sortie.
- **The read never waits.** At the arm edge app calls `vehicle.get_parameter("AAS_MOD", timeout=0.0, quiet=True)`:
  - a cached value comes back at once (`plugins/vehicle/ardupilot/parameter_reader.py:42-43`);
  - every PARAM_VALUE the link hears updates that cache (`plugins/vehicle/ardupilot/inbound_router.py:116`);
  - on a miss, it sends one request and returns None without waiting (`parameter_reader.py:56-73`, `message_store.py:135-137`), and it does not ask again for 10 s (`parameter_reader.py:16,44-45,85`).

  App fills the cache once at start-up with the normal read, as NavArgs does for its params (`nav_args.py:40`).
- **Arm edge, in core.** `NavigationCycle.run` already reads `vehicle_armed()` once per tick (`nl:125`).
  - It gains one port, `armed_changed(armed)`. The port is called before sense and `decide`, on the first tick and whenever the reading changes.
  - Core makes no product choice here.
- **Latch, in app.** A new start-up file, `app/flow/product_choice.py`, holds three things.
  - `choose_product(value)` is the only product check in `src/navpy`. `None` or `0` gives delivery's calls; any other value gives none.
  - `Sortie.armed_changed(armed)` decides what each reading means:
    - **Disarmed:** the sortie ends (`ready = False`). The calls stay on the last product, so the disarm tick's teardown (`nd:75-77`) still runs through it.
    - **Armed after a disarmed reading:** it reads `AAS_MOD`, latches the chosen calls and sets `ready = True`. The status text is "SORTIE: delivery (AAS_MOD=0)".
    - **Unknown value:** `ready = False`, with a status text that names the value. The calls keep the last valid
      record, so the guard and teardown still run; only mission work is blocked.
    - **Armed on the very first tick** (NavPy started or restarted in flight): it latches as on an arm edge, so
      NavPy flies on as today (Q4b). The hold comes with the safety work after the demo.
  - `ProductCalls` holds the seven product calls from the table above. Each product gets one record, built at start-up.
  - **Start state:** `sortie.calls` starts as delivery's record and `ready = False`, so the loop always has valid
    calls and the readiness guard (not a missing record) is what holds. Tests: start-up while disarmed, start-up while
    armed, unknown value, disarm teardown, re-arm with a new value.
- **Hold, in core.** `InactiveGuardPorts` gains `sortie_ready()`.
  - The guard checks it right after its disarm block (`nd:75-81`) and before the mode block (`nd:82`).
  - If it is False, the guard clears any active task, reports "ONHOLD: SORTIE NOT READY" and requests ONHOLD.
  - The autopilot keeps its own mode, mission and failsafes.
- **How core reaches the chosen product.**
  - App wires each of the seven product call positions to `lambda …: sortie.calls.<name>(…)`. Positions, call order and core stay as in this step.
  - The metadata refresh gives the markers to every product that was built.
  - Nothing registers into `sortie`, and neither core nor any product sees it (Q4e).
- **Built at start-up.** All products are built at start-up on the one core; today that is only delivery.
  - The controller is already built before arming (`app/runtime_composition.py:181`), and its loop runs while disarmed (`nl:124-130`).
  - Building at arming would mean rebuilding the loop thread, the network listener and the state owners while the aircraft arms.
- **Re-arm.** Every arm edge reads the value again, and a new value applies to the next sortie without restarting NavPy. Edits made while armed are ignored.
- **Forced arm.** NavPy cannot refuse arming (`docs/design/flow_runner.md:192`), and it reads only the armed bit (`app/flow/nav_runtime_composition.py:160`).
  - A forced arm (`src/gcs/backend/routes/control_vehicle_commands.py:74-90`) therefore counts as an ordinary arm edge: an unknown value holds and a known value flies (Q4c).
  - `roadmap.md:107` says a forced arm holds; that waits for the safety work after the demo, and the note goes into
    `roadmap.md` and `flow_runner.md` in that step.
  - UNVERIFIED: whether the autopilot reports a forced arm in a way NavPy could read.
- **Behaviour changes.** They all come with pickup; none is in this step.
  - **CHANGE 1:** an unknown `AAS_MOD` holds the sortie (Q4a).
  - No restart hold: a NavPy restart in flight flies on, as today (Q4b). The hold of `roadmap.md:107` and
    `flow_runner.md:192-193` comes with the safety work after the demo.
  - A missing value flies delivery, as today (Q4d).
- **Guard test, with pickup.** Inside `src/navpy`, only `app/flow/product_choice.py` names `AAS_MOD` or compares product values.
- **Why it is not built now:**
  1. The Lua param needs the GCS map line, and the GCS is untouched in this step.
  2. With one product there is nothing to choose between, so the latch would be dead code.

  It lands as the first commit of pickup's platform work (roadmap R22, `roadmap.md:55`), together with the param, the GCS line, `npm test` and `npm run build` (`roadmap.md:108`).
- **Gate for that step:** an integration test where the GCS writes `AAS_MOD`, NavPy's cache sees the new value, and the
  next arm edge latches it. If GCS writes do not reach NavPy's link, NavPy requests a fresh read on disarm through the
  existing param path, so the value is current before the next arm; the loop never waits.
- **UNVERIFIED, to check in that step:**
  - whether one "ONHOLD: SORTIE NOT READY" text appears when the loop and the guard read the armed bit on opposite sides of the arm. That one-tick hold changes no state, because the phase is already ONHOLD while disarmed.

## What moves
| Today | After | Commit |
|---|---|---|
| The approach and review half of `NavigationDecision.decide` (`nd:237-241`) and its outcome mapping | `products/delivery/flow.py` `DeliveryFlow`; the guard-and-recovery half stays in core | c6 (c3 first deletes the dead refresh, `nd:243-252`) |
| Next state after the approach (`nd:152,163,178,195-196`) | `DeliveryFlow.decide` maps `ApproachOutcome` | c5 writes it in core; c6 moves it |
| `psd` (peer yield, review dispatch) | `products/delivery/poi_status_decision.py` (`git mv`) | c6 |
| Confirmed means fly (`rel:45`) | The review requests NAV on `ReleaseOutcome.READY` | c5, c6 |
| Resume and its log after a closed review (`rej:103-110`) | The review | c5, c6 |
| Resume and its log after a geo abort (`trk:74-80`) | The review, right after the abort returns | c5, c6 |
| Restart AUTO at item 1 (`rec:89-91`) | `restart_search(vehicle)`, in the same pair hook | c6 |
| Rewind to the interrupted leg (`ntr:244-245`) | `interrupted_leg(next_waypoint)`, at the same statement | c6 |
| One-shot finish (`core/flow/nav_oneshot_completion.py`) | `products/delivery/nav_oneshot_completion.py` (`git mv`), still called at `nx:59` | c7 |
| Work order (`nta:54-67,88-92`) | `products/delivery/work_source.py` | c8 |
| `cat` | `products/delivery/mission_catalog.py` (`git mv`) | c8 (c4 first removes its reader and band) |
| `fbn` | `products/delivery/fallback_mission_navigation.py` (`git mv`) | c8 (c4 first removes its orbit sizing) |
| Hub-marker choice in the plugin (`mission_metadata.py:33-44,110-112`) | `MissionCatalog.update(markers)` | c9 (Q3) |

**Generic parts split out of delivery (they stay in core or app):**
- **Orbit sizing and scan band.** Orbit sizing (`fbn:93-105`) and the scan band (`cat:18,24`) go to `ApproachPlanner.plan_orbit_approach` and its attribute `scan_altitude_rel`. The peer and self-detect approaches use them too (`app/flow/nav_task_composition.py:147-152,197-202`).
- **Metadata read.** The metadata read (`cat:20-24`) goes to app.
  - The first read stays the first step after `compose_state_ownership` (`app/flow/nav_composition.py:61-62`). It runs before any other composition and before the "Approach strategy:" log.
  - Each task reset reads again through the existing port (`ntr:168`).
- **Plugin.** The plugin decodes every mission marker without interpreting it (c9).

**Core edits (types, return values and ports only):**
- `NavStateDecision.advance` returns `ApproachOutcome` and makes no requests; it no longer needs `status`. On a
  failure it returns FAILED without clearing the POI (`nd:151`); FAILED means "the task failed", in any phase (`nd:139`).
- `ConfirmedPoiRelease.apply` returns `ReleaseOutcome`.
- `PoiRejectionExit.apply` returns True when the review is closed, and no longer resumes.
- `TrackRecovery`'s abort stops at the DETECT request (`trk:73`).
- `flow_task_active` and `clear_flow_task` replace the catalog in `nd` and `ntr`.
- `AutoMissionResumePorts.resume_waypoint` replaces the inline rewind.
- `RecoveryAction` loses `restart_auto_mission` and `NavArgs`; `RecoveryPorts` loses `set_mode` and `restart_mission`
  and gains `min_alt` and `is_oneshot`.
- `TransitionPorts.restart_auto_mission` is renamed `return_to_search`.
- `NavTransition` takes a `finish_sortie` function instead of the one-shot object.
- `NavigationDecision` stays in core with its ports (`vehicle_mode`, `next_waypoint`), the guard and recovery, and
  takes `flow: Callable[[FlightMode], None]` instead of `nav` and `poi_status` (dependencies 8 → 4 after c3 and c6).
  `NavigationCycle` does not change. `DetectAction(choose_work)` takes a plain function.
- `NavStateDecision` loses `status` (c5) and the confirmation manager, whose only use was the POI clear at `nd:151`
  (dependencies 8 → 6).
- `NavigationTaskAction` keeps `start` and gains `try_start` (`nta:62-87`); its dependencies drop from 12 to 10 (the en-route gate keeps three).

**Stays in core, and why:**
- **Peer-task mechanism** (`pending`, `hand_off`), the en-route gate (ignore an own POI while flying to a peer's task,
  until near it or in the final approach, `nta:62-67`), the own-POI admission gates (status, re-ask and cooldown,
  `nta:68-80`) and `start`: these are swarm rules, the same for every product. Only the order of work belongs to delivery.
- **Completion classification** (`NavStateDecision`, `MissionPassPolicy`): it is on the vision path.
- **`rec`:** the floor, the RESET/RECOVERY → DETECT requests and the one-shot hold.
- **The sortie-done mechanism:**
  - the latch `NavPhaseState.oneshot_completed` and its clear at disarm (`nd:78`);
  - the ONHOLD redirect (`nx:58-63`);
  - the hold (`rec:73-77,110-115`);
  - `AAS_NAV_ONESHOT` in `NavArgs`.

  Roadmap R19 lists the sortie-done latch as a platform piece (`roadmap.md:52`). `docs/design/architecture_split.md:171` keeps the latch in `NavPhaseState` and the param in `NavArgs`.
- **The geo-hold mechanism and its abort teardown:** only the resume and its log line move.
- **The `AutoMissionResume` teardown and its AUTO write:** delivery only chooses when to resume and to which waypoint.
- **The search-waypoint gate** (`AAS_NAV_LAST_WP`, `nd:88-94`): it is a generic gate for any AUTO search.
- **Bid eligibility (lowest flight time wins):** a swarm rule with no product in it. Pickup's role bids come with
  pickup (roadmap R23), as a generic "may bid" function app passes in.
- **The dock as a target:** the detector class, its size preset and the `DOCK` task kind. See Q5.
- **The wire codec** (`plugins/vehicle/ardupilot/mission_encoding.py`): the GCS imports it (see above). The guard allows it by name.

## Core and swarm know no product
The owner's rule, checked by five audits of `core/` (flow, skills, swarm, link, perception, review, approach, ports,
kernel) and `plugins/`, each checked again by a second reviewer.

**What v4 adds to v3:**

| Item | Today | Fix | Commit |
|---|---|---|---|
| Safety holds would sit inside the flow (v3) | `nd:231-236` | Core's `NavigationDecision` runs the guard and recovery, then the flow | c6 |
| Core clears the POI on a failure | `nd:151` | Delivery clears, then requests RESET (same order) | c5 |
| Recovery reads the floor and one-shot from `NavArgs` | `rec:59,66,72,75,111` | `RecoveryPorts.min_alt`, `is_oneshot`, wired by app | c6 |
| Hook named after delivery's restart | `nth:23,61-62` | `return_to_search` | c6 |
| Core-side names say "product" | v3's ports | `flow_task_active`, `clear_flow_task`, `sortie_ready` | c4 (`sortie_ready` with pickup) |
| Docstrings and help text that name delivery or the hub | `rel:1,16`, `rej:30` (c5); `rec:1`, `core/skills/acquire/orbit_geometry.py:13` (c6); `core/flow/nav_oneshot_completion.py:20`, `core/kernel/config/nav_args.py:71` (c7); `nta:24,56`, `core/skills/acquire/peer_offset.py:1,78`, `peer_orbit_plan.py:1`, `core/swarm/peer_task_priority.py:1-7,61-62`, `core/flow/mission_navigation.py:1`, `core/flow/nav_task_availability.py:5`, `core/ports/detect_data.py:3` (c8); `plugins/vehicle/ardupilot/mission_metadata.py:83,90` and its unused import `:15` (c9) | Neutral wording | c5-c9 |
| "delivery" used as a plain noun | `core/kernel/logging/pose_cadence_debug.py:38`, `core/perception/pipeline/real_tracking.py:1`, `core/swarm/nav_network_shutdown.py:53`, `plugins/vehicle/ardupilot/pose_streams.py:5` | "hand-off" | c8 |

None of these changes behaviour, a log line the evals read, the wire, the GCS or Lua. The `-nos` help text is the only
user-visible text that changes.

**Guard** (`tests/guards/test_core_knows_no_product.py`, new in c8, about 70 lines, modelled on
`tests/guards/test_oracle_stays_in_the_bench.py`; no src code):
- Scans every `.py` and `.json` under `src/navpy/core` and `src/navpy/plugins`, except the vendored `_mavftp`.
- Identifiers (names, attributes, defs, classes, arguments, keywords, import aliases), split on `_` and camelCase,
  plus strings and comments; whole words, any case.
- Refuses: delivery, deliveries, ddh, hub, hubs, pickup, pick-up, catcher, medicine, survey, spraying, surveillance,
  patrol. Each new product adds its words.
- Allow rows are exact: path, word, count and reason. Today: `mission_encoding.py` (the codec the GCS imports; its hub
  constant and docstring) and, until c9, `mission_metadata.py`. A missing or stale row fails, so a new hit in an
  allowed file fails too, and c9 must drop its row.
- A synthetic self-test covers snake case, camel case, acronyms, strings, comments, identifiers inside strings, an
  allow-row escape, and the words that must pass ("deliver", "delivered", `AAS_DEL_*`).
- It checks words, not meaning. Each commit's Codex review still checks that no product rule hides behind a neutral name.
- Together with the c2 layer rule (no core→products import), this keeps core and swarm product-free.
- A read-only prototype on `f744062` found 61 hits in 15 files; every one is resolved by v3, by the table above or by
  the allowed codec.

**Not product knowledge (kept, with reasons):**
- `AAS_DEL_*` param names: approach-law params; AGENTS.md keeps existing names.
- The sortie-done latch and `AAS_NAV_ONESHOT`: a platform piece (roadmap R19).
- The six NavStates and the transition hooks: a generic sense-decide-act cycle. Pickup can retry a pass through
  NAV → CONFIRM without RESET, so it needs no new state for that.
- The search-waypoint gate, pass classification, AUTO resume teardown, peer-task mechanism, busy rule, bidding and
  the swarm log lines the evals read.
- The disclaimer "approach accuracy does not establish ... docking, cargo receipt, coverage" (required by AGENTS.md).
- The review's site-type labels (building, vehicle, antenna, operations site, bridge, fuel, other;
  `core/review/confirmation_media.py:12-26`): a generic description of any POI's site, also used for sim POIs. The
  codec carries the same names, but no product rule reads them.

**The dock (Q5).** The glossary makes the dock a general-purpose target, not delivery's, so the name is not product
knowledge. The real limit is that core supports only one target class, in four places:
- the class id `DOCK_DETECT_CLASS_ID = 0` and its catalog check (`core/perception/profile/vision_class_profile.py:20,66-70`);
- the size preset, read only for that class (`:119-121`, `vision_profile_loader.py:47`, key `dock_presets`);
- the swarm task kind `DOCK` on the wire (`core/link/types.py:28-48`);
- three fall-backs to class 0 (`core/skills/confirm/confirmation_action.py:98`, `core/skills/acquire/peer_geo.py:214`,
  `peer_offset.py:67`).

Pickup flies to a line, a second class. Swarm assignments already carry the detector class on the wire
(`core/link/task_assignment_msg.py:61,76`), so a second class needs no new wire kind; a line task's kind would read
`UNKNOWN` (wire 5), which every decoder accepts. A new kind (for example CATCH) is optional and would need new
decoders (`:74`) and GCS labels.

## New types
| Name | File | Why |
|---|---|---|
| `ApproachOutcome` (NONE, WAITING, ABANDONED, PASSED, FAILED) | `nd` | Core reports the approach result; delivery picks the next state |
| `ReleaseOutcome` (WAIT, ABORTED, READY) | `rel` | Tells apart "keep confirming", "geo hold aborted" and "fly" (`rel:41,43,45`) |
| `DeliveryWorkSource` | `products/delivery/work_source.py` | Delivery's order of work |
| `MissionMarker` (code, location, location_type) | `core/ports/mission_marker.py` | Lets the plugin pass markers to delivery without either side importing the other (c9) |

- **Renamed and moved, not new.** `DeliveryFlow` is `NavigationDecision` renamed. `OneShotCompletion` moves unchanged. `restart_search` and `interrupted_leg` are plain functions.
- **None of these.** There are no Protocols, registries or plugin slots.
- **With pickup only.** `ProductCalls` and `Sortie` (in app) come with the arming latch, not in this step.

## Simulation-only parts
The owner remarked on 2026-10-12 that refresh and reset are simulation behaviour. This step keeps each of them exactly as it runs today. Only the parts that are delivery's move. No simulation machinery goes into `products/`.

| Part | This step | Later |
|---|---|---|
| Mid-flight refresh | The live part stays in the core reset (`ntr:167-168`); its mission half calls app's read. The dead ONHOLD part is deleted (Q1). | With the arming latch, app wires it for sim runs only (CHANGE; owner decides). |
| Restart at item 1 | Becomes delivery's `restart_search` in the same pair hook (`nth:61-62`). It still runs on every run, real autopilots included. | After Q2, app wires it for sim runs only (CHANGE). |
| SIM_SPEEDUP restore | Untouched. It is applied at `nta:96` and restored at `core/approach/nav_exit_cleanup.py:62` and `ntr:145,174-176`. | It moves to `sim/` behind a core port. The order apply < restore < SNAP stays. |
| Hub sim POI | When the detector is simulated, the hub flight places the sim POI at the last mission item (`fbn:24-25,63-68`). It moves with the hub route (c8), behind the same ports. | Moves to `sim/` with the sim step. |
| One-shot sim disarm | Moves with delivery's finish (c7). It stays behind `OneShotCompletionPorts.disarm`, gated by `is_simulated_autopilot` (`core/flow/nav_oneshot_completion.py:13-16,36-40`). | Only the disarm moves to `sim/`; the real-autopilot warning and the teardown stay with the finish. |

## Commits
**Gate for every commit:**
- `pytest tests` and `pytest tests/gcs -o addopts=""`.
- The guards: layers, unresolved imports, vision walk, fresh imports, registry census and SOLID.
- `scripts/step1_move.py --check`. It prints "no leftovers" on `f744062`.
- A Codex review.

Build each commit in a temp worktree, because the SOLID guard scans the untracked `tools/`. Push ff-only.

**Before c2:**
- Re-record the baseline. The figure of 6608 passed with 4 known failures comes from `cc430e8` and is UNVERIFIED now.
- Run one GCS demo eval on main, from the checkout under test.

**Path edit** means that only an import, attribute or patch path changes. No assertion and no expected value changes.

| Id | What | Size |
|---|---|---|
| c0 | This doc, with the owner's answers | S |
| c1 | ~~Pin tests only~~ **Dropped** (owner, 2026-10-12) | – |
| c2 | Rename the planned layer `missions` to `products` (script, data, docs) | S |
| c3 | Delete the dead ONHOLD refresh | S |
| c4 | Core stops reading the hub record; the planner gets orbit sizing and the band; app reads the metadata | M |
| c5 | Core reports results; the choices are written once, still in core | M |
| c6 | Delivery's flow, review, restart and rewind move; vision root | M |
| c7 | Delivery's one-shot finish moves | S |
| c8 | Work source; the hub record and route move; docs | M |
| c9 | Neutral mission markers (Q3) | S |

**c0: this doc.**
- Commit `docs/design/delivery_product.md` with the owner's answers, through a temp worktree.
- `docs/design/step2_delivery.md` (untracked; the parked hooks plan) is superseded and is not committed. UNVERIFIED which chat owns it; if another chat owns it, leave it alone.

**c1: pins. Dropped by the owner (2026-10-12).** Wherever a later step names a c1 pin, the existing tests plus the
new 3-UAV delivery eval stand in for it. Kept below for reference only.

Original plan: new file `tests/modules/nav/test_delivery_pins.py`, using the rig. Every pin must be green on `f744062`. If one is not, change the pin to match today's behaviour, never the code.
- **(a) No refresh while ONHOLD lasts.** This covers ONHOLD held for more than 2 s, reached by disarm, manual mode, the waypoint gate or the one-shot hold.
  - After the tick that enters ONHOLD, there is no `args.refresh` call and no mission-item read while ONHOLD lasts.
  - With no active task at entry, there are none at all.
  - The entry tick itself may refresh: a disarm or a mode change with an active task runs the task teardown (`nd:75-77,83-84` → `ntr:167-168`), and so does the one-shot finish (`core/flow/nav_oneshot_completion.py:53`).
- **(b)** No hub flight while `peer_navigation` is set (`nta:88-92`).
- **(c)** A gated own POI never falls through to the hub (`nta:61-67`).
- **(d)** A coordinate pass sets `final_approach_nav_completed` and requests RESET (`nd:194`).
- **(e)** Work is chosen only when there is no active POI (`core/skills/search/detect_action.py:40`).
- **(f) The phase is already DETECT inside `auto_resume.run`** for:
  - reject, late deny, dwell expiry and re-acquire timeout;
  - every geo abort in review: a present re-acquire (`trk:233` → `:118`), the identity re-bind (`trk:234` → `:190` → `:118`), entry arm failed (`trk:261` → `:97`) and entry arm declined (`trk:272` → `:97`);
  - the geo abort at release (`rel:43` → `trk:118`).
- **(g)** The phase is still NAV inside the task teardown for GUIDED lost and GUIDED timeout.
- **(h)** The phase is still NAV when "RESET: PASSED POI" is logged.
- **(i) Log order.** "REJECTED: resuming AUTO mission" comes after `auto_resume.run`. "GEO-HOLD abort: …" comes after the DETECT request and before `auto_resume.run`.
- **(j) Metadata reads in the rig.** NavArgs is a mock in the rig, so the catalog is the only mission reader at start-up. The other readers are `nav_waypoints.py:19`, which only NavArgs uses, and `mission_approach_writer.py:96,155`, which only runs in flight.
  - At start-up, `vehicle.get_mission_item` is called once per item, all before the "Approach strategy:" log (`app/flow/nav_foundation_composition.py:169`).
  - Each `reset.clear()` adds exactly one more pass.
  - With a mission built by `build_mission`, as `tests/modules/nav/test_mission_metadata.py:30-40` does, the hub, hub type and band all come from that pass.
- **(k) A failed approach** clears the POI while the phase is still NAV, then requests RESET (`nd:151-152`).
- **Already pinned:**
  - the restart (`tnc:6491,8567,8672,8682`);
  - the rewind (`tnc:8040-8306`, asserts at `:8292,8306`);
  - the one-shot finish (`tests/modules/nav/test_nav_transition.py:115-185`, `tnc:7092-7502`);
  - the DDH (`tnc:7503-7582`);
  - peer priority (`tnc:7583-7715`).

**c2: layer rename.** No `src` change. The commit message says this is the owner's rename, not a weaker test.
- **Script:** `scripts/step1_manifest.py:90,226,247`. Without this, files under `products/` count as layer "old":
  - every import edge into or out of them passes (`:230,236`);
  - the vision graph cannot place them (`tests/guards/step1_vision_graph.py:74-75`).
- **Data:**
  - `docs/design/step1/manifest.tsv`: 7 mentions (final homes of rows 484, 486 and 504, plus notes);
  - `final_violations.tsv`: lines 14, 16, 17, 19 and 21;
  - `splits.json:28`.
- **Docs (a word rename only):**
  - `docs/design/step1_separation.md:19,50,69,92,177`;
  - `docs/design/roadmap.md:39,45`;
  - `docs/design/architecture_split.md:20,43,168,170,171`;
  - `docs/design/flow_runner.md:292`.
- **Tests:**
  - string renames at `tests/scripts/test_step1_manifest.py:151,164,175,216,252-254`;
  - new `allowed()` cases, all False: core→products, plugins→products, products→sim, sim→products.

**c3: dead refresh (Q1).**
- **Core:**
  - `nd`: delete the call at `:241`, `previous` at `:230`, the method at `:243-252`, `NavigationDecisionPorts.clock_s` (`:203`), and `args` and `mission` (`:217-218,226-227`). Remove the `NavArgs` import (`nd:9`).
  - `core/flow/nav_state.py`: remove `last_parameter_refresh_s` (`:28,39,44,67-75,93`).
- **Constructor:** `app/flow/nav_decision_composition.py:92,99-100`.
- **Tests:** in the rig, `:155` becomes `self.mission = self.inactive._mission`, and the unused view at `:102-108` is deleted.
- **Kept:** the class and `decide`. From here on, the bool that `dispatch()` returns is unused; c5 removes it.
- **Done when:** the gate is green.

**c4: core stops reading the hub record; the planner gets orbit sizing and the band; app reads the metadata.**
- **Producers:**
  - `core/skills/acquire/approach_planner.py`:
    - the constructor gains `approach_kind` and `scan_altitude_rel` (dependencies 6 → 8);
    - `scan_altitude_rel` becomes a public attribute;
    - add `plan_orbit_approach(poi, class_id, drone_location)` with the body of `fbn:93-105`, reading the planner's own kind and band;
    - the docstrings at `:1` and `:29` now say the planner also holds the mission's scan band.
  - `cat`: remove the vehicle argument (`:13-14`), the reader and `IVehicle` imports (`:6-7`), the band (`:18`) and `refresh` (`:20-24`). Add `update(hub, hub_type)`.
  - `nd:33-36,50,60,73`: `flow_task_active` replaces the catalog and stays the last operand. Remove the import at `:12`.
  - `ntr:65,74,101` and `:207-211,226,237,260`: a `clear_flow_task` function replaces the catalog. Remove the import at `:13`.
  - `fbn`: remove `approach_kind` (`:38,46`) and `plan_orbit_approach` (`:93-105`). At `:69`, call `self._approach_planner.plan_orbit_approach(...)`. Remove the import at `:11`. Dependencies drop from 7 to 6.
- **Constructors and consumers:**
  - `app/flow/nav_composition.py`:
    - right after `:61`, add `metadata = read_mission_metadata(vehicle)`; this stays the first read;
    - pass `metadata` to `compose_poi_mission_ownership` (`:62-68`) and `metadata.scan_altitude_rel` to `compose_vehicle_approach_ownership` (`:69-79`);
    - drop the argument at `:95`.
  - `nav_foundation_composition.py`:
    - `:103-104` becomes `MissionCatalog()` plus `update(...)`;
    - `:142-149` builds `ApproachPlanner(..., approach_kind, scan_altitude_rel)`;
    - `compose_vehicle_approach_ownership` gains `scan_altitude_rel`.

    `PoiMissionOwnership` does not change, so no stale band is kept.
  - `nav_task_composition.py`: drop the parameter at `:61` and the argument at `:79`. The lambdas at `:147-152` and `:197-202` call `approach.planner.plan_orbit_approach`.
  - `nav_decision_composition.py:59-72`: wire `flow_task_active`; drop `poi.mission` (`:67`).
  - `nav_reset_composition.py`:
    - `:45` and `:69-73` take `clear_flow_task` lambdas;
    - `:53` takes `refresh_mission=lambda: refresh_mission_metadata(vehicle, poi.mission, approach.planner)`;
    - drop `:80`;
    - add the function `refresh_mission_metadata` here. It reads the plugin, calls `mission.update`, then sets the planner band. It imports the plugin reader, which app may do.
- **Tests (path edits):**
  - 15 lines change `controller.mission.scan_altitude_rel` to `controller.approach_planner.scan_altitude_rel`;
  - 8 orbit lines (`tnc:5010,5055,5182,5242,5254,6023,6054,6082`) change to `controller.approach_planner`;
  - `tnc:6708` becomes `controller.mission.clear_navigation_task`;
  - in the rig, `self.mission = self.fallback_navigation._catalog` moves after `:191`;
  - pin (j) gets the band path.
- **Guards:** delete `docs/design/step1/exceptions.tsv:7`. Pin `test_step1_manifest.py:107` to `{"layer": 27, "vision": 15}`. This is a tighter ratchet, forced by the stale-row check.
- **Done when:**
  - the gate is green;
  - `test_mission_metadata.py` is untouched;
  - the mock at `tnc:5242-5254` and the patch at `tnc:5181-5185` still take effect, because every caller looks the method up when it runs.

**c5: core reports results; the choices are written once, still in core.**
- **Producers:**
  - `nd`:
    - `NavStateDecision` returns `ApproachOutcome` and requests nothing (`:129-196`);
    - `_mark_passed` only sets `completed` (`:194`);
    - the failure path returns FAILED before `:151`; the POI clear moves to the consumer;
    - `status` is removed (dependencies 8 → 7).
  - `rel`: `:41` keeps its CONFIRM request and returns WAIT, `:43` returns ABORTED, and `:45` returns READY.
  - `rej`: `apply` returns True when the review is closed. Remove `_resume` (`:103-110`), `resume_auto` (`:38,44`) and the `ntr` import (`:14`). Dependencies drop from 5 to 4.
  - `trk`: the abort ends at `:73`. Remove `:74-80`, `resume_auto` (`:51,60`) and the `ntr` import (`:17`). Dependencies drop from 8 to 7.
- **Consumers (still core, same positions):**
  - `NavigationDecision.decide` (`nd:237-238`) maps the outcomes and gains `status` and the POI clear. On FAILED it
    clears the POI, then requests RESET, as `nd:151-152` do today.
  - `psd.dispatch` returns None and gains `resume_auto` and `logger` (dependencies 7 → 9). It handles the results as follows:
    - if `presence.update` returns True, it writes the geo-abort log, then resumes;
    - on CONFIRMED with READY, it requests NAV;
    - on CONFIRMED with ABORTED, it writes the geo-abort log, then resumes;
    - when the review is closed, it resumes, then writes "REJECTED".
  - **Why the positions hold.** Between the abort (`trk:73`) and the return to `dispatch`, every path only returns: `trk:97-98,118-119,190,233,234,261,272`, `core/skills/confirm/poi_presence_review.py:62` and `rel:43-44`. Likewise, `rej` returns straight after its teardown and logs.
- **Constructors:** `nav_decision_composition.py:73-101`, `nav_poi_status_composition.py:46-86`, `nav_track_recovery_composition.py:34-43`.
- **Tests:** in the rig, `:177` becomes `self.poi_status._resume_auto`.
- **Docstrings:** `rel:1,16` and `rej:30` no longer say that core chooses NAV or resumes AUTO.
- **Done when:** the gate is green.

**c6: delivery's flow, review, restart and rewind move.**
- **Producers:**
  - New empty files `products/__init__.py` and `products/delivery/__init__.py`.
  - New `products/delivery/flow.py`:
    - `DeliveryFlow`, cut from `nd`: `decide(mode)` runs `NavStateDecision.advance`, maps the outcome, then runs the
      review, exactly as `nd:237-241` with c5's mapping. Dependencies: phase, approach decision, review, status and the
      POI clear (5; about 50 lines);
    - `restart_search(vehicle)`, the body of `rec:89-91`;
    - `interrupted_leg(next_waypoint)`, the body of `ntr:244-245`.
  - `git mv psd products/delivery/poi_status_decision.py`.
  - `nd`: `NavigationDecision` keeps the mode read, the waypoint read, the guard and recovery (`:231-236`), and then
    calls `self._flow(mode)` where `:237-241` were. Remove the `psd` import (`:28`) and the outcome mapping. `nl` does
    not change.
  - `rec`: remove `:89-91`, `RecoveryPorts.set_mode` and `restart_mission` (`:25-26`), the `FlightMode` import (`:16`)
    and `NavArgs` (`:10,38,44`). Add `RecoveryPorts.min_alt` and `is_oneshot`. Docstring `:1` loses delivery wording.
  - `nth`: rename `TransitionPorts.restart_auto_mission` to `return_to_search` (`:23,61-62`).
  - `core/skills/acquire/orbit_geometry.py:13`: docstring without the "passed-POI reset".
  - `ntr`: add `AutoMissionResumePorts.resume_waypoint`. `:243-245` becomes `self._ports.resume_waypoint(self._vehicle.mission_items_next)`.
- **Constructors and consumers:**
  - `nav_decision_composition.py`: imports at `:17-24`; `RecoveryPorts` at `:44-55` (drop `set_mode` and
    `restart_mission`, add `min_alt=lambda: args.min_alt` and `is_oneshot=lambda: args.is_oneshot`, drop `args`);
    `DeliveryFlow` built from the approach decision and the review; `NavigationDecision` at `:88-101` built with
    `flow=delivery_flow.decide`.
  - `nav_poi_status_composition.py:24-27`: import from `products`.
  - `nav_composition_types.py:52,59,180,187`.
  - `nav_runtime_composition.py`: `:105` wires `return_to_search=lambda: restart_search(vehicle)`; `:159-165` are unchanged.
  - `nav_reset_composition.py:69-73`: `resume_waypoint=interrupted_leg`.
- **Tests:** the 118 `controller.decision.decide()` calls do not change and still run the guard, recovery, approach
  and review in today's order. The rig's `decision._nav` and `decision._poi_status` reach the flow through
  `decision._flow.__self__` (path edits).
- **Data and vision root:**
  - `docs/design/step1/splits.json` gains a flagged part under `modules/nav/navigation_decision.py`:
    - the symbols of the part cut into `DeliveryFlow`, named as in the old file at `2c887d9` (UNVERIFIED: whether a part
      may name `NavigationDecision` while its other half stays in `nd`; if not, the part names the new `DeliveryFlow`);
    - `final` = `step1` = `products/delivery/flow.py`;
    - `vision_path` = `y`.
  - `manifest.tsv:515` changes from `move` to `split`, with a note.
  - `manifest.tsv:539`: its final home becomes `products/delivery/poi_status_decision.py`.
  - `tests/guards/step1_vision_walk.py:20`: add `"products/"` to `WALKABLE_HOMES`.
  - `tests/scripts/test_step1_manifest.py:92-93`: data pins 12 → 13 and 13 → 14.
  - New test in `tests/guards/test_step1_vision_walk.py` asserting that `products/delivery/flow.py` is in `walk().roots`.
  - The comment at `:34-35` now says 8 flagged parts, 168 files, 163 walkable.
- **Done when:**
  - the gate is green;
  - the walk gives 0 violations and 0 stale rows, with `products/delivery/flow.py` a root (re-run the end-state
    simulation first: the guard and recovery are now reached from `nl`);
  - the restart and rewind pins are unchanged;
  - an eval smoke run passes.

**c7: delivery's one-shot finish moves.**
- **Producers:**
  - `git mv core/flow/nav_oneshot_completion.py products/delivery/nav_oneshot_completion.py`. The body is unchanged, and it imports core only (`:8-9`).
  - `nx`: `oneshot: OneShotCompletion` becomes `finish_sortie: Callable[[], tuple[Exception, ...]]` (`:31,38`). `:59` becomes `errors.extend(self._finish_sortie())`. Remove the import at `:12`.
- **Constructors:** in `nav_runtime_composition.py`, the import at `:16-19` comes from `products`. `:88-98` builds `OneShotCompletion` as a local and passes its `run`.
- **Tests (path edits):** in `tests/modules/nav/test_nav_transition.py`, the import at `:13` changes, and `:108` passes `oneshot.run`. No assertions change.
- **Help text:** `core/kernel/config/nav_args.py:71` describes the one-shot flag without delivery's finish.
- **Data:** the note on `manifest.tsv:504` records the move. Its final home is already `products/delivery/` after c2. `final_violations.tsv:16` stays a valid final violation.
- **Done when:**
  - the gate is green;
  - `tnc:7092-7502` (including `:7144-7148`) and `test_nav_transition.py:115-185` pass unchanged;
  - the walk still has 0 stale rows. This was simulated: row `exceptions.tsv:39` stays reached through `nl` → `nth` → `ntr`.

**c8: work source; the hub record and route move; docs.**
- **Producers:**
  - `nta`:
    - `handle_new_poi` becomes `try_start(poi)`, with the body of `:62-87` (the en-route gate stays first);
    - remove `peer_task` and `fallback_navigation` (dependencies 12 → 10);
    - remove the imports at `:11,14` (fallback route, peer-task priority).
  - `core/skills/search/detect_action.py`: `choose_work: Callable[[Optional[DetectedObject]], None]` replaces `navigation_task` (`:20,27,41`). Remove the import at `:5`.
  - New `products/delivery/work_source.py`: the statements of `nta:54-61,88-92`, in the same order, calling `try_start`.
  - `git mv cat` and `git mv fbn` into `products/delivery/`. `fbn`'s catalog import now points to `products`.
- **Constructors and consumers:**
  - `nav_task_composition.py`: imports at `:18-21`; `NavigationTaskAction` at `:221-234`. `NavigationTaskWorkflows` gains `peer_navigation`.
  - `nav_composition_types.py:37-38,147-154`.
  - `nav_foundation_composition.py:26`.
  - `nav_runtime_composition.py:114-137` builds `DeliveryWorkSource` next to `DetectAction`, because the task stage is already at 143 of its 150 lines.
- **Tests (path edits):**
  - 10 calls of `navigation_task_action.handle_new_poi` become `work_source.handle_new_poi`: `tnc:1001,5624,7951,8419` and `tests/modules/nav/test_reask_after_timeout.py:371,381,395,412,428,527`;
  - in the rig, `:186-196` reach the objects through `work_source`.
- **Docs:**
  - the layer list in `src/navpy/AGENTS.md`;
  - `.claude/agents/architect.md:22-27` and `explorer.md:24`, then run `python scripts/gen_codex_agents.py`;
  - core docstrings that claim delivery ownership: `core/flow/mission_navigation.py:1`, `core/swarm/peer_task_priority.py:1-7,61-62`, `core/flow/nav_task_availability.py:1-7` (its "the DDH return" wording), `nta:24`, `core/skills/acquire/peer_offset.py:1,78`, `peer_orbit_plan.py:1` and `core/ports/detect_data.py:3`;
  - "delivery" as a plain noun becomes "hand-off": `core/kernel/logging/pose_cadence_debug.py:38`, `core/perception/pipeline/real_tracking.py:1`, `core/swarm/nav_network_shutdown.py:53`, `plugins/vehicle/ardupilot/pose_streams.py:5`;
  - the notes on manifest rows 484 and 519;
  - follow-ups go into `docs/design/architecture_todo.md`.
- **Guard:** new `tests/guards/test_core_knows_no_product.py` (see "Core and swarm know no product"), with
  `mission_metadata.py` allowed until c9.
- **Done when:** the gate is green, the new guard is green, the DDH and peer-priority tests pass, the swarm markers in
  `scripts/eval_gcs_demo_gates.py:34-37` are unchanged, and an eval smoke run passes.

**c9: neutral mission markers (Q3).**
- **Producers:**
  - New `core/ports/mission_marker.py`.
  - `plugins/vehicle/ardupilot/mission_metadata.py`:
    - `MissionMetadata.markers` replaces the hub fields (`:29-30`);
    - the loop at `:105-114` records every marker;
    - delete `read_default_delivery_hub` (`:33-44`).
  - `products/delivery/mission_catalog.py`: `HUB_MARKER_CODE` and `update(markers)`.
  - `mission_metadata.py:83,90`: docstrings without the hub; remove the unused import at `:15`.
- **Consumers:** the first read in `app/flow/nav_composition.py` and the refresh in `nav_reset_composition.py` pass `metadata.markers`.
- **Tests:**
  - `tests/modules/nav/test_mission_metadata.py:6,30-75,81-94,188-198` read the hub through `MissionCatalog().update(meta.markers)`. These keep the same missions and the same expected values, but they are more than path edits (Q3).
  - New test: `HUB_MARKER_CODE` equals the plugin constant.
  - The guard's `mission_metadata.py` row is removed (its stale-row check requires it).
- **Data:** the note on `manifest.tsv:486` says the reader still waits for the C3 vehicle port.
- **Done when:**
  - the gate is green;
  - "Default delivery hub navigation:" is unchanged in the eval smoke run if that run reaches the hub; otherwise the
    unit tests are the evidence, and the report says so;
  - the eval identity is re-baselined (it hashes `src`, `scripts/simtime_navigation_identity.py:11-18`).

## Guards and tests
- **Layers:** `products/` becomes a real layer in c2. The guard refuses core→products, plugins→products, products→plugins, products→sim and sim→products. One exception row is removed (c4) and none is added.
- **Vision walk:** a read-only simulation of the end state used the real graph code with every cut import and moved file, the one-shot move included. It was run from stdin and wrote nothing; it was built from `C:/Users/GARTF/AppData/Local/Temp/claude/C--Users-GARTF-Downloads-navpy-temp2/332c45a1-73c7-4891-88c4-38ee1e1677c2/scratchpad/dp2/vw_final.py`.
  - Today: 162 roots, 0 violations, 0 stale rows.
  - End state: 163 roots, 0 violations, 0 stale rows.
  - Without the delivery-flow root, `exceptions.tsv:43` (`trk` → `geo_ref_calc`) goes stale.
- **`step1_move.py --check`:** it prints "no leftovers" on `f744062`. It only flags references to pre-step-1 paths (`scripts/step1_leftovers.py:58-63,83-98`), so moves into `products/` cannot add leftovers.
- **SOLID.** These are measured with the guard's rules (`tests/guards/solid_architecture_guard.py:18-23,941-945`). The "After" values are projections (UNVERIFIED); the guard checks them at every commit.

  | Item | Limit | Now | After (commit) |
  |---|---|---|---|
  | `ApproachPlanner` (lines, methods, dependencies) | 200, 15, 12 | 154, 5, 6 | ~172, 6, 8 (c4) |
  | `approach_planner.py` | 300 | 181 | ~199 (c4) |
  | `nav_foundation_composition.py` | 300 | 262 | ~266 (c4) |
  | `create_nav_application` | 150 | 130 | ~133 (c4) |
  | `compose_navigation_task_workflows` | 150 | 143 | 143 (c4), ~138 (c8) |
  | `compose_nav_application` | 150 | 124 | ~126 (c7), ~137 (c8) |
  | `track_recovery.py` | 300 | 292 | ~283 (c5) |
  | `navigation_task_reset.py` | 300 | 277 | ~279 (c4, c6) |
  | Review / `NavigationTaskAction` / `NavTransition` dependencies | 12 | 7 / 12 / 6 | 9 (c5) / 10 (c8) / 6 (c7) |
  | `NavigationDecision` / `DeliveryFlow` / `NavStateDecision` / `RecoveryAction` dependencies | 12 | 8 / – / 8 / 5 | 4 (c6) / 5 (c6) / 6 (c5) / 4 (c6) |

  Moved files count as new against the pinned base, so every definition in them must be fully annotated (`solid_architecture_guard.py:325-334,485-541`). Every function in the moved files is annotated today (measured).
- **Pure vision:**
  - Delivery reads only the mode, the next waypoint, the mission state and core's results.
  - The flight to the hub uses GPS outside the final approach, which the owner allows.
  - The arming latch reads the armed bit and a param; neither feeds a command.
  - The vision walk now also covers the delivery flow.
- **Test edits:**
  - about 47 path edits (c3-c8);
  - three data pins: layers 28 → 27, splits 12 → 13 and 13 → 14;
  - the owner's rename (c2);
  - c9's 6 tests (Q3).

  There are no assertion edits and no skips.
- **Eval markers, byte-identical, in `uav_*_navigation.log`:**
  - the order "confirmed by ground station." < "INIT: NAV MODE" < "RESET: PASSED POI" < "SNAP(VISION-NAV" (`scripts/eval_gcs_demo_gates.py:142-163`);
  - "Self-detect orbit:", "Peer navigation started", "Task N assigned by owner M" and "GUIDED_LOITER";
  - `SIM_SPEEDUP=` in the order apply < restore < SNAP (`scripts/eval_gcs_demo_audit.py:115-172`);
  - the fatal markers, including "GUIDED not accepted within" (`scripts/eval_ideal_three_uav_vehicle.py:35`), and exactly one SNAP;
  - "Default delivery hub navigation:" only if the baseline run reaches the hub. Today's demo eval may never exhaust
    the mission, so DDH behaviour is evidenced by unit tests (pins and `test_mission_metadata.py`) unless a SITL
    scenario that reaches mission exhaustion is added; the report states which.
- **Evidence level:** unit tests plus SITL logs. There is no GCS or Lua change, so there is no npm run, live-tester run or Lua deploy.

## What pickup reuses
- **Skill results:** `ApproachOutcome`, `ReleaseOutcome`, the "review closed" flag, and `presence.update` meaning "aborted".
- **Entry points:** `try_start` and `start`, `DetectAction(choose_work)` and `NavigationDecision(flow)`.
- **The seven product call positions.** Pickup supplies its own functions, for example its own action after a catch instead of `restart_search`, and RTL as its finish instead of delivery's one-shot finish.
- **Mission data:** `ApproachPlanner.plan_orbit_approach` and the band, app's metadata read and the neutral markers.
- **Layer and arming:** the products layer rule and the arming specification. Pickup's flow file gets a flagged manifest row so that it is a vision root.
- **Templates to copy:** `flow.py`, `poi_status_decision.py`, `work_source.py` and `nav_oneshot_completion.py`.
- **Without a core edit:** its own `decide(mode)` behind core's safety holds; its own work order; its own functions at
  the seven call positions; its own floor and one-shot setting through app; a missed-pass retry through NAV → CONFIRM
  (only RESET entry clears the task, `nth:57`; CONFIRM entry only resets the gate and starts the actor, `nth:98-100`).
- **Still a core edit, each made generic when pickup adds it** (honest list):
  - detector classes: a second class (the line) needs the size preset per class and the three class-0 fall-backs
    replaced by the task's class (Q5);
  - task kinds (optional): a CATCH kind, only if the operator must see it; the class already travels on the wire;
  - arming: the generic `armed_changed` port in the loop and the `sortie_ready` hold in the guard (specified above);
  - role bids (scout, catcher, backup): a generic "may bid" function that app passes to the swarm (roadmap R23);
  - a pickup-only state between passes, if pickup's design needs one (`pickup_demo_plan.md:31` plans its own
    controller); the six states cover retry today;
  - a line zone in the shared mission codec (`mission_encoding.py:11-13`), with pickup's GCS step.
- **The seven calls must be routed through the chosen product** (the arming latch).

## Not in this step
- GCS screens, including the hub upload gate.
- Building the arming latch, the Lua param and its GCS line (specified above).
- Routing `is_oneshot` (`app/flow/nav_runtime_composition.py:100`) through the chosen product. That comes with pickup.
- The sim step: the speedup lease, the disarm and a sim-only restart.
- The approach-law contract for contractors.
- The C3 vehicle port. The reader's final home in `manifest.tsv:486` stays the long-term plan.
- Per-class target rules (Q5) and role bids.
- Pickup-step text outside core: the floor default 150 in both `nav_args.py:10` and `scripts/lua/aas_params.lua:16`,
  the Lua comment `aas_params.lua:20`, the GCS label "Disarm after attempt" (`paramSections.js:84`), the GCS
  `Literal["dock"]` (`src/gcs/backend/routes/vision_profiles.py:51`) and the dialect text in `navlink.xml:49`.
- The operator fence on AUTO writes (todo 12).
- The latent `TypeError` at `fbn:55`, and the unused imports already in `nd` (`:6,8,11`).
- Two drafts owned by other chats that still say `missions/`: `docs/design/architecture_final.md` and `pickup_demo_plan.md`.

## Owner questions (recommended default in bold)
1. **Delete the dead ONHOLD refresh?** Recommended: **Yes.** If not, it stays dead, or it is made live as a CHANGE (todo 7: GCS param edits would then apply in ONHOLD). Why it can never run:
   - It runs only when `dispatch()` returns False (`nd:239-241`), and acts only if the phase is ONHOLD (`nd:244`).
   - Every False path requests another state first:
     - no active POI → DETECT (`psd:48-50`);
     - CONFIRMED → CONFIRM (`rel:41`), DETECT through the geo abort (`rel:43` → `trk:118` → `trk:73`), or NAV (`rel:45`);
     - review still open → CONFIRM (`rej:60` via `:72`, and `rej:65-66`).
   - `request` changes the phase immediately (`core/flow/nav_state.py:77-81`).
   - All 21 `request` sites run inside `decide`, and `commit_transition` runs later in the same tick (`nl:88-104`).
   - ONHOLD is requested only by the guard (`nd:80,86,93`) and by recovery (`rec:73-77,114`). Both return before `dispatch` (`nd:233-236`).

   Pin c1(a) confirms this.
2. **After a delivery pass on a real autopilot: hold, fly to the hub, or RTL?** Recommended: **No change now** (restart at item 1, as today). Decide it together with the arming latch; the restart then becomes sim-only (CHANGE).
3. **Take the hub choice out of the plugin (c9)?** This needs 6 tests in `test_mission_metadata.py` to read the hub through delivery: same missions and same values, but more than path edits. Recommended: **Yes.** If not, c9 is dropped, and the plugin keeps the last piece of delivery knowledge outside `products/`.
4. **Arming (built in pickup's R22 step):**
   - (a) An unknown `AAS_MOD` holds the sortie until disarm. Recommended: **hold**. CHANGE: this is a new input.
   - (b) NavPy (re)started while armed: it flies on, as today. The hold comes with the safety work after the demo (your ruling).
   - (c) A forced arm counts as an ordinary arm: an unknown value holds and a known one flies.
   - (d) A missing value (no Lua param, nothing cached) flies delivery. Recommended: **delivery**, as today.
   - (e) One app variable, `sortie.calls`, holds the chosen product's calls. It is set only by the one product check at the arm edge, nothing registers into it, and core never sees it. Is that within "no plugin slots"? Recommended: **yes**. The alternative is to build at arming.
   - (f) Build it with pickup's param and GCS step. Recommended: **with pickup**. The alternative is now, with a start-up flag only.
5. **When does core learn more than one target class?** The dock stays a target name (glossary), not a product word.
   Core's one-class rules (size preset, class-0 fall-backs, the `DOCK` kind) are a limit, not product knowledge.
   - (a) **With pickup's line class** (pickup's vision and GCS step). No GCS or wire change now. **Recommended.**
   - (b) Now: the size preset per class from the vision catalog and the fall-backs from the task's class. This touches
     the profile schema the GCS reads (`src/gcs/backend/routes/vision_profiles.py:51`).

## Risks
1. **Statement order.** Every moved request, log and resume keeps its position. Pins c1(d) and c1(f)-(j) cover this.
2. **Late lookup.** A target fixed at build time would let `tnc:1790,1831,5181-5185,5242-5254,6708` pass without checking anything. Only the four bindings named above are fixed at build, and nothing patches them.
3. **`presence.update` returning True means "aborted" only implicitly** (`trk:98,119,190,233,261,272`). Mitigation: a docstring plus pin c1(f).
4. **SOLID headroom.** The task stage is at 143 of 150 lines, `create_nav_application` will be about 133/150, `compose_nav_application` about 137/150, and `ApproachPlanner` about 172/200.
5. **Path edits.** There are about 47 path edits and three data pins, and reviewers must confirm they are paths only. c9's 6 test edits depend on Q3.
6. **Baseline and checkout.** The baseline is UNVERIFIED, and the shared checkout holds other chats' untracked files, so build in a temp worktree.
7. **Arming (later).**
   - The cycle and the guard may read the armed bit on opposite sides of an arm, which gives a one-tick hold.
   - Cache freshness after a GCS write is UNVERIFIED.
8. **Size (UNVERIFIED).** About 30 src files are touched (v4 adds 8 docstring-only files), with 5 new src files,
   4 `git mv`s and about 800 lines, including moved bodies.
9. **The decision splits in two (c6).** The guard and recovery stay in core's `NavigationDecision` and run exactly as
   today; the loop still calls `act()` every tick. The 118 unchanged `decision.decide()` tests, pin c1(a) and the
   RECOVERY/one-shot tests (`tnc:7092-7502`) cover it.
10. **Guard words.** A banned word could hit a future generic use (for example "hub" in a network sense). The fix is
    a better word or a named allow row, never a weaker scan.

## Review handling
Owner ruling on v3 ("core and swarm should not know about missions"): five audits, each checked by a second reviewer.
- **Accepted:** the items in "Core and swarm know no product", the guard, and the en-route gate staying in core.
- **Not taken:** an injectable bid function now (no second user yet; it comes with pickup's roles, R23); a JSON field
  for the dock preset now (Q5); renaming `AAS_DEL_*` (AGENTS.md keeps names).
- **Judged generic, kept:** the six states, the RESET-entry teardown, the sortie-done latch and redirect, the
  search-waypoint gate, pass classification, AUTO resume, bidding, swarm constants and log markers.

Codex on v4 ("ship with changes"):
1. Returning from the loop would skip `act()`. **Accepted:** the guard and recovery stay in core's `NavigationDecision`;
   the loop is unchanged.
2. 118 `decision.decide()` test calls would break. **Accepted:** with fix 1 they stay unchanged.
3. The review's site labels. **Kept as generic**, with the reason written down.
4. The guard is a word check. **Accepted:** exact allow rows with counts, `.json` scanned, more self-tests, and Codex
   still reviews meaning at each commit.
5. Q5 mixed up the name and the limit. **Accepted:** reframed; the class already travels on the wire; the pickup list
   gains arming and the class-0 fall-backs.
6. SOLID projections. **Accepted:** `DeliveryFlow` and `NavStateDecision` rows added; the unused confirmation manager
   is dropped.
7. No pin for the failure path. **Accepted:** pin c1(k).

Checker findings on v2:

| # | Finding | Handling |
|---|---|---|
| 1 | The arming latch cannot be wired as written | **Accepted.** v3 specifies each piece: the arm edge in core's loop (a generic port at `nl:125`), the latch and the only product check in `app/flow/product_choice.py`, call positions that look up `sortie.calls`, a generic `product_ready` guard after `nd:75-81`, and a cached read that never waits. The "slot or per-call check" choice is avoided: one app variable, set only by the arm-edge check, is neither of those. It is surfaced as Q4e. |
| 2 | Q4b contradicts roadmap R22 | **Accepted.** The default is now hold, marked CHANGE, and the forced-arm case is added (Q4c). |
| 3 | The one-shot finish (R17) is not addressed | **Accepted.** c7 moves the finish into delivery. The latch, redirect and hold stay platform (`roadmap.md:52`, `architecture_split.md:171`). Row 504 now matches, and row 539 is updated in c6. |
| 4 | Pin c1(a) fails as worded | **Accepted.** Verified: `nd:75-77` → `ntr:167-168`. The pin is reworded. |
| 5 | c1(f) misses `trk:190` | **Accepted.** |
| 6 | "Late lookup everywhere" is false | **Accepted.** The four build-time bindings are named. Nothing patches them, and the rig uses `__self__` for two of them. |
| 7 | c4 leaves a dead parameter | **Accepted.** c4 removes `nav_task_composition.py:61` and `nav_composition.py:95`. |
| 8 | Docs fall out of sync | **Accepted.** The `missions/` renames go in c2, the rename commit, rather than c7. The docstring fix goes in c8. |
| 9 | The band leaves a stale value around | **Accepted.** The band is a local in `create_nav_application`, passed to the planner, and the planner's docstrings are updated. |
| 10 | Wrong citation for `mission_validator.py` | **Accepted.** It imports through waypoint_builder's re-export (`:8-11`). |

The checker's three UNVERIFIED items:
- **`step1_move.py --check`:** now run. It prints "no leftovers" on `f744062`, and by its design (`scripts/step1_leftovers.py:58-63,83-98`) it cannot flag moves into `products/`.
- **Pin (j):** reasoned from the code (the catalog is the rig's only start-up reader), but confirmed only once the pin runs.
- **The 6608-passed baseline:** still UNVERIFIED.