# Design: platform, behaviors and mission modules (draft v3, 2026-10-07)

Status: for review. This is step 1 of 2 and covers how modules are designed.
Step 2, storage, follows once this is agreed. Nothing is implemented.

v3 folds in an architecture review of v2. The review had:
- 5 alternatives, each argued by its own advocate;
- a red-team of v2;
- an audit of v2 against the code;
- a prior-art survey;
- 3 judges (safety, product, engineering cost);
- a critic who tried to refute the verdict.

## Principle
Each mission is a composition of reusable behaviors: delivery, in-flight
pickup, search and rescue (SAR), fire, patrol, survey, spraying. The vision
final approach is one of those behaviors. It is not part of the platform, and
neither delivery nor pickup owns it. SAR never builds it.

## Review verdict
- **Kept from v2 (the destination):**
  - three layers, Platform → Behaviors → Mission modules;
  - the final approach is reached only through a validated profile;
  - a supervisor above all mission logic;
  - load-time completeness checks.
  - Prior art uses the same shape: PX4 modes with mode executors, Aerostack2 behaviors, and DroneResponse.
- **Changed:**
  1. *Runtime contract.* One exclusive flight activity, plus always-on services. v2's rule "one behavior holds everything" could not reproduce today's delivery flow, and it would stop SAR coverage on every sighting.
  2. *Authority.* The autopilot comes first, then the companion supervisor, then mission logic.
  3. *Order.* Measure the pickup dive, build small seams, ship pickup and SAR, and only then extract the framework from three real modules (AGENTS.md: no abstraction before repetition justifies it).
- **Rejected:** see the table at the end.

## Why change: today's coupling
Paths are under `src/navpy/modules/` unless they start with `gcs/`, `scripts/`
or `runtime_composition.py`.

| Fact | Evidence |
|---|---|
| The vehicle runtime is one fixed flow: DETECT → CONFIRM → NAV → RESET | `nav/nav_state.py:14-20` |
| Every confirmed POI goes to the final approach | `nav/confirmed_poi_release.py:45` |
| The final-approach engine is built and started at boot for every vehicle | `runtime_composition.py:169-179` |
| `final_approach.is_active` means "a vision law is configured", which is true from boot. 17 gates read it, including the one that gates operator review on the law's preview | `navigation/navigation_final_approach.py:17-20`, `nav/confirmation_action.py:49-62` |
| RESET and RECOVERY restart AUTO at item 1 (the takeoff). A rejection resumes the leg | `nav/recovery.py:89-91`, `nav/navigation_task_reset.py:241-252` |
| Fleet tasks exist only for docks. The auction needs exactly 3 live UAVs | `comm/messages/types.py:37`, `swarm/task_actor_slots.py:11` |
| Only class 0 (dock) has class settings, and the sim renders only class 0 | `vision/vision_class_profile.py:66-70`, `vision/sim/finite_poi_projector.py:149` |
| The GCS refuses an upload without a delivery hub. Scan altitude comes from the dock preset | `gcs/frontend/src/hooks/useMissionUpload.js:47-51`, `gcs/frontend/src/utils/trackGenerator.js:11` |

## Three layers (destination)
```
Mission modules  delivery · inflight_pickup · sar · fire · patrol · survey · spraying
      | uses
Behaviors        flight activities + services
      | uses
Platform         vehicle · comms · fleet · perception · supervisor · operator channel · GCS shell
```
- Dependencies point down only. The guard is tightened in stages:
  - **Now:** mission and hook code may not import `navigation.nav.vision_nav`. Today nothing in `nav/` imports it, so the rule is free.
  - **At extraction:** widen it one package at a time.
  - Most vision→navigation imports are geo and gimbal code, which belongs to platform Perception and moves there.
- Something belongs in the platform if every mission needs it, if it carries no mission meaning, or if safety needs it to behave the same everywhere.

## Authority (three tiers)
| Tier | Owner | Holds |
|---|---|---|
| 0 | ArduPlane | Modes, fence, failsafes. Ground-compiled AUTO geometry keeps flying if the companion dies. External attitude is accepted only in GUIDED |
| 1 | Companion supervisor | Today's ordered `decide()`. The companion flies only in GUIDED and AUTO; waiting uses GUIDED loiter or AUTO loiter items. Any other mode, LOITER included, means hold. Operator tools: any mode change, "Cancel task" (ends the approach, motor stays on) and E-STOP (MANUAL plus a force-disarm). The altitude floor is per module and per phase (see below) |
| 2 | Mission logic | One flight activity at a time, the camera lease, and services |

**Safety checks belong to modules.** The altitude floor is off by default in the
platform; fire suppression, for example, must fly low. Each module plugs in its own
safety checks: a floor per phase, dive limits, and so on. The autopilot's
fence and failsafes stay the hard limit (tier 0).

The supervisor gates commands as well as states. Every mode, mission, attitude
and gimbal write goes through one fenced sink. `NavigationCommandSlot`
already does this for vision_nav, and the sink generalizes it.

## Behaviors
**Flight activities.** Exactly one owns the flight lease at a time. Activities
are chosen whole and never blended.
| Activity | Does | Outcomes | Today |
|---|---|---|---|
| search | flies the AUTO coverage; resumes at the interrupted item, not item 1 | area_done | DETECT + AUTO |
| goto, hold | fly to a point; wait for an event | arrived, failed | vehicle goto, goto_loiter |
| observe | orbits a geo point with the gimbal on it | done, lost | peer loiter + geo pointing |
| acquire | flies to a reported POI until its own vision sees it | acquired, not_found | peer orbit + ready gate |
| line_up | climbs back to the detection altitude and lines up on the crossing heading before each pass (pickup) | ready, aborted | missing |
| final_approach(profile) | pure-vision approach. Profiles: `dive` (dock), `line_dive` (capture segment) | passed, lost, aborted:<reason> | NAV + `navigation/nav/vision_nav/` |
| track (later) | follows a moving POI | — | missing |

**Services.** They are always on and hold no flight lease:
- *review:* operator, peer or automatic. It is separate from final-approach admission, and several sightings can be queued.
- *report and offer.*
- *tasking:* bids and assignments.
- *presence.*

Services may borrow the camera under a priority lease. Today AUTO keeps flying
during CONFIRM, and review and peer offers already run alongside it
(`nav/self_detected_approach.py:43-45`, `nav/confirmation_coordinator.py:47-54`).
This names what exists.

**Supervisor actions:** HOLD, RTL, finish. Each must command a safe state, not
just stop.

**Final-approach rules:**
- The request is `FinalApproachRequest(profile, local track id)`, with no geo fields. A module picks a validated profile and can't pass range, altitude or aim offsets.
- Only the vision pass detector yields `passed`. Every other end is `aborted:<reason>` and is kept out of accuracy evidence.
- Each SNAP is stamped with the profile, a hash of the law config and environment, the NavPy and firmware commits, and the outcome.
- The pure-vision guard scans the whole `vision_nav` package, with an allow-list for diagnostics. A hand-kept file list would miss files; `pitch_law.py` is missed today.
- **Decided (2026-10-09): approach laws are plugins, not legacy.** PID (AAS_DEL_CTRL 0) and PN (1), both geo-based, and vision PN (2) are our proof-of-concept laws. They can be selected through one law registry, and contractors' laws join the same way.
  - Today their branches are spread across about 40 files in `nav/`, `navigation/` and `vision/`: `legacy_*`, `roll_l1*`, `ApproachKind`, and the 21 files that read `final_approach.is_active`. The self-detect orbit is one of them.
  - Target: each law is one plugin that only takes inputs and returns commands: no vehicle access, no mode writes, no flight behavior of its own. The orbit prep, recovery climb and floor around PID/PN move into a geo-approach skill that drives those laws. The vision-approach skill drives the vision laws. Nothing outside the final approach branches on which law is selected, and a guard test enforces that.
  - Each law declares its required inputs and their quality, for example GPS with a CEP, camera frames with a rate, latency and jitter, gimbal readback, attitude, and real airspeed. Requirements can depend on the profile. The platform gives a law only the inputs it declared.
  - **Decided (2026-10-09): this all happens at setup, before the sortie.**
    - The mission builder lists the laws whose requirements the vehicle's data sources meet, and says what is missing for the others.
    - A preflight self-test measures the real quality on the ground, such as frame rate and latency.
    - The law is latched at arming. Nothing is re-checked or switched in flight.
    - If a sensor degrades in flight, the approach ends through its normal failure path (lost, then retry or handover).
  - Removing them then means deleting the package, its registry entry and its tests.

## Mission modules
**Decided (2026-10-08): a graph runner, no hooks step.** Today's divergence points
become edges and skills of each module's graph:
- after confirm (`nav/confirmed_poi_release.py:45`);
- after the approach (`nav/navigation_decision.py:193-196`);
- resume vs restart (`nav/recovery.py:89-91` against `nav/navigation_task_reset.py:241-252`);
- work source (`nav/navigation_task_action.py:50-82`);
- bid eligibility (`swarm/task_capability.py:37-51`). A backup or scout bids a sentinel cost, so it neither wins "catch" nor stalls the auction.

Rules:
- The module id and role are latched at arming as a pre-arm check, never mid-flight. ONHOLD also happens in flight.
- The final-approach engine is built only for modules whose roles use it.

**Flows are graphs.** Each role has one graph:
- nodes are skills (flight activities);
- edges are named outcomes, with optional guards (passes < N) and actions (report, offer a task).

A straight sequence is the special case with one edge per node. Retries are loops, and branches are forks.
- **Skill lifecycle:**
  - enter (safety checks, take control);
  - run (one step per loop tick);
  - outcome;
  - exit (always runs; releases control and reports).

  The supervisor can stop any skill; exit then reports `aborted:<reason>`.
- **Runner** (platform), on each tick:
  1. the supervisor goes first;
  2. then the current skill runs;
  3. when the skill produces an outcome, the runner runs that edge's actions and enters the next skill.

  Services run alongside.
- **Between drones,** graphs connect through fleet tasks: the scout's action "offer catch" becomes the catcher's event "assigned".
- **Checked at load:**
  - every outcome of every node has an edge;
  - every node is reachable;
  - each graph has a safe end.
- **Today** NavState is such a graph, hard-coded: its edges are 20 `phase.request` calls in 6 files under `nav/`.
- **Decided:** the runner arrives with pickup, which needs loops and roles. Delivery's flow is its first graph and must pass today's tests and eval unchanged. Spec: `docs/design/flow_runner.md`.

**Delivery** moves out of the platform into its own module, and is then
paused: its tests and eval stay green, but it gets no new features. When it
resumes, a dock the drone found itself is circled first, then approached
(owner's choice). Today the vision law skips that circle
(`nav/self_detected_approach.py:44`).

**In-flight pickup** has three roles: scout, catcher and backup.
```
scout:   search(line marker) -> review(operator) -> [report, offer "catch"] -> observe(line)
catcher: hold -> assigned "catch" -> acquire(line) -> line_up -> final_approach(line_dive)
         passed -> review(scout or operator) -> confirmed -> [finish] -> RTL
         rejected or lost -> passes < N ? line_up : [operator hands task to backup]
```
- RESET splits into end-of-pass and end-of-task. Today RESET clears the task, the detector identities and the task actor (`nav/navigation_task_reset.py:139-171`).
- The pass budget lives in task state, not in DO_JUMP counts or Lua.
- A miss is detected after the pass, and the catcher goes around and retries until the budget is spent. There is no automatic break-off before contact and no GPS check, because GPS is about 10 m at best. The operator can use "Cancel task" or E-STOP at any time.
- No AUTO leg and no abort target crosses the capture corridor. AUTO holds only the line-up at the detection altitude, and every pass ends in a climb-out.
- **Decided (2026-10-08): no low-level run-in.** Trees and rocks make a low straight leg unsafe, so the catcher dives from the detection altitude straight to the capture segment, like the delivery dive. The pure-vision law flies down the line of sight, which is clear wherever the camera sees the segment.
  - Open: the aircraft is still descending at contact, so the pull-out loses height. Rough estimate at 25 m/s and 2 g: about 4 m at a 20° dive and 9 m at 30°, plus the sink during the reaction time. The segment must sit that high above obstacles, or the dive must be shallow at contact. The Monte Carlo measures it.

**SAR** has no final approach.
```
search(person, sector) --sighting--> [queue review, keep searching]
review confirmed -> [report, offer "observe"]; observe task: goto -> observe -> hold or RTL
search --area_done--> [report coverage on SEARCH_STATUS] -> RTL
```
Prerequisites:
- a person detector class and a sim render for it;
- a sighting queue;
- a GCS card queue with more than one card per UAV (`gcs/frontend/src/hooks/taskConfirmationState.js:4-5`);
- coverage resume.

**Which activities each mission uses:**
| Mission | search | goto/hold | observe | acquire | line_up | final_approach | track |
|---|---|---|---|---|---|---|---|
| Delivery (paused) | ✓ | ✓ | | ✓ | | ✓ dive | later (moving dock) |
| In-flight pickup | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ line_dive | |
| SAR | ✓ | ✓ | ✓ | | | | |
| Fire | ✓ | ✓ | ✓ | ✓ | | drop method open | |
| Border patrol | ✓ route | ✓ | ✓ | | | | ✓ |
| Survey, spraying | ✓ coverage | ✓ | | | | | |

## Fleet level
- **Task.** A task has:
  - a fleet-unique id;
  - its module and kind;
  - an anchor (a point now; later a track with velocity, so bids can cost a rendezvous with a moving anchor);
  - small params;
  - the equipment it requires.
- **Market.** A single-assignment auction (CBAA-style) among reachable peers. Before any new runtime task kind is auctioned automatically, it needs:
  - removal of peers on heartbeat timeout;
  - task leases renewed by status;
  - fencing tokens for exclusive tasks, so two catchers can't both hold the line;
  - no re-offer without a release or an expired lease.
  - Until then, failover goes through the operator.
- **Roles and equipment.** The operator sets each vehicle's role in the plan. Equipment is declared per vehicle and checked for eligibility.
- **Confirm policy** comes with the task kind, not the vehicle. Actuation tasks (catch, drop, release) fail closed. Auto-confirm and "no network means confirmed" stay limited to cooperative docks and the bench.
- **Wire.** No dialect rebuild yet:
  - new task kinds use free `task_type` values (6 and up);
  - new operator commands, if a module needs them, use free `request_type` values, acknowledged on SWARM_ACK;
  - coverage reports use SEARCH_STATUS;
  - module and activity go in the heartbeat state byte, which is always 0 today.
  - Decoders must tolerate unknown kinds. Each vehicle advertises its supported kinds and module versions at check-in.
  - The one rebuild (TASK_STATUS, plus a versioned, rate-budgeted, authenticated MODULE_DATA, or TUNNEL if it routes) waits for extraction, when three modules show what it must carry.
- **GCS coordinator.** It arrives with SAR, for cross-vehicle duplicate sightings and fire clusters. It is optional: on link loss the vehicles keep working.

## GCS
- Generic planning stays in the platform: area, zones, corridor, altitude separation, fence and upload.
- For pickup:
  - the DDH upload gate moves into delivery;
  - scan altitude stops coming from the dock preset;
  - role assignment is added.
- GCS mission plugins (planner inputs, panels, map layers, operator actions, params section, exports) arrive with pickup, the first plugin (owner, 2026-10-07). Delivery's hub UI moves into its plugin at extraction. Today `App.jsx` is hard-wired, and the backend `_ROUTES` table is the one seam.
- Plans are checked against the altitude floor at upload.

## Gates
- **Before the SITL investor demo:**
  - the pickup-dive Monte Carlo is a go;
  - the altitude-floor and eval fixes are in;
  - the graph runner and the pickup graphs are in.
- **Before hardware flight:**
  - fail-closed actuation confirm;
  - a lost-companion fallback: an autopilot Lua watchdog, or a safe GUIDED target set before streaming attitude;
  - market fencing;
  - validation with the fence and failsafes configured as flown. The eval forces both off today (`scripts/eval_sim_parameters.py:36-46`).

## Order of work
Each step is designed and approved separately. Bugs go to separate tasks.
The file-by-file split, positioning sources and slice order are in
`docs/design/architecture_split.md`. Replaceable components (ports, registry, kits) are in
`docs/design/component_ports.md`. The merged step order to the demo is `docs/design/roadmap.md`.
0. **Now, in parallel:**
   - the pickup-dive Monte Carlo, which needs no code change and is a go/no-go for the demo;
   - the bug tasks already opened (floor, dead peers, unknown kinds, eval, guard, param refresh).
1. **Storage, minimal.** How the module id and role reach the vehicle. Plan metadata is bit-packed and unversioned today (`nav/mission_encoding.py:1-13`).
2. **Graph runner and split:**
   - the flow runner, with delivery's graph reproducing today's flow;
   - extract delivery into its own module (DDH, dock preset, DOCK task kind, self-detect circle, one-shot finish), then pause it;
   - build the final approach per module;
   - move goto, loiter, geo and gimbal off the `navigation/` hub;
   - scope `is_active` by role;
   - put every law behind the law registry as a plugin.
3. **Pickup on 3 UAVs in SITL:**
   - split RESET;
   - line_up;
   - the strip detector profile and its render;
   - the GCS role and DDH changes.
4. **SAR:** the person class and render, the review queue, coverage resume, and the GCS coordinator.
5. **Extraction:**
   - the behavior contract and playbook tables;
   - the N-vehicle market with leases;
   - the GCS registries;
   - the one dialect rebuild.

## Owner decisions
1. ~~Retire the legacy laws?~~ Decided: they stay as selectable law plugins (2026-10-09; see the final-approach rules).
2. ~~Delivery flow?~~ Decided:
   - a dock the drone found itself is circled first, then approached;
   - delivery is extracted from the platform into a module and paused for now.
3. ~~Is operator LOITER a hold?~~ Decided: yes. NavPy never sets LOITER mode, so LOITER leaves ACTIVE_MODES (task open).
4. ~~Altitude floor?~~ Decided: off by default in the platform; each module owns its safety checks (fire flies low).
10. ~~Hooks or graph runner?~~ Decided (2026-10-08): graph runner.
5. ~~Pickup break-off before contact?~~ Decided:
   - no break-off before contact; a missed pass is retried;
   - no GPS during the approach, because about 10 m at best is unsafe and useless at the line;
   - the operator has "Cancel task" and E-STOP.
   - GPS stays in use outside the final approach.
6. **Roles** are set by the operator in the plan; the backup is operator-handed for the demo (recommended).
7. **One active module per vehicle per sortie,** while a fleet may split across modules (recommended).
8. **Glossary now:** Platform, Behavior (one exclusive flight activity), Service, Task, Role, Equipment, Profile. Playbook and Action wait for extraction.
9. **Dialect rebuild** waits for extraction (recommended).

## Rejected alternatives
| Option | Why not | Kept from it |
|---|---|---|
| Behavior trees per vehicle | A second executor during migration. Binary results lose named outcomes. Re-evaluating every tick can abort an approach on a one-frame dropout. The blackboard is hidden shared state | durable pass budgets; named debounces |
| Hierarchical statecharts | An in-house kernel under the nav loop before the demo; delivery must be re-expressed first | exit-point ownership; state groups as the first extraction candidate |
| Concurrent behaviors with arbitration | Safety becomes emergent. Blended commands break the final approach's isolation | per-channel command fence; select-only flight arbitration |
| Fleet-first mission graph on the GCS | Mission progress stalls on RFD900 dropouts (forest SAR). The GCS becomes mission-critical | task leases; the GCS coordinator for cross-vehicle work |
| Minimal hooks only, as written | Its DO_JUMP pass loop flies blind passes if NavPy hangs; it misses RESET and `is_active` | the P2 core: seams and ground-compiled geometry |

## Next: storage (step 2), questions to answer
- **Code:** where each module's vehicle Python, GCS backend and frontend, Lua, models and tests live, and how they are deployed and kept version-compatible. Prior art: Auterion checks a platform-API level per app at load.
- **Delivery to the vehicle:** how the module id, role and initial task reach it, and how they are latched at arming.
- **Params:** per-module Lua tables, as `CHUTE_` does at key 90 (`scripts/lua/chute_deploy.lua:14-17`).
- **Plans:** none are stored today, and plan.json is unversioned.
- **Results and evidence:** stamped SNAPs per profile. `validation_evidence/` is empty.
- **Settings and profiles:** the settings model is closed, and `vision_profiles.json` is tracked in git yet rewritten at runtime.
