# Split plan: platform, approach flow, modules, positioning (draft, 2026-10-07)

Status: for review. Nothing is implemented. This plans how today's code moves into the
layers of `mission_modules.md` (v3). It is an architect's plan, revised after two reviews.
Paths are under `src/navpy/modules/` unless they start with `src/`, `scripts/`, `tests/`
or `runtime_composition.py`; `.js` files are under `src/gcs/frontend/src/`;
`mission_modules.md` is `docs/design/mission_modules.md`; `test_nav_controller.py` is
`tests/modules/nav/test_nav_controller.py`. UNVERIFIED = not checked in this repo.

In short: delivery and pickup share one flow in two configurations (ruling 2). A seam lands
with its first reader; pickup seams wait for the Monte Carlo go and the storage step
(`mission_modules.md:213,227`). Labels before moves. Positioning plugs in at EKF3. Legacy is
isolated, then moved into one package. LOITER and floor changes are their own slices.

## Target tree
```
src/navpy/
  runtime_composition.py  platform.runtime: composition root, the only place that wires
                          missions/ and the legacy laws (main.py, runtime_*.py: runtime)
  args/                   platform.config; _ARG_MODULES stays static (navpy_argparse.py:15-33)
  logger/                 platform.logging; status_logger_network.py = platform.operator
  modules/                PLATFORM root (relabelled, not renamed)
    common/, vehicle/     kernel types; platform.vehicle (IVehicle is the position reader)
    comm/                 platform.comms; class -> task kind injected [S4]
    swarm/                service.tasking, service.presence
    vision/               platform.perception; the dock class stays here
    navigation/           final_approach, relabelled in place
      nav/vision_nav/     vision law; path pinned by guards
      navigation_command_*, navigation_final_approach.py   command slot, surface [S8]
      law table {spec, build_law, runtime_factory}          [S7]
      legacy/             final_approach.legacy, one deletable package [S10]
      geo/, gimbal_*.py   perception by label; + geo/ground_location.py [S1]
      approach_strategy, peer_*, orbit_geometry            activity.acquire by label
      navigation_vehicle_commands.py                       activity.goto_hold by label
    nav/                  stays, mixed: runtime loop; supervisor (decide, recovery, floor);
                          mission.approach_flow (state machine, tables, reset); services
                          glue; final-approach NAV workflow (final_approach_*, nav_entry)
      approach_flow.py    FlowHooks contract [S3]
    positioning/          later: created by the first feeder
  missions/               mission layer [S3]
    delivery/             module.delivery, PAUSED: hooks, ddh_route, finish, task_kinds
    pickup/               pickup step
  behaviors/              later: created by line_up (pickup)   (src/gcs/: labels only now)
```

## Shared approach flow (delivery and pickup)
The flow is today's machine: search (DETECT + AUTO) -> review (CONFIRM) -> final_approach
(NAV) -> RESET, plus acquire and goto/hold (`nav/nav_state.py:14-20`). It stays in `nav/`
until extraction. Modules plug in through one contract, `nav/approach_flow.py`:
- `FlowHooks` holds per-stage factories (the DDH source needs collaborators that exist
  only mid-composition), and each hook names its thread (bid cost: network listener).
- `runtime_composition.py` passes `DeliveryHooks` through NavController. The module-id
  table of `mission_modules.md:118` waits for a second module and the storage step.
- AST-direct guard: `navpy.modules.**` never imports `navpy.missions`; modules never
  import each other; mission code never imports `vision_nav` (`mission_modules.md:57`).

| Point | Delivery (paused) | Pickup (later) |
|---|---|---|
| Final approach | `dive` (today) | `line_dive`, after the Monte Carlo go |
| Work source | own POI claims the tick, then peer assignment, then DDH (`nav/navigation_task_action.py:50-82`) | by role: scout searches, catcher takes "catch", backup holds |
| After approach | RESET ends the task; one-shot -> ONHOLD | review; a miss -> end of pass + line_up until budget N, then operator hands over to backup |
| Bids, kind | ETA; DOCK=1 | scout and backup bid the sentinel; CATCH >= 6 after the unknown-kinds fix reaches every node |
| Finish | one-shot hold, sim-only disarm | RTL |
| Circle | circle the POI until confirmed, then approach (A1) | same |
| GPS assist before the approach | off (A2) | off until a good approach module exists (A2) |
| Floor and safety checks | off by default; the module adds its own checks (A3) | own checks (line-up, climb-out) |

- Hooks born now (each has a delivery reader): work_sources, finish, reset/refresh steps,
  task_open, review labels, class -> kind. Born with pickup: after_approach, resume, bid
  cost, offer, roles, on_cancel, pass budget; after_confirm must be a pure next-state
  choice, since it runs on every CONFIRMED tick (`nav/poi_status_decision.py:51-53`).
- Pause: no features; its tests and the S0 gates guard it (no snapshot test). Resume
  (circle a self-found dock under the vision law, without the sim truth used at
  `nav/self_detected_approach.py:84-88`) is its own design item, not one config change.

## Positioning seam (ruling 7)
EKF3 is the single fused position; it fuses several sources through its source sets. Every
source feeds it; NavPy reads the result through `IVehicle`, and consumers keep their narrow
ports (`swarm/task_ports.py:17-18`, `src/navpy/utils/fly_estimator.py:17-27`). A new source
changes autopilot config and adds a feeder, not consumers. Only the supervisor switches sets.

| Source | Feed path (ArduPilot side, UNVERIFIED here) | Companion code |
|---|---|---|
| GNSS | GPS driver (today) | none |
| Visual odometry | ODOMETRY as ExternalNav (EK3_SRCn_POSXY/VELXY=6) | odometry feeder |
| Visual map/terrain matching; later magnetic-anomaly nav | MAV_CMD_EXTERNAL_POSITION_ESTIMATE while dead-reckoning | absolute-fix feeder |
| GNSS-like replacement | GPS_INPUT (GPSn_TYPE=14) | gnss_like feeder |
| Tactical INS; later quantum (cold-atom) INS | External AHRS (AHRS_EKF_TYPE=11) | none |
| Quantum clock (CSAC) | timing only | none |
| Never | GLOBAL_VISION_POSITION_ESTIMATE as a global fix (handled as local; UNVERIFIED) | - |

- Now, no behavior change: (1) this decision, written into `mission_modules.md` once
  approved; (2) the fence: the pure-vision scan covers all of `vision_nav`,
  `navigation/navigation_command_*.py` and `nav/final_approach_*.py` with a diagnostics
  allow-list (extends the open 'guard' task; CORE omits `pitch_law.py`,
  `tests/guards/test_frozen_inventories.py:18-33`),
  plus a NAV-phase test whose vehicle raises on `location()`/`home()` (T1); (3) airspeed:
  frames carry VFR_HUD airspeed (`vehicle/flight_telemetry.py:41-43`), which without a
  healthy pitot may be GNSS-synthesized (UNVERIFIED) and degrade in forests (Q6).
- Later, each on its own trigger: `modules/positioning/` with a feeder table keyed by kind
  (odometry, absolute_fix, gnss_like) with the first provider; supervisor source-set
  control with the first non-GNSS source; quality-aware bids, geo-ref stamps and GCS
  readiness when a vehicle can declare such a source, read from EKF_STATUS_REPORT (units
  UNVERIFIED), not GPS_RAW_INT h_acc, which is receiver accuracy and goes stale in denial
  (`vehicle/power_gps_telemetry.py:47-56`); sim renders from truth when degradation is
  under test (today from the EKF, `vision/sim/sim_source_composition.py:111-117`). A
  companion fusion framework: never, unless EKF3 cannot take an input.

## Final approach and legacy isolation
- One law table replaces three structures (`navigation/nav/nav_law_factory.py:37-56`,
  `navigation/navigation_law_builders.py:21-37`, `navigation/navigation_composition.py:206-210`).
  Vision registers itself; `runtime_composition.py` adds the legacy entries in one call.
- Surface on FinalApproachNavigationService: `outcome()` = passed | lost | aborted:<reason>,
  stamped with the law; only vision passes are accuracy evidence (`mission_modules.md:109`).
  `before_review()` = min-zoom freeze or legacy review prep, chosen at boot as today
  (`nav/nav_confirmation_admission_composition.py:69-72`). `approach_in_progress` stays
  flow state in `nav/` (`nav/navigation_decision.py:156-158`).
- MissionPassPolicy splits: search gate stays in `nav/`, vision pass -> `outcome()`, GPS
  pass -> legacy entry (`nav/mission_navigation.py:137-166`).
- The 17 `is_active` reads: 4 move to the surface; the self-detect circle gate becomes an
  optional legacy contribution, since that ORBIT prep is legacy
  (`nav/self_detected_approach.py:1,44-49`); the floor read goes with S12; 4 GPS-assist and
  2 entry reads wait for pickup; 5 stay inside final_approach. A guard freezes the list.
- Profiles: today's constants are `dive`. The profile record, `line_dive`,
  FinalApproachRequest and per-profile entry geometry land with pickup; until then
  ApproachPlanner keeps its law gate (`nav/approach_planner.py:150-157`), so legacy runs
  keep their radius.
- Legacy package: law, runtime, destination resolver, `roll_l1*`, `pid/`, `pn/`,
  `mission_planner*`, `nav/pass_tracker.py`, `nav/confirmation_legacy_prep.py`, ORBIT prep;
  `navigation/__init__.py:24,28-29` stops re-exporting it.
- Removal set (Q4): package, register call, tests; `src/navpy/args/pid_args.py`,
  `src/navpy/args/mission_planner_args.py`, their `_ARG_MODULES` lines; AAS_DEL_PITCH/DIR/P_KP/
  PLD/PLRD, read only by legacy code (`navigation/nav/roll_l1_composition.py:68-77`,
  `navigation/legacy_destination_resolver.py:175`); GCS, Lua and frontend param rows; eval
  and bench argv (`scripts/eval_navigation_navpy.py:60-75`). A guard pins the set.
- Proof: a clean-process test blocks the legacy modules (meta_path finder) and composes the
  vision-only runtime; unregistering the entry proves nothing while `__init__` imports legacy.

## GCS split
- Now, labels only: gcs.shell (`main.py`, `route_registration.py`, `telemetry_loop.py`);
  platform (`vehicle_*`, control, launch, params, settings); gcs.planning (`planner/`,
  `mission_validator.py`); services (`task_confirm_*`, `task_assign_listener.py`, already
  generic); gcs.module.delivery (hub settings and CSV route, DDH metadata, 7 DDH UI files).
- No moves, renames or migrations now: settings reject unknown fields
  (`src/gcs/backend/settings_store.py:138-145`), uploads forbid extras
  (`src/gcs/backend/models.py:57`), the eval posts `default_delivery_hub`
  (`scripts/eval_gcs_demo_upload.py:115-118`), and DDH assignment state runs through 11
  generic frontend files, so moving the 7 files would extract nothing.
- Pickup step: one active-module contribution read by the upload gate
  (`hooks/useMissionUpload.js:47-51`) and the track generator (scan altitude as input,
  `utils/trackGenerator.js:6,11`); a role field in `models.py` and the frontend together; the
  upload floor check (`mission_modules.md:74,209`); task_ended; mission tail items.
- Extraction: registries, folder moves, settings namespaces, per-owner param sections,
  versioned plan files (storage step).

## Slices (ordered)
| Id | What moves | Behavior change | Tests and guards | Size |
|---|---|---|---|---|
| S0 | Nothing. Record `pytest tests`, `pytest tests/gcs` (skipped by `pyproject.toml:56`; runs Vitest) and per-gate verdicts of the ideal three-UAV, GCS demo and navigation evals | none | Q1 settles the red gate. Bug tasks: re-pin PINNED_BASE (2 guard tests fail, so size and annotation limits are off); fix the eval fixture that pairs DEL_CTRL=2 with the orbit marker | S |
| T0 | Tests only: lazy NavTestRig fields; spec'd mocks for `legacy_pois` and the geo port; final-approach double built on the real service with a regime fixture; one helper for the 33 law seeds | none | no assertion edited | M |
| T1 | Tests only: work-source precedence, boot-time review-prep choice, one-shot latch commit, quiescence-gated reset order, every eval-parsed marker (nav, swarm, vehicle, perception), NAV-phase position poison test | none | a test that fails today becomes a bug task, never an edit | S |
| S1 | GeoLocator into `navigation/geo/ground_location.py`, injected from `runtime_composition.py`; the legacy resolver delegates; generic geo-ref consumers switch | none | the 6 firewall asserts move to the new port in the same commit; clean-process import | M |
| S2 | MissionCatalog split: the scan band stays in `nav/`; acquire's plan_orbit ports use it directly (`nav/nav_task_composition.py:141-146,191-196`) | none | repoint the never-called mock (`test_nav_controller.py:5198-5234`) | S |
| S3 | FlowHooks + `missions/delivery/hooks.py`, wired through NavController; work_sources keeps today's precedence | none | T1 precedence; layer guard v1 | M |
| S4 | class -> task kind: one callable owned by delivery, injected at its 3 callers; the comm -> vision lazy import goes (`comm/messages/types.py:41-48`) | none; wire values unchanged | comm and swarm suites; no new stored field on saturated classes | M |
| S5 | DDH out: hub fields, hub metadata reader, FallbackMissionNavigation -> `missions/delivery/ddh_route.py`, via reset, refresh, task_open and label hooks | none | golden metadata bytes; patch strings and rig properties | M |
| S6 | One-shot completion -> `missions/delivery/finish.py` via a finish hook (`nav/nav_transition.py:58-63`); the latch stays in NavPhaseState, AAS_NAV_ONESHOT in NavArgs | none | transition, loop, override, re-ask suites | S |
| S7 | One law table; NavLaw/NavContext leave `nav_law.py` (NavCommand* stays); the legacy runtime resets its own state | none | navigation suites; default-law test | M |
| S8 | `outcome()`, `before_review()`; pass-policy split; ORBIT prep as legacy contribution; `navigation/__init__.py` drops legacy | none | per-site equivalence; guard v2 freezes the is_active list | L |
| S9 | NavigationArgs stops building PIDArgs (`src/navpy/args/navigation_args.py:85-87,122`); the legacy factory builds it and MissionPlanner (`runtime_composition.py:168`); the test default flips to vision and legacy-dependent tests (count UNVERIFIED) get the legacy fixture, assertions unchanged | none | test_legacy_removable (meta_path); removal-set guard | M-L |
| S10 | Mechanical move of the legacy files into `navigation/legacy/` | none | path guards, patch strings, bench imports; annotations if re-pinned | L |
| S11 | LOITER leaves ACTIVE_MODES (`nav/navigation_decision.py:42`) | CHANGE: operator LOITER -> ONHOLD, task cleared | invert `test_nav_controller.py:508-514` per ruling 4; eval run | S |
| S12 | Floor per phase: the law read in RecoveryPorts -> a floor policy with the NAV exemption (Q3); RECOVERY commands a safe state that works in AUTO (attitude needs GUIDED, `mission_modules.md:65`; open floor task); GCS upload check | CHANGE: under vision the floor applies outside NAV (today off all sortie, `nav/recovery.py:50-51`) | rewrite `test_nav_controller.py:6532-6546` with owner OK; exemption tests; evals set nav_min_alt; eval run | M |

- Decided (2026-10-08): flows are graphs (`mission_modules.md`). S3 becomes the flow runner plus delivery's graph, which reproduces today's 20 `phase.request` edges. FlowHooks below is superseded. The runner design is in `flow_runner.md`.
- Order (merged into `roadmap.md`): S0, T0, T1, then S1-S10. S11 and S12 depend on neither; each lands when ruled and
  re-baselines the eval once. No slice grows a facade at its ceiling
  (`tests/guards/solid_architecture_guard.py:179-241`) without an owner-approved ratchet.
- Pickup step (not sliced here): roles and bid sentinel; CATCH; RESET split (end of pass as a
  step selection over the same transaction); line_up; line_dive and entry geometry; final
  approach built only for roles that use it (needs NavigationVehicleCommands at the root and
  a null Navigation); GPS-assist phase rule with a mechanism test; data-driven class catalog
  for the line class; fail-closed confirm per kind; module table and id; GCS items above.

## What stays put
- `vehicle/`, `common/`, `comm/` wire values (DOCK=1, UNKNOWN=5), the `swarm/` auction
  core, the `vision_nav` path and command hub (guards pin them), `nav/` until extraction.
- `vision/`, including ideal_360, and the dock class and preset name
  (`vision/vision_class_profile.py:17-21`): the dock is general-purpose (glossary), and the
  GCS loads profiles without NavPy composition (`src/gcs/backend/config.py:68-69`).
- Static arg groups; AAS_NAV_ONESHOT in NavArgs; the one-shot latch in NavPhaseState.
- All names: AAS_DEL_*, AAS_NAV_*, AAS_TARG_*, routes, WS events, `default_delivery_hub(s)`;
  eval markers byte-identical (`scripts/eval_gcs_demo_gates.py:136-145,179-189`).
- `nav/mission_encoding.py` wire layout until storage versions it; one `aas_params.lua`;
  GCS files (labels only); eval and bench scripts (on the removal checklist).
- Dead code and compatibility facades: a separate owner-approved cleanup task.

## Owner answers (2026-10-07)
- **A1 Circle until confirmed.** In delivery and pickup alike, the UAV circles the POI until confirmation, then approaches.
  - So the eval's `Self-detect orbit:` gate is right. The vision path must circle too: a new shared-flow slice with a behavior change.
  - The circle may not use sim truth (`nav/self_detected_approach.py:84-88`).
- **A2 GPS assist** before the approach stays off until a good approach module exists.
- **A3 Altitude floor:** off by default in the platform. Every module owns its safety checks; they plug in like hooks. This replaces S12's platform floor.
- **A4 Params:** a param that belongs to a module lives in that module. That covers the legacy AAS_DEL_* params, the delivery params, and their GCS and Lua rows. After the legacy package is removed, AAS_DEL_CTRL=0/1 refuses NAV with a status text (default unless the owner objects).
- **A5 Positioning** is a hardware black box for the swarm. Every source feeds the autopilot, and NavPy reads only the fused result.
- **A6 Airspeed:** without a real airspeed sensor, the final approach must not use airspeed (task open).
- **A7 GCS:** needs a plugin mechanism per mission. It lands with pickup, the first GCS plugin. Delivery's hub UI moves into its plugin at extraction.
- **Still open:** re-pin PINNED_BASE at S0, so the size and annotation guards run during the split?
