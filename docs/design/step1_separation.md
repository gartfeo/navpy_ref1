# Refactor step 1: separate modules (draft, 2026-10-09)

**Scope.** Step 1 only moves code. Each move is a `git mv` plus import updates, with no behavior change and no new abstractions. Each commit goes straight to `main` and leaves the full suite green.

**How it was planned.** Six agents mapped all 741 modules at `e0fa68d`. An architect drafted the plan, and two checkers reviewed it. A scratch copy confirmed that every module still imports after the shims are removed. Paths below are under `src/navpy/`.

## Proposed final layout (2026-10-10, awaiting owner approval)
This layout comes from the owner's rulings and the nature research (`docs/research/nature_inspired_architecture/report.md`). Once it is approved, the target tree and the commits below get re-planned to match it. Until then, they follow the earlier draft.
```
missions/    goals, on top: delivery (step 2), pickup (later); pick plugins at setup, act only through skills
core/        the platform, always there
  kernel/      shared types, settings, logging, time
  ports/       contracts every plugin implements (versioned)
  flow/        one loop per UAV, runs the mission graph
  safety/      soft reflexes: hold, most severe wins, takeover log
  skills/      search, confirm, observe, acquire, approach (each with a contract)
  world/       shared world model: own state, POIs, peers, fading scout reports
  review/      operator confirmation, used by the confirm skill
  swarm/       roles, tasks, auction plus insect rules
  link/        message set and codec between UAVs
  perception/  camera → detection → tracking pipeline, input checker, geo math, gimbal owner
  approach/    engine that runs the chosen law; profiles
plugins/     swappable behind ports; each one can become its own repo
  vehicle/ardupilot/  moves (goto, hold, orbit, climb) plus body state
  radio/ camera/ gimbal/ detection/ tracking/
  laws/        vision_pn, geo (PID + PN): inputs in, commands out
sim/         simulator versions of the plugins (dev and tests only)
app/         startup: reads the vehicle profile, picks plugins, wires core
```
- **Imports:**
  - Missions use only `core`.
  - Plugins use only `core/ports` and `kernel`, and never each other.
  - `core` never uses plugins, missions or `sim`. Today's backend if-chains stay ratchet entries until the registries replace them.
  - Only `app` wires everything together.
- **Reflexes:**
  - Hard reflexes stay on the autopilot: geofence, failsafes, command timeout.
  - Soft reflexes live in `core/safety`. A skill may loosen one only within declared limits, and only while it runs.
- **Pure laws:** the PID/PN orbit prep and recovery move into the approach skill, so the law plugins stay pure.
- **Step 1 still only moves code.** These come later in the refactor:
  - `safety/` and `world/`;
  - skill contracts and the input checker;
  - registries;
  - the insect rules.

## Target tree (earlier draft)
```
core/        shared types, logging, config, small utils; ports/ (contracts, e.g. the approach-law API)
vehicle/     ArduPilot/MAVLink adapter, incl. mission encoding and waypoint helpers
link/        radio transport, codec, all NAVLINK messages
swarm/       tasks, auction, peers
perception/  camera/ gimbal/ detection/ tracking/ geo/ pointing/; the root holds the detector pipeline and the vision controller
approach/    engine/ (final-approach surface, NAV workflow, logs)
             laws/vision_pn/
             laws/geo_l1/ (PID and PN share the L1 lateral library; they split into two plugins later)
behaviors/   goto, hold, acquire, peer approach, orbit
             geo_approach/ (PID/PN orbit prep, recovery, mission writer)
flow/        today's nav runtime: loop, state machine, recovery, task reset; review/ (operator POI review)
sim/         simulator (ships until the sim-seam step)
app/         composition root, argparse
main.py, main_gui.py stay. The ESP32 trigger and simulator move to src/gcs/backend.
```

## Boundary rules (guard test)
- `core` imports nothing else. `vehicle` and `link` import only `core`.
- `perception` imports `core` and `vehicle`. `swarm` imports `core`, `vehicle` and `link`.
- Laws import only `core`. `vision_pn` never imports `vehicle`, `perception` or `sim`.
- `approach/engine`, `behaviors` and `flow` have no order among themselves yet. The flow-runner step sets it. None of them imports `sim` or `app`.
- No product module imports `sim`. `app` may import anything.
- **Ratchet.**
  - About 50 of today's wrong-way imports are listed, each with the step that fixes it.
  - A new wrong-way import fails, and so does a listed entry that no longer exists.
  - Any unresolved `navpy.*` import fails.

## Commits
| # | What |
|---|---|
| c00 | Guard, move manifest, move tool (git mv plus import and string rewrite) and message-registry census |
| c01 | Delete compat shims and re-export facades (repo rule: no shims) |
| c02 | Pure-move splits of mixed files |
| c03-c04 | `core`: types and utils, then logging and config |
| c05-c06 | `vehicle`; `link` (ESP32 code goes to the GCS) |
| c07-c10 | `perception`: camera + gimbal, geo + detection, tracking + pointing, root |
| c11 | `sim` |
| c12-c13 | `approach`: laws + `core/ports`, then engine |
| c14 | `behaviors` + `geo_approach` |
| c15 | `swarm` |
| c16-c17 | `flow/review`, then `flow` |
| c18 | `app`. Delete the old folders. Update agent docs and active design-doc paths. Reinstall. Eval smoke + live GCS check |

## Not in step 1
- **`missions/delivery`.** The delivery files mix generic and DDH logic, so moving them whole would make `flow` depend on delivery. Delivery gets its own folder in step 2, with small splits.
- Registries, the law plugin API and the flow runner.
- Renames (`legacy_*`, `*_fleet`) and dead-code removal.
- The repo split (PEP 420 packages).

## Checker findings the commits must keep
- **Message registry.** Import side effects fill `MsgRegistry`. Without the facades, the GCS backend registers 4 message types instead of 11. The c00 census pins this, and explicit imports fix it.
- **c01 test updates:**
  - Update the tests that read deleted files: the vehicle-logger caps, the `test_clean_imports` strings, and the tests that use facade module objects or patch strings.
  - Delete only identity asserts. Keep size inventories, because the Testing Policy forbids weakening tests.
- **What the move tool rewrites:** attribute references, `mock.patch` strings and logger-name literals. It deletes emptied folders, including `__pycache__`. The resolution guard reads the source files themselves, not `find_spec`.
- **Path depth.** `real_detector_factory` keeps `parents[4]`. `vision_profile_loader` sits next to `vision_profiles.json`. Unpatched path tests cover the model, `.sim` and the JSON.
- **SOLID guard.** It compares AST shapes after mapping moved names, so a pure rewrite is not "material". `test_public_adapter_facets` scans `src/navpy` and asserts it found something.
- **Placement fixes:**
  - `mission_encoding` goes to `vehicle`, not `core`.
  - `calc_data` comes from `navigation/`.
  - `confirmation_legacy_prep` and `peer_poi_mission_writer` go to `geo_approach`.
- **Docs.**
  - c18 updates all `.claude/agents/*.md` that cite moved paths, then runs `gen_codex_agents.py`.
  - c18 rewrites the paths in the active design docs; `docs/validation` stays as history.
- **Evals.** Source identity hashes change, so the evals are re-baselined after c18.

## Before c00 (owner)
1. **Re-pin the size guard.** Done in `145d9a8`.
2. **Approve the layout** above, then re-plan the commits to match it.
3. **Pause other work.** The camera chat (PR #15) has uncommitted changes in files that step 1 moves. Merge or park it first. Pause other chats and GCS stacks while step 1 lands.
