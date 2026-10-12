# Refactor step 1: separate modules (draft, 2026-10-11)

**Status.** Step 1 is complete (c00-c18): every manifest row is at its step-1 home and the old folders are gone.
The SOLID base is re-pinned to `e9b1591` and the normalizer is removed; the old-path columns below are history.
"Exceptions after step 1" stays as the ratchet.

**Scope.** Step 1 only moves code: `git mv` plus import updates, no behavior change, no new abstractions.
Each commit goes straight to `main` and leaves both test suites green. Paths are under `src/navpy/`.

**How it was planned.** Seven mappers placed all 761 files at `9db0a26`. An architect planned, two critics checked,
a scratch dry-run moved everything, and the architect applied the findings. Replaces the 2026-10-09 draft.
Inputs in `docs/design/step1/`: `manifest.tsv` (per file: old path, final home, step-1 home, action, vision path,
reason), `splits.json` (split symbols, per-part vision flag), `reexports.tsv`, `exceptions.tsv`, `final_violations.tsv`.
c00 adds the tools that read them: `scripts/step1_move.py --commit cNN [--dry-run]` (and `--check` for leftovers),
the loader `scripts/step1_manifest.py`, and the guards `tests/guards/test_step1_*.py`.

## Final layout (approved 2026-10-11)
```
missions/<name>/   goals on top: mission graphs, mission logic (delivery: DDH, one-shot finish; pickup later)
core/              the platform, always there, ours
  kernel/          shared neutral types, settings, logging, time, small pure utils
  ports/           contracts every plugin implements + the neutral types they exchange
  flow/            one loop per UAV: runtime loop, state machine, graph runner, recovery, task reset
  safety/          soft reflexes: supervisor hold, most severe wins, takeover log
  command/         one owner per actuator: vehicle commander, gimbal owner; safety filter right after
  skills/          search, confirm, observe, acquire, approach, plus geo_approach (PID/PN prep, mission writer)
  world/           shared world model: own state, POIs, peers, fading scout reports
  review/          operator POI review, used by the confirm skill
  swarm/           roles, tasks, auction, peers, task actor
  link/            message set and codec between UAVs (NAVLINK messages, registry); not the transport
  perception/      camera -> detection -> tracking orchestration, input checker, geo math, pointing
  approach/        final-approach engine: ingress, frames, command execution, liveness, hold, reset, logs
plugins/           swappable organs behind ports; each can become its own repo
  vehicle/ardupilot/  MAVLink adapter: moves, telemetry, mission encoding
  radio/ camera/ gimbal/ detection/ tracking/   one subfolder per implementation
  laws/            vision_pn, geo (PID + PN): only the math, inputs in, command out
sim/               simulator versions of plugins + sim world (dev and tests only)
app/               composition root: argparse, profiles, backend choice, wiring (+ main.py, main_gui.py)
```
- **Imports.** `kernel` uses only `kernel`; `ports` uses `kernel`, `ports`; other `core` folders use only `core`. A plugin
  uses `ports`, `kernel`, its own family. Missions use `core`, their own folder. `sim` uses `core`, `plugins`, `sim`.
  Only `app` uses anything; nothing imports `app`.
- **Laws** hold only the math; their runtime lives in `core/approach`. **Missions** act only through skills, never on actuators.
- `core/link` is messages and codec, `plugins/radio` only transport. Our messages in MAVLink `TUNNEL` (stock firmware): later task.
- **Reflexes.** Hard reflexes stay on the autopilot (geofence, failsafes, command timeout). Soft reflexes live in
  `core/safety`; a skill may loosen one only within declared limits, and only while it runs.

## What step 1 reaches
Files after step 1, not counting new empty `__init__.py` files (56 new packages; 58 in the final layout with
`missions`, `missions/delivery`), which keep every folder a regular package as today.

| Folder | Files | Subfolders (files) |
|---|---|---|
| `core/kernel` | 39 | flat 17, `config` 10, `logging` 12 |
| `core/ports` | 29 | flat (6 are new split files) |
| `core/link` | 21 | flat |
| `core/perception` | 147 | root 21, `camera` 25, `pipeline` 33, `zoom` 21, `geo` 15, `tracking` 14, `profile` 7 (with the JSON), `pointing` 6, `detection` 5 |
| `core/command` | 18 | command slot + goto/loiter sender 2, `gimbal` 16 (today's gimbal owner) |
| `core/approach` | 72 | root 28, `vision_nav` 22, `logs` 16, `geo` 6 (geo-law runtime) |
| `core/skills` | 37 | `confirm` 15, `acquire` 11, `geo_approach` 8, `search` 3 |
| `core/review`, `core/swarm`, `core/flow` | 18, 35, 20 | flat |
| `core/safety`, `core/world` | 0 | built later; the inactive-navigation hold stays in `core/flow` for now |
| `plugins` | 133 | `vehicle/ardupilot` 71, `gimbal` 21, `laws` 14, `radio` 11, `tracking` 7, `camera` 6, `detection` 3 |
| `sim` | 119 | `detection` 52, `determinism` 24, `gimbal` 17, `camera` 14, `debug_view` 6, `radio` 3, `world` 3 |
| `app` | 41 | root 9, `perception` 15, `flow` 13, `approach` 4; `main.py`, `main_gui.py` stay at the root |
| `src/gcs/backend` | 2 | ESP32 trigger client and ESP32 simulator |
| deleted | 40 | 20 shims/facades (c01), 20 package inits of folders that dissolve |

- **`missions/` is not created.** Delivery code waits in `core/flow` (catalog, DDH fallback route, one-shot finish)
  and the DDH metadata reader in the vehicle plugin until step 2. Final homes are in the manifest.
- **Placement rules:** backend choice and wiring go to `app` now; name lists (law, detector backend) are config in
  `core/kernel`; one folder per concept; approach set-up is the `acquire` skill, `geo_approach` keeps only PID/PN needs;
  `core/command` holds today's arbiters and owners only; a shared type plugins need moves to `core/ports` or
  `core/kernel` by a pure-move split (12 files); importers name the defining module, lines that only re-export go.

## Exceptions after step 1 (ratchet)
After step 1, **28 wrong-way import lines (27 file-to-file edges) in 18 files** remain (the layer rules on the
post-c02 graph). Each has a reason and fixing step in `step1/exceptions.tsv` (`guard=layer`).

| From -> to | Lines | What | Fixed by |
|---|---|---|---|
| `plugins/radio` (7), `core/ports` (3), `plugins/vehicle` (1) -> `core/link` | 11 | transports and `NetworkAbc` handle `MsgABC`, filter, registry | C3 codec port |
| `core/approach` -> `plugins/laws` | 5 | engine uses vision_pn plan, anchor, law | R8 law registry |
| `core/perception` -> `app` | 2 | capture factory, `build_models` | C1 registry |
| `core/perception` -> `plugins/detection`, `plugins/vehicle` | 3 | deep search builds YOLO; charuco config type; pose-stream request | C1 registry |
| `plugins/radio` -> `sim` | 1 | serial transport picks its sim client | C1 registry |
| `plugins/vehicle` -> `core/perception` | 4 | gimbal telemetry types (3, TYPE_CHECKING only); terrain lookup (1) | C3 gimbal split; C1 |
| `core/kernel`, `core/flow` -> `plugins/vehicle` | 2 | `NavArgs` waypoint indices; catalog reads our mission metadata | C3 vehicle port (+ R16) |

- **By step:** C3 codec 11, C1 registry 7, R8 law registry 5, C3 gimbal 3, C3 vehicle port 2.
- **Final layout:** 35 lines in 23 files (`step1/final_violations.tsv`): 27 of the above (the catalog edge goes when step 2
  moves catalog and reader together), 4 `core` -> `missions/delivery` (step 2 hooks, R16, R17), 1 metadata reader ->
  `mission_encoding` (C3), 3 `core` -> `sim` (`navigation_speedup`, C1 sim seam).
- The plain checker sees split files as one unit and imports through facades: 103 lines in 76 files. c01-c02 remove the gap.
- **Guard.** A new wrong-way import fails; a listed edge that disappears fails until removed; an unresolved `navpy.*`
  import fails (read from source). Keyed by file pair; files still in the old tree are skipped.

## Pure-vision guard
- **The set:** 161 manifest rows flagged `vision_path=y`, per-part flags in `step1/splits.json`; after step 1, 168 files
  (core 134, plugins 29, app 5 that only wire): frames -> `VisionFrame` -> law -> command, completion decisions,
  adapters (zoom readback table and attitude history included).
- **Rule:** from each flagged `core`/`plugins` file, follow runtime imports through any `core`/`plugins` file. Reaching
  `core/perception/geo`, `core/world`, `sim`, `core/approach/geo`, `core/skills/geo_approach` or `plugins/laws/geo`
  fails, unless the path crosses one of 15 named edges (`step1/exceptions.tsv`, `guard=vision`):
  - for good: `vision_nav/diagnostics` -> geo distance for the SNAP log row, after the command is formed (1);
  - C3 gimbal split: geo pointing composed with the detector and gimbal owner (`real_detector_ports`,
    `real_detector_controls`, `real_gimbal_adapters`, 5 `core/command/gimbal` geo files) (8);
  - R9: legacy pass tracking in `mission_navigation` and task reset (3); R8: `navigation_lifecycle` resets
    `LegacyNavigationState` (1); R10: the NAV decision calls the confirm skill's track-loss geo hold (1);
    C1: the vehicle facade reaches the terrain lookup (1; the same edge is a layer row).
  - A named edge that no flagged file reaches any more fails until removed.
- **Name rule:** the scan for compass yaw/heading, ground speed, velocity, location, altitude, wind and bbox height keeps
  running on the law and engine command files (`test_vision_source_boundaries`); R2 widens it. `sim` is exempt.

## Commits
c01 runs before any move: moved code must not import a kept facade (in the dry-run, 16 modules failed to import
that way). Then bottom-up, so each layer is checked as it lands. The move tool rewrites every reference in the same commit.

| # | What | Files |
|---|---|---|
| c00 | Guards (layers, ratchet, unresolved imports, pure-vision walk, fresh-process import of every module, registry census) reading `step1/`, move tool, SOLID normalizer, location-independent scanning tests | 0 |
| c01 | Delete 20 shims/facades, empty 10 facade inits, strip 95 re-exported names on 43 lines in 29 kept modules (`step1/reexports.tsv`), drop the `DetectorAbc` base from 3 classes; importers in `src/navpy`, `src/gcs`, `scripts`, `tests`, `tools` name the defining module; the GCS imports its 7 message modules | 20 deleted |
| c02 | 12 pure-move splits into 12 new files (13 parts; `Detection` and `FrameDetector` share `core/ports/detection.py`) | 12 new |
| c03 | `core/kernel` (types, utils, settings, logging) | 39 |
| c04 | `core/ports` | 23 |
| c05 | `core/link`, `plugins/radio`, `sim/radio`; ESP32 code to `src/gcs/backend`; `add_navlink_message.py` paths by hand | 37 |
| c06 | `plugins/vehicle/ardupilot` (its `__init__` replaces the empty one c02 made) | 70 |
| c07 | `perception/camera`, `plugins/camera` | 31 |
| c08 | `perception/detection` and `tracking`, `plugins/detection` and `tracking` | 28 |
| c09 | `perception/geo`, `zoom`, `pointing`, `profile` (new `vision_profiles.json` path named in the message) | 49 |
| c10 | `perception/pipeline` and the perception root | 54 |
| c11 | `plugins/gimbal`, `core/command` | 39 |
| c12 | `plugins/laws`, `approach/vision_nav`, `approach/geo` | 41 |
| c13 | `approach` root and `logs` | 44 |
| c14 | `core/skills`, `core/review` | 55 |
| c15 | `core/swarm` | 35 |
| c16 | `core/flow` (with the delivery pieces that wait for step 2) | 19 |
| c17 | `sim` | 116 |
| c18 | `app`. Delete the old folders. Re-pin the SOLID base to this tip, drop the normalizer. Update docs and `.claude/agents`, run `gen_codex_agents.py`. Reinstall. Eval smoke, live GCS check (`live-tester`), eval re-baseline | 39 |

## Checks every commit keeps
- **Gate.** `python -m pytest tests` and `python -m pytest tests/gcs -o addopts=""` (pyproject skips `tests/gcs`),
  each matching the baseline; the guards; a fresh-process import of every `navpy` module (about 25 s; it catches the
  vehicle-init cycle in-process loops miss); the move tool's leftover-path scan.
- **Message registry.** Classes register on import; without the facades the GCS registers 4 types instead of 11.
  The c00 census (`tests/guards`, fresh subprocess, exact class names) pins GCS 11, `navpy.main` 12 and the swarm-ack
  probe. c01 imports `check_msg`, `swarm_ack_msg`, `swarm_heartbeat_msg`, `swarm_request_msg`, `task_assignment_msg`,
  `task_availability_msg`, `task_confirmation_msg` at module level in a GCS module `main` loads; not `log_status_msg`.
- **c00 scanning tests** become location-independent and assert they found something; every assert stays:
  `test_frozen_inventories` (per-file paths instead of the `vision_nav` folder), `test_vision_nav_prediction` (each home
  of the vision_nav files, which must exist), `test_public_adapter_facets` (all of `src/navpy`, at least today's facet
  count), and the segment-built paths in `test_poi_zoom_solid_regressions`, `test_zr10_profile_intrinsics`, `test_upload_progress_js`.
- **c01 test updates.** Tests switch to the defining modules; only identity asserts go (`test_nav_swarm_facade_identity`,
  `test_task_message_facade` with its size entry for the deleted file, the 4 `DetectorAbc` asserts). The vehicle
  abstract-surface test repoints to `vehicle_contract.IVehicle`; `test_clean_imports` imports the defining modules;
  the sim-alias test becomes "`SimTimebase`, `sim_time`, `sim_timebase` are referenced nowhere". Size inventories stay.
- **Move tool** works on the AST: imports, `module.attr`, dotted strings with implicit concatenations (`mock.patch`,
  logger names such as `peer_orbit_plan`'s), `src/navpy/...` paths, `from pkg import submodule` per submodule. A patch
  on a split module follows the part that looks the name up (16 `real_detector_models` targets go to the builder); on
  a facade, the defining module, or the consumer when it binds at import time. It keeps line endings, never edits
  vendored files, adds an empty `__init__.py` only where no row brings one, deletes emptied folders with their caches
  (any other file left there, hidden ones too, stops the commit untouched), and fails on any leftover old directory
  string, segment-built path or patch prefix.
- **Splits** keep `from __future__ import annotations`, import only names their code uses (not docstring words), keep
  statement order (a merged part takes its sources in `splits.json` order), and each new file imports on its own.
  c02 moves the `TrackedObject` passive-record key to `core/ports/tracks.py`.
- **SOLID guard.** c00 compares each file with its old path at the pinned base: manifest map (with `gcs:` rows),
  per-symbol map for splits, AST normalization of import modules, facade names and dotted strings. A synthetic test
  shows a pure move, a facade rewrite and a split are not material.
- **Path depth.** `dem_tile_repository` (`parents[5]`), `real_detector_factory`, `sim_detector_factory` (`parents[4]`) keep
  their depth; `vision_profiles.json` sits next to both readers, still ships as package data; fix `scripts/gcs_demo_comparison.py`.
- **Docs.** c18 rewrites paths in active design docs, `README.md`, `src/navpy/AGENTS.md`, 7 `.claude/agents/*.md`, the
  `plannerConfig.js` and `pyproject.toml` comments; `docs/validation` stays as history. **Evals:**
  `simtime_navigation_identity.py` hashes `src/navpy`, so they are re-baselined after c18.

## Not in step 1
- `missions/delivery` (step 2: DDH, metadata reader, fallback route, one-shot finish); registries (C1), the law plugin
  API (R8), the flow runner (R9, R10, R18); logic in `core/safety` and `core/world`, skill contracts, the input
  checker, insect rules; renames (`legacy_*`, `*_fleet`); dead code and dead imports (`navigation_data`,
  `observable_gate`, the VISCA controller); TUNNEL; moving `tests/` to mirror the layout; the repo split.

## Before c00
1. **Layout approved** by the owner on 2026-10-11. **Camera chat landed** in `9db0a26`; its worktree
   (`.claude/worktrees/wizardly-dijkstra-550966`) is clean and can go.
2. **Fast-forward the main checkout** from `e91e881` to `origin/main` (`9db0a26`).
3. **Pause other chats and GCS stacks** (`python scripts\gcs_launch.py --list`): every commit touches most of `src/navpy`.
4. **Commit or stash `vision_profiles.json` edits** in every clone and deployed checkout; the GCS rewrites it in place.
5. **Record the baseline:** both suites (known failures: 4 `test_board_detector` tests and 2 annotation violations in
   `camer_calibration3.py`, separate tasks), SOLID guard, registry counts, eval identity.
6. **Owner decisions (2026-10-11):**
   - `core/world` stays empty in step 1; the POI status store and peer roster move there later.
   - Delete the `DetectorAbc` marker and the 4 asserts that only check it stays empty (no shims).
   - `self_detected_approach` ends in `skills/geo_approach`.
   - `InactiveNavigationGuard` stays in `core/flow` until `core/safety` has a contract.
