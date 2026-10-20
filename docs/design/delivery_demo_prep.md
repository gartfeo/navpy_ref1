# Delivery demo by hand: prep command and the stale-after-approval failure

Status: v3 shipped (`26cb913`); waypoint docks v1 (2026-10-20) replaced by the normal-run demo v4 below.

## Goal

The owner drives the 3-UAV delivery fixture (`scripts/eval_delivery_scenario.json`)
in the GCS UI: press Launch, approve docks, and see d1 kept, d2 offered, one
peer win d2 and one UAV go home. One command sets the stack up; the eval's
helpers do the work.

## Why the 2026-10-19 manual run failed

Run: `C:\Users\GARTF\Downloads\navpy-demo\.logs\delivery-demo`, 09:16-09:18.

1. **Stale missions.** Settings had `companion_lifecycle auto`, so connect
   started NavPy on the missions left in SITL ("existing mission valid" in
   `gcs/backend.log`). The settings had one hub at the launch point. So no
   UAV flew the fixture: UAV 2 approached `home-middle` (obj 3) 581 m abeam,
   UAV 3 `home-west` (obj 4).
2. **`NAV_FAIL: source_command_stale` soon after approval** (dock armed to
   disarmed: UAV 2 about 190 ms, UAV 3 about 540 ms wall, debug CSVs). See
   below.

## The stale failure: what is proven

Mechanism (UAV 2, task 1, obj 3; CSV wall times, not the cached log times):

- NAV issued 4 fresh commands from frames at source time 1004.324, .464,
  .664 and .944 (`source-time/pose_cadence_worker_uav_2_navpy.csv`). No newer
  frame reached the NAV ingress: later observations still carry frame 1004.944
  ("duplicate", `..._observation_uav_2_navpy.csv`).
- The held command (`vision_nav/command_hold.py:142`) checks
  `FinalApproachCommandFreshness.is_fresh` (`command_freshness.py:29`): source
  age <= 1.0 s **and** receipt age <= 1.0 s / speedup of wall time, counted
  from the frame's pose receipt. Frame 1004.944's pose arrived at wall 105.9422
  (`..._attitude_arrival_uav_2_navpy.csv`); its 100 ms receipt budget ended at
  106.0422. At disarm the source age was only 0.96 s (debug CSV, source
  1005.904). So, as reconstructed (the shared latch does not record which
  check failed), the **receipt** check failed: the hold failed, the liveness
  latch set, and the NAV tick logged `source_command_stale`
  (`final_approach_nav_workflow.py:120-125`). The pump's own timeout logs
  another reason (`source_receipt_timeout`).
- The frames that did reach NAV were 3.7-52.0 ms (median 40.0) past their
  pose acceptance on UAV 2, and 14.1-56.8 ms (median 32.0) on UAV 3. In a
  passing wall eval the median is 2.2-4.5 ms, 18.5 ms at most. Much of the
  100 ms budget was gone before a frame was used.

Window per UAV: its first DOCK_ARMED to its first DOCK_DISARMED (debug CSV).
Polls: `paired` rows of all UAVs, from 60 s before each window to its end;
nearest-rank percentiles. Reference: the passing wall eval
`.logs/delivery_move_ref/wall_green_c8_6953798` (report `passed: true`).

| | demo run | passing wall eval c8 |
|---|---|---|
| window (wall) | 0.19 s (UAV 2), 0.54 s (UAV 3) | 2.86-2.87 s |
| frames accepted / reaching NAV | 7 / 4 (UAV 2), 20 / 16 (UAV 3) | 429-495 / 391-465 |
| pose acceptance -> NAV ingress, median | 40.0 ms, 32.0 ms | 2.2-4.5 ms |
| detector poll work, median / p99 / max | 1.96 / 18.5 / 49.7 ms | 0.35 / 7.8 / 37.5 ms |

"Emitted" in the pose CSV and `TRACK: IN_FRAME` in the log are written before
render and publication (`pose_frame_clock.py:110`,
`sim_detection_pipeline.py:101`), so they do not show that a frame was
published.

## The stale failure: what is not proven

Why frames took 30-50 ms instead of 1.6 ms to reach NAV. At NAV start, polls
took 26-45 ms wall with thread CPU time at 0 (quantized to 15.6 ms), so the
thread waited. It could have waited on OS scheduling, the GIL or a lock; the
CSVs cannot tell which. NAV start also lowers the process switch interval to
1 ms (`scheduler_cadence.py:224`, from `navigation_command_worker.py:124`);
the eval does the same. Load seen on the PC at the time: the GCS page open,
another session's stack on slot 1 (`sleepy-mirzakhani-74de4d`: 3 SITL, a
backend and 3 NavPy from 09:17:27). At about 09:25:30 the demo's poll median
fell from about 1.4 ms to 0.46 ms, for a reason the logs do not show. None of
these is measured as the cause.

The speedup is the same as the eval's (10x), and so is the NavPy env. This is
todo 31's mode; todo 65 found starvation of this kind under load.

**Next step (follow-up task, not in this change):** trace one approach with
timestamps at pose acceptance, render, publication (accepted or rejected),
snapshot read and ingress, plus lock waits, with the page open and closed, on
a quiet PC. That separates late publication, a rejected publication and an
old snapshot read by a late NAV tick. The finding goes into todo 31.

**Speedup (owner, 2026-10-19): the hand demo runs at 10x, like the eval.** A
lower speedup would widen the receipt budget (1/S s of wall time), but it is
a mitigation, not a fix. The live test reports pose-receipt-to-command age and
source age per command with the page open.

## Design: `scripts/delivery_demo.py`

Two steps. The slot belongs to the checkout that holds the script, so the
script's path picks it. For the owner's checkout (after the merge):

```powershell
C:\Users\GARTF\Downloads\navpy-temp2\.venv\Scripts\python.exe C:\Users\GARTF\Downloads\navpy-demo\scripts\delivery_demo.py launch
C:\Users\GARTF\Downloads\navpy-temp2\.venv\Scripts\python.exe C:\Users\GARTF\Downloads\navpy-demo\scripts\delivery_demo.py prep
C:\Users\GARTF\Downloads\navpy-temp2\.venv\Scripts\python.exe C:\Users\GARTF\Downloads\navpy-demo\scripts\gcs_stop.py
```

`launch` starts the stack with the dock world and home, no browser; `prep`
puts the scenario on it, then opens the UI. Each prints the next command.

**`launch`** (refuses if this directory already owns a stack):

- run dir `.logs\delivery-demo\<YYYYmmdd-HHMMSS>` (fresh: the SITL root and
  logs need one);
- `load_fixture` and `delivery_scenario`; `prepare_sim_docks` writes
  `fleet_docks.json` and checks the fixture runtime;
- settings = `build_settings(scenario)` plus `default_delivery_hubs` from the
  fixture's `inputs.delivery_hubs`. It keeps the eval's `auto_connect false`
  and `companion_lifecycle explicit`, so nothing starts NavPy on old
  missions. Each UAV's own hub comes with its mission, not from this list's
  order;
- env = `build_environment(...)` plus `NAVPY_SIM_DOCKS`;
- `prepare_sitl_root(run_dir)`: fresh eeprom, so no stored mission;
- `launch_stack(..., headless=False, browser=False)`: the eval's
  launch with the scenario home, `-n 3` and a launch token, with the frontend
  but no browser. The browser opens only after prep, because a page open
  before the upload sees empty missions and never shows a plan to launch
  (`planDownloadRun.js:98`, `useVehicleConnection.js:479`);
- waits up to 60 s for the frontend to answer: the launcher does not, and
  a dead `npm run dev` (no `node_modules`, as in a fresh worktree) logs
  nothing;
- writes `demo_launch.json` in the run dir: launch token, home, and
  the sha256 of the fixture, of `fleet_docks.json` and of the settings file.

**`prep`**:

- finds the run whose `demo_launch.json` token equals the slot's verified
  registry `sitl_launch_token`; refuses with the stop and launch commands if
  none;
- refuses if the fixture, the dock file or the settings file changed since
  launch (sha256), or if the backend's settings differ from the file in hubs,
  lifecycle or zoom;
- builds the `StackContext` from the registry entry (shared with
  `launch_stack`); refuses if a companion is already running;
- runs the eval's `run_mission` steps up to the launch: `connect_vehicles`,
  `configure_vehicle_params` (`aas_params_for`: `nav_oneshot`, `nav_last_wp`,
  `targ_wps=0`; ROLL_LIMIT_DEG), `upload_scenario_missions` (each UAV's
  mission and hub, raw readback), `verify_aas_profile`, `start_companions`,
  `start_esp32_simulator`, `wait_for_missions`, `resolve_live_plan`;
- opens the frontend URL and prints: "Ready. If a GCS tab was already open,
  reload it. Press Launch, then approve each dock (30 s each, wall time)."

What the token shows: the running stack is the one this `launch` started,
with the env, home and SITL root it wrote. It assumes nobody restarts the
backend in between; the sha256 and settings checks catch edited files.

Stop as always: the same checkout's `scripts\gcs_stop.py`.

Changes to eval code for reuse (the eval keeps its behavior; default argv
unchanged):

- `launch_command` and `launch_stack` take `headless=True` and
  `browser=True` keywords.
- the end of `launch_stack` (backend health wait, generation re-check, API
  close on failure, `StackContext` build) becomes
  `owned_stack_context(env, entry, launch_token, backend_log,
  navigation_dir, clock=, scenario=)`, used by both (prep passes the
  existing `backend.log`, which `backend_log_file` would refuse).

Module size: about 170 lines. Tests (no SITL): argument parsing; refusals (no
stack, token not found, a hash changed, backend settings differ, companion
running); settings carry the fixture hubs and explicit lifecycle; env carries
`NAVPY_SIM_DOCKS`; prep calls the eval steps in order and opens the browser
last (fake context); `launch_command` default argv unchanged and the new
keywords map to `--no-browser` and no `--headless`; the extracted
`owned_stack_context` keeps the health wait and the API close, and fails
when the registry generation changes during the health wait.

Docs: `src/gcs/AGENTS.md` (how to run it); todo 31 gets this run's evidence and
the open trace.

## Verification

1. Unit tests above; the eval test suite unchanged.
2. live-tester on this worktree's slot, quiet PC: `launch`, `prep`, then the
   UI's Launch and approvals with the page open; pass = d1 kept, d2 offered
   and won by one peer, one UAV homes, no stale NAV_FAIL; report per command
   the receipt age and source age. Also open the page before `prep` and
   reload it after: the plan must show and Launch must be enabled. The owner
   allowed stopping slot 0 first.
3. Push to main fast-forward only; the targeted tests on main.

## Owner decisions (2026-10-19)

1. Speedup 10x.
2. The live test may stop slot 0 (`navpy-demo`) first.
3. The trace of the 30-50 ms frame delay is a separate task.

## Waypoint docks for the hand demo (v1, 2026-10-20)

**Owner (2026-10-20):** the hand demo puts d1 and d2 on mission waypoints, so the
GCS map shows them and the run is easy to check. The shared dock world
(`NAVPY_SIM_DOCKS`) stays for the delivery eval and for later.

**Why the map shows nothing today.** The map draws a sim dock only at the track
point of each bit in a vehicle's `targ_wps` (`dockResolver.js:36-81`, filled from
`/params` in `useVehicleConnection.js:620-647`). The demo sets `targ_wps=0` on
every UAV and keeps d1 and d2 in the dock file, which the GCS never reads.

**How waypoint docks work** (no `NAVPY_SIM_DOCKS`; the rectangle eval's mode):
- Each NavPy builds its own sim world from its own `AAS_TARG_WPS` waypoints
  (`navigation_poi_args.py:42-56`, `poi_provider.py:115-145`), height
  `AAS_TARG_ALT` above ground (demo: 0). Dock ids are per UAV.
- A peer that accepts an offered task gets one dock at the task message's
  position, which in sim is the offerer's truth (`peer_approach_start.py:30-35`,
  `nav_network.py:159-167`).
- A UAV that reaches its hub row gets a dock at the hub
  (`products/delivery/fallback_mission_navigation.py:60-65`).
- Dock location rule: navigation never reads d1/d2 before detection or a task
  message; they feed only the sim renderer. The hub is the rule's exception:
  fallback navigates to the operator-assigned hub before seeing it
  (`fallback_mission_navigation.py:54-66`), as today.

**Change (demo only; the eval is untouched):**
- d1 and d2 sit mid-segment on the owner's (zone-0) northbound leg at lon
  44.4737067, between NAV ordinals 3 and 4. The demo inserts two NAV waypoints
  there, at the fixture's d1 and d2 positions. The points are collinear, so the
  path is the same. d1 becomes ordinal 4, d2 ordinal 5; the hub row stays last.
  `nav_last_wp` (ordinal 1) and detection start are unchanged. Peers' missions
  are unchanged.
- The demo scenario is the fixture scenario with that owner mission and
  `poi_nav_waypoint_ordinals (4, 5)`. The delivery gate profile refuses any POI
  ordinal (`eval_gcs_demo_gate_profiles.py:43`), so the demo builds it in one
  named function: missions through `scenario_missions(..., (4, 5))`, which checks
  the ordinals against the owner track, then `dataclasses.replace`. No eval code
  path builds it; prep runs no delivery gate. Then the eval's `aas_params_for` gives the
  owner `targ_wps=24` and the peers 0, and the eval's upload writes the new
  mission with readback.
- `launch` does not set `NAVPY_SIM_DOCKS` and does not call `prepare_sim_docks`
  (its runtime check refuses `targ_wps != 0`, `eval_delivery_fixture.py:135`);
  the launch record and the hash check drop the dock file.

**Expected run:** UAV 1 sees d1 and d2 in its own world, keeps the first
(`poi_selection.py:72-88`) and offers the other; the peer that wins gets that dock
from the task message; the third UAV flies to its hub and gets the hub dock. The
map shows d1 and d2 on UAV 1's track and the three hubs (UNVERIFIED: the map
uses the plan's track, rebuilt from the downloaded mission; the live test checks
that it includes the two inserted points).

**Limits:** peer and hub docks are not drawn (peers' `targ_wps` is 0); any RESET
drops a peer's or hub's added dock (`navigation_task_reset.py:157-161`), but a
failed approach ends like DOCKED in sim, without a RESET. Which of d1/d2 UAV 1
keeps depends on which it detects first and its selection rules
(`poi_selection.py:72`, `poi_priority.py:29`); the scenario's rule (keep one,
offer one) holds either way, if both are seen.

**Tests:** the demo scenario's owner mission has the two inserted points at the
fixture's d1/d2 positions and the same path; its ordinals are (4, 5) and the
owner's params carry `targ_wps=24`, peers 0; `launch` sets no `NAVPY_SIM_DOCKS`
and writes no dock file; prep's hash check covers the fixture and settings only;
`resolve_live_plan` accepts the demo scenario (prep's steps through plan
resolution, fake context).

**Verify:** live-tester runs `launch` and `prep`, checks the map shows d1 and d2
on UAV 1's track, then Launch and approvals: UAV 1 keeps one dock, a peer wins
the other, one UAV flies home.

## Normal-run demo (v4, 2026-10-20)

**Owner (2026-10-20):** the hand demo must feel like a normal `AAS_GCS.bat` run.
The app auto-connects, NavPy and the ESP32 sim start and restart through the
`auto` lifecycle, and the owner builds or loads a plan in the UI and presses
Launch. In today's run (`navpy-demo\.logs\delivery-demo\20261020-150146`), the
eval's `explicit` lifecycle and `prep` forced the fixture missions, so the owner
E-stopped and relaunched by hand. **Owner choice (A + X):** the demo plan uses
150 m line spacing so two waypoint docks are seen together, and it flies the
normal camera-profile altitude.

**What a normal run already does:**
- Connect starts NavPy per vehicle (`navpy_sim.py:251-256`), and the telemetry
  loop restarts dead ones (`navpy_sim_runtime.py:176-218`).
- Browser discovery starts the ESP32 sim when `launch_type` is `container`
  (`App.jsx:476`, `vehicles.py:92`, `control.py:253-276`).
- A UI upload writes each UAV's `targ_wps`/`nav_last_wp` from the plan's
  `sim_dock_wps`/`detect_after_wps`, then restarts and checks the companions
  (`useMissionUpload.js:168-190`). Other AAS params survive that restart
  (`missionUploadActivation.js:114`).
- Without `NAVPY_SIM_DOCKS`, each NavPy builds docks from its own
  `AAS_TARG_WPS` waypoints only (`poi_provider.py:129`), so peers learn of a
  dock only from an offer.

**Differences from a normal run, with reasons** (all else is normal):

| Kept | Why |
|---|---|
| Run-local SITL root (`prepare_sitl_root`, unchanged) | The demo writes params; the owner's `~/ardupilot/N` eeproms would keep them and stale missions. The root runs the repo's Lua. Its `SITL_DEFAULT_OVERRIDES` restate the shared template eeprom's values (tuning, `ARMING_CHECK 0`, `THR_FAILSAFE 0`, stream rates), so a fresh eeprom flies like the normal one. It also fixes the sim clock epoch, seeds the repo's terrain tiles (`eval_gcs_demo_sitl_root.py:181`, so dock ground heights are the eval's), and uses single-boot/loopback flags. |
| `-n 3`, home = fixture home | 3 UAVs. The home equals `run_swarm.sh`'s default and is stated. |
| Per-run settings file: the checkout's `gcs_settings.json` (if any) plus pins | Demo edits (the plan load writes hubs) leave the owner's settings alone. |
| Pins: `sim_mode`, `simulated_vehicle_count 3`, `uavs_per_set 3`, `takeoff_altitude_m 40`, `altitude_separation_m 20`, `auto_connect true`, `companion_lifecycle auto` | Planner inputs and the normal lifecycle, whatever the owner's file holds. |
| `launch_type container`, `esp32_host 127.0.0.1`, port 9 | The scenario is proven with container launch, which also starts the ESP32 sim. A loopback endpoint keeps a LAN ESP32 untouched. |
| Static AAS params + `ROLL_LIMIT_DEG` (`configure_vehicle_params`, plain fixture scenario: `nav_oneshot`, `targ_alt 0`, `del_*`, …; `targ_wps 0`, `nav_last_wp 1` until upload), checked by `verify_aas_profile` | The UI writes only `targ_wps`/`nav_last_wp`. They are written before the browser opens; the upload's restart applies them. |
| Env: the shell's env minus `NAVPY_SIM_DOCKS`, plus the settings path and run log dirs (incl. source-time CSVs) | `NAVPY_SIM_DOCKS` would replace waypoint docks (`sim_render_composition.py:133`). Logs are evidence for the open stale trace. |

**Dropped:** `prep`, the `explicit` lifecycle, `auto_connect false`, the env
allow-list and thread pins, `demo_launch.json` and its hash checks, the camera
pins (the owner's profile is used; altitude X), the settings hubs (the plan file
brings them), and `delivery_demo_docks.py`.

**Plan file** `scripts/delivery_demo_plan.json`: a normal UI plan file written
by `scripts/delivery_demo_plan.py` (`--check` fails when it is stale). It keeps
the fixture's area height, launch point, pattern, 3 UAVs, partition angle,
hubs and per-zone hub index [2, 1, 0]. The area is narrowed to 1.35 km around
the launch longitude, at `route_offset_m` 150, so each UAV still flies 3 lines
of 3 km. `detect_after_wps {z: 0}` (`nav_last_wp 1`), `sim_dock_wps {2: [1, 2]}`:
the south-end turn waypoints of zone 2, 150 m apart, like d1–d2. Zone 2 is the
lowest (`planner.js:280`). The UI planner takes its altitudes from the camera
profile named in the settings (`plannerConfig.js` `configurePlanner`). The
owner's settings name none, so it uses its built-in defaults: zones 0–2 at
240/220/200 m, close to the fixture's 220 m. The live run flew these, and both
docks were detected there. If the owner selects the ZR10 profile, the zones go to 403/383/363 m. With
the fixed -14° search pitch, only 363 m leaves a window, about 682–730 m out,
where both docks are in frame and in range. At 403 m the frame's near edge
(757 m) lies past the range (709 m) (Codex, calibrated ZR10). Zone 2 is the
only choice that suits both cases, but the ZR10-profile case is not live-tested.
The zone goes to the third vehicle in the UI list.

**One command** `delivery_demo.py`:
1. Run dir, settings and SITL root.
2. `launch_stack(..., headless=False, browser=False)`, which refuses if the
   checkout already runs a stack.
3. Frontend wait, then `wait_for_vehicles` (auto-connect connects them).
4. `configure_vehicle_params` and `verify_aas_profile`.
5. Open the UI, then print the plan file path and the stop command.

Stop with `gcs_stop.py`.

**Tests:**
- Settings: the owner file plus pins; owner keys kept, pins win, no hubs.
- Env: the source env plus paths, with `NAVPY_SIM_DOCKS` removed.
- Steps: in order, browser last, no connect, upload or companion start (fake
  context).
- Plan file: the committed file equals the builder's output.
- A vitest loads the file through the real loader and checks 3 zones of 6
  waypoints each, the default altitudes 240/220/200 m, docks 150 m apart on
  the lowest zone (2), and hubs [2, 1, 0].

**Live test:**
1. Run the one command, load the plan file, upload, Launch, approve.
2. Pass: NavPy restarts on upload, the ESP32 sim is running, the zone-2 UAV keeps one
   dock, a peer takes the other, one UAV homes.
3. Result (2026-10-20, run `20261020-192919`, PC at 99–100% CPU from other
   work): start, auto-connect, NavPy and ESP32 auto-start, plan load, upload
   and companion restart all passed. The zone-2 UAV detected both docks,
   offered one and DOCKED at 0.46 m. The peer accepted the offer, then lost
   detection 7 m out (`no_detection`, DOCK_FAILED). That was an endgame miss
   that began before the offerer's reset. The homing UAV hit the known
   `source_command_stale` bug.
