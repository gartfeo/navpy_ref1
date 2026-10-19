# Delivery demo by hand: prep command and the stale-after-approval failure

Status: design v3 (after Codex reviews 1-2), 2026-10-19. Owner OK pending.

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

**Operating choice for the hand demo:** a lower speedup widens the receipt
budget (1/S s of wall time: 250 ms at 4x). That is a mitigation, not a fix,
and its margin is checked live: the live test reports pose-receipt-to-command
age and source age per command with the page open.

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
- `load_delivery_scenario(FIXTURE_PATH)`; `prepare_sim_docks` writes
  `fleet_docks.json` and checks the fixture runtime;
- settings = `build_settings(scenario)` plus `default_delivery_hubs` from the
  fixture's `inputs.delivery_hubs`. It keeps the eval's `auto_connect false`
  and `companion_lifecycle explicit`, so nothing starts NavPy on old
  missions. Each UAV's own hub comes with its mission, not from this list's
  order;
- env = `build_environment(...)` plus `NAVPY_SIM_DOCKS`;
- `prepare_sitl_root(run_dir)`: fresh eeprom, so no stored mission;
- `launch_stack(..., headless=False, browser=False, speedup=S)`: the eval's
  launch with the scenario home, `-n 3` and a launch token, with the frontend
  but no browser. The browser opens only after prep, because a page open
  before the upload sees empty missions and never shows a plan to launch
  (`planDownloadRun.js:98`, `useVehicleConnection.js:479`);
- writes `demo_launch.json` in the run dir: launch token, speedup, home, and
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

- `launch_command` and `launch_stack` take `headless=True`, `browser=True`
  and `speedup=SIM_SPEEDUP` keywords; the registry check compares with the
  requested speedup.
- the end of `launch_stack` (backend health wait, generation re-check, API
  close on failure, `StackContext` build) becomes
  `owned_stack_context(env, entry, clock, scenario)`, used by both.

Module size: about 170 lines. Tests (no SITL): argument parsing; refusals (no
stack, token not found, a hash changed, backend settings differ, companion
running); settings carry the fixture hubs and explicit lifecycle; env carries
`NAVPY_SIM_DOCKS`; prep calls the eval steps in order and opens the browser
last (fake context); `launch_command` default argv unchanged and the new
keywords map to `--no-browser`, no `--headless` and `--speedup`; the registry
check accepts the requested speedup and rejects another; the extracted
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
   reload it after: the plan must show and Launch must be enabled. The
   owner's slot 0 stack still runs.
3. Push to main fast-forward only; the targeted tests on main.

## Owner decisions

1. Default speedup for the hand demo: 10x (eval parity; the same budget that
   failed) or 4x (recommended; 250 ms budget, checked in the live test).
2. May the live test run while slot 0 runs, or do you stop slot 0 first?
3. The trace of the 30-50 ms frame delay as a separate task: yes or later.
