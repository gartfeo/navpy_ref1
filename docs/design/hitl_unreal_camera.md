# HITL camera stream from Unreal

A simulated camera for hardware-in-the-loop runs: the HITL bridge's plant pose drives Unreal, Unreal
renders the fixed camera from it, and navpy receives every frame stamped with the board time of the pose
it shows. First step: live stream during HITL (replay mode comes later).

```
board / SITL hil --HIL_ACTUATOR_CONTROLS--> bridge (JSBSim) --HIL_STATE_QUATERNION--> board
                                              |  after the reply, one pose per 20 ms of board time
                                              v  UDP 17110, vehicle_state.v1 + board_time_us + sequence
                                  Unreal NavPyVehicleStateReceiver (exact pose, no smoothing)
                                              |  gimbal actor captures with the applied pose's stamp
                                              v  TCP 18110, NPYC v2 (pixels + board_time_us)
                         navpy npyc:// backend --(board time -> ATTITUDE receipt clock)--> FrameProvider
```

## Rules

- Simulator truth goes to Unreal only. navpy gets pixels and the board TIME of the rendered pose, never
  the pose (Pure Vision Approach Constraint); `tests/guards/test_unreal_pose_isolation.py` holds it.
- The board never waits: the pose is sent after the reply, on a non-blocking socket; a dead or slow
  Unreal only counts dropped poses (`tools/tuning/hitl_bridge/unreal_pose.py`).
- Altitude: navpy, the bridge and the board use MSL; every Unreal/Cesium height (georeference origin,
  receiver origin, `--pose-at`, the dock-terrain log) is a WGS84 ellipsoid height. One undulation joins
  them: `--unreal-pose HOST:PORT ALT_OFFSET_M` always carries it; there is no default. aas-hitl measured
  23.5 m at Yeghvard (EGM96 cross-check 24.8 m). The Yeghvard origin is the Cesium terrain there
  (1316.29 m ellipsoid, sampled 2026-10-14) minus 23.5: 1292.8 m MSL, georeference origin 1316.3 m
  ellipsoid, so an aircraft at rest on the origin stands on the Cesium terrain.

## Wire formats

- `navpy.unreal.vehicle_state.v1` gains optional `board_time_us`, `sequence` and `session_id`; older
  receivers ignore them. One datagram per 20 ms slot of board time (`POSE_PERIOD_US`), the first frame
  in each slot, so a 19.992 ms SITL loop is not halved.
- NPYC v2 header (56 bytes) = v1 (36 bytes) + `board_time_us` u64 + `pose_sequence` u64 + `pose_flags`
  u32 (bit 0 stamp valid, bit 1 exact: the rendered pose IS that sample). The plugin writes v1 unless
  `NativeCameraHeaderVersion=2` / `-NavPyNativeCameraHeaderVersion=2`, so v1-only readers keep working.

## Stamp mapping (navpy)

`src/navpy/core/perception/camera/board_clock_map.py`: the board time and ATTITUDE `time_boot_ms` are
the same autopilot clock. Over the newest boot's recent ATTITUDE samples (64, at least 10),

    wall(board_t) = board_t + min(receipt_time_s - time_boot_s)

the lower envelope of the link. A frame is EXPOSURE-stamped with that only when its pose is stamped and
exact and the board time is within 2 s of the newest ATTITUDE; every other frame gets a READ stamp at
receipt. Calibration consequence: for an `npyc://` source, `attitude_link_delay_s` is the typical
ATTITUDE delay above that minimum, not the full link delay.

## Unreal (C:\repos\unreal-projects\AasSimProject)

- Receiver `bPresentLatestPoseExactly` / `-NavPyExactVehiclePose=1`: smoothing, buffering and the
  interpolation delay off; the applied pose's board time is kept for the camera.
- Gimbal actor `bFixedBodyCamera` / `-NavPyFixedBodyCamera=1`: rigid mount at the Initial angles
  (pitch -35 deg), roll included; gimbal commands ignored. Native camera connects without blocking the
  game thread (a blocking connect to a closed port cost about 2 s per frame on Windows).
- `UNavPyDockBoardSubsystem` places the ChArUco board on the dock at play time (the level is not edited):
  2.4 x 3.2 m, 40 cm squares, navpy's default board (`DICT_4X4_50`, 5 x 7, marker 0.74 of a square).
  Texture: `tools/unreal/charuco_board_texture.py`; assets: the project's
  `Scripts/create_charuco_board_assets.py`. Metric use needs the profile override
  `detector.charuco: {square_mm: 400, marker_mm: 296}`.

## Running it (PowerShell 5.1)

1. Unreal (game, UAV1 camera on 18110), without sound and pixel streaming, capped at 60 fps and below
   the bridge in priority (see "Sharing the PC with the board link"):
   `$ue = Start-Process -PassThru 'C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe' -ArgumentList 'C:\repos\unreal-projects\AasSimProject\AasSimProject.uproject','/Game/AAS/Levels/Yeghvard_Default','-game','-windowed','-ResX=1280','-ResY=720','-NavPyEnableNativeCameraStream=1','-NavPyExactVehiclePose=1','-NavPyNativeCameraHeaderVersion=2','-NavPyFixedBodyCamera=1','-NoRayTracing','-NoVSync','-nosound','-ini:Game:[/Script/NavPyUnrealBridge.NavPyGimbalCameraActor]:bEnablePixelStreaming=False','-ini:Game:[/Script/NavPyUnrealBridge.NavPyOperatorViewActor]:bEnableTailPixelStreaming=False','-ini:Game:[/Script/NavPyUnrealBridge.NavPyOperatorViewActor]:bEnableGimbalInset=False','-ExecCmds="t.MaxFPS 60"'; Start-Sleep -Seconds 5; $ue.PriorityClass = 'BelowNormal'`
2. The bridge as usual plus `--unreal-pose 127.0.0.1:17110 23.5`, with the Yeghvard origin
   (`--origin-lat 40.3117412 --origin-lon 44.4552112 --origin-alt 1292.8`) on both the bridge and the board
   (runner `--origin 40.3117412,44.4552112,1292.8,0`; a board's `SIM_OPOS_ALT` 1292.8).
3. navpy with frame source `npyc://127.0.0.1:18110`, or the check tool:
   `Set-Location C:\Users\GARTF\Downloads\navpy-temp2; $env:PYTHONPATH = "$PWD\src;$PWD"; .venv\Scripts\python.exe -m tools.unreal.npyc_live_check --mavlink udpin:0.0.0.0:9640 --seconds 60 --out C:\tune_hitl\cam_check`
   (`--mavlink`: any pymavlink link carrying the board's ATTITUDE, e.g. the runner's `--mp-mirror` port
   9640; frames.csv, summary.json and sample PNGs land in `--out`).

## Dock passes (`sitl_driver dock-pass`, 2026-10-14)

A test card that flies the camera past the board (`tools/tuning/dock_pass.py`). It is a test harness: the
board's known position shapes only this card's path, and the truth is used only offline for scoring. navpy's
navigation and mission modules get no dock information.

- **Board.** Centre lat 40.31205630, lon 44.45815163, 1320.365 m ellipsoid = 1296.865 m MSL (Unreal log
  `NavPy ChArUco board centre`). That is 250 m east, 35 m north and 4.07 m above the Yeghvard origin. It lies
  flat, 2.4 m east-west by 3.2 m north-south (deck and board yaw 0, Unreal +X east).
- **Card.** After the catapult takeoff, AUTO flies 8 straight passes at 24 m/s (`DO_CHANGE_SPEED`):
  - above the board: 40 m heading 0, 25 m heading 90, 15 m heading 180 and 270, 10 m heading 0, 90, 180 and 270;
  - each pass is approach (900 m before the board), entry (450 m) and exit (150 m past it);
  - heights above home are taken from the autopilot's `HOME_POSITION` after the takeoff (set on the rail,
    0.9 m above the origin in SITL);
  - the pass window opens at the entry and closes at the exit; the run's JSON records it in board time as each
    pass ends;
  - a loiter at the last exit keeps AUTO until the runner switches to RTL. A last waypoint or a
    `NAV_RETURN_TO_LAUNCH` item switched to RTL before the last `MISSION_ITEM_REACHED` was read.
- **Floor.** The run fails below `--min-height` (default 7 m above home) and recovers in RTL, as port-check
  does. A floor within 5 m of the lowest pass (about 13 m above home) is refused before the flight, and again
  against the autopilot's home after the takeoff.
- **Commands (PowerShell 5.1).** Unreal as in "Running it" step 1. Then the live check, with a duration
  longer than the run, every frame checked and each detected frame saved:
  `Set-Location C:\Users\GARTF\Downloads\navpy-temp2; $env:PYTHONPATH = "$PWD\src;$PWD"; .venv\Scripts\python.exe -m tools.unreal.npyc_live_check --mavlink udpin:0.0.0.0:9640 --seconds 2400 --detect-every 1 --save-detected --out C:\tune_hitl\dock3_frames`
  In a second window, the card (the `port-check` command of flight_tuning.md B, Yeghvard origin):
  `Set-Location C:\Users\GARTF\Downloads\navpy-temp2; $env:ARDUPILOT_ROOT = "\\wsl.localhost\Ubuntu-24.04\home\gart\ardupilot"; .venv\Scripts\python.exe -m tools.tuning.sitl_driver dock-pass --board COM15 --erase-params --mp-mirror --origin 40.3117412,44.4552112,1292.8,0 --workdir C:\tune_hitl\dock3 --params-file "$env:LOCALAPPDATA\navpy-hitl-tools\params\peron15_161.parm" --bridge "{python} -m tools.tuning.hitl_bridge --device COM11 --plant package --reconnect-interval-s 0.5 --model-dir $env:LOCALAPPDATA\navpy-hitl-tools\models\Peron15_hitl4 --servo-speed-s 0.03 --origin-lat {lat} --origin-lon {lon} --origin-alt {alt} --heading {hdg} --exit-on-stdin-eof --log-csv {link_csv} --unreal-pose 127.0.0.1:17110 23.5"`
  Once the check has written its summary, the report:
  `.venv\Scripts\python.exe -m tools.unreal.dock_pass_report --run-json C:\tune_hitl\dock3.json --link-csv C:\tune_hitl\dock3\link.csv --frames C:\tune_hitl\dock3_frames --out C:\tune_hitl\dock3_report`
- **Report.** `tools/unreal/dock_pass_report.py` joins each frame with the bridge's `link.csv` row of its pose's
  board time. It projects the board into the fixed body camera (`tools/unreal/dock_pass_geometry.py`):
  - pitch -35 deg in the body, 90 deg horizontal FOV, 1280 x 720;
  - 2.5 m ahead of and 0.3 m below the vehicle position, measured on static bench poses with the same ChArUco
    PnP (2.33-2.61 m, 0.10-0.38 m); without it the truth outline sat tens of pixels above the board;
  - a frame is "in view" when all four board corners fall inside the picture.
  
  The report counts detections per pass and per slant range. On the saved frames it scores the ChArUco
  solvePnP board pose against the truth. Sample frames show the truth outline in green and the detected corners
  in red.
- **SITL stand-in.** Run the fork's SITL `hil` model directly with `--serial8=tcp:8168`, and add `SIM_RC_FAIL 1`
  to its defaults. Without it, SITL's default sticks put `FLTMODE_CH` 5 high and select AUTOTUNE before the
  runner's overrides start, and the loaded gains then differ. Connect the runner with
  `--board tcp:127.0.0.1:8160` and the bridge with `--device tcp:127.0.0.1:8168`.

Measured on the real board (dock2, 2026-10-14, Peron15_hitl4, Unreal BelowNormal, pixel streaming off). These are
simulation numbers on rendered frames, not a hardware camera. All 8 passes flown, lowest height 11.3 m above home,
no floor breach. Link gate passed: stale 3.4%, age max 14.8 ms. Frame latency in the windows: median 110 ms,
p90 122 ms, max 182 ms. "In view" counts frames from far out (most are beyond 60 m), hence the low per-pass rates.

| Pass | Height, heading | Frames | In view | Detected | Max detected slant | Position error median / p90 |
|---|---|---|---|---|---|---|
| 1 | 40 m, 0 | 422 | 329 | 0 | - | - |
| 2 | 25 m, 90 | 416 | 332 | 0 | - | - |
| 3 | 15 m, 180 | 409 | 333 | 0 (1 partly out of view) | - | - |
| 4 | 15 m, 270 | 424 | 344 | 1 | 18.0 m | - |
| 5 | 10 m, 0 | 433 | 350 | 2 | 14.3 m | 0.09 / 0.14 m |
| 6 | 10 m, 90 | 421 | 335 | 4 | 14.7 m | 0.10 / 0.13 m |
| 7 | 10 m, 180 | 420 | 330 | 4 | 14.7 m | 0.14 / 0.21 m |
| 8 | 10 m, 270 | 444 | 338 | 3 | 13.9 m | 0.13 / 0.23 m |

By slant range (camera to board centre): below 15 m, 13 of 13 in-view frames detected (position error median
0.11 m, p90 0.21 m; tilt error at most 1.7 deg); 15-20 m, 1 of 24; 20 m and beyond, 0 of 2654. The SITL stand-in
(dock_sitl6, same card) agreed: 8 of 10 below 15 m, 2 of 18 at 15-20 m, none beyond. The pose errors are against
the camera offset calibrated with the same ChArUco PnP, so they bound consistency, not absolute accuracy. At
19 fps and 24 m/s the board is within 15 m for under a second per pass, so a pass gives at most a few detections.

- The board's DataFlash log of this run was 175 MB; its MAVLink download took about 60 minutes after landing.
- The first board attempt (dock1) stopped after the takeoff: the home on the rail, 1.1 m above the origin, put
  the lowest pass 0.05 m inside the 5 m margin above the then 8 m floor. The floor is now 7 m and that check fails the run cleanly.


## Measured (2026-10-13, laptop RTX PRO 1000, Unreal -game windowed)

Unit and loopback tests only prove the mechanism; these are simulation numbers, not hardware.

| Run | fps | index gaps | EXPOSURE | receipt - mapped board time p50 / p90 / p99 / max |
|---|---|---|---|---|
| SITL hil + bridge + Unreal, 60 s | 17.1 | 0 | 877 / 1024 | 79 / 117 / 149 / 175 ms |
| Bench pose over the dock, 30 s | 17.0 | 0 | 511 / 511 | 90 / 120 / 167 / 181 ms |

The 147 READ frames of the SITL run all came before ATTITUDE started streaming. ChArUco was detected
in 158 of 170 checked frames of the dock run (from below the terrain, before the altitude fix).

After the altitude and exposure fix (2026-10-14, same laptop): SITL hil + bridge + Unreal 60 s, 24.9 fps,
0 index gaps, 1472 / 1494 EXPOSURE, 144 / 156 / 168 / 187 ms, mean grey 142 with nothing saturated, the
aircraft at rest 1.0 m (the rail) above the terrain. Bench pose 15 m above the board: ChArUco in 434 of
435 frames, mean grey 105. The dock base sits on the terrain (+0.00 m, Unreal log
`NavPy dock terrain check`).

## Sharing the PC with the board link (2026-10-14, real board, port-check, check 600 s)

The board counts a 5 ms loop stale when the bridge's reply arrives after its next frame (gate: at most
5% stale, age at most 50 ms). The first board run with the stream (cam1) failed the gate and the camera:

| Run | Unreal / bridge | stale | age max | camera connections | fps | EXPOSURE | latency p50 / p99 |
|---|---|---|---|---|---|---|---|
| hitl4, no Unreal | - / normal | 0.5% | - | - | - | - | - |
| cam1 | pixel streaming on, old sender / normal | 14.0% | 19.8 ms | 44 | 20.4 | 4804 of 12211 | 105 / 169 ms |
| cam2 | streaming off, -nosound, new sender / normal | 9.3% | 15.1 ms | 1 | 19.0 | 4539 of 4545 in span | 115 / 164 ms |
| cam3 | same, Unreal BelowNormal / bridge High | 2.8% | 12.5 ms | 1 | 19.1 | 4515 of 4523 in span | 115 / 147 ms |
| cam4 | same, the bridge raising itself (committed) | 3.2% | 12.2 ms | 1 | 19.0 | 4441 of 4449 in span | 114 / 156 ms |

- Stale loops were spread evenly, not in bursts, and followed the bridge's reply time: its processing
  per frame went from 0.6 ms (median, armed) without Unreal to 1.1-1.2 ms with it. The project's default
  pixel streaming (1080p gimbal every frame, 60 fps tail, inset) kept Unreal at about 10 cores; off, 3.
- Reconnects: the camera actor closed the socket whenever a send would block (the reader a frame
  behind), and each reconnect lost the frames in flight. It now keeps one frame in flight, finishes it
  on later ticks and drops new frames while it is pending (logged at powers of two); only a real socket
  error closes it.
- EXPOSURE: a frame is EXPOSURE only when its pose's board time is within 2 s of the newest ATTITUDE. The
  check ran 600 s, the flight about 210 s; before it Unreal showed the previous run's last pose, after
  it the pose stayed frozen while ATTITUDE went on, so those frames were READ. `exposure_span` in the check's summary counts the frames from the first to the
  last EXPOSURE frame.
- The bridge raises itself to the HIGH priority class on Windows (`process_priority.py`, at the process
  entry only); Unreal goes to BelowNormal (command above). A bench screen with the board idle did not
  show the difference; the board runs did.
- The capture runs on every other game tick (game about 37 fps, capture gate 1/30 s), hence about 19 fps.

## Known limits

- The native camera runs at about 17-25 fps on this laptop, not the configured 30 (about 19 fps beside
  the board link, above).
- Exposure is fixed, never auto: `NativeCameraExposureBias=-13.0` in the project's `DefaultGame.ini`
  (one stop per step, `-NavPyNativeCameraExposureBias=<bias>`); the level's cameras keep no own value.
- With the fixed body camera at -35 deg the board's ChArUco is detected up to about 15 m above it and
  not from 16.6 m; that limit is the board's pixel size, not the exposure.
- The current fork SITL puts the HIL link on SERIAL8 (base port + 8), which `sitl-backing --model hil`
  also gives SERIAL5; the live check ran SITL directly with `--serial8=tcp:8168` and serial0 held inside WSL.
