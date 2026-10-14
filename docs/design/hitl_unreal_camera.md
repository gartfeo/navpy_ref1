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
  (pitch -35 deg), roll included; gimbal commands ignored.
- Gimbal actor `bNativeCameraUseIntrinsics` / `-NavPyNativeCameraIntrinsics=1`: the native camera renders
  through a pinhole with `NativeCameraFxPx`, `FyPx`, `CxPx`, `CyPx` (pixels at 1280 x 720, a custom projection
  matrix, so the off-centre principal point and fx != fy are exact). The defaults are the SIYI ZR10's zoom-1
  calibration on its 1280 x 720 stream (navpy `vision_profiles.json` `siyi_zr10`): 48.9 x 28.8 deg.
  `-NavPyNativeCameraZoom=<factor>` multiplies fx and fy. Off, the camera keeps its FOV angle (90 deg) and
  the demo's zoom commands. No lens distortion is rendered. The capture's show flags turn motion blur, TAA,
  DOF, bloom, lens flares, grain, vignette and eye adaptation off; the HDR scene-colour capture already has
  none of them and no temporal jitter. Native camera connects without blocking the
  game thread (a blocking connect to a closed port cost about 2 s per frame on Windows).
- `UNavPyDockBoardSubsystem` places the ChArUco board on the dock at play time (the level is not edited):
  2.4 x 3.2 m, 40 cm squares, navpy's default board (`DICT_4X4_50`, 5 x 7, marker 0.74 of a square).
  Texture: `tools/unreal/charuco_board_texture.py`; assets: the project's
  `Scripts/create_charuco_board_assets.py`. Metric use needs the profile override
  `detector.charuco: {square_mm: 400, marker_mm: 296}`.

## Running it (PowerShell 5.1)

1. Unreal (game, UAV1 camera on 18110), without sound and pixel streaming, capped at 60 fps, at normal
   priority (the bridge raises itself to HIGH; see "Sharing the PC with the board link" and "Unreal freezes"):
   `$ue = Start-Process -PassThru 'C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe' -ArgumentList 'C:\repos\unreal-projects\AasSimProject\AasSimProject.uproject','/Game/AAS/Levels/Yeghvard_Default','-game','-windowed','-ResX=1280','-ResY=720','-NavPyEnableNativeCameraStream=1','-NavPyExactVehiclePose=1','-NavPyNativeCameraHeaderVersion=2','-NavPyFixedBodyCamera=1','-NavPyNativeCameraIntrinsics=1','-NoRayTracing','-NoVSync','-nosound','-ini:Game:[/Script/NavPyUnrealBridge.NavPyGimbalCameraActor]:bEnablePixelStreaming=False','-ini:Game:[/Script/NavPyUnrealBridge.NavPyOperatorViewActor]:bEnableTailPixelStreaming=False','-ini:Game:[/Script/NavPyUnrealBridge.NavPyOperatorViewActor]:bEnableGimbalInset=False','-ExecCmds="t.MaxFPS 60"'`
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
  - pitch -35 deg in the body; the ZR10 pinhole (`--camera zr10`, default) or the 90 deg camera of the runs
    before the intrinsics (`--camera fov90`: dock1, dock2, dock_sitl1-7);
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

## Camera model and range (2026-10-14)

**Cause of the 15 m limit: pixels per square.** The native camera rendered Unreal's 90 deg FOV (fx 640 px at
1280 x 720). At 15 m and the mount's 35 deg depression a 40 cm square is 17 px across and 10 px along the line of
sight, a marker bit cell 1.2 px along it. ArUco cannot read 4 x 4 bits at that size, and OpenCV's default
`minMarkerPerimeterRate` 0.03 (38 px at 1280) drops the markers there too. Not the cause:
- motion blur: scene captures turn it off, and a static pose gives the same limit as the flight;
- post effects (TAA/TSR, DOF, bloom, auto exposure): the HDR scene-colour capture is taken before them, with a
  fixed exposure and no temporal jitter;
- compression: NPYC carries raw BGR.
An offline render of the board texture through the same pinhole reproduces the limits: 10 corners at 12 m, none
at 15 m for fx 640; 17 at 25 m, 5 at 30 m for fx 1407.

**Fix: the ZR10 camera.** The HITL launch adds `-NavPyNativeCameraIntrinsics=1`, so Unreal renders with the
ZR10's zoom-1 calibration (fx 1407.42, fy 1401.91, cx 670.95, cy 369.78; 48.9 x 28.8 deg). The report and the
static check project and solve PnP with the same K (`dock_pass_geometry.zr10_camera`, read from the profile).

**Static range check** (`tools/unreal/static_range_check.py`): vehicle level and still, heading 0, board centre
on the optical axis (35 deg depression), 10 frames per range. "Detected" = ChArUco pose (at least 6 corners over
2 rows and 2 columns).

| Slant | 90 deg camera: px/square across x along, detected | ZR10: px/square, detected, PnP range error |
|---|---|---|
| 10 m | 26 x 15, 9/10 | 57 x 33, 9/10, -0.10 m |
| 12 m | 22 x 13, 0/10 (5 corners) | 47 x 27, 10/10, -0.15 m |
| 15 m | 17 x 10, 0/10 | 38 x 22, 10/10, -0.10 m |
| 20 m | 13 x 7, 0/10 | 28 x 16, 10/10, -0.07 m |
| 25 m | 10 x 6, 0/10 | 23 x 13, 10/10, -0.09 m |
| 30 m | 9 x 5, 0/10 | 19 x 11, 10/10, -0.33 m |
| 35 m | - | 16 x 9, 0/10 |
| 40, 50, 80 m | 6 x 4 and less, 0/10 | 14 x 8 and less, 0/10 |

The first frame after start shows the board's material still compiling (no pattern); the 9/10 at 10 m is that.
These ran with OpenCV's default marker detection; navpy's detector now uses the settings below.

**Dock passes with the ZR10 camera.** Same card, report `--camera zr10`. In view and detected by slant range:

| Run | below 15 m | 15-20 m | 20-25 m | 25-30 m | 30 m and beyond | position error median / p90 |
|---|---|---|---|---|---|---|
| dock_sitl6, SITL, 90 deg | 8 / 10 | 2 / 18 | 0 | 0 | 0 | - |
| dock2, board, 90 deg | 13 / 13 | 1 / 24 | 0 / 29 | 0 / 28 | 0 / 2597 | 0.11 / 0.21 m |
| dock_sitl8, SITL, ZR10 | - (never in view) | 15 / 16 | 12 / 21 | 15 / 22 | 0 / 154 | 0.09-0.12 / 0.15-0.23 m by bin |
| dock3, board, ZR10 | - (never in view) | 15 / 19 | 16 / 23 | 11 / 25 | 0 / 171 | 0.10-0.13 / 0.15-0.23 m by bin |

- dock3 (real board, Peron15_hitl4, 2026-10-14): all 8 passes, lowest 11.3 m above home, no floor breach; link
  gate passed (stale 3.1%, age max 11.9 ms); frame latency in the windows median 111 ms, p90 122 ms, max 185 ms.
  Detections in passes 3-8 (both 15 m and all 10 m passes), farthest 28.8 m, tilt error at most 1.6 deg; none at
  40 and 25 m (the whole board in view only from 58 m and 36 m).
- dock_sitl8 (SITL stand-in): detections in passes 4-8, farthest 29.2 m; pass 3 had only 17 frames in its window,
  none with the board in view.
- The ZR10's 28.8 deg vertical field keeps the whole board out of the picture close in: at 10 m height it is in view
  only from about 15.5 m slant (dock3), so the bin below 15 m is empty.
- Unreal froze for 22-34 s a few times per run: no capture and no game tick, nothing in its log (dock_sitl8 four
  times, in the pass 3, 7 and 8 windows and once between passes; dock3 twice, in pass 3 and between passes 5
  and 6). Missing frames are not counted as misses. Cause and fix: "Unreal freezes" below.
- dock_sitl8's own record was lost (its JSON path was held open by the output redirect), so its report ran on a
  record rebuilt from the runner's `[driver]` pass lines (same windows).

**The real ZR10 (theory, not measured).** At zoom 1 the real camera has the same K, so the same pixels per
square at the same geometry: about 30 m at 35 deg depression, set by about 11 px per square along the line of
sight. Range scales with the focal length (zoom 2 about 60 m), and with the sine of the depression along the
line of sight. The simulation is the best case. It has none of these:
- lens distortion: k1 -0.27, about 7 % at the corners; the board near the centre is barely affected;
- H.264/H.265 compression of the 1280 x 720 RTSP stream;
- exposure-time blur: v t f / R px, e.g. 24 m/s x 1 ms x 1407 px / 20 m = 1.7 px;
- rolling shutter, lens MTF, sun glare and auto exposure.

## Detector settings for range (2026-10-14, offline)

`detector.charuco_detection` (`CharucoDetectionSettings`, defaults below) sets how the ChArUco detector finds
markers; the live check, the dock-pass report and the static check use the same settings. Defaults:
`min_marker_perimeter_rate` 0.015 (OpenCV 0.03), `pixels_per_cell` 8 (OpenCV 4), `refine_markers` true
(OpenCV false); everything else stays OpenCV's. Unchanged: a detection needs 4 ChArUco corners and the profile's
`conf`; the report's pose needs 6 over 2 rows and 2 columns.

Sweep (scratch tooling, results in `C:\tune_hitl\det_sweep*`): 23 settings over 108 static and 159 in-view pass
frames (ZR10 and 90 deg runs), 1021 frames without the board (841 Unreal, 173 real camera frames from
`tools/cam/calibration` incl. the ZR10, checkerboards and faces, 7 `.sim` images), each raw, blurred (sigma 1 px +
noise) and blurred then H.264 4 Mbit/s, H.265 4 Mbit/s and H.264 2 Mbit/s CBR (the ZR10's stream bitrate is not
measured; frames encoded shuffled, so each gets about one frame's share of the rate: pessimistic). Plus 150
offline renders of the board (supersampled pinhole, 35 deg depression, yaw 0/30/90 deg) at zoom 1 and 2, raw,
soft (sigma 0.6 px), and soft + H.264 4/8 Mbit/s or H.265 4 Mbit/s. The renders reproduce the Unreal limits
(default 30 m, 35 m gap, 40 m with the smaller floor, nothing at 45-50 m).

The pass frames are the ones the live check saved: frames OpenCV's defaults detected, plus every 150th frame.
They are biased toward frames the old settings found, so they compare the settings but do not give a pass's
detection rate or range.

| Frames (pose rule: 6 corners over 2 rows and columns) | OpenCV default | navpy settings |
|---|---|---|
| Unreal static, raw: farthest 9/9 range | 30 m | 40 m (35 m: 0, render aliasing) |
| Unreal passes in view, detected: raw / blur / H.264 4M / H.265 4M / H.264 2M | 99 / 26 / 7 / 6 / 0 | 110 / 50 / 24 / 26 / 10 |
| Unreal static 9/9 range, H.264 4M | 12 m | 18 m |
| Render zoom 1, farthest: raw / soft / soft+H.265 4M / soft+H.264 8M | 30 / 25 / 22.5 / 25 m | 40 / 37.5 / 30 / 30 m |
| Render zoom 2, farthest: raw / soft / soft+H.265 4M / soft+H.264 8M | 55 / 52.5 / 47.5 / 47.5 m | 82.5 / 77.5 / 62.5 / 62.5 m |
| Boards in frames without one (5105 frames incl. variants) | 0 | 0 |
| Corners more than half a square off the truth | 0 | 0 (max 0.24 square) |
| Single markers in frames without a board (all ids outside the board's 0-16, no corner) | 9 | 10 |
| Position error p90 vs truth, Unreal passes | 0.14-0.22 m | 0.15-0.18 m |

- Rejected: `minMarkers` 1 (more corners, position error p90 up to 1.2 m on compressed frames), contour corner
  refinement (OpenCV 5.0 throws `_interpolate2Dline` on some frames), subpixel marker refinement (no range,
  error 0.32 m), other threshold windows and `errorCorrectionRate` 0.3 (no change). No pose-plausibility gate:
  no wrong board was found to tune one against.
- Cost: 12 ms per 1280 x 720 frame single-threaded on the laptop, 9 ms with OpenCV's default.
- Real camera: compression and blur cut the range far more than the settings add (H.264 4 Mbit/s: 12-18 m at
  zoom 1 in Unreal frames). Range scales with zoom: zoom 2 doubles it in the renders. Suggestion, not measured
  in flight: zoom 2 for the approach phase; its view of about 25 x 15 deg halves the time the board is in the picture.


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
  entry only). A bench screen with the board idle did not show the difference; the board runs did. Unreal
  ran BelowNormal in cam3, cam4 and the dock runs; it now runs at normal priority ("Unreal freezes"). The
  bridge's HIGH class is above both, but the gate has not been re-measured on the board with Unreal at
  normal priority.
- The capture runs on every other game tick (game about 37 fps, capture gate 1/30 s), hence about 19 fps.

## Unreal freezes (2026-10-14)

Unreal froze for 22-55 s, a few times per run, on board runs and on replays of a link.csv's poses into it
(no flight controller, no bridge). Measured with Unreal Insights (`-trace=default,task,loadtime,file
-statnamedevents`) on a replay of dock3's poses with the launch above at BelowNormal (as it was then) and
`-ResX=800 -ResY=450` (a window that fits the remote desktop; the native capture stays 1280 x 720):

- A freeze is one game frame (33 s and 35 s in that trace). The game thread sits in TG_PrePhysics,
  `ProcessUntilTasksComplete`, running queued game-thread tasks: 1675 vehicle moves in the 33 s, the 50 Hz
  pose rate. The render thread, the RHI thread and the GPU wait for the game thread.
- Each received pose was its own game-thread task (`AsyncTask` per UDP packet), and each move updates the
  vehicle's 440 attached mesh components (navigation and physics). A move costs about 5 ms; when the game
  thread gets less CPU than a move every 20 ms needs, the queue grows faster than it drains and the frame
  does not end until the load goes away and the backlog is gone.
- What took the CPU in the board runs was not recorded. Any process at normal priority that keeps all cores
  busy leaves Unreal at BelowNormal almost no CPU. Reproduced with 20 busy-loop processes for 40 s during a
  replay: one 55 s freeze; without load, two full replays had no gap over 1 s.

Fix: the vehicle receiver (`NavPyVehicleStateReceiver`, aas-sim-unreal) queues the parsed poses on its socket
thread and its tick moves the vehicle once, to the newest; the camera ticks after its receiver, so a frame
shows the pose its stamp names (exact pose mode unchanged). Unreal runs at normal priority. Frame gaps of
Unreal's capture times, full dock3 pose replays (12 min, same window) with three 40 s all-core load bursts
(about 16 fps):

| Run | max gap | gaps > 1 s | p99 / p99.9 |
|---|---|---|---|
| old plugin, BelowNormal | 54.6 s | 1 | 77 / 126 ms |
| old plugin, normal | 0.14 s | 0 | 77 / 98 ms |
| new plugin, BelowNormal | 6.7 s | 36 (during the bursts) | 76 / 5128 ms |
| new plugin, normal (2 runs) | 0.19 / 0.20 s | 0 / 0 | 87 / 143, 111 / 153 ms |

With the new plugin alone (BelowNormal) gaps of up to 6.7 s remained, most of them during the bursts
(7 came before the first burst, from a load not recorded); at normal priority no gap exceeded 0.2 s. Replay numbers only, not a board run.

## Known limits

- The native camera runs at about 17-25 fps on this laptop, not the configured 30 (about 19 fps beside
  the board link, above).
- Exposure is fixed, never auto: `NativeCameraExposureBias=-13.0` in the project's `DefaultGame.ini`
  (one stop per step, `-NavPyNativeCameraExposureBias=<bias>`); the level's cameras keep no own value.
- The board's detection range is set by its pixel size ("Camera model and range" below).
- The current fork SITL puts the HIL link on SERIAL8 (base port + 8) on a board. The dock_sitl1-7 runs ran
  SITL directly with `--serial8=tcp:8168` and serial0 held inside WSL; `sitl-backing --model hil` works too
  (HIL on SERIAL5, TCP 8168), with `SIM_RC_FAIL 1` in a `--defaults` file.
