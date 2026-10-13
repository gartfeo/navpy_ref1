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

1. Unreal (game, UAV1 camera on 18110):
   `& 'C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor.exe' 'C:\repos\unreal-projects\AasSimProject\AasSimProject.uproject' /Game/AAS/Levels/Yeghvard_Default -game -windowed -ResX=1280 -ResY=720 -NavPyEnableNativeCameraStream=1 -NavPyExactVehiclePose=1 -NavPyNativeCameraHeaderVersion=2 -NavPyFixedBodyCamera=1 -NoRayTracing -NoVSync '-ExecCmds="t.IdleWhenNotForeground 0"'`
2. The bridge as usual plus `--unreal-pose 127.0.0.1:17110 23.5`, with the Yeghvard origin
   (`--origin-lat 40.3117412 --origin-lon 44.4552112 --origin-alt 1292.8`) on both the bridge and the board
   (runner `--origin 40.3117412,44.4552112,1292.8,0`; a board's `SIM_OPOS_ALT` 1292.8).
3. navpy with frame source `npyc://127.0.0.1:18110`, or the check tool:
   `Set-Location C:\Users\GARTF\Downloads\navpy-temp2; $env:PYTHONPATH = "$PWD\src;$PWD"; .venv\Scripts\python.exe -m tools.unreal.npyc_live_check --mavlink udpin:0.0.0.0:9640 --seconds 60 --out C:\tune_hitl\cam_check`
   (`--mavlink`: any pymavlink link carrying the board's ATTITUDE, e.g. the runner's `--mp-mirror` port
   9640; frames.csv, summary.json and sample PNGs land in `--out`).

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

## Known limits

- The native camera runs at about 17-25 fps on this laptop, not the configured 30.
- Exposure is fixed, never auto: `NativeCameraExposureBias=-13.0` in the project's `DefaultGame.ini`
  (one stop per step, `-NavPyNativeCameraExposureBias=<bias>`); the level's cameras keep no own value.
- With the fixed body camera at -35 deg the board's ChArUco is detected up to about 15 m above it and
  not from 16.6 m; that limit is the board's pixel size, not the exposure.
- The current fork SITL puts the HIL link on SERIAL8 (base port + 8), which `sitl-backing --model hil`
  also gives SERIAL5; the live check ran SITL directly with `--serial8=tcp:8168` and serial0 held inside WSL.
