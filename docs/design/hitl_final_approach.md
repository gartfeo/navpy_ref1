# HITL final approach to the Unreal dock (design v6, 2026-10-15)

**Status.** v6 applies the owner's answers of 2026-10-15 (below) to the Codex-approved v5.6; building (the router of section 2 has an uncommitted first cut in the
worktree, not reviewed). Owner's "What next?" item 3 ("Full final approach in HITL"). Builds on `hitl_unreal_camera.md`
(camera stream, dock passes, detector range) and `flight_tuning.md` (HITL runner, bridge, gate). Line numbers are on
`feat/hitl-flight-tuning` 7d36929; Unreal lines on `aas-sim-unreal` 48ad8dd (`C:\repos\unreal-projects\AasSimProject`);
aas-hitl lines on its checkout `C:\repos\aas_repos\hitl_repos\aas-hitl` (read only). v5 applies the owner's camera
decision of 2026-10-15: **the approach uses the simulated SIYI ZR10 gimbal camera, as on the aircraft**, not a fixed
camera. Kept: board 2.4 x 3.2 m, a PC router shares the board's USB link, pass mark 10 of 10. Evidence level of every
result: simulation (SITL-HIL or board-HIL). It never proves physical docking.

**Goal.** One UAV flies navpy's vision final approach (parking navigation) to the Unreal dock with navpy running as on
the aircraft: real detector on camera pixels, navpy's SIYI client talking SIYI protocol to a ZR10 (here an emulator),
gimbal search, lock and track after detection, the `vision-nav-pn` law, `SET_ATTITUDE_TARGET` in GUIDED over MAVLink.

```
board (Pixhawk 6X, HIL fw) <-COM11-> bridge (JSBSim Peron15_hitl4, HIGH) --pose UDP 17110--> Unreal (normal)
   | COM15 (USB)                                                              ^ gimbal_state UDP 17212 | NPYC v2
 router (HIGH) --UDP--> runner (params, takeoff, search mission, floor, RTL)   |                        | TCP 18110
               --UDP--> navpy --SIYI UDP 127.0.0.1:37260--> ZR10 emulator ----+                        |
                          ^------------------------------------------------- npyc://127.0.0.1:18110 --+
```

## 1. What runs
- **navpy:** `python -m navpy.main` (`src/navpy/main.py:7`), headless, single UAV:
  `--detector-type real --detector-backend charuco --vision-profile siyi_zr10_hitl --pitch-controller vision-nav-pn`,
  `-c udpin:127.0.0.1:<navpy's client port>` (the router sends to each client's listening port and accepts replies only
  from it, `mav_router.py:60, 160, 202`), `-ss` the board's system id (navpy is component 191, `conn_args.py:7-11`). The GCS
  launch hardcodes `--detector-type sim` (`src/gcs/backend/navpy_process_launch.py:125`); GCS and operator approval come
  later, so the first runs auto-confirm (`-tac`, default true, `nav_args.py:12`). No GCS stack, no `swarm_run.py --eval`.
- **ZR10 emulator (new, navpy `tools/hitl/zr10_emulator.py`, section 4):** SIYI protocol on UDP 127.0.0.1:37260, gimbal
  and zoom plant, publishes the true gimbal state to Unreal (17212).
- **Runner:** a new `sitl_driver approach` subcommand: board setup as `dock-pass` (params, erase, bridge,
  `--unreal-pose`, Yeghvard origin), then starts the emulator and navpy, takeoff, search mission, AUTO, waits for navpy's
  terminal record or a timeout, stops navpy, RTL, gate, logs. It keeps draining MAVLink the whole time, so `HeightFloor`
  (a hook on received GLOBAL_POSITION_INT, `flight_steps.py:137`) always fires.

## 2. MAVLink link: one USB port, a router on the PC (owner)
Unchanged from v4. The board's USB (COM15, SERIAL0) is the only link; no adapter. `tools/tuning/mav_router.py` owns
COM15 in its own process and serves each client on its own loopback UDP port (bands as `mp_mirror.RESERVED_PORT_BANDS`).
Not MAVProxy (own streams and heartbeats) and not `mp_mirror.py` (one client, `:118`, forwards only as the runner reads).
- **Threads, no starvation:** one reader thread reads COM15 and only parses and sends (UDP `sendto` on loopback); one
  writer thread drains a single FIFO that each client's receiver thread fills. HIGH priority beside the bridge. Counters
  per endpoint, the board's MAVLink sequence gaps (USB loss) and each client's `mav_loss`, reported per run.
- **Routing:** targeted board packets go only to the client that sent from that system (and component when both are
  non-zero): ArduPlane addresses COMMAND_ACK to the sender (`GCS_Common.cpp:5188-5191`, `:5655-5658`). Untargeted
  packets go to every client. Client packets go only to the board.
- **Stream rates are per channel, and both clients share channel 0.** `REQUEST_DATA_STREAM ALL` re-sets every message
  (`GCS_Param.cpp:148-161`, `GCS_Common.cpp:6500-6514`), so the runner's 10 Hz request (`vehicle.py:65`, `board.py:203`)
  would undo navpy's intervals (`pose_streams.py:116`), and ArduPlane saves it (`persist_streamrates`,
  `ArduPlane/GCS_Mavlink.h:43`). The router keeps each client's `SET_MESSAGE_INTERVAL` claims; the board gets the fastest
  positive claim per message, never slower than the fastest `REQUEST_DATA_STREAM ALL` rate asked; 0/-1 withdraws that
  client's claim only. After every forwarded `REQUEST_DATA_STREAM` and every new boot the router re-sends the effective
  intervals in its own name (system 250, own component) and drops the acks addressed to it.
- **Reboots:** COM15 vanishes on reboot (`board.py:144`); the router reopens it with `OPEN_RETRY_S` (`board.py:41`) and
  drops client packets while closed, counted. The first cut has a write/close race (the writer checks `_link_up`,
  `mav_router.py:181`, while the reader closes the transport, `:133`, and `transport.py:78` asserts an open handle,
  outside the writer's `OSError` handler): H0 gives the serial handle one owner (writes and reopen under one lock) and
  tests a reboot under write load.
- **Quiescence at the board boundary:** the router queues raw bytes apart from navpy's lifecycle (`mav_router.py:86,
  176, 227`), so a command fenced in navpy can still reach the board after its terminal record. A router barrier on a
  loopback control port (HITL only; navpy gets its address as a run option): after its local fence and **before** it
  publishes the terminal record, navpy asks the router to close its endpoint; the router stops accepting navpy's
  packets, discards those still queued, waits for any write in progress to return, and acks with the discarded count
  and the time of its last navpy write. Only then does navpy write the record. No ack within a bound: navpy writes a
  barrier-failed terminal record and the run counts as failed. The router logs the write time and source of every
  packet sent to COM15, so "no navpy packet written after the terminal record" is checked at the board boundary. Tests:
  delayed queue, write in progress at the barrier, barrier timeout, ordering of ack and record.
- **Authority:** navpy flies GUIDED + `SET_ATTITUDE_TARGET` (`attitude_command.py:94`); the runner keeps its RC overrides
  (SYSID_MYGCS 250) at centred sticks, throttle low. H1 checks the GUIDED attitude response under them first.
- **Handoff at the end:** RESET→DETECT restarts AUTO (`navigation_transition_handler.py:61`, `recovery.py:89`), which can
  race the runner's RTL. navpy holds after its terminal record (7); the runner stops it (`NAVPY_STOP_V1` on stdin,
  bounded wait; no ack on worker error, `runtime_resources.py:239`, so on timeout it terminates its own recorded navpy
  pid). Before any RTL the runner itself closes navpy's router endpoint (the same barrier, on the runner's control
  connection; idempotent if navpy already closed it) and waits for its ack, so after a crash, a forced stop or a
  failed navpy barrier no buffered navpy packet can follow the RTL; no ack means the router process is terminated
  before RTL is sent with the runner opening COM15 itself, and the run is failed. Then RTL. Regression: buffered UDP from navpy across a
  forced stop and the RTL. A floor breach or timeout takes the same path (today `HeightFloor` recovers directly,
  `flight_steps.py:162`); the stop time counts in the floor clearance.
- **One-shot stays false, checked:** one-shot force-disarms a simulated autopilot (`nav_oneshot_completion.py:36-75`)
  and the HIL firmware has SIM_* parameters (ArduPilot `libraries/SITL/SITL.cpp:231`). The runner sets a board
  `AAS_NAV_ONESHOT` to 0 if present; H0 adds the effective one-shot and confirmation settings to navpy's start record
  (not logged today, `runtime_composition.py:87`) and the runner asserts them before takeoff.
- **SITL stand-in:** the router on the SITL instance's serial TCP port instead of COM15.

## 3. Frames and clocks
- Profile `camera_index: "npyc://127.0.0.1:18110"` (`frame_capture_npyc.py`). EXPOSURE stamps from the v2 header's board
  time via `AttitudeBoardClock` (`real_detector_runtime_composition.py:121`, `board_clock_map.py`).
- **B1, no calibration:** `RealDetectorFactory` sets no `capture_calibration` (`real_detector_factory.py:146-158`), so
  every frame is `real_frame_capture_uncalibrated` (`real_frame_association.py:144-150`) and `frame_projection.py:45`
  drops it. Fix: a certified `CaptureLookupCalibration` (`capture_lookup.py:45-62`) for the npyc source on navpy's link
  (`:103-106`); residual within 8 ms (`:30`). H0 adds the tool; H1 certifies it under full load.
- **NPYC is EXPOSURE-only:** a calibration also admits READ frames (`capture_lookup.py:149`) and NPYC falls back to READ
  when mapping fails (`frame_capture_npyc.py:170`). Rule: an npyc source admits only EXPOSURE frames; tested with
  invalid pose flags, mapping loss, board reboot.
- **Gimbal state with frames (new in v5).** Two halves, kept apart (Pure Vision: gimbal readback is camera state only):
  - *What Unreal renders.* Today an external gimbal_state is applied on arrival through an exponential smoother
    (`FInterpTo`, speed 24, `NavPyGimbalCameraActor.cpp:558-598`; first state snaps, `:525`), no timestamp is read
    (`:466-556`), while the vehicle is drawn at an interpolated board time 40 ms behind (`hitl_unreal_camera.md`). So a
    frame's gimbal angle and its vehicle pose belong to different instants during a slew. Fix (Unreal, U2): gimbal_state
    gains an optional sender time on the PC's monotonic clock; the actor keeps a short history and renders the gimbal at
    the frame's pose instant (board time mapped to that clock by a lower envelope over pose receipts),
    interpolated, no smoothing (the emulator owns the dynamics).
  - *One time axis.* Every process on the PC stamps with one monotonic clock (QPC: Unreal `FPlatformTime::Seconds`,
    Python `time.perf_counter`; UE 5.7 adds 16777216 s to QPC seconds, `WindowsPlatformTime.h:29`, so H0 removes that
    epoch and then tests that they agree, and N3 moves the SIYI attitude stamp off wall `time()`). A
    frame has one board time b. Unreal maps it as b + d_pose (pose-receipt envelope), navpy as b + d_att
    (ATTITUDE-receipt envelope, which keeps the minimum ATTITUDE link delay, `board_clock_map.py:9-16`); they differ by
    the two transports' offsets. Unreal logs b and its mapped instant per frame; H0's calibration tool (with B1) measures
    navpy's offset to it and navpy applies it as a certified `gimbal_frame_offset_s`. Budget: 0.3 deg at the 90 deg/s
    slew limit is 3 ms; residual and drift checked in H1; an asymmetric-delay regression test covers it. Each frame's rendered gimbal angles and zoom go to an
    Unreal-side log for scoring only; navpy never reads them (guard test as `test_unreal_pose_isolation.py`).
  - *What navpy believes.* The SIYI client polls 0x0D every 20 ms (`siyi_sdk.py:68-69, 252-266`) and stamps attitude
    with wall `time()` on receipt (`siyi_sdk.py:561`; zoom uses `monotonic()`, `:690`). Association takes the latest
    gimbal/zoom sample when the frame is associated, no interpolation (`camera_mount_frame_state.py:116-135`,
    `real_frame_association.py:207`; `real_detected_object_builder.py:155-159`). On the real ZR10 the report also lags and
    the yaw report is 0.92-0.94 x true (4). Fix (navpy, N3): gimbal and zoom samples on the frame clock, each re-stamped
    at receipt minus its own report lag (`gimbal_attitude_report_lag_s` for 0x0D, `zoom_report_lag_s` for 0x18,
    measured separately; aas-hitl lags attitude only, `dynamics.py:19`, `device.py:318`; HITL = the emulator's
    configured lags; real = to be measured), then interpolated at the frame's exposure time; **lens-travel rule (the
    only one):** a frame in lens travel is used only when the two zoom readings bracketing its exposure are equal; then
    K is that reading's K. The wire carries 0.1 x steps (`siyi_sdk.py:687`), so the HITL emulator moves the rendered
    magnification in exact 0.1 x steps, rendering the value it reports; equal brackets then give the rendered K
    exactly. Unequal brackets: the frame is refused. On the real ZR10 the reading is a rounded label, so its travel
    frames stay unused until a calibrated reading model exists; a frame whose
    bracket has no later sample yet waits for it up to a bound, else it is refused (as ATTITUDE,
    `real_frame_association.py:133-174`). **Report scale:** navpy uses the reports directly as camera orientation
    (`hardware/state.py:84-93`, `visual_ray_projection.py:20`), so a yaw report of 0.94 x physical is 5-7 deg off at
    90 deg yaw. The profile gains a report→physical calibration (scale and offset per axis) applied before association;
    the emulator runs scale 1 first, and a scaled emulator runs only with the matching calibration (an uncompensated
    run is a labelled stress case). **Signs:** SIYI yaw is left-positive in 0x0E commands and 0x0D reports, while a
    positive 0x07 rate lowers the report and turns the camera right; Unreal yaw is right-positive (aas-hitl
    `zr10.example.toml:75`, `tests/zr10/test_gimbal_state_publish.py:104`, `tests/zr10/test_commands.py:124`; rig step 0).
    navpy passes absolute yaw and reads reports unchanged (`hardware/commands.py:30`, `hardware/state.py:84`). The
    emulator is wire-faithful to the real unit; the SIYI plugin owns one tested conversion each way (absolute command,
    report, rate sign), tested end to end: wire command → emulator → Unreal camera motion → calibrated frame-local ray.
    Score per
    frame: |navpy's associated gimbal angle - Unreal's rendered angle|; H1 sets the budget from the 0.3 deg tracker
    settle tolerance (`gimbal_rate_types.py:34`).

## 4. Camera: simulated ZR10 gimbal (owner, 2026-10-15)
- **Smallest path: a SIYI emulator in navpy tools, Unreal renders its state.** navpy's real client (`GimbalSiyi`,
  `plugins/gimbal/siyi/gimbal_siyi.py:23`; `siyi_sdk.py:24` takes ip and port) talks to `127.0.0.1:37260`, so the
  protocol path is the aircraft's. The emulator reuses navpy's SIYI codec (`siyi_message.py`) and navpy's deterministic
  gimbal plant (`sim/gimbal/siyi/gimbal_physics.py`, angle and zoom plants), stepped at 20 ms, and adds what the plant
  lacks: command dead time, report lag, report scale, the command→reading zoom table. **Aircraft attitude in:** the
  plant needs the aircraft attitude for LOCK stabilisation and the mechanical limits (`gimbal_angles.py:55, 174`; the
  sim steps it first, `gimbal_siyi_step.py:60`). The emulator is simulator side and gets the plant's attitude from the
  bridge (a second copy of the board-time-stamped `vehicle_state.v1` datagram, today sent to Unreal only), never via
  navpy. Its output angles are in the frame Unreal composes with the vehicle attitude
  (`NavPyGimbalCameraActor.cpp:624-645`); conventions are fixed by tests against navpy's sim plant with level, banked
  and turning cases. It sends gimbal_state.v1 (schema
  `NavPyGimbalCameraActor.cpp:43-45`; required `sys_id`, `device_id`, `session_id`, `sequence`, `attitude_deg`,
  `:466-556`) to 17212 every step. Commands: 0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x0A, 0x0C, 0x0D (reply, and the 0x25
  stream), 0x0E, 0x0F, 0x18, the set navpy sends (`hardware/commands.py:30-75`, `sdk_owner.py:117-121`) and the
  aas-hitl emulator answers (`aas_hitl/zr10/device.py:81-94`).
  - *Not chosen:* running aas-hitl's emulator (`aas_hitl/zr10`, CLI `py -3.11 -m aas_hitl.zr10 --config`) as is: it
    already speaks SIYI and drives 17212, but it is another repo we may not change, its 0x0F is ignored by default
    (`device.py:308-316`), its optics (`hfov` 45.96 deg) and dynamics (145 deg/s, 0.6 s 0x07 dead time, 0.145 s report
    lag, all TBC, `zr10.example.toml`) are not navpy's, and navpy tests cannot cover it. H1 runs it once against navpy,
    read only, as an independent protocol cross-check. *Not chosen:* SIYI in the Unreal plugin (C++, more code, the
    plant would exist twice).
- **Unreal changes (aas-sim-unreal, U1-U4):** U1 the native intrinsics camera takes zoom from gimbal_state (new optional
  `zoom` = magnification, K x zoom about the same principal point, as `-NavPyNativeCameraZoom`,
  `NavPyGimbalCameraActor.cpp:1266-1280`); today external FOV does not change it (`.h:171-175`). U2 the stamped history
  of 3. U3 the per-frame rendered-state log. U4 gimbal mount: `-NavPyFixedBodyCamera=0`, the actor at the vehicle
  reference (today spawned at (0,0,-80) cm, `Scripts/setup_yeghvard_scene.py:146-158`), pitch limits -90..+25 as the ZR10
  (today -90..30, `.h:82-85`).
- **B3, real gimbal path:** `vision_profile_composition.py:48` hardcodes `use_sim_gimbal=True`, so a `siyi` profile
  builds `GimbalSiyiSim` (`vision_camera_factory.py:128-137`; todo 18). Fix: the real path builds `GimbalSiyi` from the
  profile's `siyi_ip`/`siyi_port`; tested so a real run never gets the sim gimbal.
- **Dynamics, from what is measured (stated per value).** navpy's measurement folders hold video timing only
  (`docs/measurements/zr10-latency-20260830/REPORT.md`: LED to published frame median 99.8 ms, p99 121.5 ms;
  `zr10-strobe-phase/REPORT.md`: yaw readback drift about 2 deg per 90 s). Gimbal numbers come from the gimbal-controller
  rig work (owner's notes, 2026-10-08..14) and are copied into the emulator config with their source:

  | Parameter | Value | Source |
  |---|---|---|
  | Limits | yaw -135.0..+132.8, pitch -88.9..+23.9 deg | rig limit sweep 2026-10-14 |
  | Slew rate, 0x0E and full 0x07 | 90 deg/s | unmeasured: navpy sim plant (`gimbal_angles.py:11`); aas-hitl uses 145 TBC |
  | 0x07 dead time | 0 first, then 0.6 s | unmeasured: aas-hitl TBC; rig stop latency 8.2 ms p99.9 is the send path only |
  | Report lag (0x0D) | 0 first, then measured | unmeasured: aas-hitl's 0.145 s is command-to-reported-motion onset, not telemetry lag (`zr10.example.toml:29`) |
  | Report scale | pitch 0.995, yaw 0.92-0.94 x true | rig step test 2026-10-08; aas-hitl: report 0.995 x command - 1.00 deg, true 1.056 x command - 0.81 deg (`zr10.example.toml:41-43`) |
  | 0x07 yaw sign | yaw+ turns the yaw report down | rig step 0 2026-10-14 |
  | 0x07 deadband, low-speed response | speeds up to 4 units give no motion | aas-hitl bench observation (`zr10.example.toml:26`, `dynamics.py:53`); navpy's plant moves for any nonzero speed (`gimbal_angles.py:138`) and the tracker emits 1-unit corrections (`gimbal_rate_command.py:44`); the emulator models it; measured for the final series |
  | Absolute zoom (0x0F) | non-monotonic command→reading table, 2-6 s lens travel | profile `zoom_calibration`; rig sweep 2026-10-08 (91 commands). Conflict: aas-hitl records 0x0F as acknowledged and ignored (`zr10.example.toml:35`, older gimbal-controller text); the sweep is newer; reconciled on the rig before the final series |
  | Manual zoom (0x05) | 0.5 x/s near 1x; stop settles < 0.3 s | aas-hitl `zr10.example.toml:34` (measured near 1x only); rig sweep. navpy's plant value 5 levels/s (`gimbal_zoom_plant.py:11`) is not used |
  | Not rendered | autofocus hunting after jumps (17 % blurred), creep, 1 of 40 moves 10.6 s late | rig |

  Runs start ideal (no dead time, no lag, scale 1) and add the effects one at a time (H2), so a failure is attributed.
  Runs with unmeasured rows are exploratory and labelled so. **The final ten run under the measured configuration:**
  slew rate, 0x07 dead time, deadband and low-speed response, both report lags, zoom speed over the range and 0x0F behaviour measured on the rig (its
  step-0 and limit data pulled in first). If a row cannot be measured, the series is reported as not done, not passed
  with a placeholder; only the owner can change that scope.
- **Zoom 1-10x and K per zoom.** Unreal renders K = magnification x the ZR10 zoom-1 K (fy 1401.9 at 1280 x 720,
  48.9 x 28.8 deg) and the emulator's 0x18 reading is that magnification (sim: the reading is faithful, as measured:
  same reading, same scale within 0.45 %). The emulator maps a 0x0F command to a reading through the profile's
  `zoom_calibration` table, so the real non-monotonic command space is reproduced. navpy today maps the reading back to a
  command and interpolates K in command space (`camera_mount_zoom_readback.py:56`, `zoom_calibration.py:48-80`), which on
  the real ZR10 gives -7..+11 % fx error even with exact rows. A profile change alone cannot fix this: the runtime
  converts every reading to a command key before it freezes K (`camera_mount_zoom_readback.py:52`,
  `camera_mount_frame_state.py:75`); command 2.0 reads 1.8 (`vision_profiles.json:531`), an 11 % error on Unreal's
  image. N4: a separate optical lookup keyed by the 0x18 reading (profile `intrinsics_key: "reading"`), the command table
  kept only for commanding. The HITL profile `siyi_zr10_hitl` (new, extends `siyi_zr10`) uses it, with rows exact for
  Unreal and **zero distortion at every zoom** (the base zoom-1 row has distortion, `vision_profiles.json:456`; Unreal
  renders none; navpy undistorts the navigation pixel, `real_track_observation.py:32`). The real ZR10 profile keeps its
  lookup until it has a per-reading calibration (follow-up, not in this plan). On the real ZR10 only zoom 1 is measured
  (24 views, 0.140 px); rows 2-10 are preserved historical fabricated values, not current zoom-1 multiples (fx(2) 2066.5
  vs fx(1) 1407.4, `test_zr10_profile_intrinsics.py:1-31, 53-61`); separately, the rig's high-zoom fits are weak (a
  zoom-10 fit stuck in a local minimum, -4.7 % fx; owner's rig notes 2026-10-08). A real run above 1x needs its own
  calibration first.
- **Search, lock, track (existing navpy behaviour, no new laws).** Search with the gimbal: navpy has no gimbal scan
  pattern today (none in src). N5 adds a small one, active only while nothing is detected: a profile-defined yaw sweep
  at a fixed search pitch and the search zoom step (pattern, rate and limits from the profile; never from the mission or
  a dock position), stopped on the first detection; after a loss the existing recovery runs. H1 runs a fixed
  forward-down pose first, then the sweep. After detection: `GimbalVisualTracking`
  (`command/gimbal/gimbal_visual_tracking.py:39`) with `GimbalRateTracker` (bw 2.5, max rate 100, slew 90 deg/s, lead
  0.15 s, `gimbal_rate_types.py:30-39`) locks the board; on loss a zero-rate hold, then recentre
  (`gimbal_loss_recovery.py:104-154`). Tracking enabled selects approach kind ORBIT (`approach_strategy.py:34-47`): an
  orbit around the board's geolocation from navpy's own detection (allowed after detection; GPS allowed before NAV),
  confirm, then NAV. **That orbit is infeasible with this board and reader (Q2):** its radius is at least 500 m
  (`MIN_APPROACH_STANDOFF_M`, `peer_orbit_plan.py:59`, applied in `approach_planner.py:157`; tuned with the simulated
  detector, whose comment says 10x holds the POI at about 48 px at 500 m), while the ChArUco reader needs about 11.5 px
  per 40 cm square: horizontal range at most 48.8 z sin(d) cos(d) <= 24.4 z, i.e. 244 m at 10x. Auto-confirm also has
  no lock or orbit-established gate (`confirmation_action.py:49`, `confirmed_poi_release.py:34`); TRACKING is no such gate
  (returned on the first accepted sample, `gimbal_rate_tracker.py:114, 166`; the 0.3 deg tolerance quantises commands,
  `gimbal_rate_command.py:42`). H0 adds an explicit lock: the same POI, measured (not predicted) samples fresh within a
  named age, pointing error (board centre to principal point) under a named angle for a named settle time, at a settled
  zoom; and orbit established (on the circle within a named tolerance, bank steady; GPS allowed before NAV). Confirm
  only while both hold. Constants named in the profile and tested, including confirmation attempts during an
  acquisition slew and during orbit capture. The orbit centre is the camera ray on the ground (reference height 0,
  `self_detected_approach.py:14, 58`); a board 20 m up (Q1) puts it tens of metres beyond the board (N8 below
  replaces it with the board's own pose, never with dock truth). NAV entry stops geo pointing (`nav_entry.py:44-50`); visual rate tracking
  continues in NAV; the law gets the gimbal readback as camera state only (`real_detected_object_builder.py:118-143`).
- **Zoom steps, not auto-zoom, first.** The track + auto-zoom design (continuous 0x05, `r = k log(s_req/s_obs)`) was
  written for the gimbal-controller repo and awaits the owner (Q4). Until then zoom moves in fixed steps by measured
  size. navpy's `PoiZoomTracker` must exist (acquisition refuses without it, `gimbal_geo_acquisition.py:55`), but it
  picks continuous control first for SIYI (`siyi/hardware/facets.py:70`, `poi_zoom_update.py:154`), has no policy
  selector (`poi_zoom_types.py:35`), and its discrete fallback compares readings with command labels and sends a label
  as a command (`poi_zoom_setpoint.py:133`). N6, a fixed-step policy: `PoiZoomTrackerConfig.policy = "steps"`; steps
  are optical readings that the command table can reach exactly (1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 10.0: each the reading of a
  0x0F command in `zoom_calibration`); the command for a step comes from that table; a step is reached when the
  0x18 reading equals it within 0.05 on two consecutive polls; until then the zoom is in transition, and its frames follow N3's
  single lens-travel rule. Up or down on the measured square size (the smallest side of the detected board quad
  over its square count, so foreshortening counts; the geo selector's whole-object sizing,
  `gimbal_geo_zoom_selector.py:59`, is not used), with hysteresis. Zoom per range without a range: pure vision, no dock
  position. Search starts at the profile's search step. **Owner (2026-10-15): zoom out as it gets close.**
  NAV entry today freezes the zoom at the minimum (`nav_entry.py:47`, `gimbal_zoom_control.py:73-84`). N9: with the
  profile's `final_approach_zoom: "steps"`, NAV entry keeps the current step and the same step policy runs in NAV,
  stepping down only, so the whole board stays in view: step down when the detected board quad spans more than 60 %
  of the picture on either axis (each axis normalised by its own side) or any corner comes within 10 % of a picture
  edge (as `poi_zoom_geometry.py:54` checks all four edges), to the next step down, by 0x0F to that step's command (lens travel per N3). Profiles
  without the setting keep today's freeze at the minimum.
- **Range per zoom, flat board.** Detection needs about 11.5 px per 40 cm square along the line of sight (dock4,
  `hitl_unreal_camera.md`). A flat board seen at depression d from slant range R gives 1401.9 z x 0.4 sin(d) / R px,
  so R <= 48.8 z sin(d), and the height above the board h <= 48.8 z sin^2(d):

  | Zoom | R at d = 30 / 45 / 60 deg | h above board at 45 / 60 deg | Picture at 1280 x 720 |
  |---|---|---|---|
  | 1 | 24 / 35 / 42 m | 24 / 37 m | 48.9 x 28.8 deg |
  | 2 | 49 / 69 / 85 m | 49 / 73 m | 24.9 x 14.4 deg |
  | 4 | 98 / 138 / 169 m | 98 / 146 m | 12.6 x 7.2 deg |
  | 10 | 244 / 345 / 423 m | 244 / 366 m | 5.1 x 2.9 deg |

  So at zoom 1 in NAV the flat board is read only within 24-35 m on a 30-45 deg dive (1.0-1.5 s at 24 m/s); at 2x,
  49-69 m. A wide 1x search reads the board only from within 24 m above it at 45 deg. The board fills the picture
  height (3.2 m) from 6.2 z m; closer, ChArUco decodes a subset (`charuco_detector.py:117`), and the ZR10 profile's
  tracker (BoT-SORT, `vision_profiles.json:406`) coasts empty detections for only 0.30 s
  (`tracker_backends_boxmot.py:100`, `track_export_memory.py:18`).
- **Feasibility contract: search, circle, NAV entry and tail as one plan.** A run's plan (search height and pitch,
  circle radius r and height h above the board, zoom held from lock to contact, tracker settings) is accepted only if a
  pre-flight check (H0 tool, pure arithmetic on profile and plan; no dock position) passes every condition, else the run
  is refused:
  1. *Readable everywhere on the circle:* with sin(d) = h/R, the reader needs R <= sqrt(48.8 z h), for the largest
     horizontal distance to the board, r plus the worst orbit-centre error (below).
  2. *Flyable under the effective NAV limits:* r >= the planner's own minimum (`r_nav_min`, `orbit_geometry.py:67`,
     kappa 2 and eta 0.65, `orbit_geometry.py:34`) evaluated with the NAV bank cap of 35 deg (`command_anchor.py:78-79`)
     and the board's `PTCH_LIM_MIN_DEG`: turn term 2 v^2 / (g tan 35) = 167.7 m at 24 m/s; descent term
     h / tan(0.65 |min pitch|) (164 m at h 80 m and -40 deg). With 1, the horizontal reach is at most
     sqrt(48.8 z h - h^2) <= 24.4 z, so z >= 7 for any circle; only the 7.7, 8.8 and 10 steps can pass.
  3. *Trackable:* on the circle the bearing turns at v/r (8.1 deg/s at 170 m). Today's rate law commands bandwidth x
     image error (`gimbal_rate_command.py:27`); the tracker's sample holds only image errors and time
     (`gimbal_tracking_sample.py:56`) and it differentiates those (`gimbal_rate_tracker.py:99`), so at a constant lag the
     error rate is zero while the camera must keep turning: steady error (v/r)/bw = 3.2 deg at bw 2.5, outside the
     2.6 deg half-width of the 10x picture. N7: a camera-motion-compensated line-of-sight rate, the derivative of
     (gimbal readback angle + pixel angle) in the gimbal's stabilised frame (camera state and pixels only; if the yaw
     report is body-relative, body yaw rate from the gyro, ATTITUDE yawspeed, never heading), fed forward as the
     commanded rate, so steady error + frame-delay error stays under the lock tolerance, well inside the half field of
     view. Regression: pixels constant while the camera turns gives a feedforward equal to the camera rate; plus a
     simulated circle.
  4. *Zoom out in time (N9):* the trigger's height test, the board's 3.2 m side at most 60 % of the picture height
     (432 px; the 2.4 m side at 60 % of 1280 px is looser), needs
     z <= R / 10.4 (fy 1401.9 z); closing at 24 m/s this lowers the zoom at 2.3 x/s on average. The check uses the
     profile's lens speed for 0x0F moves and the zoom report lag, and requires the step-down to keep the board whole
     to the minimum step; the emulator runs ideal first, then its measured lens speed. The real ZR10's speed over the
     range is unmeasured (0x05 0.5 x/s near 1x; 0x0F jumps 2-6 s): a real-camera approach may fail this condition, and
     the check says so instead of flying.
  5. *Tail:* at the minimum step (reading 1.1) the board fills the picture from 6.9 m, the last 0.29 s; the offline
     check (planned approaches rendered through the real detector, BoT-SORT and the homography centre) must show
     measured centres at least every 0.3 s to contact, inside BoT-SORT's 0.30 s coast and the 1.0 s command freshness
     (`command_freshness.py:12`).
- **Orbit centre without the ground-plane error (N8).** The circle centre is the camera ray on the ground (reference
  height 0, `self_detected_approach.py:14, 58`); for a board 20 m up seen from 80 m above it at 170 m, that is about
  40 m beyond the board, which breaks condition 1 at every zoom. N8: centre on the board's own position from its ChArUco
  pose (own detection plus the aircraft's GPS, both allowed before NAV). The pose uses the rendered board's ChArUco
  geometry, bound explicitly in the HITL profile: 5 x 7 grid, 400 mm squares, 296 mm markers (`charuco_board_texture.py:50`),
  the panel margin kept apart; not `detector_class_dimensions` (3.5 x 2.5 m, `vision_profiles.json:5`) and not the
  `CharucoBoardSpec` defaults (50/37 mm, `charuco_board.py:28`, an eightfold scale error). This is a target-type
  property, not a dock location. A metric-pose consistency test checks the reprojection and the square size, with the ground ray as fallback that refuses the circle
  if condition 1 fails. The camera position must be taken at the frame's exposure time too: today the image uses the
  exposure stamp but the GPS camera location is read at association and stored unchanged
  (`real_frame_association.py:210`, `real_detected_object_builder.py:76`), about 2.8 m at 115 ms and 24 m/s; N8
  interpolates GLOBAL_POSITION_INT at exposure. H1 bounds the total world-position error of the centre (offline, against
  Unreal's board position), and condition 1 uses that bound.
- **Worked plan (owner-approved values):** circle 170 m, 80 m above the board on its 20 m mast, zoom 10 on the
  circle and at NAV entry, stepping down on the dive (N9).
  1: farthest slant with 8 m centre error 195 m against a limit of 198 m; 2: 170 >= 167.7 m and >= 164 m (descent,
  assuming -40 deg); 3: needs N7; 4: steps begin near 104 m and reach 1.1 near 11 m, about 3.9 s at 2.3 x/s average; 5: 6.9 m,
  0.29 s. Every condition is tight: 10x is the ZR10's optical maximum, so there is no
  zoom margin left. Search: the same height and step, gimbal at -25 deg (the circle's depression), across-track view
  about 17 m at 188 m slant, so the sweep (N5) carries the coverage. If no plan passes the check, H1 reports that as a
  finding with the numbers for the owner (the board size is fixed), not tuned around.
- **Navigation pixel = board centre:** ChArUco reports the centre of its clipped bbox (`charuco_detector.py:124-130`),
  which becomes the navigation pixel (`real_track_observation.py:32`). Carry the homography-projected board centre; bbox
  for display only; test partial views. Measured centres are marked apart from tracker predictions and carry their own
  measurement time (`real_detected_object_builder.py:189`).
- **Pure Vision:** no compass yaw, ground speed or altitude in the command path; gimbal roll-yaw-pitch readback is camera
  state only; Unreal's rendered gimbal state and the vehicle pose never reach navpy (guard tests).
- **Other cameras, for later (profiles only, no run in this plan).** A fixed camera is data: resolution, K per zoom (or
  nominal pixel pitch and focal range), mount pitch, maximum optical zoom, and a `calibration` label (`measured` or
  `nominal`); results from a nominal profile are labelled so. Two new profiles: `novoxy_sk_ahd2200` (owner's camera:
  1/3" IMX323, 1920 x 1080, 2.8 um, 5-50 mm: fx = f / pitch = 1786 px at 5 mm, about 56.5 x 33.6 deg; 17857 px at
  50 mm, about 6.2 x 3.5 deg; nominal, uncalibrated; AHD analog output through a capture device, so READ stamps only
  until a capture timing method exists) and `placeholder_18x` (ZR10 zoom-1 K, max 18x, labelled placeholder). Note:
  the existing profiles `novoxy10` (1920 x 1080, measured-looking zoom 1-3 rows, fx 1978.6 at zoom 1, i.e. 51.8 deg wide,
  11 % off the 5 mm nominal) and `novoxy18` (zoom 1-2) have no recorded source (initial commit 16ced48); whether
  `novoxy10` is this camera is unknown, so it is left unchanged. The fixed-camera path (v4's B2: a factory for one fixed K
  without a zoom command) is not needed for the gimbal runs and is deferred with these profiles.

## 5. How the approach starts
- **Dock location rule:** the search mission is fixed and never derived from the board position: a straight AUTO leg at
  a fixed height toward a fixed bearing. The board's along-lane distance, lateral offset and heading are drawn per run
  inside the lane's bounds and given only to Unreal and to scoring. `-mwp` gates detection to the lane.
- **Detection to NAV:** detection → lock (rate tracker) → ORBIT around navpy's own geolocation → confirm while locked →
  NAV requests GUIDED → the law flies; the gimbal keeps the board in the picture, so v4's collision-course margin
  problem of a fixed camera is gone. The circle, its height and the held zoom satisfy the visibility condition of 4;
  today's 500 m floor does not (Q2). The time on the law is set by NAV entry geometry and the zoom held in NAV (Q3); H1
  measures acquisition range and time in NAV.
- **Search height** is the harness's job (mission altitude, refused before flight); `-ma` does not gate it with the
  vision law selected (`recovery.py:49-52`).

## 6. Floor and safety
- In NAV navpy has no floor (pure vision). NAV ends on DOCKED or NAV_FAIL (`no_detection`, `missed_dock`,
  `source_command_stale` at 1.0 s, `navigation_rejected`; `final_approach_command_dispatch.py:89-122`).
- **The board must sit above the floor (Q1).** The Unreal board lies flat 4.07 m above the origin
  (`hitl_unreal_camera.md`); contact means the aircraft within 0.5 m of it, and the runner floor (`HeightFloor` → RTL)
  is 7 m above home (`dock_pass.py:55`), so no approach can reach contact today. Proposal: the same flat 2.4 x 3.2 m
  board on a sim-only mast, centre 20 m above the ground (Unreal only): contact 13 m above the floor, more than a
  20-30 deg dive's 4-9 m pull-out (pickup plan). H2 measures the height loss. The floor comes from the terrain and that
  loss, never from the dock position; a lane that would trip it is refused before flight.
- ArduPlane's own failsafes stay as they are (owner ruling 2026-10-12). Gimbal limits are the ZR10's own; the emulator
  clamps to them as the hardware does.

## 7. Success and metrics
- **B4, contact sensor in the real path.** The 0.5 m sensor (`dock_contact_sensor.py:38-40`) is wired only by the sim
  detector factory (`sim_detector_factory.py:58-76`); the real one returns None (`real_detector_factory.py:65-68`). Wire
  it in the real path for a simulated autopilot with a HITL dock world (the run's board centre; scoring only,
  `test_dock_contact_isolation.py` extended); GPS-based, as the owner ruled for sim.
- **Identity binding:** contact arming uses the POI's `obj_id` (`dock_contact_episode.py:52`, exact match
  `dock_contact_sensor.py:215`); real detections carry tracker ids (`real_detected_object_builder.py:50`). Bind outside
  the law: one HITL dock → the world's single dock. Tests: tracker ids, reacquisition, wrong dock, episode resets.
- **Terminal channel and fence:** DOCKED is a batched debug-stream row (`navigation_log_streams.py:77`), NAV_FAIL a
  general log line (`final_approach_command_dispatch.py:130`); a FAILED latch does not end command work
  (`episode_latch.py:34`). In HITL navpy fences attitude commands, mission/mode writes and gimbal commands, holds, passes
  the router barrier (2), then writes one flushed terminal record (outcome, task, episode, aircraft time) the runner reads; tested together.
- **Truth cross-check, offline:** closest approach of the bridge's 250 Hz `link.csv` path to the board centre; DOCKED
  counts only with a matching DOCK_ARMED.
- **Pass mark (owner): 10 of 10.** Ten consecutive approaches in one series, each a simulated contact (DOCKED, truth
  distance within 0.5 m), zero commands after the terminal record, clean stop and RTL. Every run kept
  (`C:\tune_hitl\<series>\<n>`: runner record, router counters, emulator log, Unreal rendered-state log, navpy logs,
  frame log, `link.csv`, DataFlash); a miss is reported with its cause and fails the series. Runs refused before takeoff
  are reported apart.
- **Per approach:** acquisition range and zoom, lock time, time in NAV, frames used (EXPOSURE share, drops), gimbal
  association error (3), command rate and age, command-to-`ATT.DesRoll/DesPitch` delay, miss distance (sensor, truth),
  outcome. **Per run:** link gate, timing certification, router counters, frame latency, Unreal gaps, emulator step
  jitter, navpy CPU and reader lag.

## 8. Determinism and load
- Physics is deterministic in board time; nothing else is (Unreal real time, frame latency median about 115 ms; navpy
  and the emulator on the PC clock). Results are statistics over N runs; each failure gets a stated cause.
- CPU: bridge and router HIGH; Unreal, emulator and navpy normal (OpenCV threads capped). The stale gate and the timing
  certification must hold with everything running.
- `main` is merged (1bad278).

## 9. Steps
| Step | What | Done when |
|---|---|---|
| H0 | Q1-Q4; router (2); B3 real SIYI path; ZR10 emulator with aircraft attitude from the bridge, the dynamics table and zoom table (4); gimbal/zoom readback lag-corrected, scale-calibrated and interpolated at exposure (N3); optical K by reading (N4); gimbal search sweep (N5); fixed-step zoom policy (N6); SIYI sign contract; one PC clock and the certified gimbal frame offset; confirm only while locked and on the circle; plan feasibility check (4); zoom steps in NAV (N9); tracker rate feedforward (N7); orbit centre from the board pose (N8); offline partial-view tail check with BoT-SORT; router single-owner serial handle and the barrier before the terminal record; HITL profile `siyi_zr10_hitl` (zero distortion); B1 calibration tool and EXPOSURE-only rule; board-centre navigation pixel; B4 contact sensor + identity binding; terminal fence (incl. gimbal commands), hold and record; observation-mode inhibit; effective settings in the start record; `sitl_driver approach`; report tool; Unreal U1-U4 and the board mast (Q1); profiles `novoxy_sk_ahd2200`, `placeholder_18x` | Unit + guard tests (Pure Vision, contact isolation, no dock data or rendered state in navpy, EXPOSURE-only, real path never sim gimbal, emulator protocol against navpy's client and against recorded aas-hitl replies, router routing and stream claims, terminal handoff interleavings), full suite |
| H1 | SITL stand-in + router + bridge + Unreal + emulator + navpy, observation only: an explicit inhibit at NAV entry (search, lock, zoom steps and the acquisition orbit's GUIDED loiter run; no NAV, no attitude command; tested), or the harness flies the orbit with navpy's mode writes inhibited. Bounded GUIDED attitude test. aas-hitl emulator cross-check, read only | Calibration certified under load; gimbal association error within budget; acquisition range and zoom on the mast board; lock holds through the orbit; router clean; gates pass; lane bounds set |
| H2 | SITL approaches: ideal gimbal, then each effect added (yaw report scale with its calibration, report lag, 0x07 dead time, zoom speed); NAV zoom stepping down (N9); then 10 under the measured configuration (4) | Per setting: acquisition, time in NAV, outcome; 10 of 10 (7); pull-out height loss; every run logged |
| H3 | Real board (COM15 via the router, COM11 bridge; Mission Planner must not hold COM15), 1 then 10 | As H2, results beside H2 |
| H4 | Later: auto-zoom (owner: after HITL), real ZR10 K by reading after a per-reading calibration, GCS approval, moving dock, Novoxy/18x fixed-camera runs | Separate designs |

## Owner answers (2026-10-15)
1. Board: keep 2.4 x 3.2 m. → 4, 6.
2. Link: share the board's USB through a PC router. → 2.
3. Pass mark: 10 of 10. → 7.
4. Camera: first a fixed nose camera (v4), then the Novoxy SK-AHD2200 (10x, 5-50 mm) and a bigger 18x mentioned, then
   "Let's do gimbal instead": the approach uses the simulated ZR10 gimbal. Novoxy and 18x stay as profiles for later. → 4.
   v4's Q5 (stand the board up for the fixed camera) is withdrawn: the gimbal looks down at a flat board.

## Owner answers to v5 (2026-10-15)
5. Board mast: "Yes, 20 m mast" (same flat board, sim-only mast about 20 m up). → 6.
6. Circle: "Yes, 170 m" (about 170 m, 80 m above the board). → 4.
7. Final-approach zoom: "Zoom out as it gets close" (step down with range so the whole board stays in view). → N9, 4.
8. Auto-zoom: "Fixed steps now, auto-zoom later". → 4, H4.

No open owner questions.

## Codex review v5 r1 (2026-10-15): "rework", all applied in v5.1
P1: router needs `udpin` (1); report-lag sign (3); the emulator needs the aircraft attitude (4); the 500 m orbit is
outside the reader's range and confirm has no lock gate (4, Q2); reading-keyed K needs a runtime lookup, not only a
profile (N4); gimbal search was deferred (N5). P2: discrete steps need the zoom tracker and square-size sizing (4);
yaw report scale needs a calibration (3); zero distortion in the HITL profile (4); the mast moves the ground-plane orbit
centre (4); H1 inhibit must allow the acquisition orbit (9); final ten under the measured configuration (4). P3: the
zoom 2-10 rows are historical fabricated values (4).

## Codex review v5 r2 (2026-10-15): "rework", all applied in v5.2
Closed from r1: udpin, lag sign, emulator attitude, N4, N5, zero distortion, mast offset, H1 inhibit, P3. New or
incomplete: P1 fixed steps need an explicit policy and reading→command mapping (N6); circle, height and NAV zoom
need one visibility condition (4, Q2, Q3). P2: TRACKING is not a lock (explicit lock and circle gate); SIYI yaw sign
contract; one time axis for Unreal and navpy (certified offset); separate attitude and zoom report lags; no
placeholder in the final ten; quiescence at the board boundary (router); router write/close race (2).

## Codex review v5 r3 (2026-10-15): "rework", all applied in v5.3
Closed from r2: N6, lock gate, SIYI signs, time axis, separate lags, no placeholders, router race. Open and fixed:
P1 circle geometry must include the planner's minimum radius, the 35 deg NAV bank cap and the orbit-centre error
(feasibility contract, N8); P1 router barrier must precede the terminal record (2). P2 tracker capacity on the circle
(N7, condition 3); 0x07 deadband (4). P3 the profile's tracker is BoT-SORT, 0.30 s coast (4).

## Codex review v5 r4 (2026-10-15): "rework", all applied in v5.4
Closed from r3: router barrier, deadband, BoT-SORT. Fixed: P1 the circle must meet the 35 deg NAV bank cap (turn
minimum 167.7 m, so zoom >= 7; worked plan now 170 m, 80 m up, 10x); P2 N7 feeds forward a camera-motion-compensated
LOS rate, not the image-error rate; P2 N8 takes the camera position at exposure and H1 bounds the total centre error;
P3 UE's QPC epoch offset.

## Codex review v5 r5 (2026-10-15): "rework", applied in v5.5
All four r4 findings closed. P2: N8's pose must use the rendered board's ChArUco geometry (400/296 mm, 5 x 7), not the
class catalog or the 50/37 mm defaults.

## Codex review v5 r6 (2026-10-15): "rework", applied in v5.6
r5 closed. P1: after a forced stop the runner must close navpy's router endpoint itself before RTL (2).

## Codex review v5 r7 (2026-10-15): "approve"
r6 closed; no new findings. Approval covers the design contract, not HITL performance.

## Codex check of v6 (2026-10-15): "rework", applied
P2: one lens-travel rule (N3) and 0.1 x quantisation, exact in HITL; N9's trigger tests both axes and edge margins.
Round 2: "approve".
