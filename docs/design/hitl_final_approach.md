# HITL final approach to the Unreal dock (design v6, 2026-10-15)

**Status.** v6 applies the owner's answers of 2026-10-15 (below) to the Codex-approved v5.6; building. Done: the
router of section 2 (`tools/tuning/mav_router*.py`, Codex-reviewed), the profile `placeholder_18x`, the
terminal fence, router barrier and record in navpy with the runner's stop-before-RTL (section 2, "Built"), and B3
with the gimbal command fence (sections 2 and 4), and the ZR10 emulator (`tools/hitl/`, section 4,
Codex-reviewed; FPV refused). 2026-10-16: the owner's 7 points on the real ZR10 (section 4, "Real ZR10
behaviour"): yaw-outer readback, wire yaw sign, stops clamp, own attitude feed, 0x0D push; Codex-reviewed.
Owner's "What next?" item 3 ("Full final approach in HITL"). Builds on `hitl_unreal_camera.md`
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
  from it, `mav_router_core.py:127, 179`), `-ss` the board's system id (navpy is component 191, `conn_args.py:7-11`). The GCS
  launch hardcodes `--detector-type sim` (`src/gcs/backend/navpy_process_launch.py:125`); GCS and operator approval come
  later, so the first runs auto-confirm (`-tac`, default true, `nav_args.py:12`). No GCS stack, no `swarm_run.py --eval`.
- **ZR10 emulator (new, navpy `tools/hitl/zr10_emulator.py`, section 4):** SIYI protocol on UDP 127.0.0.1:37260, gimbal
  and zoom plant, publishes the true gimbal state to Unreal (17212).
- **Runner:** a new `sitl_driver approach` subcommand: board setup as `dock-pass` (params, erase, bridge,
  `--unreal-pose`, Yeghvard origin), then starts the emulator and navpy, takeoff, search mission, AUTO, waits for navpy's
  terminal record or a timeout, stops navpy, RTL, gate, logs. It keeps draining MAVLink the whole time, so `HeightFloor`
  (a hook on received GLOBAL_POSITION_INT, `flight_floor.py:56`) always fires.

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
  drops client packets while closed, counted. Built: one owner of the serial handle (open, write and close under one
  lock; a failed open releases the handle). Fail closed across transitions (link up or down, barrier, reopen): each
  receiver keeps the state it accepted before its wait, a datagram read across a change is dropped, and every
  transition wakes the receivers so they take the new state at once; a stale packet changes no stream claim. A
  datagram carries whole packets, so a partial packet is dropped, never joined to the next datagram. Tested through
  the real receiver: reboot under write load, a datagram read across a transition, partial UDP, failed open,
  repeated barriers, reopen of an open endpoint.
- **Quiescence at the board boundary:** the router queues raw bytes apart from navpy's lifecycle (`mav_router_core.py:39,
  168, 179`), so a command fenced in navpy can still reach the board after its terminal record. A router barrier on a
  loopback control port (HITL only; navpy gets its address as a run option): after its local fence and **before** it
  publishes the terminal record, navpy asks the router to close its endpoint; the router stops accepting navpy's
  packets, discards those still queued, waits for any write in progress to return, and acks with the discarded count
  and the time of its last navpy write. Only then does navpy write the record. No ack within a bound: navpy writes a
  barrier-failed terminal record and the run counts as failed. The router logs the write time and source of every
  packet sent to COM15, so "no navpy packet written after the terminal record" is checked at the board boundary. Tests:
  delayed queue, write in progress at the barrier, barrier timeout, ordering of ack and record. Built: with
  `--router-control HOST:PORT` (and `--router-client`, default `navpy`) the episode latch's first outcome
  (`episode_latch.py`, DOCKED, MISSED or FAILED) closes a fence on navpy's MAVLink connection (`LinkFence` wraps its
  `write`, where every pymavlink send ends: attitude, mode, mission, params, heartbeats), waits for the write in
  progress, closes the router endpoint (`router_barrier.py`, 2 s bound, resent every 0.25 s) and prints
  `NAVPY_TERMINAL_V1 {json}` (`terminal_handoff.py`). The fence stays closed: that is the hold, so RESET's AUTO restart
  never reaches the board. Built with B3: the gimbal command fence (`SiyiCommandFence`,
  `plugins/gimbal/siyi/hardware/command_fence.py`) wraps the SIYI SDK's single send point (`sendMsg`) of every SDK the
  real path creates, reconnects included. The same first outcome closes it; once closed only queries (0x01, 0x02,
  0x0A, 0x0B, 0x0D, 0x18, 0x25) and the two stops (0x07 0,0 and 0x05 0) pass, every other frame is dropped and counted.
  A SIYI rate or manual-zoom command keeps acting until the next one, so per gimbal endpoint (own send lock) the
  handoff waits for a send in progress and sends one zero rate and zoom hold, all under one 1 s bound
  (`GIMBAL_STOP_TIMEOUT_S`: a stuck send never holds the record or another gimbal's stop), then the barrier; the
  record adds `gimbal_stop` (sent, failed, timeout, no_gimbal; anything but sent or no_gimbal fails the run) and
  `fenced_gimbal_commands`.
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
  `flight_floor.py:74`); the stop time counts in the floor clearance. Built: `tools/tuning/navpy_handoff.py`
  (`NavpyHandoff.before_rtl`: stop with a 10 s bound then terminate, the barrier, and on no ack the router ended and RTL
  sent on the board's own port, confirmed by its heartbeat) and `HeightFloor(before_recovery=...)`; wired into
  `sitl_driver approach` when that is built.
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
- **Real camera path: capture time, not arrival (owner, 2026-10-15, fix B; with N3/N7).** The lateral LOS rate is a
  raw two-point difference, (bearing - prev_bearing) / dt (`lateral_rate.py:119-127`), with no low-pass
  (`LATERAL_RATE_FILTER_TAU_S = None`, `law.py:49`) and no runtime minimum dt. Real frames are stamped READ, `rt.time()`
  on arrival (`frame_capture_ffmpeg.py:116,126`, `frame_capture_opencv.py:196,206`, `capture_lookup.py:149-161`). The
  ZR10 rig (gimbal-controller `out/step0-ev5`, 231k frames) measured camera PTS steps of exact 33.3 ms multiples but
  bunched arrivals after decoder or network stalls: 1.41 % of gaps under 20 ms, minimum 6 ms. A 6-20 ms dt amplifies
  per-frame angle error 2-5x into one-frame roll-command spikes (SITL shows the same at dt 0.02 s). Rule: before its
  pixels feed the LOS rate, a real frame is timed by its capture time, the stream PTS mapped to the PC clock (the rig's
  `gimbal/freshness.py`: 7.5 ppm drift, 7.6 ms scatter), i.e. an EXPOSURE stamp as for npyc; READ frames do not feed
  the rate. `capture_lookup.py`'s `min_frame_interval_s` (>= 10x variation) is checked on the calibration file only,
  not at runtime. Separate: the sim pose reader pairing attitude with another boot's GLOBAL_POSITION_INT (fix A,
  owner-plan chat); a dt-weighted rate filter (C) is deferred.
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
    navpy's yaw is right-positive: `plugins/gimbal/siyi/siyi_convention.py` negates yaw in 0x0E commands and 0x0D
    reports; 0x07 rates pass unchanged (positive turns right on both). The
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
  - *Real ZR10 behaviour (owner, 2026-10-16; facts from gimbal-controller main 45104e3).* (1) The gimbal is
    yaw-outer: camera-to-body = Rz(yaw) Ry(pitch) Rx(roll), navpy seq `"ZYX"` (`siyi_convention.py`). The real
    client, the sim plant readback (`gimbal_frame_transform.py` `matrix_to_zyx`) and the siyi profile use it; a siyi
    profile with another seq is refused. Roll sign unmeasured. (2) 0x0E in LOCK with a tilted base is unmeasured, so
    it stays world angles, as navpy's sim. (3) Stops: the camera is clamped at the body limits (yaw -135.0..+132.8,
    pitch -88.9..+23.9); the plant target is kept, so the camera returns when the aircraft levels; `stop_steps`
    counts steps at a stop; the run is not invalidated. Roll stops and the real behaviour at a stop are unmeasured.
    (4) Attitude: the bridge's `--vehicle-state-copy HOST:PORT` sends a copy of `vehicle_state.v1` to the emulator
    (the unit's own IMU), with or without `--unreal-pose`. Whether the real unit uses 0x22 is unmeasured. (5) 0x0D:
    one request gets its reply and starts a push at `attitude_push_hz` (50, one per plant step; real about 50.5 Hz;
    0 = request/reply only); push expiry and a second request's effect are unmeasured. (6) No runner stop/summary
    protocol yet: the HITL runner does not launch or read the emulator. Follow-up: when it does, use the bridge's
    stdin-EOF stop (`--exit-on-stdin-eof`). (7) `fov_h_rad` is not sent; `sender_time_s` is the scheduled step time.
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
  profile's `siyi_ip`/`siyi_port`; tested so a real run never gets the sim gimbal. Built: `--detector-type real`
  selects the real path (a real run gets no simulated gimbal of any kind, guard over every profile); with
  `--router-control` its SDK is built through the gimbal command fence (2).
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
  `nominal`); results from a nominal profile are labelled so. New profile: `placeholder_18x` (ZR10 zoom-1 K, max
  18x, labelled placeholder). Novoxy SK-AHD2200: dropped (owner, 2026-10-15: concentrate on the ZR10). The
  fixed-camera path (v4's B2: a factory for one fixed K without a zoom command) is not needed for the gimbal runs and
  is deferred.

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

## 9. Layers and steps
Four layers, in the owner's order (2026-10-16). Each has its own plan; do not mix their results.
1. **Flight like the real Peron**: the simulated aircraft flies like the real flights in the logs. Section 10.
2. **Real camera stream** instead of fake pixels: `hitl_unreal_camera.md`.
3. **Vision as close to real as possible**: the ZR10 emulator, detector and gimbal effects (H0-H3 below).
4. **Other things**: other contributors, H4.

| Step | What | Done when |
|---|---|---|
| H0 | Q1-Q4; router (2); B3 real SIYI path; ZR10 emulator with aircraft attitude from the bridge, the dynamics table and zoom table (4); gimbal/zoom readback lag-corrected, scale-calibrated and interpolated at exposure (N3); optical K by reading (N4); gimbal search sweep (N5); fixed-step zoom policy (N6); SIYI sign contract; one PC clock and the certified gimbal frame offset; confirm only while locked and on the circle; plan feasibility check (4); zoom steps in NAV (N9); tracker rate feedforward (N7); orbit centre from the board pose (N8); offline partial-view tail check with BoT-SORT; router single-owner serial handle and the barrier before the terminal record; HITL profile `siyi_zr10_hitl` (zero distortion); B1 calibration tool and EXPOSURE-only rule; board-centre navigation pixel; B4 contact sensor + identity binding; terminal fence (incl. gimbal commands), hold and record; observation-mode inhibit; effective settings in the start record; `sitl_driver approach`; report tool; Unreal U1-U4 and the board mast (Q1); profile `placeholder_18x` | Unit + guard tests (Pure Vision, contact isolation, no dock data or rendered state in navpy, EXPOSURE-only, real path never sim gimbal, emulator protocol against navpy's client and against recorded aas-hitl replies, router routing and stream claims, terminal handoff interleavings), full suite |
| H1 | SITL stand-in + router + bridge + Unreal + emulator + navpy, observation only: an explicit inhibit at NAV entry (search, lock, zoom steps and the acquisition orbit's GUIDED loiter run; no NAV, no attitude command; tested), or the harness flies the orbit with navpy's mode writes inhibited. Bounded GUIDED attitude test. aas-hitl emulator cross-check, read only | Calibration certified under load; gimbal association error within budget; acquisition range and zoom on the mast board; lock holds through the orbit; router clean; gates pass; lane bounds set |
| H2 | SITL approaches: ideal gimbal, then each effect added (yaw report scale with its calibration, report lag, 0x07 dead time, zoom speed); NAV zoom stepping down (N9); then 10 under the measured configuration (4) | Per setting: acquisition, time in NAV, outcome; 10 of 10 (7); pull-out height loss; every run logged |
| H3 | Real board (COM15 via the router, COM11 bridge; Mission Planner must not hold COM15), 1 then 10 | As H2, results beside H2 |
| H4 | Later: auto-zoom (owner: after HITL), real ZR10 K by reading after a per-reading calibration, GCS approval, moving dock, 18x fixed-camera runs | Separate designs |

## 10. Layer 1: the simulated Peron flies like the real flights (v4, 2026-10-17)
**Goal.** Fewer real re-flights: a flight in the bridge model looks like a real flight of the same kind. The reference
is the real logs. Launch and catapult are low priority. Flight tuning (`flight_tuning.md`) is a separate, parked feature.
**Evidence level of everything below:** logs and an offline replay of the model (JSBSim `Peron15_hitl4`, open loop).
No closed-loop board run against a real log exists yet. The bands were approved by the owner on 2026-10-16 and `Peron15_hitl4` is scored below.
v4 (2026-10-17) corrects v3 after a deeper log analysis by Codex that Claude checked against the logs; its findings are in section 10a.

**Step 1: inventory (read only).** Facts on disk (2026-10-17), not names from memory:
- 31 flight-log files in `Downloads\Logs`. Group = SYSID in the log, except that the seven `peron_160_*` files (SYSID 157) form their own group "160". A SYSID is a parameter: it names neither a board nor a frame. Dates are GPS time from the log.
- Short log names in this section: "log N" alone is the SYSID 158 file `gdrive_<date>_000000NN` (logs 6, 10, 11, 15, 16, 21, 25, 30, 32; "log 30" = `gdrive_20250927_00000030`);
  '"160" log N' is `peron_160_000000NN`; a SYSID before "log N" names that SYSID's file with that number ("151 log 22" = `gdrive_151_00000022`). Other files are named in full.
- The folder of other aircraft (`Downloads\Logs_other_frames`, 104 logs, not used) is no longer on this PC. Its two `peron_160_*` files (00000002, 00000006) were byte copies of
  `peron_157_20241019_00000002` and `peron_157_20241102_00000006` (owner, 2026-10-16).
- Text `.log` copies of the 31 BIN files were made on 2026-10-17 by someone else. They match the BIN files record by record, except one metadata record (the first UNIT record) per file, and they hold floats to 7 significant digits. The BIN files stay the source.
- No log file on this PC is named for 153, 154, 156, 161, 162 or 163. For 161-163 only preflight parameter files exist (`Peron\Tune\20260711_Preflight`), plus one Mission Planner telemetry log of 163
  (`Mission Planner\logs\FIXED_WING\163\2026-10-12 18-42-28.tlog`): 7.5 s, disarmed, ground speed 0, so no flight.
- Aircraft mass is not recorded in an ArduPilot log; 7.5 kg is the model value. No log holds a MODE record, but the mode class (automatic throttle, FBWA-like,
  AUTOTUNE, MANUAL) can be measured for most of the flight time (section 10a, B1). Segments are picked by what the aircraft did (rates, speed, sink, bank). The
  per-log table and the dive table are in the session notes of 2026-10-16
  (`%APPDATA%\Claude\side-session-notes\local_5f42d6d8-c1df-43a1-9cb4-6cd20c3e9c83\layer1-log-inventory.md`). They predate v4: they call the SYSID groups
  airframes, and their per-log table gives 3D speeds that include wind. Where they differ from this section, this section holds. The 38 dives with their heights
  are also in `verify\_resolve\R7a\table.md` of the section 10a output.

| SYSID group | Files / flying logs | Dates | Flight min | Note |
|---|---|---|---|---|
| 151 | 5 / 4 | 2023-11 to 2024-01 | 62 | 1100-1900 us throttle range; two APJ 50 boards (P on 2023-11-11, Q from 2023-11-18) |
| 152 | 3 / 2 | 2023-12, 2024-08 | 47 | the Durandal board of 157 and "160" |
| 155 | 1 / 0 | 2024-09 | 0 | one failed launch only |
| 157 | 3 / 3 | 2024-10 to 2024-11 | 58 | the Durandal board of 152 and "160" |
| 158 | 11 / 11 | 2024-11 to 2026-01 | 174 | 2 logs on an APJ 50 board (2024-11, very probably board P), 9 on a Pixhawk 6X (2025-26); 2026-01-18 is the model's one fit flight; throttle about +0.06 vs model on the 6X logs, about -0.01 on the two 2024 logs (cause not in the logs) |
| "160" (file SYSID 157) | 7 / 7 | 2025-01 to 2025-09 | 117 | the Durandal board of 152 and 157; frame not identified; own AHRS trim and servo trims; throttle about -0.17 from "160" log 29 |
| 149 | 1 / 1 | 2023-05 | 10 | other model, not used except 3 of the 15 throttle-off drag windows (section 10a, A4) |

**Which boards and settings are alike (log fingerprints; corrected 2026-10-17).** The logs identify boards and parameter states, not frames. No logged value names a frame:
BRD_SERIAL_NUM is 0 in all 31 files. The files hold no MSG and no MODE records although both formats are defined, so these copies were probably filtered.
The original SD-card logs should hold the board's start-up messages (with its MCU ID) and the mode records; ask the owner for them. File names are archive labels.
- Owner practice (2026-10-16): there are few boards and parts are reused, so a board moves to a new frame together with its airspeed sensor and ESC. The level (AHRS trim) is
  redone after a new frame with an old board, and sometimes again on the same board. So a new AHRS trim hints at a new frame but does not prove one.
- Board models. The IMU device IDs (INS_ACC*_ID, INS_GYR*_ID) encode bus, address and sensor type, so they name a board model, never one board. In the 31 logs:
  APJ 50 (FMUv5 class, IMU type A, INS_ACC2_ID 2688010) in 9 logs; Durandal (APJ 139, type B, 2819082) in 13 logs; Pixhawk 6X (APJ 53, type C, three ICM45686) in 9 logs.
  (A scan of the 137 logs then on disk on 2026-10-16 (these 31, the 104 of the other folder and its two `peron_160_*` copies) found type A and type B on many other SYSIDs from 2021 on; that folder is gone and the scan was not repeated.)
- What can name a board. Parameter files carry values from board to board: values match after a 7-digit text round trip (for example ARSPD_RATIO 1.8086971 of the Durandal on 2024-10-05
  becomes 1.808697 on the same Durandal from 2024-10-19 and on two other boards: the 2024-11 SYSID 158 board (very probably board P) and the 6X until 2025-04-05). So accelerometer offsets, AHRS trim and airspeed ratio cannot name a board. Two kinds of values do follow a board: the boot gyro offsets, which are
  measured again at every boot, and the AP_Stats counters (STAT_RESET, BOOTCNT, FLTTIME, RUNTIME). The counters are parameters and can be rewritten, so they are strong evidence, not proof.
- Durandal (APJ 139): SYSID 152 in 2023-12 and 2024-08, then SYSID 157 from 2024-10, including all seven `peron_160_*` files. All 13 logs share one AP_Stats history
  (the same STAT_RESET stamp of 2022-01-28, BOOTCNT rising from 216 to 475). From `peron_157_20241019_00000002` to `peron_160_00000035` (2025-09-27) each file's flight time starts where the previous
  file ended (gap 0 s in 8 pairs). So very probably one board flew all these logs (strong evidence, not proof), and no flight time of it in that period is missing from the archive.
  Its AHRS trim changed six times from 2023-12 to 2025-05: between 2023-12-24 and 2024-08-31 (still SYSID 152), at the SYSID change (between 2024-08-31 and 2024-10-05), slightly between 2024-10-05 and 2024-10-19,
  then between 2024-11 and 2025-01, by 2025-04-05 and by 2025-05-04; it is unchanged from 2025-05-04 to 2025-09-27. Each change fits a re-level, which hints at a frame change but does not prove one. "160" may be a frame name, but no logged value shows it.
- SYSID 151 was two APJ 50 boards: P on 2023-11-11 and Q from 2023-11-18 to 2024-01-13. Q carries P's calibration through a text parameter file. The two
  `peron_152_20231120_*` files are SYSID 151 and were recorded on 2023-11-18 by GPS time (file 04 holds no flight).
- SYSID 158 on 2024-11-02 (`peron_158_20241102_*`, APJ 50): its boot gyro offsets match board P (0.5-0.9 mrad/s apart at 25-27 C), so these two logs very probably flew on
  board P, with parameters from board Q's line. Its accelerometer calibration is new. The v3 statement that they share 151's offset (0.41) and trim (0.042) rested on one component and is withdrawn.
- SYSID 158 on the Pixhawk 6X (2025-01-28 to 2026-01-18): the AP_Stats counters restart between 2025-05-18 and 2025-05-25. The 2025-05-25 log is also the first with a new airspeed sensor setup, a CAN
  compass, a new accelerometer calibration and a new AHRS trim; the logs do not show whether these came on one day. The compass offsets are the old values after a
  7-digit text round trip (COMPASS_OFS_X -51.3976135 to -51.3976097), which points to a parameter-file load, not a new compass calibration. The logs cannot tell a parameter reset on the same board from a second 6X; the boot gyro offsets continue across the break, which favours the same board.
  A second counter drop on 2026-01-18 (firmware 4.6.3) is not explained.
- Two aircraft flew at the same time on three dates (both logs flying, GPS fixes matched within 0.1 s): 2025-01-28 (235 s, 0.2-2.2 km apart), 2025-08-16 (1328 s, median 0.51 km
  apart and 99 m in height) and 2025-09-27 (325 s, 0.46 km, 97 m). So the `peron_160_*` logs and the SYSID 158 6X logs are two aircraft on those dates.
- The SYSID groups used later in this section are not independent boards: 152, 157 and "160" are very probably one Durandal board; "158 with all its dates" is very probably board P in 2024 and a 6X in 2025-26; 151 is boards P and Q.
- By how the aircraft behaved (each log's throttle offset against the model, as the replay applies it: the mean over that log's trim bins at 20 m/s or more, all banks): 151, 152, 157
  and 158 (2024) lie within -0.04 to +0.03 (the one level bin of 151 log 22, at 26 m/s, reads +0.11); 158 (2025-26) needs +0.01 to +0.11 more throttle (median +0.06); the
  `peron_160_*` logs go from +0.04 ("160" log 11) to -0.03 ("160" log 21, the only one of logs 18-22 with a bin) to -0.17 ("160" logs 29-35). These per-log values come from the
  v3 replay data and were not re-derived in v4. This is not a gradual drift. "160" log 11's high throttle is mostly its low pack voltage. Between "160" log 22 (2025-05-18) and
  "160" log 29 (2025-08-10) the throttle-to-power map steps: at the same power and speed, throttle x pack voltage falls by about 28 %, and at 0.7-0.8 throttle the current is
  about 100 A in "160" logs 29-35 against 50-56 A in "160" logs 18-22. The throttle settings checked (servo output range, THR_MAX, battery compensation, output protocol,
  current scaling) did not change; the throttle RC calibration moved by 1-2 us (RC3 990/2012 to 988/2011 us), and a full parameter comparison is still open. The cause (ESC calibration or replacement, motor, or propeller such as the APC 14x8.5) is not in the logs: there is no RPM or ESC telemetry.
- Which frame carried the Durandal board in 2025 is not shown. A dated flight or maintenance record would settle it. In this section "160" is only the name of the seven `peron_160_*` logs as their own group.

| Item that matters | Covered by | How much |
|---|---|---|
| (a) roll and pitch response | all flying logs; 25 Hz ATT and rate-controller rates | 241 roll and 83 pitch windows (peak rate > 30 and > 12 deg/s). No clean doublets; the one MANUAL segment is 80 s of roll (section 10a, B1). |
| (b) trim and speed | steady level and turn bins; throttle steps | 82 bins (all banks), 0-4 level bins per log (11 of the 29 replayed logs have none); 5-8 s steady windows; elevator per g in 25 turn pairs |
| (c) dive and pull-out | see below | **Yes, real dives exist** |
| (d) wind and gusts | EKF wind estimate (XKF2), every flying log | 1.1-8.8 m/s median per log (the 155 launch-only log, 1.0 m/s from 19 samples, left out); gust is only an EKF-smoothed estimate |

**Are there real dives? Yes.** 38 dives of 4-10 s with mean pitch -30 deg or steeper, in 11 logs of SYSID groups 151, 152, 157, 158
and "160"; 23 of them steeper than -45 deg (22 without the one left out). That one (`peron_160_00000018` at 2387 s, 15-20 m/s, throttle 0) is a low-speed departure
that ends in a ground impact; it is not used. Speeds are air-relative (EKF velocity minus the EKF wind). Heights lost are from IMU height fitted to GPS (section 10a, A2); EKF height reads high under load, so it gives 1-17 m less per dive:
- `gdrive_151_00000022`: 6 steep dives, 8-10 s, pitch -59 to -61 deg, throttle 0.08-0.17, 24-29 m/s to 55-59 m/s, 286-375 m lost.
- `gdrive_20250927_00000030` (158): 10 dives, 4-7.4 s, pitch -31 to -61 deg, throttle 0.91-0.97, 28-33 m/s to 58-68 m/s, 137-238 m lost. The last one (from 3606.04 s) runs
  into the in-air event at 3611.8 s that came before the ground impact at 3615.5 s, and section 10a excludes this log's data from 3611.0 s. Without that dive the ranges are the same,
  except the height lost (137-234 m).
- `gdrive_20250518_00000015` (158): 4 dives, 8.1-8.9 s, pitch -49 to -50 deg, throttle 0.45-0.48, 23 m/s to 51-52 m/s, 241-259 m lost.
The passes of one log repeat closely: the same entry point, height and heading (in `gdrive_20250518_00000015` the four entry points lie within 16 m, and the dive
tracks extended to the home-height plane end within 25 m). The tracks run parallel, so this shows a repeatable manoeuvre, not steering to a common ground point. In
`gdrive_20250927_00000030` the end point moves with the saved AAS_TERM_PITCH value (passes from one spot with -30 and -55 end 307 m apart). In 41 of 49 steep intervals the NTUN
target at dive entry is the aircraft's own position at the moment the target was set, 1-28 s before the dive (in 20 of them at a logged GUIDED request), so it is not an
aim point. The sticks are centred in most dives and the logged pitch demand follows the dive, so the demand came from an automatic source. Of the 47 steep intervals with
automatic demand, 21 start 4-50 s after an accepted GUIDED request and 26 have no logged mode request (log 30 has none at all); in 16 of the 47 the pitch stick goes nose-up
at recovery (STICK_MIXING 1; section 10a, B1). The owner says 50-70 m/s approaches were flown and some dives may have tested maximum load; mode and pilot are not known
(owner decision 3).
Limits: these speeds are 50-70 m/s, far above the replay windows and trim bins behind rows 1-12 (window start EAS 16-36 m/s, real speed in them at most 36 m/s; trim bins 18-31 m/s); only the dive replay of rows 13-15 compares the model with real flight above that (model peaks up to 61 m/s as corrected, 64 m/s with the elevator offset only). In 22 dives of logs 30, 151 log 22, "160" logs 11 and 30, and log 16 (after its autocal)
the raw pitot TAS (ARSP airspeed x CTUN.E2T) is 0.97-1.02 of GPS velocity minus EKF wind (median over each dive; a 0.2 s GPS velocity lag lowers these ratios by 1.6-2.5 %); log 25 reads 1.04-1.10, and log 15 1.26-1.31 (inferred: a sensor read on the wrong pressure scale, section 10a, B3). Comparing peaks, the raw pitot reads higher than the EKF air speed: 1.01-1.09 in log 30, 1.04-1.12 in "160" log 30 and 1.09-1.18 in log 25. In the scored dive of "160" log 18 at 2130 s the replay TAS peak is 12 % above the EKF peak. The EKF wind is partly
fitted to the pitot, so this is a consistency check, not an independent calibration. (v3 quoted "within 6 %, for example 70.3 against 68.1 m/s": that pair is the first log-30 dive,
the replay's CTUN-based TAS peak against the air-relative peak. The v3 check used that replay TAS throughout, and in log 15 and the 151 logs it is the EKF's own synthetic airspeed.) Against a GPS wind triangle that uses no compass and no EKF wind, the logged TAS
of a flight is 0.92-1.07 of the GPS air speed (median 0.995, 22 logs); it varies 6-8 % between logs of one unchanged setup and up to +-7 % within one log.

**Step 2: scorecard (approved 2026-10-16).**
How the bands are built:
- Ratios are model / real from the open-loop replay (corrected mode, one value per SYSID group: 151, 152, 157, 158 with all its dates, "160").
  With one fixed model, the spread of these ratios between groups is the best available picture of how much real aircraft differ
  (plus window and noise effects). The shared offset of the fleet median is the model error. So the fleet-median band judges the model
  and the per-group band ("each group" in the table) allows for real spread. They are engineering targets set from these spreads, not measured tolerances. The ratio spread still mixes aircraft differences, the operating
  condition of each window and noise, so step 3 also reports it by condition (speed, bank, throttle).
- The groups are not independent aircraft (fingerprint paragraph): 152, 157 and "160" are very probably one Durandal board; 151 is two APJ 50 boards (P and Q); 158 is very probably board P in 2024 and a Pixhawk 6X in 2025-26, so 151 and 158 very probably share board P. No logged value maps a group to a frame,
  so a frame-weighted fleet median cannot be derived from the logs; the bands stay as approved, and this limits what the per-group spread shows.
- "Reference" shows the old `Peron15_hitl3 -> hitl4` numbers from the validation reports (`compare.txt`), for context only. It is not a score.
- Rows with small samples are marked provisional; step 3 recomputes every spread with a bootstrap interval before the bands are fixed.

| # | Item | Measure | Real spread (the basis) | Band | Reference |
|---|---|---|---|---|---|
| 1 | Roll-rate gain | group medians | old-model ratios by group 0.69-0.83 (range 0.14); IQR inside 158: 0.70-0.91 | fleet median 0.95-1.05; each group 0.85-1.15 | 0.69-0.83 -> 0.85-1.03 |
| 2 | Roll lag | model - real | group medians -50..0 ms; 25 Hz = 40 ms steps | each group within +-40 ms; fleet median within +-30 ms; step 3 first measures the lag uncertainty | -50..0 -> -15..+33 ms |
| 3 | Pitch-rate gain | as 1 | old-model ratios 0.82-0.92 (range 0.10); only 1-5 windows on 158-fit, 152, 158 (2025) | as 1, provisional | 0.82-0.92 -> 0.86-1.00 |
| 4 | Pitch lag | as 2 | group medians -30..+30 ms | as 2 | -30..+30 -> -25..+30 ms |
| 5 | Rate-response shape | RMS error and correlation next to gain and lag | held-out roll gain 0.81 -> 1.00 while RMS went 35 -> 46 deg/s | report only until step 3 shows the floor; a row 1-4 pass with a worse RMS is flagged | RMS 20-30 deg/s |
| 6 | Elevator per g in turns | median of per-pair ratios real/model, pairs in one log | 25 pairs, real -0.06..-0.51, median -0.181 | ratio 0.85-1.15, provisional | 0.96 -> 1.11 |
| 7 | Level trim offset | real - model at each bin's own EAS: throttle; elevator (us) | throttle offsets by group -0.13..+0.07 (fleet median +0.01); elevator -42..+36 us (10 mm CG = 19 us) | fleet-median offset: throttle within +-0.03, elevator within +-20 us; each group's own offset inside the real range (throttle -0.16..+0.07, elevator -45..+40 us); a group with fewer than 3 bins is inconclusive (151 with 1 bin, 152 with 2) | thr -0.13..+0.07; elev -42..-6 us |
| 8 | Trim attitude | pitch in the AHRS frame | real - model offsets -0.5..-3.0 deg; AoA is an EKF estimate with its own reference | report only (reference frame unknown) | model 2-3 deg nose-higher |
| 9 | Trim slope with speed | per m/s: throttle; elevator | sessions with 4-8 level bins: throttle 0.010-0.026, elevator 0.004-0.010 (157, 158-2024, 158-fit, 160; 158 pooled over 7 flights is not used because each flight has its own trim, elevator offsets -67..+41 us) | model slope inside each range, provisional | thr 0.012-0.025; elev 0.009-0.010 |
| 10 | Throttle needed in a 30-45 deg turn | (turn - level) at the same EAS, model - real, matched pairs; also check climb and acceleration in the pair | 9 pairs, real median +0.006, range -0.02..+0.05 | median difference within +-0.03 | model +0.051 (0.02-0.09) |
| 11 | Speed after a throttle step, 8 s | EAS error, model - real, below 30 m/s | no real repeat spread known | engineering target +-2 m/s, provisional; above 30 m/s report only (known pitch-up) | -8..+6 m/s |
| 12 | Steady cruise and turn, 5-8 s | EAS error; height error (steady windows exist for 151, 152, 158, "160"; none for 157) | no real repeat spread known | engineering target EAS +-1.5 m/s, height +-8 m (hitl4 group medians were -9..+1 m in the validation reports), provisional | EAS -0.6..+0.8; height -9..+1 |
| 13 | Dive speed gain | air-relative speed gain 5 s after dive start and peak speed; replay from the logged state with the logged elevon and throttle (free response; also report pitch and flight-path angle) | families: 151 steep (6): gain 20.4-22.6 m/s, peak 54.8-58.7; 158 log 30 shallow (5): gain 23.7-29.5, peak 58.3-63.8; 158 log 15 (4): gain 23.1-25.4, peak 51.4-51.9 | gain within +-15 % and peak within +-8 % of the matched real dive (family repeat spread is +-5 to +-11 % and +-0.5 to +-4.5 %) | not measured |
| 14 | Dive time history | RMS of speed, flight-path angle, pitch and height over the whole dive and pull-out | none yet | report only until step 3 sets the floor (separates drag, lift and control errors) | not measured |
| 15 | Pull-out | from the first pitch rate above 25 deg/s: height lost to the lowest point; peak pitch rate (the pull-out start is not the dive start) | 158 log 30 shallow: 9-12 m, rate 121-131 deg/s; 151 steep: 27-32 m (one 65), rate 92-103; 158 log 15: 86-114 m, rate 89-103 (a slower recovery: it depends on the logged control history and the entry state, so replay the logged elevator). Heights from IMU height fitted to GPS (2026-10-17); v3 used EKF height, which reads 1-15 m less here | height within +-25 % or 5 m, whichever is larger; peak rate +-20 %, provisional | not measured |
| 16 | Wind input | what the bridge applies | EKF wind median 1.1-8.8 m/s per log, p95 up to 10.5; 5 s change p95 0.2-2.1 m/s | capability test: steady wind 0-12 m/s applied to air-relative velocity, direction and frames checked; gust realism stays unvalidated (EKF smooths gusts) | not tested |

Notes on the bands:
- Event definitions. Dive: flying, climb < -3 m/s and pitch < -20 deg for at least 4 s, mean pitch <= -30 deg. Pull-out start: the first pitch rate
  above 25 deg/s after 2 s of dive; the height loss and the peak rate are taken over the next 10 s. Speed gain: over 5 s, or to the dive end if the dive is shorter (to be flagged; the v3 outputs do not flag them: 3 of the 37 scored dives, 4.08-4.72 s, in logs 25 and 30).
- Rows 1-5 and 11-12 reuse the validation replay windows. Rows 6-10 come from the trim bins (row 6 from turn and level bin pairs). Rows 13-15 need a dive replay. Row 16 is an input check.
- A band is no tighter than the measurement allows: rigging offsets (elevator -42..+36 us, aileron about +-40 us), 25 Hz sampling, the airspeed scale (0.92-1.07 per log, up to
  +-7 % within one log), the EKF wind, and repeated windows of one log are not independent samples. Fleet bands (the model) and per-group bands (real spread) are kept separate.
- Pull-out and dive heights come from IMU height fitted to GPS, not from barometer or EKF height: every barometer reads falsely high under load (section 10a, A3).
- The old "held-out" log (`peron_157_20241005_00000100`) has been looked at, so it is a regression log. Final acceptance uses logs not replayed
  so far (flights still to come; the dive logs do not count: their roll windows were in the hitl4 fit and step 3 scored their dives), chosen by a rule written down in step 3 before any change.
- Known open gaps the card should show, not hide: row 10 (model needs more throttle in a turn), the model's pitch-up above 30 m/s, and everything
  above 40 m/s (never validated). Launch, catapult and the < 17 m/s region stay out of scope.

**Step 3 result: `Peron15_hitl4` scored on the card (2026-10-16).** The owner approved the bands as proposed ("Bands approved"). No model file was changed.
Scripts and outputs: `%LOCALAPPDATA%\navpy-hitl-tools\validation\2026-10-16-layer1\` (outside the repo). Evidence level: logs and offline open-loop replay; descriptive, not independent validation.
- Rows 1-5 and 11-12 reuse the existing hitl4 replay windows, and rows 6-10 its trim bins (`trim.json`). The roll and pitch windows were also used to fit hitl4 (roll +25 %, 35 ms delay, pitch +15 %), so rows 1-5 are in-sample.
- Rows 13-15 are new: 37 real dives replayed from the logged state with the logged elevon and throttle (the departure dive of `peron_160_00000018` is left out). "Corrected" means the log's own
  trim offsets (elevator, aileron, throttle, AoA) are removed, as in the other rows. They are the n-weighted mean of all trim bins of that log at EAS 20 m/s or more and any bank
  (20.1-30.6 m/s, 79 bins in all); seven logs with no such bin take their group mean. (v3 said "fitted at 20-28 m/s"; the replay code does the above.)
- No aggregation rule was set in the card for rows 5-16, so each verdict gives the median and how many windows or dives are inside the band. "Mixed" means the median is inside and many are not.
- Acceptance logs: `gdrive_20250927_00000030` is not untouched: its roll and pitch windows were in the hitl4 fit, and its dives are now scored. It stays a known-failing family; final
  acceptance needs logs that no fit and no scoring has used (flights still to come; `peron_160_00000002` and `_00000006` are copies of 157 logs already used, so they do not count).
- Bootstrap intervals are 95 % over windows inside each SYSID group (151, 152, 157, 158 all dates, "160"). Windows of one log are not independent.
- The replay inputs were checked again on 2026-10-17 (section 10a, A1-A2). Rows 14-15 were rescored then against a better real height, with the stored model values (no model rerun).
  The other rows are scored as run; the effects of the input errors found in A1 are estimates, not rescores.

| # | Result | Verdict |
|---|---|---|
| 1 Roll gain | fleet median 0.97 (CI 0.96-1.04); groups 0.88 (157) to 1.03 (160) | pass (in-sample) |
| 2 Roll lag | fleet median 0 ms (CI -5..+3); groups -15 (152) to +30 ms (157) | pass (in-sample) |
| 3 Pitch gain | fleet median 0.94 (CI 0.91-0.99); groups 0.86 (152, 3 windows) to 1.00 | fail, narrowly (band 0.95-1.05); with the compass-free wind-triangle airspeed on every log about 0.954, a narrow pass (estimate, low confidence, 10a A1) |
| 4 Pitch lag | fleet median -15 ms (CI -20..-5); groups -25 to +25 ms | pass (in-sample) |
| 5 Shape | roll RMS 21-31 deg/s, correlation 0.89-0.96; pitch RMS 5.5-25 deg/s, correlation 0.85-0.97 | report only (no earlier model to compare in this run) |
| 6 Elevator per g | median 1.11 (CI 0.95-1.20, n 25 pairs); 158 alone 1.31 (n 13), 160 0.95 (n 11) | pooled median inside the band; CI crosses the edge and 158 is outside: weak pass as scored. Not settled: the replay air density comes from E2T, which uses the airspeed sensor's temperature at arming (B3); with ISA air the median would be about 1.26 (ISA+15 K: 1.20), out of band (estimate, 10a A1) |
| 7 Level trim offset | fleet median (157, 158, 160) throttle +0.012; elevator -22 us (band +-20); 151 and 152 have under 3 bins | throttle pass; elevator fail, narrowly |
| 8 Trim attitude | real - model -2.0 to -2.9 deg on every group | report only (model flies 2-3 deg nose-higher) |
| 9 Trim slope | model slopes inside both ranges on all 4 sessions; on "160" the model throttle slope is 0.024 against 0.010 real | pass (the band is wide); with ISA air density the 157 model throttle slope may be about 0.030, above the 0.026 band top (estimate, low confidence, 10a A1) |
| 10 Throttle in a turn | model - real median +0.040 (n 9; real median +0.006, model +0.051); acceleration in the pairs not checked | **fail** as scored (band +-0.03). Not settled: with ISA air density about +0.029, at the band edge (ISA+15 K: +0.034) (estimate, 10a A1) |
| 11 Throttle-step speed | median error +0.1 m/s, but only 51 % of 81 windows within +-2 m/s; 151: +8.2 m/s (7 windows), 152: +11.5 (1 window) | mixed; 151 fails |
| 12 Steady cruise and turn | EAS median -0.2 m/s, 89 % of 36 windows within +-1.5; height median -0.3 m, 78 % within +-8 m; 151 height -9 m (4 windows); none for 157. One window (log 6 at 3783.8 s) is not flight (10a A1); without it, EAS 32 of 35 and height 28 of 35 inside (a recount, not an estimate) | mixed to pass |
| 13 Dive speed | **Corrected (card convention): 12 of 37 dives have gain (+-15 %) and peak (+-8 %) both inside; median gain -16 %, peak -7 %.** By logged throttle: below 0.5, 10 of 13 inside (gain -4 %); 0.5-0.85, 2 of 10; 0.85 and up, 0 of 14 (gain -31 %, peak -16 %). With the elevator offset only: 22 of 37 inside (gain -6 %, peak -3 %); 7 of 14 at throttle 0.85 and up (gain -16 %). Input sensitivity (estimates, section 10a A1): the start speed biases the peak by -5.1 to +4.0 % per dive; started at the EKF speed, corrected 14-16 and elevator only 22-23 of 37; at ISA density, corrected 9 and elevator only 22 (19 of 27 without log 30). Still scored although 10a C excludes their data: log 30's last dive (window to 3612.16 s) and the 2023-11-11 and log 16 dives, which end in the last 1 s of their recording; without these three, corrected 11 of 34 and elevator only 21 of 34 (a recount). The log 25 dive at 4109 s flew on a depleted pack (10a A4) | **fail as scored**; the cause is unresolved: the throttle offset alone changes little, while the AoA and aileron offsets together make the gain much worse; the dive convention for those two offsets, thrust and drag remain open (see "What the scores say") |
| 14 Time history | only endpoints measured: pitch at the dive end model - real +6.7 deg median (absolute 11.6); dive height loss model/real 0.82 against IMU+GPS height, 1.08 with the elevator offset only (v3 against EKF height: 0.86 and 1.14). This includes the four dives that end the flight (row 15) and the three that run into excluded data (row 13); without either set it is 0.83 | not fully measured |
| 15 Pull-out | peak rate: 27 of 37 within +-20 % (median +1 %); height against IMU+GPS: 14 of 37 within 25 % or 5 m (raw 3, elevator offset only 3, throttle offset only 3; v3 against EKF height: 16, 3, 5, 3). Four scored dives do not reach a lowest point in flight: in three the recording stops inside the 10 s window (2023-11-11, log 16, `peron_152_20240831_00000096`), and in log 30's last dive the aircraft hits the ground (3615.5 s). Their "pull-out" is only the height lost before that; without them: height 12 of 33, rate 25 of 33 | mixed; height fails |
| 16 Wind input | the bridge applies one constant horizontal wind (`--wind-speed-m-s`, `--wind-from-deg`) 5 s after boot; the 3 existing wind tests pass (direction convention, plant velocity); not run at 12 m/s; no gust or turbulence exists | steady wind: partly checked; gust: not available |

What the scores say:
- The gap that matters for the final approach is row 13, high-throttle dives. The model is slow there: log 30 (logged throttle 0.91-0.97) gains about 20 m/s in 5 s where the aircraft gains about 30,
  and peaks at 50-56 m/s where the aircraft reaches 58-68. At low throttle the model matches (151, logged throttle 0.08-0.37, median 0.14: gain -4 %, peak -5 %).
- What drives it is only partly known. Evidence for throttle: within the same pitch band, dives below 0.5 applied throttle have gain -4 % (steep, n 10) and +17 % (shallow, n 3), dives at 0.85 and
  up have -33 % (n 7) and -29 % (n 5). Against a clean conclusion: the high-throttle dives come from few logs (25, 30, "160" 11), and the correction itself matters:
  the throttle offset of the log (+0.07 to +0.08 on logs 16 and 25, -0.17 on "160" log 30) was measured at 20-31 m/s EAS and should not carry to full throttle. Applying only the elevator offset (and dropping the throttle, aileron and AoA offsets together) improves 12 to 22 of 37.
  No run dropped the throttle offset alone. The runs that apply only some of the offsets point away from it: the throttle offset alone moves the median gain from -21 % (raw) to -22 %, while the AoA and
  aileron offsets together take it to -53 % (`score_dives2.txt`). So the cause is open: the dive convention for the AoA and aileron offsets, thrust or drag at 40-70 m/s, or a mix. Step 5 has to separate them (see steps).
- Row 10 (the model needs more throttle in a turn; not settled until the outside air temperature is known, 10a A1) and row 11 for 151 are the next gaps. The roll and pitch response (rows 1-4) passes except pitch gain, which fails narrowly (0.94 against 0.95); all four rows are in-sample.
- The pull-out height depends on the combined trim correction (corrected 14 of 37 inside, raw 3, one offset alone 3), so it measures rigging offsets as much as the model; treat it as soft.
  With the elevator offset only (the step-5 dive convention) the stored model values lose a median 299 m in the 10 s window where the aircraft loses 32 m (section 10a, A2).
- Not verified here: rows 1-6 on logs the fit did not see (only `peron_157_20241005_00000100`, already used once), gusts, and the replay machinery beyond speed summaries (the 151 dive speeds agree to -4 % gain and -5 % peak; mapping, timing and full histories are not independently checked).

**Step 5, first part: row 13 on temporary model copies (2026-10-16, owner approved the test).** In these runs only the elevator offset is applied. The throttle, aileron and AoA offsets are dropped in every dive. The owner accepted dropping the throttle offset in full-throttle dives ("Cruise throttle offset is not applied
to full-throttle dives"). Dropping it in the other dives, and dropping the aileron and AoA offsets, has not yet been put to the owner (decision 6). Same 37 dives. The `Peron15_hitl4` folder is unchanged (sha256 c7f6cea3...); each variant is a temporary copy.
Output: `validation\2026-10-16-layer1\score_variants.txt`.

| Variant (temporary copy) | Dives with gain and peak both inside | Gain, median, by logged throttle: < 0.5 / 0.5-0.85 / >= 0.85 |
|---|---|---|
| none (hitl4, elevator offset only) | 22 of 37 | -1 % / 0 % / -16 % |
| propeller thrust coefficient x1.25 | 22 | -1 % / +4 % / -13 % |
| thrust curve stretched in advance ratio x1.12 | 18 | +4 % / +7 % / -9 % |
| x1.18 | 17 | +9 % / +10 % / -5 % |
| x1.25 | 14 | +13 % / +13 % / -2 % |
| drag coefficient x0.85 at -4 to 7 deg alpha | 19 | +3 % / +4 % / -13 % |

- No single thrust or drag change beats the unchanged model overall: every change that closes the high-throttle gap opens a gap at low and middle throttle. So these two terms are not the whole answer.
- The high-throttle gap is concentrated in one log: gain error -25 % median on `gdrive_20250927_00000030` (10 dives, speeds up to 68 m/s, the fastest in the set), -10 % on `gdrive_20250816_00000025` (3), -4 % on `peron_160_00000011` (1).
  That looks like a condition of that flight (SYSID 158 on the Pixhawk 6X board, 2025-09-27: battery, ESC, propeller, or the pitot above 60 m/s) more than a model-wide fault, but the data cannot tell which.
- The next check (below) found where the gap sits: log 30 draws 1.7 times the power of log 25 in matched full-throttle descents and the same power in level cruise.

**Row 13 force check (read only, 2026-10-16; corrected 2026-10-17).** The check covers five dive logs. At states taken from the log (the replay speed, the real attitude and controls at 1.5, 2.5 and 3.5 s into a dive; the model run 0.3 s)
the model's along-path acceleration was within about 1 N (7.5 kg) of the log, as run, in four of the five: 151 (-0.5 N), 158 log 15 (-0.8), 158 log 25 (+1.1), "160" log 11 (-0.8).
The log 25 samples include three throttle-off points and its depleted-pack dive (4111-4113 s, 15.8-16.2 V), which list C excludes for propulsion; without them its median is +1.2 N.
Log 30 (`gdrive_20250927_00000030`) had a median real excess of 14 N per sample (median accelerations: real 7.2 m/s2, model 5.6, at 47 m/s); 13.9 N without its last dive.
- Start speed (section 10a, A1): the model starts at the replay TAS, which in logs 30 and 25 is 0.9-4.2 m/s (median 2.4) below the EKF speed that the real acceleration uses.
  Started at the EKF speed, the estimated deficit is about 20 N in log 30 and about 5 N in log 25, so "within 1 N" probably fails for log 25 as well. These are estimates from the model's own speed slope, not reruns.
- Power (log facts): in matched full-throttle descents (35-45 m/s air-relative, 24.5-27 V), log 30 drew 2.04 kW (79 A, 7 dives). Log 25 drew 1.20 kW (47 A, 3 dives) on the same SYSID 158 board with the same INA monitor settings.
  "160" log 11 (SYSID 157, analog monitor) drew 1.60 kW (65 A, 1 dive, 24.6 V). In level cruise logs 30 and 25 drew the same power (425 and 422 W).
  The v3 figures (1850 W against 941 W) were medians of mixed samples: log 25's included throttle-off and depleted-pack samples, log 30's partial throttle and the terminal dive.
- Net force (log facts): under the same rule, (T-D)/m is +1.75 m/s2 in log 30 (EKF; +1.90 from GPS, +2.06 from the IMU), against -0.21 to +0.12 in log 25, "160" log 11, `peron_160_00000030` and `peron_157_20241102_00000006`.
  So log 30 is the outlier in net force. In power it stands out only against log 25, which is the low one: the three SYSID 157 logs draw 1.6-2.2 kW.
- Inference, not a log fact (mass assumed 7.5 kg): with the throttle-off drag of section 10a A4, the implied propulsive efficiency is 0.76-0.78 in the two SYSID 158 INA logs and 0.49-0.56 in three SYSID 157 analog logs.
  The logs cannot separate the causes: an INA current that reads low, a different mass, drag or propulsion set.
- So log 30's extra net forward force comes with more power draw than log 25 on the same board with the same INA monitor settings. It points to a condition of that flight: more thrust (battery, ESC, propeller or motor) or less drag.
  An EKF wind error is less likely, because the GPS and IMU variants agree, but it is not ruled out (with a pitot speed gate the difference falls to +1.35 m/s2). The logs cannot tell which.
  This does not clear the model: started at the EKF speed, log 25 may also have a deficit of about 5 N (an estimate, not a rerun), and under the elevator-only convention
  the stored model values lose far more height in the pull-out than the aircraft (section 10a, A2; end values only, no traces).

Without log 30, row 13 with the elevator offset only is 19 of 27 dives inside (gain -2 %, peak +1 %; the 4 high-throttle dives are all inside). Source: `var_base.json` (the step 5 base run).
Consequence: do not fit thrust or drag to log 30. Under the card's corrected convention, row 13 still fails without log 30 (12 of 27 inside, median gain -11 %).
With the elevator offset only, log 30 holds 7 of the 15 dives outside, including all 7 high-throttle ones. With the start-speed correction, log 25 may also show a deficit of about 5 N (an estimate).

**Steps (the owner approved the bands on 2026-10-16).**
3. Done above. Prepare and score. First fix, before any replay, the acceptance logs and their exclusion rules. Log 30 can no longer be one: it was used in the hitl4 fit and in this scoring.
   Final acceptance needs logs that no fit and no scoring has used (flights still to come). Use a different dive family (151) for the machinery check. Check the bridge and replay: signs, scaling, timing, limits, units, elevon and throttle mapping, start state, wind. Then score `Peron15_hitl4` on the card with one script and write the result next to the card. No model change in this step.
   Also list which unknowns move the failing rows (mass, CG, elevon angle against PWM, static thrust, servo speed under load, `validation/2026-10-13/REPORT.md` section 5).
4. Ask the owner for the measurements that those rows depend on (at least mass and CG, static thrust) before fitting, so a wrong parameter cannot hide behind a passing row.
5. Close the biggest failing rows one change at a time, starting with row 13: first decide the dive convention for the aileron and AoA offsets (decision 2 covers the throttle offset only in full-throttle dives, but the step 5 runs drop it in every dive; decision 6 asks about the other dives and about the aileron and AoA offsets).
   Then rerun the dive replay with saved model traces, to see whether under the elevator-only convention the model fails to pull out or pulls out late
   (from stored end values it loses a median 299 m in the 10 s window where the aircraft loses 32 m; row 15 height 3 of 37; section 10a, A2).
   Then test a thrust-only and a drag-only change on a temporary copy of the model, each checked on the whole card, with rows 13-15 scored together for every variant
   (the +15 % elevon pitch power of hitl4 cut the model's elevator per g by about 13.5 % and so moved the elevator-per-g ratio from 0.96 to 1.11).
6. Closed loop, in stages: cruise, then a shallow dive, then a steep dive, each with a stated stop rule. Step 3 checked the bridge only in part: before this, fix the replay inputs found in section 10a, A1
   (start speed, pitot lag, one EKF core for all states, density), and check the mapping, timing and full histories. Fly the board and
   bridge through the legs of a real log and compare. This is the first test of the autopilot loop with the model and is needed before the final approach is called ready.

**Owner decisions needed.**
1. Target: **fleet median** with per-group spread (owner, 2026-10-16; the groups are not airframes, see step 1). Per-airframe propulsion profiles (propeller, motor) stay a later option, not part of the fleet model. The `peron_160_*` logs stay a group of their own: they come from the Durandal board (one AP_Stats history with 152 and 157), not 158; the frame is not identified (see the fingerprint paragraph); a dated flight record would settle it.
2. Step 5 starts with the high-throttle dive gap (row 13). May it test thrust and drag changes on a temporary copy of the model first (the model files stay untouched until you agree)? Do you accept that the cruise throttle offset is not applied to full-throttle dives? Owner, 2026-10-16: the test on temporary copies
   is approved, and "Cruise throttle offset is not applied to full-throttle dives" (step 5). The aileron and AoA offsets are a new question (item 6).
3. What were the steep dives for (target, mode, who flew)? Is a 50-70 m/s dive in scope for the HITL final approach? Owner, 2026-10-17: approaches at 50-70 m/s have been flown, so the range is in scope; some of the dives may have been tests of the aircraft's maximum load. Mode and pilot are still unknown.
   This scope change touches parts written for 24 m/s: the zoom and timing plan of section 4, and the 4-9 m pull-out margin of section 6. The real
   pull-outs (row 15 rule, IMU+GPS height, the 33 recovered ones) lose 9-12 m after the shallow (about -30 deg) log 30 dives that peak at 58-64 m/s, 19-36 m after the steep log 30 dives and the three short log 25 dives,
   27-65 m after the six steep dives of 151 log 22 (83 m after its shallow, pilot-flown one) and 86-114 m in log 15; the whole set spans 9-216 m. In 11 of the 13 that lose more than 70 m the rule's start falls 1.9-6.1 s before the dive ends, so part of that loss is still dive.
   Sections 4 and 6 need a recheck at 50-70 m/s.
4. Are logs for 161-163 or 153/154/156 held elsewhere? None are on this PC. What changed on 158 for the flight of 2025-09-27 (battery, propeller, ESC)? Owner, 2026-10-16: not known; log 30 stays an unexplained outlier, but some airframes may fly an APC 14x8.5 Thin Electric propeller, or a different motor. A larger propeller at the same pitch would explain more thrust and more current at high speed; this is an unverified hypothesis (no propeller type is in the logs). Check: compare static thrust or rpm per airframe, and the model's propeller table against an APC 14x8.5 table.
   2026-10-17: log 30 is the outlier in net force; in power it is high only against log 25 and lies inside the 1.6-2.2 kW of the three SYSID 157 logs (row 13 force check). Its INA current probably went over the 90 A range at times (inferred, not read from the driver; for the 19 terminal records the cause is open, section 10a A6): 83 records read 0 A in flight at full throttle
   (64 in climbs, 19 in the terminal sequence; none in the matched descents; section 10a, A4).
5. Are you willing to supply the measurements of step 4 (weigh and balance, static thrust, elevon angle at five PWM values)?
6. New from the log analysis (2026-10-17, section 10a):
   - What does the SERVO5 pass-through operate (a recovery parachute?), and on which flights was it used on purpose?
   - The airspeed sensor history of SYSID 158: which sensor flew from 2025-04-05 to 2025-05-18, and what changed on 2025-07-13 (the pressure sign flips)?
   - Frame, board, propeller and motor records per date, and the outside air temperature per flight (the replay density depends on it, A1).
   - What happened in the air at 3611.8 s in log 30 and at 2453.9 s in `gdrive_20250128_00000006`, and what was that log doing at 3659-4151 s (on or near the ground)?
   - Do the GUIDED request, dive and AUTO sequences come from an onboard script? The logs carry AAS_TERM_* parameters, and the log 30 dive end moves with AAS_TERM_PITCH. Not all requests can come from a script: in `peron_152_20240831_00000096` the own-SYSID requests came with scripting off (SCR_ENABLE 0). 26 of the 47 steep intervals with automatic demand have no logged request (B1).
   - Can you supply GCS telemetry logs (tlogs), or logs that hold MODE and MSG records, for the dive flights, and the GCS and companion setup behind the senders 255/0, own SYSID/0 and 255/190? They would show which mode was active (B1).
   - How did the flights of 2023-11-11 (SYSID 151), 2025-05-25 (log 16) and 2024-08-31 (`peron_152_20240831_00000096`) end: an impact, a net catch or a power cut? Each recording stops while armed in a fast descent. Do you agree that these three dives and the last dive of log 30 stay out of the row 15 pull-out score?
   - The step 5 runs drop the aileron, AoA and throttle trim offsets in every dive; decision 2 covers only the throttle offset in full-throttle dives. Should the aileron and AoA offsets be dropped in dives, and the throttle offset in dives below full throttle?
   - Can the SYSID 158 INA current be bench-checked against a clamp meter? It would show whether the 0.76-0.78 implied efficiency is a sensor error (A4).
   - For future flights: raise BATT_MAX_AMPS or fit a higher-range shunt, so full-throttle current is measured (A4), and set LOG_DISARMED to 1, so the end of a flight after an in-air disarm is recorded (B4).
   - What is the battery pack spec (cell count, capacity)? The cell count is not logged (A4).

## 10a. Independent log analysis (Codex, checked by Claude, 2026-10-17)

**Method.** Codex 0.163 at reasoning effort xhigh analysed the 31 BIN files with its own parsers, read only. Claude agents then re-derived 40 claim groups (C01-C38 without C30, with C07 in four parts) with pymavlink; a critic listed 28 issues,
and Codex resolved them in seven rounds (R1-R7b). Claude spot-checked the key numbers against the logs (log 30 heights and zero-current records, the log 25 autotune, the 2023-11-11 log end,
the 2025-01-28 overlap, matched power, one 151 pull-out height). Output: `%LOCALAPPDATA%\navpy-hitl-tools\validation\2026-10-17-codex-deep\` (topic folders; checks in `verify\`, resolutions in `verify\_resolve\`).
No model was run. "Log fact" means read from the logs; "inferred" and "estimate" mean it is not. This section has not had an independent review: Codex found the facts, and Claude checked and wrote them.

### A. For the model

**A1. Replay inputs (R7b).**
- The 151 logs fly with ARSPD_USE 0, so their replay airspeed is the EKF synthetic airspeed, not a pitot reading.
- In logs 10, 11 and 15 ARSPD_USE went to 0 in flight, so the replay uses synthetic airspeed. The cause is inferred: the airspeed consistency monitor (ARSPD_OPTIONS 11), since the logged health probability fell below 0.1 at that moment. No message record names the cause. In logs 10 and 11 the per-flight scale makes the replay TAS 6-9 % low.
- In log 16 the ratio changed in flight (autocal, B3): the replay TAS is +6.7 % before 877.6 s and -4.4 % after 1174.7 s.
- `gdrive_20250128_00000006` at 3659-4151 s is not flight (it holds the replay's flights 2-4: 3734.6-3833.3, 3989.4-4041.0 and 4119.3-4150.6 s): pitot 2.6-3.0 m/s at 13-21 m/s ground speed, baro height 2.6-15.9 m (inferred: moved on or near the ground).
  The row 12 window at 3783.8 s comes from it. Without it, row 12 is EAS 32 of 35 and height 28 of 35 inside (a recount). Without all five flagged row 12 windows (logs 6, 11 and 16), EAS 29 of 31 and height 25 of 31.
- Density: the replay takes density from E2T, which uses the airspeed sensor's temperature at arming (B3); this gives 0.85-1.01 of ISA at the logged pressure. If the air was at ISA, the model's dynamic pressure is 0-15 % low.
  Estimated effects: row 6 1.11 to 1.26 (out of band), row 7 elevator -22 to -27 us, row 9 the 157 model throttle slope 0.025 to 0.030 per m/s, above the 0.010-0.026 band (low confidence), row 10 +0.040 to +0.029, row 13 corrected 12 to 9 of 37. Rows 6 and 10 are not settled until the outside air temperature is known.
- EKF cores: the replay takes attitude from ATT (the active core) but velocity and wind from core 0. Their TAS agrees within 0.6 m/s (p99) except in log 15 at 964-1012 s (up to 14.6 m/s) and one `peron_157_20241102_00000006` throttle window (2.3 m/s). Take all states from one core.
- Row 11 without the flagged windows: 36 of 70 inside, median +0.01 m/s. Rows 1 and 3: correcting the TAS of logs 10, 11 and 16 moves the roll fleet median from 0.967 to 0.977; with the compass-free wind-triangle factor on every log, roll is 0.989 and pitch 0.954 (from 0.945), so row 3 may be a narrow pass (low confidence; the flagged logs give no pitch windows).
- Replay TAS lags the EKF speed by 0.38 s in log 30 and 0.22 s in log 25. The model starts each dive at the replay TAS, -2.2 to +2.2 m/s off the EKF speed per scored dive. That biases the row 13 peak by -5.1 to +4.0 %; started at the EKF speed, row 13 is corrected 14-16 and elevator only 22-23 of 37 (estimates). For the force, see the row 13 force check.

**A2. Pull-out and dive heights (R7a).** v3 measured height loss with EKF height. Against IMU height fitted to GPS (three parameters per window, GPS lag 0-0.13 s), EKF height understates the
pull-out loss by 0.8-14.5 m (median 4.4 m) and the dive loss by 1-17 m. Rows 14 and 15 above use the new heights. With the elevator offset only, the stored model values lose a median 299 m
(0-555 m) in the 10 s pull-out window, where the aircraft loses 32 m. This comes from stored end values only; rerun the replay with saved traces before using it for tuning.

**A3. Barometer and EKF height under load (C25, R5).**
- Every barometer reads falsely high in pull-outs. Log 30 BARO0: +26-34 m, with steps of 9.6-13.7 m in 0.1 s. Other logs: +13-38 m at 5.8-9.1 g, about 2-3.5 m per g.
- Log 30 against its RTK GPS (GPS1 RTK fixed in 6103 of 6132 in-flight samples): barometer and EKF height read about +19 m high in steady flight as well. The barometer error grows with dynamic pressure, about 21 m per kPa (R5; other checkers got 16-30 m per kPa, so the size depends on the method).
- In log 30 every pull-out fails the EKF height test (SH 2.0-2.7 in d01-d09, 5.0 in d10; the gate fails at SH 1) and the EKF switches core, 10 of 10. Logs 15, 25 and 151 log 22 also switch at most pull-outs (log 15 4 of 4, log 25 5 of 6, 151 log 22 5 of 7), with nearly equal SH on the two cores. In the other seven dive logs the EKF does not switch core in any of their section 10 dives. In log 30 the EKF height error peaks 1.6-2.9 s after the load peak. Across the 33 recovered pull-outs it peaks 1.8-9 s after the pull-out start; the long pull-outs of logs 15 and 25 and the "160" logs reach their lowest point only after 3-9 s.
- So barometer or EKF height is not height truth at speed or under load; use GPS, or IMU height fitted to GPS.

**A4. Propulsion and power (R4, C10, C16, C17).**
- Log 30 against log 25: 1.71 times the power in matched full-throttle descents (1.70-1.74 under other gates), the same power in level cruise; log 30 is also the outlier in net force (row 13 force check).
- The INA monitor of the SYSID 158 6X logs (BATT_MONITOR 21, BATT_MAX_AMPS 90) logs exactly 0 A at full throttle near its range. This is inferred, not read from the driver: an over-range sample counts as 0 inside a 4-sample average (the readings next to the zeros are 22.4 A and 44.8 A, which are 1/4 and 1/2 of about 89.6 A). Log 30 has 83 such records while flying at full throttle
  (64 in climbs at 2391-2800 s, 19 in the terminal sequence at 3611.85-3613.65 s); its maximum is 89.74 A. The logged charge does not count those records. Log 25 has none (maximum 78.6 A).
  For log 30 current, drop the zeros and the records within 0.5 s of them; the current behind them is only estimated.
- The analog monitor of the Durandal logs (BATT_MONITOR 4) has a ceiling of 3.3 V x 36.364 A/V = 120 A. This was checked in "160" logs 29, 30 and 35 only. In each of them one record touches the ceiling (119.97-119.99 A, an isolated spike at 17-20 m/s). Their 96-106 A medians at 25-30 m/s are not clipped.
- Throttle-off drag from 15 windows in 4 logs (SYSID 151 and 157, plus 3 windows from SYSID 149, which is another model): D = 0.0154 V^2 N at an assumed 7.5 kg (24.6 N at 40 m/s; per-log medians 24.7-28.3 N; rms 3.1 N).
  It includes the drag of the unpowered propeller (windmilling or stopped; no RPM is logged); 13 of the 15 windows are in dives (-25 to -60 deg path angle), and 11 end 0.1-4.3 s before a 4-9 g pull-out; 4 reach 1.3-1.6 g; nothing above 49 m/s.
- Implied propulsive efficiency 0.76-0.78 (SYSID 158, INA) against 0.49-0.56 (SYSID 157, analog); the cause is not separable (row 13 force check).
- In the `peron_160_*` logs the current at 0.7-0.8 throttle and 25-30 m/s doubles between "160" log 22 and "160" log 29 (50-56 A in "160" logs 18, 21, 22; 101-105 A in "160" logs 29, 30, 35) with the same monitor settings. At full throttle the current is similar in both groups (97-110 A; the later logs draw about 8 % more at matched voltage), so the change is mainly at part throttle. The cause is not logged.
- Log 25's pack is depleted after about 4040 s (15.6-16.3 V in the last dive). Charge used exceeds the configured capacity in 5 logs. The voltages fit an 8S pack (the cell count is not logged).

**A5. Controls (R6).**
- Elevon mixing uses MIXING_GAIN 0.5, and the PWM spans either side of the trim differ. Put the saturation at the pre-mixer +-4500 limit: pre-mixer rails are about 27 times more common than PWM endpoints.
- Where SERVO_AUTO_TRIM is on (19 logs throughout, 4 more for part of the log), the saved trims are controller state, not rigging: auto-trim keeps them within 40-60 % of the servo range (1400-1600 us for 1000-2000 limits). In the 8 Durandal logs with auto-trim off throughout, the trims never change. Drive the model surfaces from RCOU in all logs.
- 12 gain sets across the logs, and autotune changes gains in flight. In log 25 the last 13 s flew with roll P 0.045 instead of 0.130 (an autotune value that was not saved). Use the gains in force at each time.
- Demand-to-rate lag is about 0.10-0.12 s (median over logs; per log 0.07-0.28 s in roll and 0.02-0.19 s in pitch). Roll is close to one delay of about 0.10 s. In pitch the delay rises with frequency, from about 0.03 s below 0.6 Hz to about 0.16 s at 1.5-2.5 Hz. It is a closed-loop number: compare it with SITL or HITL on the same gains.
- Elevator trim gradient: about 62 us per unit of n (24/V)^2.
- Simple regressions give no usable damping or control power from these logs.
- The roll oscillation of about 5-6.5 Hz (a broad peak, not one line) in 151 on 2023-11-18 belongs to that session only.

**A6. Loads.**
- Log 30: 9 recovered pull-outs at 11.2-12.5 g (raw -AccZ, IMU with a 20 Hz filter; 80 ms running mean 11.1-12.3 g; Codex's 25 Hz grid metric, a mean of 5 samples over 80 ms, 10.9-12.1 g). Log 25: 7.5-9.7 g. Peak load per unit dynamic pressure is similar in log 30, log 25 and 151 log 22, about 5-6 g per kPa (log medians 5.3-5.8). So speed explains most of log 30's higher loads (inferred; mass, CG and CL are unknown).
- The last dive of log 30: normal load peaks at 14.1 g at 3611.954 s; in the air the IMU shows axial decelerations of -12.35 g (3611.939 s) and -18.31 g (3612.384 s); ground contact is at 3615.459 s (24.6 g).
  Both in-air events (this one and `gdrive_20250128_00000006` at 2453.91 s, 10.9 g in level flight) show the current reading 0 A while the voltage drops. In log 30 it drops by up to 7.4 V for 1.8 s (3611.85-3613.65 s). In `gdrive_20250128_00000006` it drops by 6-9 V for about 8 s, from 2454.77 s, just after the shock, through the ground hits to 2462.6 s. In both logs the 0 A may also come from the INA monitor going over its range (A4): `gdrive_20250128_00000006` has the same 90 A INA monitor (BATT_MONITOR 21, BATT_MAX_AMPS 90). The cause is not logged.

### B. Identity and configuration

**B1. Modes (R1, C01, C32, C34, C36).**
- There are 0 MODE records. AUXF has 199 mode-function records; of the 4,191 MAVC records, 231 are DO_SET_MODE requests in 15 logs (227 accepted, 4 failed; 92 after merging repeats within 2 s). The flight-mode slot reads FBWA in 90.9 % of flight, but TECS runs in 77.2 % of that time, so the slot does not give the mode.
- Mode classes from TECS, ATRP and the demand records: automatic throttle 70.9 %, FBWA-like 20.3 %, AUTOTUNE 8.5 %, MANUAL 0.3 %. A replay of the mode events agrees with them in 99.15 % of the airborne samples it can label. The other 15.7 % of airborne time comes before the first logged event in its log: it has a class but no mode name.
- The one MANUAL segment: `peron_158_20241102_00000002` at 4687.13-4767.63 s; roll -105.5 to +38 deg, roll rate up to 213 deg/s; between 4697 and 4757 s the EKF often rejected GPS velocity (about 400 of 2,018 records per core), and at 4710.6-4717.8 s also GPS position (innovation up to 151.5 m). Do not use EKF velocity, position, AOA/SSA or the wind triangle there.
  It can serve roll-axis identification from IMU, attitude, pitot and the elevon outputs (AETR) only.
- Steep-pitch intervals (C32: pitch below -30 deg for at least 2 s, armed, GPS speed above 12 m/s): 49 intervals in 14 logs, 47 manoeuvres. This is a wider set than the 38 dives of section 10. 2 are flown by the pilot with large stick input (`gdrive_151_00000022` at 1489.8-1494.7 s, `peron_157_20241102_00000006` at 761.4-766.8 s). In the other 47 intervals the sticks are centred or nearly so (36 fully centred; stick activity in under 20 % of samples) and the pitch demand follows the dive, so it came from an automatic source. Of these 47, 21 follow an accepted GUIDED request 4-50 s before and 26 have no logged request. Within 3 s after the interval, nose-up pitch stick of 200 us or more appears in 16 of the 47 (STICK_MIXING 1): use the recorded RC.
- RC overrides in 6 logs, including 747 s of machine square waves during autotune; an RC failsafe in `peron_157_20241005_00000100` (7.3 s); fence errors in 4 logs. In the 2026 log the mission is reset to item 1 from outside 17 times (16 times from item 4, LOITER_UNLIM, which never ends by itself, and once from item 2); what sent the resets is not logged.

**B2. Boards.** See "Which boards and settings are alike" in step 1: three board models; one Durandal board carries SYSID 152, 157 and all `peron_160_*` files in one AP_Stats history;
SYSID 151 is two APJ 50 boards; SYSID 158 is very probably board P in 2024 and a Pixhawk 6X in 2025-26. Values copied by parameter file do not identify a board.

**B3. Airspeed sensor (R3, C07a-c, C37).**
- k is the fitted pitot coefficient: the ARSPD_RATIO that would make the pitot read the GPS air-relative speed (GPS wind triangle, no compass, no EKF wind). It is not the logged ARSPD_RATIO. Each k is given with the logged E2T and at ISA. k scales as 1/T (about 3.6 % per 10 K), and the outside air temperature is not logged, so neither value is the true coefficient.
- SYSID 158 has four epochs: (1) to 2025-01-28, MS4525 driver (TYPE 1, logged ARSPD_RATIO 1.8087): k 1.89-1.98 with the logged E2T, 1.95-2.03 at ISA; (2) 2025-04-05 to 2025-05-18, the pressure gain falls 2.78-2.86 times, which fits a +-10 inH2O sensor read on the MS4525 scale
  (inferred), so do not use logs 10, 11 and 15 at the logged scale; (3) log 16, autocal moves the ratio from 1.99 to 1.60 in flight; that moved it toward the truth only if the outside air was warmer than about 21 C; (4) from 2025-07-13, RawPress has the opposite sign: k 1.70-1.80 with the logged E2T, 1.70-2.10 at ISA.
- SYSID 157 (9 logs, TYPE 1): k 1.63-2.12 with the logged E2T, 1.85-2.26 at ISA, with no logged configuration change.
- Log 30 DiffPress reaches 2456 Pa, 99 % of the 2491 Pa full scale of a +-10 inH2O sensor, which the TYPE 9 / PSI_RANGE 0.36 setting implies (the part is not logged). No clipping is seen.
- E2T is not the outside air. In ArduPlane 4.3-4.5 it is built from GndTemp, the airspeed sensor's temperature at arming, held fixed for the flight (4.5-57 C). In log 32 (4.6.3) it comes from ISA.

**B4. Flight ends (R2).**
- 32 flights: 9 normal landings; 13 slow descents after the SERVO5 pass-through output (functions 59, 60, 62) went high, which fits a recovery parachute (inferred; CHUTE_ENABLED is 0 in all logs);
  5 with pulses but no slow descent; 3 recordings that stop while armed in a fast descent; 2 in-air events followed by ground impact.
- Logging stops about 15 s after an in-air disarm (LOG_DISARMED 0): 14.96-15.00 s after 14 of the 15 in-air disarms, and 17.9 s in `peron_158_20241102_00000002`, where a switch arm attempt restarted the timer. So the touchdown is missing in 10 of the 13 slow descents. It is recorded in `gdrive_20250713_00000021` (never disarmed), `peron_158_20241102_00000002` (disarmed at 14.1 m) and `peron_158_20241102_00000003` (re-armed in the air).
- The barometer re-zeroes every 5-6 s after an in-air disarm (HOME_RESET_ALT 0): use GPS height there.

**B5. Archive.** File names do not always match the SYSID in the log (the `peron_152_20231120_*` files are SYSID 151 and were recorded on 2023-11-18 by GPS time; file 04 holds no flight). The text `.log` copies match their BIN files record by record, except the first UNIT record per file, and hold floats to 7 significant digits (step 1).
Gaps inside a file come from the disarm logic (LOG_DISARMED 0), not from records the logger dropped: the dropped-record counter (DSF.Dp) does not change across any gap. Inside a gap no sensor or state data is recorded (B4).

### C. Data not to trust (exclusion list)

- From the first rise of the pass-through output above 1700 us (or the GCS servo command) to the end of that flight: 21 events in 20 flights.
- Everything after an in-air disarm, and the in-air re-arm of `peron_158_20241102_00000003` at 2183.77-2203.07 s.
- Log 30: everything from 3611.0 s; its last dive (d10) in force fits, from its commanded dive start at 3605.67 s; its barometer and EKF height as height truth at any time (A3).
- `gdrive_20250128_00000006` from 2453.91 s to the end of the file: the in-air event and the impact, then 3659-4151 s on or near the ground (inferred; the replay's flights 2-4, A1), which includes the low pitot readings at 3661.8-3668.8 s.
- `peron_160_00000018` from about 2374 s; `gdrive_151_2023-12-24_13-46-44` at 2870-2872.6 s (launch hop).
- The last 1 s of the three recordings that stop while armed (`gdrive_2023_11_11_2023-11-11_17-06-18`, `gdrive_20250525_00000016`, `peron_152_20240831_00000096`).
- Raw ARSP in logs 10, 11 and 15, and in log 16 up to the end of its autocal (1175.6 s). Use log 16 only with a per-window k (B3).
- Log 25 after 4040 s for propulsion.
- Log 30 current at the 90 A range (A4).
- Barometer and EKF height as height truth at speed or under load (A3). In the log 30 pull-outs (d01-d09) the barometer error lasts to about 3 s and the EKF height error to about 8 s after the load peak. Across the 33 recovered pull-outs the EKF height error peaks 1.8-9 s after the pull-out start, so in long pull-outs 8 s may not be enough (inferred).
- RC override periods and RC failsafe periods as pilot input (B1). During a failsafe RCIN does not show the pilot's sticks: roll, pitch and yaw show the firmware's RC trim values, and throttle keeps its last value: `peron_157_20241005_00000100` at 1012.06-1019.34 s, and very probably `peron_152_20231224_00000087` at 863.65-867.17 s.
- Periods that STAT marks as flying on the ground: log 25 at 4484-4854 s, and all 16 STAT "flying" records of `gdrive_155_2024-09-14_12-10-49` (a ground movement at about 2 m/s, 1535.6-1536.8 s, and the failed launch at 1953.2-1954.8 s). Gate "flying" with airspeed or GPS speed as well as STAT.
- Accelerometer samples within +-0.25 s of a clip-count increase.

## Owner answers (2026-10-15)
1. Board: keep 2.4 x 3.2 m. → 4, 6.
2. Link: share the board's USB through a PC router. → 2.
3. Pass mark: 10 of 10. → 7.
4. Camera: first a fixed nose camera (v4), then the Novoxy SK-AHD2200 (10x, 5-50 mm) and a bigger 18x mentioned, then
   "Let's do gimbal instead": the approach uses the simulated ZR10 gimbal. Novoxy and 18x stay as profiles for later. → 4.
   Later the same day: "Skip novoxy, we should concentrate on zr10": the Novoxy profile is dropped. → 4.
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

## Codex review of the router code (2026-10-15): rounds 1-4 "rework", round 5 "approve"
Fixed: packets read before a reboot, barrier or reopen reaching the board after it (receipt stamp, then the whole
waiting backlog dropped); partial packets (a datagram carries whole packets); a failed open keeping the handle; double
counted barrier drops; stale or link-down packets changing stream claims; NaN intervals stopping a receiver; a client
named `router`; reopen of an open endpoint. A regression test for each, each failing without its fix.

## Codex review of the terminal handoff (2026-10-15): rounds 1-2 "rework", round 3 "approve"
Fixed: a raising runner handoff let HeightFloor send RTL through the router while navpy's endpoint was not known
closed (now no router RTL; a step error before the confirmed close takes the no-ack path); a failure while ending the
router left a cached record without the direct-RTL mark. A regression test for each, each failing without its fix.
Open then: the gimbal command fence, built with B3 (2).

## Codex review of B3 and the gimbal command fence (2026-10-15): rounds 1-2 "rework", round 3 "approve"
Fixed: the gimbal stop could hold the terminal record (now bounded); two sim scripts lacked the new
`use_sim_gimbal` argument (an AST test checks every caller); only the last SIYI SDK got the stop, and one stuck
endpoint blocked another's stop (now one send lock and one stop thread per endpoint, one deadline). A regression
test for each, each failing without its fix.

## Codex review of the real ZR10 behaviour (2026-10-16): round 1 "rework", round 2 "approve"
Fixed: the bridge sent the vehicle_state copy and Unreal's pose on one UDP socket, so on Windows a closed emulator
port could surface as a reset on the next send and drop Unreal's pose (now one socket per destination). A regression
test models the delayed reset and fails on a shared socket. Static review; Codex could not run the tests.
