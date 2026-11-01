# GCS preflight: camera rows and companion wording

Status: v4, approved by the owner 2026-10-25 (all ten recommendations);
built 2026-10-26, see "As built" for where the code differs from this text.
Todos 70 and 72 part (a).
Codex reviewed v1 to v3 (changes needed each time, no blocker after v1);
v4 answers each finding (end of this note). Mock: `gcs_preflight_camera_row_mock.html`.

## Problem (origin/main 3c3034ba)

- Preflight rows are prearm, companion, mission, GPS, throttle, battery, EKF,
  RC and sensors (`PreflightReadiness.jsx:12`). No camera row.
- The GCS learns about cameras only through the MAVLink camera protocol.
  `video_streams` lists streams only after discovery finished
  (`video_camera_discovery.py:105-110`). "Camera seen", "discovery failed"
  (`video_streams.py:197-205`) and "URI rejected" (`:166-176`) are only logged.
- NavPy makes a camera component only for a mount whose profile device has
  `video_streams` (`camera_stream_composition.py:15-35`,
  `camera_stream_publisher.py:77-79`), and its HEARTBEAT is always
  `MAV_STATE_ACTIVE` (`camera_stream_messages.py:77-84`). No profile in
  `vision_profiles.json` has streams, so a real UAV announces no camera, and
  nothing says whether the detector gets frames.
- A lost camera is deleted (`video_streams.py:128-137`), and a NavPy restart
  shorter than `CAMERA_LOST_AFTER_S` keeps the old discovery generation
  (`:88-94`), so changed streams are not rediscovered.
- Two UAVs announcing the same RTSP host and path are not caught.
- The companion row "Active" (`en.json:1057`) and the CC badge tooltip
  "Companion active" (`en.json:681`, `VehicleStatusCard.jsx:29-33`) only mean
  that NavPy's 1 Hz heartbeat (`vehicle_mav.py:45`) arrived in the last 5 s
  (`vehicle_manager.py:233-249`, `vehicle_status.py:40-41`). They read like
  "approach running".

## Rows

Two rows after Companion (option A, recommended):

- **Detector camera**: the camera NavPy's detector reads. Does NavPy get
  frames?
- **Video stream**: the operator display stream (vehicle window). Is it
  advertised by this UAV's companion, and is it unique in the fleet?

Option B: one Camera row with both lines in each cell. The cell can show only
one colour, the worse of the two, so a caution hides which part it is about.

Both rows are caution at worst in v1. They never read NO-GO and never change
LAUNCH (`control_readiness.py` untouched). They join the GO/CAUTION header,
the action strip and the sidebar readiness button through
`computeVehicleReadiness`.

### Detector camera states

| Text | Status | When |
|---|---|---|
| Ready | GO | NavPy camera, HEARTBEAT `ACTIVE` |
| Simulated | GO | NavPy sim camera, `ACTIVE` |
| No frames | CAUTION | NavPy camera, `CRITICAL`: no frame within `DETECTOR_FRAME_FRESH_S` |
| Lost | CAUTION | seen, then no HEARTBEAT for `CAMERA_LOST_AFTER_S` |
| Not advertised | CAUTION | companion up for `CAMERA_ADVERTISE_GRACE_S`, no camera ever seen |
| Checking… | grey | no camera yet within the grace, or its `CAMERA_INFORMATION` not yet in |
| Not reported | grey | not a NavPy camera, or a NavPy camera with another status |
| — | grey | companion down (its own row says so) or link down |

"NavPy camera" and "NavPy sim camera" come from `CAMERA_INFORMATION`
`vendor_name` ("NavPy", "NavPy sim"). Only those cameras' `system_status`
means detector readiness: older NavPy sends `ACTIVE` with an empty vendor
whatever its detector does (`camera_stream_messages.py:77-84`, `:126`).

Each camera takes the first state that applies:

1. lost: Lost;
2. no `CAMERA_INFORMATION` yet: Not reported if its discovery failed, else
   Checking…;
3. vendor not NavPy's: Not reported;
4. NavPy's: `ACTIVE` gives Ready or Simulated, `CRITICAL` No frames, any
   other status Not reported.

The row: — while the companion is not "ok" or the link is down. With no
camera: Checking… within the grace, Not advertised after it. With cameras,
the worst one wins, ranked Lost and No frames, then Checking…, then Not
reported, then Ready and Simulated; the text is the worst camera's, with
"(1 of 2)" when more than one camera is in the list.

### Video stream states

| Text | Status | When |
|---|---|---|
| 1 stream / 2 streams | GO | discovery done, every stream valid and unique |
| Lost | CAUTION | a camera that had a stream is lost |
| None configured | grey | every camera's `CAMERA_INFORMATION` says no stream, none expected |
| Checking… | grey | discovery running (it ends done or failed on its own), or expected and the grace not over |
| Discovery failed | CAUTION | retries exhausted |
| URI rejected: *reason* | CAUTION | a stream failed `validate_rtsp_uri` (reason as logged) |
| Same stream as UAV *n* | CAUTION | another UAV announces the same RTSP host and path |
| Not advertised | CAUTION | expected, no stream after the grace and no discovery running |
| — | grey | no camera seen and none expected (the detector row says why) |

The row takes the first that applies:

1. companion not "ok" or link down: —;
2. a stream of a not-lost camera has another UAV's host and path: Same
   stream as UAV *n*;
3. a not-lost camera rejected a stream: URI rejected;
4. a not-lost camera's discovery failed: Discovery failed;
5. a lost camera had a stream (`has_video` true): Lost;
6. a not-lost camera's discovery is running: Checking…;
7. not-lost cameras have valid streams: *n* streams;
8. expected: Checking… within the grace, Not advertised after it;
9. cameras seen, all with `has_video` false: None configured;
10. else —.

A lost camera's streams leave the count and the duplicate check. "Expected" means the sim test source is on for this
sys_id (`sim_video_companion.sim_video_env`). On a real UAV the companion
announces its own profile's cameras, so the GCS does not read the fleet-wide
`camera.vision_profile` (a real UAV's profile may differ from it).

What the stream checks prove: the stream was announced by this UAV's own
companion (`video_streams.py:160-164`) and no other UAV announces the same
host and path. They do not prove which aircraft's camera the pictures come
from: two companions announcing swapped URIs pass both checks. That needs a
UAV name in the Jetson overlay (later, decision 4). The GO text says only
"1 stream", never "verified".

Hover hints (`preflight.fix.*`), and the cell text carries the reason, so the
row reads right without the hover: No frames: "NavPy gets no detector frames:
check the camera cable, power and NavPy log". Lost: "The camera stopped
announcing itself: check NavPy's log". Not advertised (detector): "NavPy
announces no camera: check its vision profile and log". Discovery failed:
"Stream discovery failed: stop NavPy for more than 5 s, then start it" (works
for older NavPy too; see Backend). URI rejected: "Fix the
stream URI or video.allowed_networks". Same stream: "Two UAVs announce one
stream: give each UAV its own stream path". Not advertised (stream): "Test
source is on but this UAV announces no stream: restart its NavPy". Lost
(stream): "The stream camera stopped announcing itself: check NavPy's log".

## Companion wording (72 a)

Row: "Connected" / "Verifying…" / "Disconnected". The row label and a GO cell
get the tooltip "NavPy heartbeat in the last 5 s; not mission or approach
state". CC badge tooltip (locale only): "Companion connected: NavPy heartbeat
in the last 5 s; not mission or approach state". Verifying and Disconnected
keep their wording.

## Mechanism

Display only. Nothing on the navigation command path reads any of it (Pure
Vision Approach Constraint).

### NavPy

- Every camera mount becomes a camera component (decision 1 c), not only
  mounts with streams. `CAMERA_INFORMATION` sets `HAS_VIDEO_STREAM` only for a
  mount with streams; a stream request to one without gets DENIED
  (`_requested_streams` finds none). `vendor_name` is "NavPy", or "NavPy sim"
  for the sim detector.
- Camera HEARTBEAT `system_status` (decision 3 a): `ACTIVE` when the mount's
  detector reports ready, else `CRITICAL`. Each detector factory gives its
  mount a narrow health port that the publisher reads at each heartbeat:
  - real: ready only when the capture is running, the newest publication
    holds a frame (`FrameSnapshot.frame` not None; publishing None also stamps
    a time, `frame_publication.py:52-75`), that publication came after the
    current capture start (its `sequence` is above the one recorded at start:
    a stop does not clear the slot, `frame_publication.py:46-50`, so a quick
    restart would otherwise reuse the old frame), and
    `rt.time() - published_at_s <= DETECTOR_FRAME_FRESH_S` on the
    publication's own clock (`rt.time()`, not the heartbeat's `rt.monotonic()`).
    Otherwise `CRITICAL`;
  - sim: always ready.
- Camera HEARTBEAT `custom_mode` carries NavPy's boot id: a random
  per-process id drawn once at start, never 0 (0 means older NavPy). `custom_mode` has no meaning
  for a camera, nothing reads it from a non-autopilot component
  (`inbound_router.py:122-132`, GCS backend), and it adds no bytes.
  NavPy's swarm heartbeat also carries the boot id, but only from DETECT on
  (`navigation_transition_handler.py:57-59`), so it is absent preflight.
- Wiring: `FramePublicationStore.set_running` records the newest publication
  sequence at each start and answers the ready question
  (`has_fresh_frame`); `FrameProvider` applies the threshold.
  `VisionDetectorNode` gets a `camera_health` port
  (`navpy/core/ports/camera_health.py`: the vendor and a ready reading), set
  by each detector factory (real: "NavPy", through the detector's
  diagnostics, which already hold the frame provider,
  `real_detector_diagnostics.py:39-50`; sim: "NavPy sim", always ready).
  `vision_controller_composition.py` hands the ports to `streaming_cameras`;
  the camera publisher draws the boot id once per process.
- `DETECTOR_FRAME_FRESH_S` = 1.0 s, provisional: at the ZR10's 25-30 fps it is
  25 or more frame periods, and a stall shows within 2 s at the 1 Hz
  heartbeat. Nominal fps does not give the publication jitter: the Orin and
  ZR10 bench check confirms or changes it (follow-up todo).

### Backend

- Boot rules, per camera record. The boot id in a camera HEARTBEAT is a
  NavPy convention (MAVLink gives a camera's `custom_mode` no meaning), and
  random ids have no order, so "newer" cannot be read from the value:
  1. each camera component has a lasting record (the boot ids it has sent,
     whether it has answered as NavPy's, lost or not) and a discovery
     generation (phase, information, streams, requests) that is replaced on
     a restart or a return from Lost;
  2. a component that has once answered as NavPy's stays NavPy-qualified
     across generations (a later answer with another vendor replaces it).
     For it the boot id is checked before anything else: a HEARTBEAT with
     an id it has sent before is dropped whole (no liveness, status or
     request update), a late packet of an old process; a non-zero id new to
     it means NavPy restarted: a new generation starts with that id, even
     within `CAMERA_LOST_AFTER_S`;
  3. a component not yet qualified, or another vendor's, ignores
     `custom_mode` (today's rule). One that qualifies after a restart
     catches up on its next HEARTBEAT (rule 2), at the cost of one more
     information request;
  4. boot id 0 (older NavPy) keeps today's rule: rediscovery only after
     `CAMERA_LOST_AFTER_S` of silence. The "stop for more than 5 s" hint
     works for both;
  5. a record is never dropped: a lost camera stays Lost until an admitted
     HEARTBEAT starts a new generation, for the life of the vehicle's entry
     in the GCS. While Lost it takes no discovery replies
     (`CAMERA_INFORMATION`, `VIDEO_STREAM_INFORMATION`, `COMMAND_ACK`) and
     sends no requests: deletion did that before
     (`video_streams.py:185-195`). So a camera that does not come back after
     a NavPy restart stays a caution; one removed from the profile on
     purpose stays Lost until the GCS restarts.

  Limit: `CAMERA_INFORMATION` and stream replies carry no boot id, so a reply
  of the old process still in flight right after a restart can be taken as
  the new process's answer. Both processes read the same profile, so the
  answer is the same unless the profile changed in between.
- A lost camera stays as "lost" instead of being deleted (rule 5), so
  losing one of two cameras cannot turn the row back to GO. Seen again, it
  starts a new discovery generation as today.
- `CameraDiscovery` also keeps the last HEARTBEAT's `system_status`, the
  vendor and `HAS_VIDEO_STREAM` from `CAMERA_INFORMATION` (null until it
  arrives), and each rejected stream id with its reason.
- New snapshot keys (`video_streams` unchanged):
  - `cameras`: per camera component `{component, lost, vendor:
    "navpy"|"navpy_sim"|"other"|null, status: "active"|"critical"|"other",
    discovery: "running"|"done"|"failed", has_video: bool|null, rejected:
    [{stream_id, reason}]}`;
  - `camera_wait_over`: `CAMERA_ADVERTISE_GRACE_S` = 15 s have passed since
    the companion came up (companion status left "down"). It applies only
    while no camera is recorded; after a NavPy restart the old records stay
    and turn Lost if their cameras do not come back. It is a policy for the first advertisement, not a bound on
    discovery: the start of the publisher after NavPy's own heartbeat
    (vision composition, then mount start, `vision_component_lifecycle.py:83-99`)
    has no bound in code. A running discovery shows "Checking…" whatever the
    grace says; discovery ends done or failed by its own retry rule. The live
    run measures the companion-to-camera delay in SITL; the bench check
    measures it on the Orin. Timed in the backend because the telemetry loop
    broadcasts only changes (`telemetry_loop.py:30`): a bool that flips once
    reaches the browser, a ticking age would not, and the browser clock is
    not the backend's;
  - `video_stream_expected`: sim test source on for this sys_id;
  - `video_stream_duplicates`: `[{key, sys_ids}]`, set in
    `VehicleManager.get_all_snapshots` (decision 4) by grouping every
    vehicle's valid streams by their WHEP URL, which the backend builds from
    the stream's host and path alone (`video_streams.py:171`), so one URL is
    one host and path. Only other UAVs count.
- The boot rules, the wait clock and the duplicate grouping live in new
  small modules, so
  `vehicle_manager.py` stays under its size limit.
- `_has_changed` gets the four keys.

### Frontend

`computeDetectorCameraCheck` and `computeVideoStreamCheck` in
`preflightReadiness.js`, added to `computeVehicleReadiness` (signature kept);
two grid rows, cell text, hints and the action-strip priority (after the
launch blockers) in `PreflightReadiness.jsx` and `preflightRemediation.js`;
en and hy locale keys.

### Files

NavPy: the new port `core/ports/camera_health.py`; `frame_publication.py`,
`frame_provider.py`, `real_detector_diagnostics.py`,
`detector_public_diagnostics.py`, `vision_detector_factory_ports.py`,
`real_detector_factory.py`, `sim_detector_factory.py`,
`vision_controller_composition.py`, `camera_stream_composition.py`,
`camera_stream_messages.py` and `camera_stream_publisher.py`. Backend: new
`camera_preflight.py` (wait clock, expected flag, duplicates) and
`video_camera_record.py` (boot rules, Lost); `video_streams.py`,
`video_camera_discovery.py`, `vehicle_snapshot.py`, `vehicle_manager.py`,
`telemetry_loop.py`, `sim_video_companion.py`. Frontend:
`preflightReadiness.js`, `preflightRemediation.js`, `PreflightReadiness.jsx`,
en and hy. None is on the not-editable list.

## As built

Where the code differs from the text above, and why:

- The health port is `camera_health` (vendor and ready reading together),
  not `camera_ready`: the publisher needs both per mount, and one port keeps
  the factory, node and composition interfaces to one new field.
- The start sequence is recorded in `FramePublicationStore.set_running`, not
  `FrameCaptureRuntime`: the store already holds the sequence and the
  running flag under its one lock.
- A mount with no gimbal device id has no camera component id to send from,
  so it stays out (one with `video_streams` fails as before).
- `vendor_name` is `uint8_t[32]`: pymavlink gives it as a list of byte
  values. The backend decodes it up to the first NUL; a NavPy-to-GCS wire
  test packs and decodes both vendors.
- `FrameProvider.has_fresh_frame` is one more method on the detector facade:
  the SOLID guard's allowance for it goes from 40 to 41, with a comment
  naming this design (the project's ratchet rule).
- The backend launch-gate differential runs the real entry through each
  camera state (`test_camera_preflight.py`).
- Lockstep pair, origin/main b804147b against the branch, `hw` seed 2, test
  source off (2026-10-26): setup bytes identical; mission (171.8 s) per SITL
  +3,775 to +3,784 B (about 22 B/s: the camera HEARTBEAT plus one
  `CAMERA_INFORMATION` and ACK), backend +132 B (three 44 B requests); held
  and coordinator pending bytes 0 on both. Both runs fail the same workflow
  check ("no reset marker" for each UAV), seed 2's todo-77 signature
  (`radio_budget.md:186-190`), so main fails it too; runs in
  `.logs/preflight-camera-radio`.
- Live check, SITL, 3 UAVs (2026-10-26, `.logs/preflight-camera-live`):
  test source on, each UAV "Simulated" and "1 stream"; off, "None
  configured"; an injected camera with a userinfo URI, "URI rejected:
  userinfo" (amber), then "Lost" 5 s after it stopped; an injected copy of
  UAV 1's stream, "Same stream as" on both UAVs; LAUNCH enabled throughout;
  the "Connected" wording and both tooltips. Camera traffic at the GCS: 21
  B/s per camera steady, 248 B `CAMERA_INFORMATION` per generation (226 B
  stream information with a stream). First camera HEARTBEAT 0.6 s after the
  companion's at startup, 0.7 s after a NavPy restart, well inside the 15 s
  grace.
- Decision 8 counted four more `e.currentPoi` files; three remain
  (`SettingsModal.jsx`, `WaypointEditorOverlay.jsx`, `VirtualParamGrid.jsx`),
  todo 139. Follow-ups: the bench check is todo 141, the router filter todo
  142.

## Radio cost (decision 1)

Measured framed sizes (pymavlink, MAVLink 2, unsigned, first send): HEARTBEAT
21 B, `CAMERA_INFORMATION` 248 B, `COMMAND_LONG` 44 B (a resend 45 B),
`COMMAND_ACK` 22 B.

- New steady traffic: one camera HEARTBEAT per second for each mount that has
  no stream: every mount of a real UAV today; with the sim test source off,
  every sim mount; with it on, every mount except the first (the test source
  adds its stream to the first device only, `sim_video_test_source.py:40-53`).
  Three UAVs with one new mount each: 63 B/s, about 0.5 kbit/s, 0.8 % of the
  RFD900x's raw 64 kbit/s air rate before slot, framing and contention
  overhead (`radio_budget.md:35-49`); per UAV 0.5 % on top of the 3.9 kB/s
  serial1 send.
- Once per camera generation (new camera, new boot, lost then seen): 44 B up,
  270 B down (no stream requests for a mount without streams); resends after
  loss or a rejecting ACK, at most 3 sends per request.
- Status needs no new message: it rides on that heartbeat.
- Peers: the router filter blocks source component 100 only
  (`radio_budget.md:73-88`), so a second mount's camera (101 and up) also
  reaches every peer's serial1: 21 B/s per extra mount per peer. Only
  `novoxy_dual` has two mounts today. Follow-up todo: block camera
  components 100-105 in the router filter (sim and Orin).
- Checks: a NavPy unit test pins one heartbeat per mount per period. The live
  run counts framed bytes of every camera component per UAV at startup, in
  steady state and after a NavPy restart. A lockstep run pair, main and the
  branch, on the radio acceptance settings (`hw` seed 2,
  `radio_budget.md:173-185`), test source off so every mount is newly
  advertised: framed bytes per node including the backend, setup and
  mission apart (`air_load`, `eval_gcs_demo_air_load.py:85-111`); held
  bytes; coordinator pending bytes stay 0 on both. NAVLINK latency is read
  by hand today (todo 101), and a 21 B/s change is below its run-to-run
  spread, so it is not compared. The second mount stays computed: the eval
  profiles have one mount per UAV.

## Isolation

- The health port lives in NavPy's perception layer. A new guard test scans
  every module under `src/navpy` (AST) and fails if any module outside the
  wiring list (frame provider, diagnostics, factories, composition, camera
  publisher and messages) imports the new module or names the new readiness
  method or port. A read through an existing object would need that name, so
  this covers more than an import check.
- Assembled-path differential test: the sim vision controller (built as in
  `tests/modules/vision/test_vision_controller.py`, `ideal_360` profile)
  runs the same scripted vehicle states twice, with the camera publisher
  running and the `camera_health` reading forced ready in one run and not
  ready in the other;
  the coordinator's detections, the only vision output navigation reads,
  must be identical. Admitted camera HEARTBEATs (next point) leave the
  vehicle state navigation reads unchanged. With both inputs identical, the
  law's commands and completion decisions are too; the test does not run
  the law itself.
- Differential tests: every camera state, with all other inputs fixed, gives
  the same `computeLaunchReadiness` result (frontend) and the same backend
  launch issues; a camera caution never makes the header NO-GO.
- Camera HEARTBEATs with `ACTIVE` and `CRITICAL` and a non-zero boot id in
  `custom_mode` through the real inbound path leave the autopilot heartbeat count, mode, armed state and vehicle
  discovery unchanged (today's filters: `inbound_router.py:122-132`,
  `mav_bus_heartbeats.py:18-37`).
- Live: LAUNCH stays enabled with a camera caution showing.

## Owner decisions (recommendation first)

1. Expected camera: (c) the companion announces every camera, plus the sim
   test source per sys_id; cost above. A per-vehicle setting only when fleets
   become mixed.
2. Severity: caution only in v1; gate on the detector camera after the demo,
   per mission module.
3. Detector readiness: (a) camera HEARTBEAT `system_status` from frame
   freshness, read only from cameras whose `CAMERA_INFORMATION` vendor marks
   them as NavPy's.
4. Stream ownership: own-companion rule plus the same host/path check; the
   row does not claim physical ownership. A UAV name in the Jetson overlay
   later.
5. Sidebar readiness button shows the camera caution too (no edit needed).
6. Companion wording "Connected" with the tooltip above, row and CC badge.
7. Rows: option A, two rows.
8. `e.currentPoi` (the target-to-POI rename, e7d2c7f3, broke
   `e.currentTarget`): hovering a red or amber preflight cell throws, so no
   hint shows; backdrop click does nothing in four more files. Recommend
   fixing the three lines in `PreflightReadiness.jsx` here (these rows' hints
   need it) and the other files as a separate task.
9. Isolation test (Codex and the author disagree): (a) the reference guard
   plus the vision-boundary differential test (detections and vehicle state
   identical); the camera status has no path into the law, and the guard
   fails any module that names it. (b, Codex) also run the navigation law to
   commands and completion with fixed inputs; needs a new assembled
   harness through navigation. Recommend (a).
10. Radio checks (Codex and the author disagree): (a) the main-vs-branch
    lockstep pair (framed bytes per node, held and pending bytes), live byte
    counts, the second mount computed (21 B/s per extra mount per peer) and
    the router follow-up. (b, Codex) also by-hand NAVLINK drainage and
    latency checks at startup and rediscovery (todo 101 is not built) and a
    two-mount radio fixture. Recommend (a).

## Not in this change

- No NO-GO and no launch gate.
- UAV name in the Jetson overlay; physical stream ownership.
- Router filter for camera components 101-105 (follow-up todo).
- Real detector readiness and the two thresholds on the Orin and ZR10 rig: a
  bench check, logged as a follow-up todo; this change claims unit and SITL
  only.

## Checks

- `test_preflight_camera_rows_js.py`: every state in both tables and each
  step of both evaluation orders; two cameras (worst wins, count, one lost);
  a lost stream camera's stream out of the count; the grace flag on both
  sides; every camera state with other inputs fixed leaves
  `computeLaunchReadiness` unchanged and the header never NO-GO; hints and
  action-strip priority; en and hy keys. `PreflightReadiness.test.jsx`:
  companion wording for go, warn and nogo, the tooltip, the camera cells.
- Backend: per-camera status, vendor, has_video, failed and rejected in the
  snapshot; an empty vendor gives "other"; one of two cameras lost stays
  lost; lost then seen starts a new generation; boot rules: a NavPy record
  A to B rediscovers within the 5 s window; A, B, late A, B does not
  rediscover again; two cameras, one qualified and one pending at the
  restart, both end on B with one extra request for the pending one;
  another vendor's changing `custom_mode`, including an id NavPy used,
  changes nothing; a camera that does not come back after a restart turns
  Lost; a late A before B's information arrives and after loss leaves
  liveness, status and discovery unchanged; delayed information, stream
  information and a rejecting ACK after loss send nothing and keep Lost; boot id 0: a restart below `CAMERA_LOST_AFTER_S` keeps the failed
  discovery, at it rediscovers; `camera_wait_over` one tick before and at
  the grace, reset by companion down;
  two vehicles with one host and path both flagged, a lost camera's stream
  not; `_has_changed` sees each key; backend launch issues unchanged by
  camera keys.
- NavPy: ready at a frame age of exactly the threshold, not just past it,
  not before any frame, not after a None publication, not with the capture
  stopped, not after a restart until a new frame arrives; "NavPy sim" vendor
  for the sim detector; the same non-zero boot id in `custom_mode` for every
  mount and heartbeat of one process; every mount is a component,
  `HAS_VIDEO_STREAM` only with streams; one heartbeat per mount per period;
  camera heartbeats leave the autopilot heartbeat state unchanged; the
  reference guard; the assembled-path differential test.
- `npm test`, `npm run build`, `pytest tests`, `pytest tests/gcs`.
- Live (live-tester, SITL): test source on, each UAV "Simulated" and "1
  stream"; off, "None configured"; one URI outside `video.allowed_networks`,
  caution with its reason, LAUNCH still enabled; companion row and CC tooltip
  wording; camera-component framed bytes per UAV (startup, steady, after a
  NavPy restart); delay from companion up to first camera heartbeat.
- Lockstep run pair, main against the branch (Radio cost).

## Codex round 1 (2026-10-25) and answers

1. Blocker, older NavPy reads Ready: holds (`camera_stream_messages.py:77-84`
   sends `ACTIVE` always). Readiness now only from cameras whose
   `CAMERA_INFORMATION` vendor is NavPy's; others read "Not reported".
2. A None publication stamps a time; clock mix: holds
   (`frame_publication.py:52-75`). Readiness needs a frame; age on
   `rt.time()`; threshold provisional until the bench.
3. Losing one of two cameras restores GO: holds (`video_streams.py:128-137`).
   Lost cameras are kept within a NavPy boot.
4. "Restart NavPy" does not rediscover within 5 s: holds
   (`video_streams.py:88-94`). Records reset on a new boot id in the camera
   HEARTBEAT (the swarm heartbeat's boot id is not sent preflight).
5. 15 s is not a discovery bound: holds. Restated as a first-advertisement
   policy with start and reset events; running discovery takes precedence;
   measured live.
6. Duplicate check does not prove ownership: holds. Wording and this note
   say what it proves; the overlay name stays later.
7. Isolation not proven by the checks: holds. Import guard, differential
   tests, heartbeat-admission test and a live LAUNCH check added.
8. HIL flag means hardware in the loop: holds. Dropped; vendor name instead.
9. Radio accounting: holds. Sizes qualified, test-source case corrected,
   retries counted, raw-rate caveat added. Air-load run: see round 2,
   point 6.

## Codex round 2 (2026-10-25) and answers

Codex: round 1 points 1, 3, 6 and 8 resolved; no reader of a camera's
`custom_mode` found. New points:

1. A quick capture restart reuses the old frame: holds
   (`frame_publication.py:46-50`). Ready needs a frame published after the
   current capture start.
2. Another vendor's `custom_mode` could reset NavPy's cameras: holds. The
   boot id counts only for NavPy-qualified components, and a reset clears
   only their records.
3. A late heartbeat of the old boot would reset again: holds. Retired boot
   ids are ignored; the in-flight reply limit is stated.
4. "Restart NavPy" fails for older NavPy: holds. The hint says stop for more
   than 5 s, which works for both; legacy boundary tests added.
5. No navigation-command differential test: partly. No navigation input
   takes the camera status, so such a test has nothing to vary. The guard
   now scans all of `src/navpy` for the new names, not only imports, which
   also catches a read through an existing object. Boot ids go through the
   heartbeat admission test.
6. Radio checks miss discovery and peers: holds. Bytes counted live at
   startup, steady and after a restart; lockstep air-load report on main and
   the branch; the second mount reaches peers (component 101 passes the
   router filter): computed, router follow-up logged.
7. State tables leave outcomes open: holds. Both rows now have an evaluation
   order; a lost stream camera has its own state and leaves the count.

## Codex round 3 (2026-10-25) and answers

Codex: round 2 points 1, 4 and 7 resolved.

1. Boot ids across two cameras (one pending at the restart) and an id
   collision with another vendor: holds. The rules are now per camera
   record, so no camera resets another; only NavPy-qualified records read
   the id; a pending record catches up after it qualifies.
2. The reference guard is not a behavioural check: holds. An
   assembled-path differential test on the vision controller is added; it
   compares detections and vehicle state, the law's only inputs, not the
   law's commands.
3. Radio checks miss totals, peers and backlog: partly. The run pair now
   covers every node including the backend, setup and mission, held and
   pending bytes, on the radio acceptance settings. NAVLINK latency (read by
   hand, todo 101) and a two-mount run are not added: 21 B/s is below the
   latency spread, and no eval profile has two mounts.
4. Mock states table: holds, fixed.

## Codex round 4 (2026-10-25) and answers

Codex: round 3 point 4 resolved, point 1's cross-camera and mixed-vendor
cases addressed, no blocker.

1. Qualification and boot history must outlive a discovery generation:
   holds. Rule 1 splits the lasting record from the generation; rule 2
   keeps NavPy qualification and drops a known old id before any update.
2. Lost records would take delayed replies and send requests: holds
   (`video_streams.py:185-195`). Rule 5: a Lost record takes no replies and
   sends nothing.
3. Radio: Codex asks for by-hand NAVLINK drainage and latency checks at
   startup and rediscovery, and a two-mount radio fixture. Not agreed:
   owner decision 10.
4. Isolation: Codex asks for the differential test to run through the
   navigation law to commands and completion. Not agreed: owner decision 9.
