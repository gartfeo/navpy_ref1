# GCS video streams per vehicle

Status: design v8 approved by the owner 2026-10-18 (D1 (a): follow moves only the map; display codec: software H.264 on the Jetson) and shipped as PR #18. v10 draft (2026-10-19: one bottom-left vehicle window with a Camera / Synthetic switch, minimize and maximize-swap; it replaces the v9 draft, design C) awaits owner approval; its open questions are at the end.

Owner request (2026-10-16): "we should be able to see video streams in our GCS if
those are available."

Goal: the monitor view shows live video from any vehicle that has a camera stream,
and for vehicles that have none says "No camera" in plain text, never an empty
or broken picture.

Video is **operator display only**. Nothing here feeds frames, pixels or stream
state into NavPy's navigation or the final-approach command path (Pure Vision
Approach Constraint). The vision pipeline keeps reading its own camera input.

## What exists today

- Frontend: `VideoPanel.jsx` + `useWhepStream.js` play **one** WebRTC (WHEP)
  stream, bottom-left of the monitor view, hidden during manual control. The URL
  is the build-time `VITE_VIDEO_WHEP_URL` (Jetson tracking-overlay publisher's
  MediaMTX); absent, the panel is hidden. The WHEP client is MediaMTX's own
  `reader.js` v1.20.0, vendored with lifecycle patches. Not per vehicle; a URL
  change needs a rebuild.
- Backend: per-vehicle camera telemetry already arrives from NavPy's camera
  components (`MAV_COMP_ID_CAMERA..6`, one per gimbal device id):
  `GIMBAL_DEVICE_ATTITUDE_STATUS`, `CAMERA_FOV_STATUS`, `CAMERA_SETTINGS`
  (`vehicle_camera.py`, filtered to the vehicle's own companion). No
  `CAMERA_INFORMATION` / `VIDEO_STREAM_INFORMATION` handling anywhere, and the
  camera components send no `HEARTBEAT`.
- Companion inbound path: `InboundMessageRouter.ingest`
  (`inbound_router.py:109`) drops every standard message whose source sysid is
  not its own aircraft, so a GCS request cannot reach the companion today.
- pymavlink has `VIDEO_STREAM_INFORMATION` with `uri`, `type` and `encoding`
  (H264, H265) and `CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM`.
- Hardware: ZR10 RTSP `rtsp://192.168.144.25:8554/main.264` carries HEVC today
  (2026-10-15 recordings). The SIYI SDK can switch codec (`encode_codec_set` in
  gimbal-controller). The Jetson (Orin Nano) runs MediaMTX v1.20.0, which serves
  the tracking overlay over RTSP and WHEP.
- Sim: plain SITL renders no camera. With Unreal, aas-hitl's video service makes
  RTSP H.265 through FFmpeg + MediaMTX.

## Approaches compared

| Option | Latency | Cost here | Multi-vehicle fit |
|---|---|---|---|
| Discovery: MAVLink camera protocol v2 (`HEARTBEAT` of the camera -> `CAMERA_INFORMATION` -> `MAV_CMD_REQUEST_MESSAGE(269)` -> `VIDEO_STREAM_INFORMATION`) | n/a | small; what QGC does | per sysid + camera component |
| A. Browser WHEP straight to each aircraft's MediaMTX | ~0.1-0.3 s | lowest: client vendored, server already on the Jetson | one server per aircraft; on demand by nature |
| B. Ground MediaMTX gateway pulling RTSP, WHEP to browser | ~0.1-0.3 s | new process, ports, lifecycle, pull allow-list | fan-out to several viewers; any RTSP camera |
| LL-HLS / HLS | 1-3 s / 5-10 s | low | too slow for flying |
| WebSocket MJPEG from backend | low | decode + JPEG per stream in Python, 5-10x bandwidth | poor at 3+ streams |

Sources: [MAVLink camera protocol](https://mavlink.io/en/services/camera.html),
[MediaMTX WebRTC](https://mediamtx.org/docs/read/webrtc),
[MediaMTX WebRTC browser codecs and connectivity](https://mediamtx.org/docs/features/webrtc-specific-features),
[MediaMTX H.265 WebRTC discussion](https://github.com/bluenviron/mediamtx/discussions/4396).

**Choice: A.** Our aircraft carry the Jetson with MediaMTX in both product lines,
and its WHEP sender only runs while a reader is attached, so unwatched streams
cost no radio bandwidth. B is the fallback if we later need many viewers of one
stream or cameras without a MediaMTX in front of them.

## Design

### 1. Discovery (MAVLink camera protocol v2)

Aircraft side (NavPy companion):

- Each camera mount spec in the vision profile gets an optional `video_streams`
  list: `stream_id`, `name`, `uri` (RTSP, for standard GCSs such as QGC),
  `encoding`, resolution, framerate. No list = nothing below is sent for that
  camera and the GCS shows nothing.
- Each camera component with streams sends `HEARTBEAT` (`MAV_TYPE_CAMERA`) every
  `CAMERA_HEARTBEAT_PERIOD_S`, from its own component id.
- Inbound admission is widened narrowly: `COMMAND_LONG` with
  `MAV_CMD_REQUEST_MESSAGE` for message 259 or 269, `target_system` = own sysid
  and `target_component` = one of its camera components, from any source. All
  other foreign-sysid traffic stays dropped.
- The camera component answers with `COMMAND_ACK` (from that component, to the
  requester), then `CAMERA_INFORMATION` (flag `HAS_VIDEO_STREAM`) or
  `VIDEO_STREAM_INFORMATION` (param2 0 = every stream, each with `count`; else
  one id).

GCS backend, per (sysid, camera component), serialized one request at a time:

1. Camera `HEARTBEAT` seen while the camera is unknown or lost: start a new
   discovery generation (drops the previous one's streams).
2. Request `CAMERA_INFORMATION`. A send fails when its ACK or its payload has
   not arrived within `CAMERA_REQUEST_TIMEOUT_S`; up to 3 sends (the camera
   protocol's own rule), so an ACK without the payload still retries.
3. If `HAS_VIDEO_STREAM`: request all streams; collect ids against `count`;
   request any missing id singly, again up to 3 sends each.
4. Exhausted retries leave the camera "discovery failed" until its next
   lost -> seen transition (logged once).
5. No camera `HEARTBEAT` for `CAMERA_LOST_AFTER_S`: camera lost, its streams
   removed. This is per camera, so one dead camera disappears while the vehicle
   and its other cameras stay.

v1 supports independent camera components only (NavPy's, one camera per
component). Autopilot-proxied cameras (several cameras behind component 1,
told apart by camera device id) are out of scope.

Timing constants live in one shared module used by NavPy and the backend, each
documented and boundary-tested. MAVLink leaves heartbeat timing to the channel,
so these are our radio-link defaults, not protocol values:

- `CAMERA_HEARTBEAT_PERIOD_S = 1.0`, the rate QGC and PX4 use;
- `CAMERA_LOST_AFTER_S = 5.0`, five missed heartbeats;
- `CAMERA_REQUEST_TIMEOUT_S = 2.0`, above the radio round trip plus the companion's
  loop, to be checked against the measured link RTT on hardware.

### 2. Playback URL and validation

- The browser plays WHEP, not RTSP. The backend derives the WHEP URL from the
  announced RTSP URI by MediaMTX's own layout: same host and path,
  `http://<host>:<whep_port>/<path>/whep`. `whep_port` is `video.whep_port` from
  settings (default 8889, MediaMTX's default); in sim with the test source on,
  the backend uses its own slot's sim WHEP port from `instance_ports.py` (it
  already knows its slot), so no extra launcher wiring is needed.
- `uri` is untrusted text. Accepted only when: scheme `rtsp`, no userinfo, host a
  literal IP inside `video.allowed_networks` (settings; default the SIYI/radio LAN
  `192.168.144.0/24`; loopback added only in sim mode). Anything else is logged
  and ignored. No DNS names, so there is nothing to re-resolve. The path must
  be `[A-Za-z0-9._-]` segments separated by `/`, with no empty or `..`
  segment, no query and no fragment.
- The browser is held to that URL too (a new "NavPy patch" in the vendored
  reader, listed in its `NOTICE`): every fetch uses `redirect: 'error'` and
  `credentials: 'omit'`, and the session `Location` from the WHEP answer must
  resolve to the same origin and lie under `<path>/whep/`, else the session is
  dropped as an error. No other URL is ever fetched.
- The backend adds `video_streams` to each vehicle's telemetry snapshot:
  `key` (`"<camera component>:<stream id>"`, unique per vehicle, since two
  cameras may both have stream 1), `camera_component`, `stream_id`, `name`,
  `encoding`, `whep_url`. The backend itself never connects to the stream.

### 3. Codec

- Browser WebRTC needs H.264 without B-frames for every browser; H.265 plays only
  in recent Chrome/Safari with a hardware decoder. The aircraft display stream
  must therefore be H.264, constrained baseline or main without B-frames.
- The Orin Nano has no hardware video encoder (NVENC), so the display stream is
  a software H.264 encode (x264, `zerolatency`, no B-frames, keyframe interval
  2 s or less) at a reduced size such as 1280 x 720 and about 2 Mbit/s. Hardware
  acceptance: Jetson CPU load, added latency and unchanged vision capture timing
  are measured with the encoder running before it is used in flight.
- The window checks `RTCRtpReceiver.getCapabilities('video')` against the
  announced `encoding` before connecting (unknown encoding: try), and treats a
  failed negotiation or no frames as the error state below.

### 4. Reachability

- Supported deployment: the browser runs on a machine on the same IP network as
  the aircraft (the radio LAN), with no NAT between them. The aircraft MediaMTX
  must advertise its radio-LAN IP as ICE host candidate (`webrtcAdditionalHosts`)
  and list the GCS origin in `webrtcAllowOrigins` (the v1.20.0 key, a list).
  Both are aircraft config, set
  once in the publisher's MediaMTX config.
- The handheld (Termux) case is the same browser path, so it needs nothing extra
  beyond being on that network.

### 5. UI: one vehicle window

Owner, 2026-10-16: build each feature for a purpose; in the demo any UAV may
have a stream but one plays at a time; Cesium FPV must work without video; no
need for FPV and video together. v8 (2026-10-17): every element was walked
through a 3-UAV delivery mission by a field operator, a delivery operations
adviser, a UX designer and Codex, then debated with Codex; elements without a
concrete operator use were cut. v10 (owner, 2026-10-19): one window at the
bottom left that the operator minimizes or maximizes, instead of a camera panel
plus a top-right FPV button.

Purpose of the two sources:

- **Camera (real video)**: inspect the real scene at the two delivery moments
  the map cannot show: the dock confirmation (does the live scene still match
  the detector's still?) and the final approach (is there a reason to Cancel
  task?). Watching is optional; the map stays the main view.
- **Synthetic (Cesium nose view)**: orientation, and a vehicle view where there
  is no usable video (SITL demo, UAV without camera, failed feed), without
  taking RC authority. It shows attitude over terrain, not what the camera or
  detector sees, and never confirms anything. Named "Synthetic", not "FPV":
  DJI FlightHub 2 and UgCS use "FPV" for a real camera, and no ground station
  checked shows a synthetic first-person view in its camera window.

So the monitor view has one **vehicle window** at the bottom left. It shows one
UAV and one source, **Camera** (default) or **Synthetic**, never both. The map
stays the main view and keeps chasing the followed UAV; only RC turns the main
map first-person. One rule: **badges and View camera pick the window's UAV for
both sources; the follow selector moves only the map** (D1, decision 5).

Practice it follows (research 2026-10-19): QGroundControl's `PipView` (one
bottom-left window; hide leaves a restore button; a click swaps map and video;
both choices saved); DJI FlightHub 2 and UgCS (the source switches inside the
window and stays on the same vehicle; text states when there is no video);
NUREG-0700 Rev. 4 section 2.5.2 and EASA CS-AWO SVGS.102(c), by analogy only
(vehicle label at the top, the active mode shown at once, synthetic imagery
plainly marked, a window never covers commands or warnings).

#### Controls

| Control | v1 | Operator use |
|---|---|---|
| UAV badges in the window header, one per UAV in the fleet | yes | watching UAV 1's approach, check UAV 2 and tap 1 to come back, without moving the map. A badge picks the UAV for both sources and keeps the source. Selected badge: outline + fill + the full "UAV n" label, never colour alone. A badge never changes follow or RC |
| **View camera** on each task confirm card | yes | a card arrives: tap, compare the live scene with the still, then Approve/Deny on the card. It sets that UAV and the Camera source and restores a minimized window, so the operator does not watch the wrong UAV; it does not prove the camera still shows the same dock (it may have moved since the still). Neutral button beside the still, away from Approve/Deny; "Viewing" when the window already shows that UAV's camera and is not minimized; on tap the old picture clears and "connecting · UAV n" shows. The countdown keeps running (the card says so); the tap never approves, denies or pauses. Pending cards only; shown only when that UAV has a stream and RC is off |
| **Camera / Synthetic** switch in the window | yes | SITL demo or a dead feed: a vehicle view without RC, for the window's UAV. Synthetic only by a tap here: never automatic, never saved |
| Live state: LIVE (frames arriving), connecting, stale shown on the picture as text ("NO FRAMES 7 s"), failed, **Reconnect video** | yes | before relying on the picture at confirm or approach: a frozen frame looks like a calm scene. LIVE means frames arrive, not low latency |
| **Minimize** / restore and **Maximize** / restore | yes | minimize gives the map back and leaves a handle; maximize swaps window and map to judge detail at confirmation while the map stays visible, small. No tap-anywhere toggle on the window |
| "No camera · UAV n" | yes | that UAV advertises no stream: said in text, never a black box; Synthetic is one tap away |
| "Camera video lost" vs "Vehicle link lost" | yes | camera only: keep monitoring; link: check the vehicle, consider Cancel task or E-STOP. Both shown when both happen, link first. Video loss says nothing about NavPy or the detector; neither changes mission state |
| Header names the view from stream metadata (e.g. "UAV 2 · tracking overlay") | yes | the operator knows the picture is the Jetson overlay with detector boxes, not the raw camera |
| Stream picker (two views on one UAV) | later | no delivery decision needs it; the data model keeps all streams, v1 plays the UAV's default stream: the lowest stream id, and the companion config gives the display stream id 1 |
| Synthetic offered inside a failed feed | cut | it made synthetic imagery look like the answer to lost video; the switch is enough |
| Top-right FPV button | removed (v10) | the switch in the window replaces it |
| Full screen | later | maximize covers the confirmation need; QGroundControl's full screen ends when the link is lost |
| Auto-switch to a confirming UAV, with a pin | later | pulls the view off another UAV's approach; View camera is one tap |
| Snapshot / recording | later | after-action review only; an image does not prove docking or cargo receipt |
| Gimbal or zoom control | later, payload module | NavPy owns the gimbal during approach; zoom changes vision calibration; needs an ownership design |
| Camera and Synthetic together; mission phase in the header | no | no decision needs them; phase belongs in the sidebar |
| Tiles | later option (owner, 2026-10-16) | only with a new operator need once several streams play at once |
| Flight or mission actions in the window | no | Cancel task and E-STOP already exist; observation is not outcome |

Generic vs delivery: badges, the switch, live state, loss wording, minimize and
maximize serve any mission module; View camera depends on the task-confirmation
flow.

Touch and layout (handheld): the window's controls are 44 px targets with text
labels, no hover-only actions, nothing within 8 px of Approve/Deny. Header text
14 px on an opaque background. The header line holds only badges, the "UAV n"
title and the state; a control row under it holds the switch (left) and
Maximize and Minimize (right); a failure or held view goes on a row under that
(reason, Reconnect video, Dismiss). Narrow map pane (under 480 px): badges show
the number, the full "UAV n" stays in the title, badges that do not fit move
into one menu button, and Maximize and Minimize show their standard glyphs with
an accessible name. The badge room is computed for the whole fleet. In monitor
mode confirm cards and E-STOP are in the sidebar, not over the map, so neither
the window nor the small swapped map can cover them. The top-right bar goes
back to its pre-PR #18 controls at 32 px (owner, 2026-10-18); with the FPV
button gone it holds no new control, so the 44 px rule does not apply to it.

#### Vehicle window

- Where: bottom-left of the map pane, monitor view only, whenever the fleet has
  a UAV; hidden in manual control (RC). Three sizes: normal (v8's panel box:
  width min(432, pane - 68) px, stopping below the top-right controls),
  minimized (a 44 px handle) and maximized (the swap, below).
- Layout, top to bottom: header line; control row; a status row when needed;
  the picture (16:9, shrinking to fit the height limit). The UAV label stays at
  the top in both sources. A state with no picture (no camera, failed, lost,
  waiting for pose) collapses the picture area at normal size, so the window
  never shows a black box; maximized, the large area shows that state as text.
- Which UAV: badges and View camera pick it, for both sources. First pick, when
  the fleet gets its first UAV: the followed UAV if it has video, else the first
  UAV with video, else the first UAV (lowest sys_id). The first pick stays
  automatic until the operator taps a badge, the switch or View camera; while
  automatic and on "No camera", the window moves to the first UAV that starts
  advertising a stream (nothing is on screen to switch away from, so "never a
  silent switch" holds). The pick re-arms when the fleet becomes empty. Later
  follow changes never move the window (D1).
- Camera states:
  - Stream: video with its live state.
  - **No camera · UAV n**: that UAV advertises no stream (plain SITL opens
    here). No Reconnect or Dismiss. If that UAV starts advertising, its video
    starts.
  - **Camera video lost · UAV n**: the shown stream went away. That UAV's other
    stream plays if it has one; else this state, with Reconnect video and
    Dismiss, until a stream returns. Dismiss goes to "No camera · UAV n" for
    the same UAV. Never a silent switch to another UAV.
  - The GCS learns of streams only from the camera protocol: a camera that is
    expected but never advertises shows "No camera"; preflight reports it
    (`architecture_todo.md` 70).
- Deadlines: no first frame within `VIDEO_FIRST_FRAME_DEADLINE_S` (default 10 s:
  ICE setup plus one keyframe interval, which the display stream keeps at 2 s or
  less), or no new frame for `VIDEO_STALL_DEADLINE_S` (default 5 s, a few
  keyframe intervals), becomes the failed state. Measured by frame progress
  (`requestVideoFrameCallback`, fallback `getVideoPlaybackQuality`), not only the
  video element's waiting/stalled events. Both named in one frontend constants
  file, with fake-timer tests. Failed or codec not supported: a short reason and
  Reconnect video.
- Minimized: a 44 px handle in the window's corner reading "UAV n" and its
  state from telemetry and stream announcements ("Camera", "No camera",
  "Synthetic", "Vehicle link lost"; never LIVE). A tap restores. Minimizing
  closes the camera reader and destroys the synthetic widget: an unwatched
  stream costs radio bandwidth (v8's collapse did the same); restore reconnects
  or rebuilds the kept source.
- Maximize swaps the window and the map (QGroundControl, UgCS, DJI Pilot 2):
  the window takes v8's expanded box (pane width minus 12 px margins, from
  below the top-right controls to the bottom margin) and the map shrinks into
  the window's normal box, drawn above it. The window's Maximize reads Restore
  while swapped. A tap anywhere on the small map swaps back: a transparent
  button covers it (accessible name "Show the map large"), so the small map is
  never panned, zoomed or picked, and no accidental drag becomes the saved
  startup view. While swapped the main map's
  `screenSpaceCameraController.enableInputs` is false, which also stops the
  inertia of a gesture made just before maximizing
  (`ScreenSpaceCameraController.js:511`); the chase still moves the camera
  programmatically. It is set back to true on the swap back.
  - CSS only: the map wrapper stays mounted and only its box changes. Never
    remount `CesiumMap` (the viewer cleanup does not destroy the viewer, so a
    remount leaks a WebGL context), never animate the size (Cesium resizes its
    canvas on its render tick, so a transition reallocates it every frame),
    never hide the map or size it to 0 (rendering and the chase would stop).
    Cesium picks up the new size on its next tick.
  - While swapped: the follow selector still moves the small map and the chase
    keeps working; the top-right bar stays usable (the window starts below it);
    reset view is hidden (a fit made in the small box uses its aspect ratio and
    is wrong after the swap back); markers keep their size, so a 3-UAV map is
    crowded: it is an overview.
  - Automatic fits wait for the large map. The chase-to-Free auto-centre
    (today `useFpvView.js:112-117`, also reached when the followed UAV leaves,
    `App.jsx:142-149`) and any other `centerView` call made while swapped are
    deferred; maximizing during a fit's flight cancels it
    (`camera.cancelFlight`) and defers it the same way. A pending fit is
    dropped when follow or RC takes the camera. On the swap back the map calls
    `viewer.resize()` (public, synchronous), then runs the pending fit once,
    only if the map is still Free. A fit in the small box would use its aspect
    ratio and be wrong after the swap back; shrinking itself changes only the
    frustum, so a move end while small saves the unchanged pose.
  - Swap back on: Restore, a tap on the small map, Minimize (swap back, then
    minimize), RC entry, leaving the monitor view, an empty fleet. Never saved:
    a reload starts unswapped.
- Credits: Cesium's credit line (the Cesium ion logo, which the engine treats
  as required, and the data attribution; Cesium ion terms 2.2.3) is never
  covered. Each Cesium view keeps its credits at its own bottom-left (Cesium's
  default), and the small item (the window or its handle normally, the map when
  swapped) sits 12 px above the top of the large view's credit line, measured
  from the large viewer's credit container the way the top controls are
  measured. Camera video large has no credit line, so the margin is 12 px.
  Maximized Synthetic fills the window, so its credits sit at the window's
  bottom-left. The small map's own credits stay visible inside it; its tap
  button blocks their "Data attribution" link until the swap back.
- Saved: only minimized or normal, under the browser storage key
  `gcs-vehicle-window`, every access in try/catch, default normal (as
  `gcs-camera`). Never the source, never the swap.
- The selected UAV leaves the fleet: its badge disappears and the selection
  stays. Camera: "Vehicle link lost · UAV n" and "Camera video lost" as today.
  Synthetic: the last pose holds with "View held · vehicle link lost". Both
  resume if the UAV returns; the window never switches UAV or source by itself.
- One reader per browser; the demo has one GCS, so no radio-wide arbitration in v1.
- Removed (v8): `VITE_VIDEO_WHEP_URL`, `videoConfig.js` and the single-feed
  wiring (no shim). `useWhepStream` and the vendored reader stay; `src/gcs/AGENTS.md` and
  the Jetson config notes are updated.

#### Synthetic view

Owner, 2026-10-18 (design C): a second Cesium view while the map keeps chasing;
2026-10-19: it is a source of the vehicle window.

- Selection: only a tap on the switch. Switching to Synthetic closes the camera
  reader (one reader per browser); switching back reconnects (fresh connection)
  and shows the kept camera state, a lost feed included. A stream appearing,
  failing or going away while on Synthetic never changes the source.
- Which UAV: the window's UAV, picked by the badges; follow plays no part. A
  badge tap moves the same widget to the new UAV's pose.
- Rendering: a second public `CesiumWidget` filling the picture, created on
  entering Synthetic (normal or maximized), destroyed on Camera, minimize, RC,
  leaving the monitor view and failure (at most one, also under React
  StrictMode). One Scene has one camera and CesiumJS has no public
  multi-viewport API, so a second widget is Cesium's own pattern (Sandcastle
  "Multiple Synced Views"); the private `SharedContext`, `View` and
  `Framebuffer` are not used. Content: shaded terrain in one ground colour and
  no imagery, the way aircraft synthetic vision draws terrain; it looks
  plainly unlike camera video and loads no imagery tiles. No imagery also
  because Bing attributions cannot show in two views at once: Cesium caches
  Bing metadata, and with it each attribution's `Credit`, across provider
  instances (`BingMapsImageryProvider.js:83-90,160-169`), and a `Credit` owns
  one DOM element that each display re-parents (`Credit.js:91-100`,
  `CreditDisplay.js:111-124`). Options (Cesium 1.138.0 installed):
  `baseLayer: false` (never `imageryProvider`, ignored since 1.107, and never
  undefined, which adds Ion world imagery); `terrainProvider` is the widget's
  own instance of the main map's terrain source, requested with vertex normals
  for shading, never the main map's instance, so its Ion attribution is its
  own `Credit` (`IonResource.js:152-164`, `Credit.js:153-156`). The main map
  swaps in Ion terrain asynchronously (`useCesiumViewer.js:226-242`), so an
  open widget follows that upgrade with its own new instance and unsubscribes
  on teardown; with no Ion terrain the ground is the flat ellipsoid under the
  horizon. Its own tile requests are counted by the load gate. Lighting from
  one fixed `DirectionalLight` (not the sun, so shading does not change with
  the time of day) with `globe.enableLighting`; `requestRenderMode` with
  `maximumRenderTimeChange` Infinity, the main map's target frame rate as one
  shared constant, `msaaSamples` 1, `showRenderLoopErrors` false, camera
  inputs off; sky atmosphere on (the horizon is the attitude cue), sun, moon
  and sky box off. No entities; peers, docks and tracks stay on the chasing
  map.
- Pose: the same interpolated position and orientation the map's UAV model
  uses (copies through a narrow source ref from `CesiumMap`, wired like the
  map camera ref), through today's first-person math moved into one pure helper
  shared with RC FPV. The widget applies a pose only when the GCS telemetry
  socket is connected and that UAV's snapshot has been replaced since the
  socket last opened, the snapshot passes the hold check (`fpvHoldReason`) and
  the copied position and orientation are finite; otherwise it holds the last
  good pose. A closed socket leaves the cached snapshots, `link_ok` included,
  unchanged, and reopening keeps them until new telemetry arrives
  (`telemetryStore.js:93-125,135-139`); the store replaces a vehicle's snapshot
  object on each update for it, so "replaced" needs no timer. Both checks go
  into the shared hold check, so RC FPV gets the same held reason. A UAV removed from the fleet
  has no source entry, which the source reports as "no pose", never as a throw.
  Prerequisite: the shared source validates before it builds or advances a
  pose. Today it skips only a missing lat/lon and fills a missing altitude or
  attitude with 0 (`useUavMarkers.js:37-56`), then interpolates towards that
  pose (`lerpAnim.js`); a UAV with no altitude drops to the terrain on the map.
  The source skips the update unless lat, lon, altitude, heading, roll and
  pitch are all finite numbers, so the model, the chase and the synthetic view
  keep the last valid pose and resume from it. One render request per
  telemetry update. The widget saves no camera, so a nose pose is never the
  startup view.
- Wrong-UAV guard: the picture shows the widget only while its pose belongs to
  the window's UAV. After a badge tap, until that UAV has a valid pose, an
  opaque layer over the picture reads "Waiting for pose · UAV n"; the previous
  UAV's view is never shown under the new label.
- Failure: the widget fails to create, `scene.renderError` fires, the canvas
  loses its WebGL context, or the widget's own pose callback throws -> widget
  destroyed, source back to Camera, notice "Synthetic view unavailable". The
  callback needs its own catch: Cesium raises `preRender` outside its error
  wrapper (`Scene.js:4440`), and a throw there stops the widget's render loop
  with no event (`CesiumWidget.js:73-80`, error panel off). The main map keeps
  rendering. Retry is a new tap; no automatic retry.
- Look: title "UAV n · synthetic view"; **SYNTHETIC** in the state slot (never
  LIVE, never a green dot); an accent inner frame (video never has one); a
  fixed tag **SYNTHETIC · not camera** at the top-left of the picture. While
  the view holds its last pose, a row under the header reads **View held ·
  telemetry disconnected**, **View held · vehicle link lost** or **View held ·
  pose unavailable** with the seconds held. The handle reads "UAV n · Synthetic".
- RC: unchanged. Body FPV of the RC target on the main map with today's
  full-pane tag, frame and held banner (`FpvViewOverlay`, now RC only); the
  window is hidden.
- Map camera ownership: only RC takes the map camera. The saved map camera
  and the persistence suspension (`useCesiumViewer.js`) apply to RC only.
- Load gate before merge, on the demo PC: main-map frame time, GPU process
  memory, tile and Ion request counts with Camera, with Synthetic at normal
  size, and with Synthetic maximized (map small). Then switch to Synthetic and
  back 20 times, and maximize and restore 20 times on Synthetic: no extra
  canvas is left in the DOM, a heap snapshot holds no stale widget, scene or
  WebGL context, and GPU and JS memory return near the first-Synthetic level
  (Cesium frees its resources on destroy but does not release the WebGL
  context explicitly, `Context.js:1729-1750`). Throughout, the main map
  renders with no context loss and no render-loop stop. The owner accepts the
  measured cost before merge; the handheld is measured before handheld use. If
  the cost is too high, the fallback is a 2D attitude indicator in the window
  (QGroundControl and Mission Planner show the vehicle view this way).

| Event | Result |
|---|---|
| First UAV joins (monitor view) | window normal, or minimized if saved; first pick as above; source Camera |
| Badge tap | that UAV in the current source; the map does not move |
| Switch to Synthetic | camera reader closes; the window's UAV in the synthetic view, or "Waiting for pose · UAV n" |
| Switch to Camera | widget destroyed; that UAV's camera state, fresh connection |
| View camera on a pending confirm card | that UAV, source Camera, a minimized window restored; the swap stays; follow unchanged |
| Minimize | swap back if swapped; reader closed, widget destroyed; handle shown; saved |
| Restore from the handle | normal size; the kept source reconnects or rebuilds; saved |
| Maximize | map and window swap; nothing reconnects or rebuilds |
| Restore, or tap on the small map | swap back; nothing reconnects or rebuilds |
| Follow changed or set to Free | the map only; the window does not change; while swapped, the Free auto-centre waits for the swap back |
| Telemetry disconnected, link lost or pose not finite (Synthetic) | the last pose holds with the held row; resumes on the next valid pose |
| Selected UAV removed from the fleet | badge gone, selection kept; Camera: link lost and video lost; Synthetic: held; resumes if it returns |
| Stream appears, fails or goes away | Camera: the camera states above; Synthetic: the source stays, the camera state shows after switching back |
| Widget fails, render error, WebGL context lost or pose callback throws | widget destroyed; source Camera with "Synthetic view unavailable"; the main map keeps rendering |
| RC entry | swap back; window hidden; widget destroyed; source Camera; today's RC FPV of the RC target on the main map |
| RC exit | Free map with the saved map camera; the window returns as it was (normal or minimized), source Camera |
| Leaving the monitor view, or the fleet becomes empty | swap back; window gone; source Camera; an empty fleet re-arms the first pick |
| Page reload | normal or minimized as saved; source Camera; unswapped; the startup view is the saved map camera |

- Code impact (no shims): `useFpvView` shrinks to RC FPV and RC-only map
  camera ownership; the top-right FPV toggle, `toggleFpv`, `changeFollow` (the
  follow selector calls `setFollowSysId` directly), `pickFpvTarget`, the view
  half of the camera save and restore, and their tests go. The window is
  visible when monitor, not RC and the fleet is not empty; the badge count is
  the fleet size; a new `noCamera` kind; Dismiss on a lost feed goes to it;
  "Viewing" needs source Camera and a window that is not minimized.
- Deferred: full screen; a camera-direction synthetic view from gimbal attitude
  + FOV (needs verified mount axes and stream-to-device association); resizing
  the window by drag.

#### Found by the walkthrough, outside this design

Logged in `architecture_todo.md` (67-72): a pending-confirm cue and card order
by time left; the confirm card says what happens at zero (code and demo config
reject on expiry: `aas_params.lua:19`, `eval_gcs_demo_config.py:144`); E-STOP
and the right UAV's Cancel task reachable when cards stack; a per-UAV camera
row in preflight, including an expected camera that never advertises; the
still's capture time and its dock; operator status clarity (what the companion
indicator measures, mission phase, dock assignment, honest completion wording).

Unchanged and not part of this design: confirm cards and their countdown stay
over the map in RC (`App.jsx:771-795`). Deferred: an audit of who decided each
confirm (operator or timeout), recipient authorization for real delivery,
fleet separation checks (a mission-level verification), and outdoor
glare/glove acceptance (the 44 px and 14 px rules are built now).

### 6. SITL vs real hardware

| Setup | Stream source | Discovery |
|---|---|---|
| Plain SITL, default | none | nothing announced -> "No camera · UAV n" |
| Plain SITL, `simulation.video_test_source` on (default off) | one sim MediaMTX for the slot, a path per UAV with `runOnDemand` FFmpeg H.264 test pattern labelled "SIM TEST UAV n" | NavPy sim companion announces `rtsp://127.0.0.1:<slot port>/uav<n>` |
| SITL + Unreal (aas-hitl video service) | Unreal camera -> FFmpeg -> MediaMTX | the configured URI (needs an H.264 output to play everywhere) |
| Real aircraft | Jetson MediaMTX over the radio LAN | NavPy companion on the Jetson |

The sim MediaMTX is the only new process. The launcher starts it only when the
setting is on, records its PID in the slot's registry entry like backend and
frontend, and `gcs_stop.py` kills its tree (FFmpeg children included). Its RTSP,
WebRTC HTTP and WebRTC UDP ports are new slot-derived bands in
`instance_ports.py`, checked against all existing bands for slots 0-84; every
other MediaMTX listener is disabled: RTMP, HLS, SRT, MoQ, API, metrics, pprof,
playback, and RTSP's UDP and multicast transports (RTSP is TCP only, and FFmpeg
publishes over TCP). A test checks that the generated config opens only the
three allocated ports. A start
failure leaves video off and is logged; the rest of the stack still runs.

## Test plan

- NavPy unit: camera heartbeats only for cameras with streams; admission accepts
  only `REQUEST_MESSAGE` 259/269 to own sysid + own camera component and still
  drops other foreign traffic; ACK source and target; `CAMERA_INFORMATION` flag;
  `VIDEO_STREAM_INFORMATION` count, ids, param2 0 vs one id; nothing when none
  configured.
- Backend unit: request timeout boundaries, generation start on heartbeat, serialized requests, retry on a
  missing payload after an ACK, missing-id recovery, retry exhaustion, one camera
  lost while another stays, companion restart, URI validation (scheme, userinfo,
  DNS name, outside network, loopback outside sim, bad path), WHEP URL
  derivation incl. the sim slot port, snapshot `key` with two cameras both
  advertising stream 1 (remove one, the other stays). A check that no navigation module imports the video code.
- Integration: request and answer through the real link (GCS -> router -> SITL ->
  companion serial0 and back) in SITL, since the message routing is the risky
  part.
- Launcher: sim MediaMTX recorded and stopped by `gcs_stop.py`, two slots at once
  without port clashes, start failure leaves the stack running.
- Reader patch tests: a redirect, a foreign-origin `Location` and a `Location`
  outside `<path>/whep/` each end in error with no further fetch.
- Frontend (vitest): the window in monitor view whenever the fleet has a UAV,
  also with no streams ("No camera · UAV n", no Reconnect or Dismiss), hidden
  in manual control and outside the monitor view, gone with an empty fleet;
  badges for every UAV; the first pick (followed UAV with video, first UAV with
  video, first UAV) and the automatic move from "No camera" to the first UAV
  that starts advertising, which stops after any operator tap; a follow change
  never moves the window and a badge never moves the map; first-frame and
  stall deadlines (peer connected but no frames; frames then freeze; frames
  stop while the element raises no waiting/stalled event); codec-not-supported
  and error states with Reconnect video; stream removal: the same UAV's other
  stream, else "Camera video lost · UAV n" with Reconnect video and Dismiss,
  Dismiss to "No camera · UAV n", never another UAV; minimize closes the reader
  and destroys the widget, restore reconnects or rebuilds; the saved state is
  only minimized or normal (a throwing storage falls back to normal), never the
  source or the swap; maximize and restore keep the same map element (no
  remount, no animation, never size 0) and the same reader; the button over the
  small map swaps back and passes no input to the map; maximizing during
  inertia or an active touch gesture leaves the camera to the chase alone and
  saves no moved pose, and camera inputs are back on after the swap back; reset view hidden while
  swapped; Minimize, RC entry, leaving the monitor view and an empty fleet each
  swap back; Free and the followed UAV leaving while swapped: the fit runs once
  after the swap back with the large pane's aspect, a fit in flight when
  maximizing is cancelled and rerun, Follow -> Free -> Follow -> Restore runs
  no fit, a pending fit then RC entry is dropped, and the saved camera is never
  a small-box fit; the small item's bottom offset follows the measured credit line;
  every row of the transition table. Synthetic: entered only by the switch,
  never on any automatic path; SYNTHETIC state and tag, accent frame, never
  LIVE; held rows with the time held; the telemetry socket closing and
  reopening with no snapshot in between keeps holding with "telemetry
  disconnected", and the next snapshot for that UAV resumes it; "Waiting for pose · UAV n" after a badge
  tap until that UAV has a valid pose; a removed UAV holds. The widget (Cesium
  mocked): one per mount under StrictMode, `baseLayer: false`, its own terrain
  provider instance (never the main map's) that follows the Ion terrain
  upgrade, no imagery layer, no entities, inputs off, the shared pose helper gives
  today's RC FPV values, the shared pose source (real `useUavMarkers` and
  `lerpAnim`) skips a sample with a missing or non-finite lat, lon, altitude or
  attitude, keeps the last valid pose and resumes from it while the main map
  keeps rendering, no pose applied while the snapshot fails the hold check or
  the copy is not finite, a missing source entry reads as "no pose", one render
  per update, the same widget on a badge tap, unsubscribed and destroyed on
  unmount, and
  a create error, render error, context loss or a pose-callback throw each
  return to Camera once with the notice, never touching the main camera or
  stored view. A real `CreditDisplay` (Cesium not mocked; Ion and Bing
  endpoint metadata mocked and cached) with the main map and the widget
  rendering at once, then the widget torn down: the main map keeps its Bing
  and terrain attributions and ion logo in its own container throughout, and
  the widget shows its own terrain attribution and ion logo. RC FPV unchanged with its
  SYNTHETIC tag and the new held reason; RC exit restores the
  saved map camera; a nose pose is never stored as the startup view. View
  camera on a pending confirm card sets that UAV and Camera and restores a
  minimized window, shows "Viewing" only when that UAV's camera is shown and
  the window is not minimized, is absent on approved cards, when that UAV has
  no stream and during RC; tapped midway through a countdown it keeps the
  original deadline, expiry runs the existing timeout action once, the tap
  calls neither approve nor deny. LIVE dot only while frames arrive, "no
  frames for N s" after; "Camera video lost" and "Vehicle link lost" shown
  separately, both when both. The top-right bar has no FPV button and the
  follow selector changes only the map follow. `npm test`, `npm run build`.
- Layout: at desktop size and a 375 px wide map pane (a 695 x 812 touch
  viewport: the 320 px sidebar does not shrink, `styles.js:92`), with 3 UAVs, a UAV
  with two streams, the window normal, minimized and maximized, and three cards
  in the sidebar: header on one line (badges or the menu), 44 px window
  targets, the map buttons fit with the plan button showing, Approve/Deny and
  E-STOP not covered, the Cesium credit line visible in every state, no overlap
  (vitest layout contract plus a `live-tester` screenshot check).
- Live (`live-tester`): 3-UAV SITL with the test source, from two slots at once
  (each plays its own slot's patterns): each badge plays its labelled pattern,
  switching works, killing a path's FFmpeg shows the failed state and Reconnect
  video recovers; maximize, tap the small map to swap back, minimize and
  restore while video plays. With the source off: the window opens on "No
  camera · UAV 1", no console errors; Synthetic shows the nose view while the
  map chases, switch UAVs by badge while they fly (the map keeps its follow),
  link loss shows the held row, maximize Synthetic (the small map keeps
  chasing), back to Camera, RC entry and exit, force a WebGL context loss on
  the widget canvas (back to Camera with the notice, the map keeps rendering),
  then reload the page: the startup map view is the map, the window is on
  Camera in its saved size. Screenshots at desktop size and under 480 px. With
  the test source on: Synthetic replaces the video, Camera reconnects, View
  camera from Synthetic shows that UAV's camera. Then the load gate (Synthetic
  view section), on a quiet PC.
- Hardware (later, with the owner): Jetson H.264 stream over the radio from the
  laptop and from a second device (WHEP OPTIONS/POST accepted from the real GCS
  origin): glass-to-glass latency (on-screen clock),
  bitrate, and vision capture timing unchanged while the GCS watches.

## Owner decisions

1. Approach approved (2026-10-18): camera protocol discovery, browser WHEP
   straight to each aircraft's MediaMTX, one feed at a time, FPV toggle for
   Cesium FPV of any UAV (v10: replaced by the Synthetic source, decision 5).
2. D1 (a) (2026-10-18): the follow selector moves only the map and FPV; badges
   and View camera move the video; follow picks only the first feed (v10: the
   follow selector moves only the map; decision 5).
3. Display codec (2026-10-18): the Jetson publishes a software H.264 display
   stream; the ZR10 and the vision input stay untouched (Jetson CPU measured
   first).
4. v9 direction (2026-10-18): design C, FPV in the bottom-left panel while the
   map keeps chasing.
5. v10 direction (2026-10-19): one bottom-left vehicle window that the
   operator minimizes or maximizes; badges pick the UAV for both sources and
   the follow selector moves only the map; maximize swaps map and window; plain
   SITL opens on "No camera". The top-right FPV button goes; the synthetic
   source is named Synthetic, chosen by hand only and never saved; only
   minimized or normal is saved.

## Open for the owner (v10)

1. Minimizing stops the camera video and the synthetic view (recommended: an
   unwatched stream costs radio bandwidth and Jetson encoding; restore then
   waits for the first frame, up to the 10 s deadline). The alternative keeps
   video playing while minimized, so restore is instant.

2. The synthetic view shows shaded terrain without imagery (recommended:
   Cesium cannot show Bing attributions in two views at once, and aircraft
   synthetic vision also draws terrain without photos). The alternative keeps
   imagery through a provider wrapper that gives the widget its own copy of
   every credit: more code that depends on how Cesium caches credits.

Decision 5 closes v9 questions 2 and 4: follow no longer drives the synthetic
view, and the top-right bar is back to its pre-PR #18 controls at 32 px. v9
question 1 keeps its recommended answer unless the owner says otherwise: a
second Cesium view behind the load gate (fallback: a 2D attitude indicator).
Question 2 above replaces v9 question 3.
