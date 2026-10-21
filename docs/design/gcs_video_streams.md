# GCS video streams per vehicle

Status: design v8 approved by the owner 2026-10-18 (D1 (a): follow moves only the map; display codec: software H.264 on the Jetson) and shipped as PR #18. v10 (2026-10-19: one bottom-left vehicle window with a Camera / Synthetic switch, minimize and maximize-swap; it replaces the v9 draft, design C) approved by the owner 2026-10-19 (decision 6) and shipped as PR #20. **v11** (owner request, 2026-10-19): the vehicle window moves to the top right under the action bar and holds the UAV picker, each UAV marked with a camera or map icon; a UAV without a camera opens on Synthetic; the follow selector and the map chase go. This reverses D1 / decision 5. Approved by the owner 2026-10-20 with the recommended answers to the open questions (decision 7) and implemented 2026-10-20.

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
plus a top-right FPV button. v11 (owner, 2026-10-19): the follow selector and
the window's UAV badges did almost the same job (pick the UAV to look at; a
map chase is close to a synthetic view), so one control is left, and it lives
in the window: the window moves to the top right, under the action bar, holds
the UAV picker, and the follow selector and the map chase go. A UAV without a
camera opens on Synthetic by itself.

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

So the monitor view has one **vehicle window** at the top right, just under
the action bar. It shows one UAV and one source, **Camera** or **Synthetic**,
never both. Until the operator picks a UAV it reads **Select a UAV for
streaming** and lists the fleet. The map stays the main view and never follows
a UAV; only RC turns the main map first-person. One rule (v11, reverses D1 /
decision 5): **the UAV buttons in the window and View camera on a confirm card
pick the window's UAV; each UAV button shows a camera icon when that UAV
advertises a stream and a map icon when it does not, and a pick opens what the
icon shows: Camera, or Synthetic.**

Practice it follows (research 2026-10-19): QGroundControl's `PipView` (one
small window; hide leaves a restore button; a click swaps map and video;
both choices saved); DJI FlightHub 2 and UgCS (the source switches inside the
window and stays on the same vehicle; text states when there is no video);
NUREG-0700 Rev. 4 section 2.5.2 and EASA CS-AWO SVGS.102(c), by analogy only
(vehicle label at the top, the active mode shown at once, synthetic imagery
plainly marked, a window never covers commands or warnings).

#### Controls

| Control | v1 | Operator use |
|---|---|---|
| **UAV buttons** in the window, one per UAV in the fleet, each with a camera or map icon | yes (v11) | a glance tells which UAVs have video; tap UAV 2: its camera plays; tap UAV 3 (map icon): its synthetic view opens. Before a pick they fill the window under "Select a UAV for streaming"; after a pick they stay in the header as the quick switch. The icon follows the UAV's stream announcements and changes when they change; a UAV whose link is lost also reads "link lost" in text. Selected button: outline + fill + the full "UAV n" label, never colour alone. A button never moves the map and never changes RC |
| **View camera** on each task confirm card | yes | a card arrives: tap, compare the live scene with the still, then Approve/Deny on the card. It picks that UAV with the Camera source (from the picker too), so the operator does not watch the wrong UAV; it does not move the map and does not prove the camera still shows the same dock (it may have moved since the still). Neutral button beside the still, away from Approve/Deny; "Viewing" when the window already shows that UAV's camera; on tap the old picture clears and "connecting · UAV n" shows. The countdown keeps running (the card says so); the tap never approves, denies or pauses. Pending cards only; shown only when that UAV has a stream and RC is off |
| **Camera / Synthetic** switch in the window | yes | change the source for the picked UAV: Synthetic for a UAV with a camera, or Camera to check a map-icon UAV again. Never saved |
| Live state: LIVE (frames arriving), connecting, stale shown on the picture as text ("NO FRAMES 7 s"), failed, **Reconnect video** | yes | before relying on the picture at confirm or approach: a frozen frame looks like a calm scene. LIVE means frames arrive, not low latency |
| **Close** and **Maximize** / restore | yes | Close stops the picture and goes back to "Select a UAV for streaming"; maximize swaps window and map to judge detail at confirmation (or to see Synthetic large) while the map stays visible, small. No tap-anywhere toggle on the window |
| Follow selector in the action bar, and the map chase | removed (v11) | the UAV buttons pick the UAV; the map no longer needs a chase of its own (Synthetic maximized is the large vehicle view) |
| Minimize to a handle | removed (v11) | Close replaces it; the picker state is small (open question 3) |
| "No camera · UAV n" | yes | shown when the operator switches a map-icon UAV to Camera: said in text, never a black box |
| "Camera video lost" vs "Vehicle link lost" | yes | camera only: keep monitoring; link: check the vehicle, consider Cancel task or E-STOP. Both shown when both happen, link first. Video loss says nothing about NavPy or the detector; neither changes mission state |
| Header names the view from stream metadata (e.g. "UAV 2 · tracking overlay") | yes | the operator knows the picture is the Jetson overlay with detector boxes, not the raw camera |
| Stream picker (two views on one UAV) | later | no delivery decision needs it; the data model keeps all streams, v1 plays the UAV's default stream: the lowest stream id, and the companion config gives the display stream id 1 |
| Synthetic offered inside a failed feed | cut | it made synthetic imagery look like the answer to lost video; the switch is enough |
| Top-right FPV button | removed (v10) | the switch in the window replaces it |
| Full screen | later | maximize covers the confirmation need; QGroundControl's full screen ends when the link is lost |
| Auto-switch to a confirming UAV, with a pin | later | pulls the view off another UAV's approach; View camera is one tap |
| Live preview tiles per UAV | no (owner, 2026-10-19) | several streams at once break the one-stream rule and cost radio bandwidth per UAV; the icons say enough |
| Snapshot / recording | later | after-action review only; an image does not prove docking or cargo receipt |
| Gimbal or zoom control | later, payload module | NavPy owns the gimbal during approach; zoom changes vision calibration; needs an ownership design |
| Camera and Synthetic together; mission phase in the header | no | no decision needs them; phase belongs in the sidebar |
| Tiles | later option (owner, 2026-10-16) | only with a new operator need once several streams play at once |
| Flight or mission actions in the window | no | Cancel task and E-STOP already exist; observation is not outcome |

Generic vs delivery: the UAV buttons, the switch, live state, loss wording,
close and maximize serve any mission module; View camera depends on the
task-confirmation flow.

Touch and layout (handheld): the window's controls are 44 px targets with text
labels, no hover-only actions, nothing within 8 px of Approve/Deny. Header text
14 px on an opaque background. Picker state: the line "Select a UAV for
streaming", then the UAV buttons, wrapping onto more rows when they do not fit
(the window grows; past its height limit the list scrolls). Viewing state: the
header line holds the UAV buttons, the "UAV n" title and the state; a control
row under it holds the switch (left) and Maximize and Close (right); a failure
or held view goes on a row under that (reason, Reconnect video, Dismiss).
Narrow map pane (under 480 px): header buttons show the icon and the number,
the full "UAV n" stays in the title, buttons that do not fit move into one menu
button (v10's badge room rule, computed for the whole fleet), and Maximize and
Close show their standard glyphs with an accessible name. Icons always carry
an accessible name ("UAV 2, camera" / "UAV 3, no camera, synthetic view").
The overflow menu opens downward under its button (today it opens upward,
`VehicleWindow.jsx:138`, which at the top right would run into the action
bar), stays inside the window (at most down to the window's bottom edge, then
it scrolls) and draws above the small swapped map; a pick, Escape or a tap
outside closes it, and Escape returns focus to the menu button.
Focus: Close removes the control that has focus, so focus moves to the picked
UAV's button in the picker; if that UAV has left the fleet, to the first UAV
button. (With an empty fleet the window is gone.) In monitor mode confirm cards and E-STOP are in the
sidebar, not over the map, so neither the window nor the small swapped map can
cover them. The top-right bar keeps its pre-PR #18 controls at 32 px (owner,
2026-10-18), without the follow selector; it wraps on a narrow pane
(`MonitorMapOverlay.jsx:64-78`) and reports its bottom edge, so the window
always starts 12 px below it.

#### Vehicle window

- Where: top right of the map pane, right edge 12 px from the pane edge, top
  12 px below the action bar's measured bottom (`onTopControlsBottom`, as v10
  measures it), monitor view only, whenever the fleet has a UAV; hidden in
  manual control (RC). Width min(432, pane width - 24) px. Its bottom stops
  12 px above the reset-view button (bottom right, `App.jsx:900-905`), so it
  never covers it; the picture shrinks to fit that height limit. It does not
  reach the RC button (top left) or the main map's credit line (bottom left).
  Two sizes: normal and maximized (the swap, below).
- Layout, top to bottom (viewing): header line; control row; a status row
  when needed; the picture (16:9, shrinking to fit the height limit). The UAV
  label stays at the top in both sources. A state with no picture (no camera,
  failed, lost, waiting for pose) collapses the picture area at normal size,
  so the window never shows a black box; maximized, the large area shows that
  state as text. Picker: the prompt line and the UAV buttons only, no
  picture.
- Which UAV: the UAV buttons and View camera pick it, for both sources. Only
  the operator picks: v10's automatic first pick and its move from "No
  camera" to the first UAV that starts advertising are removed; the window
  starts on the picker. Picking the selected UAV again does nothing.
- Source on a pick (owner, 2026-10-19): a UAV with a stream (camera icon)
  opens on Camera; a UAV with none (map icon) opens on Synthetic, marked as
  in the Synthetic view section. This is the only automatic entry into
  Synthetic; View camera always opens Camera. Once picked, the source changes
  only by the switch, Dismiss (below) or a synthetic failure: a stream that
  appears while on Synthetic only turns the icon into a camera; a stream that
  goes away while on Camera shows "Camera video lost", never a silent switch
  to Synthetic.
- The picked UAV leaves the fleet (removed from the backend's `active_ids`,
  `telemetryStore.js:128-133`): its button disappears and the selection stays
  (v10). Camera: "Vehicle link lost · UAV n" and "Camera video lost".
  Synthetic: the last pose holds with "View held · vehicle link lost". Both
  resume if the UAV returns; the window never switches UAV by itself. Close
  goes back to the picker.
- Camera states:
  - Stream: video with its live state.
  - **No camera · UAV n**: the operator switched a map-icon UAV to Camera. No
    Reconnect or Dismiss. If that UAV starts advertising, its video starts.
  - **Camera video lost · UAV n**: the shown stream went away. That UAV's other
    stream plays if it has one; else this state, with Reconnect video and
    Dismiss, until a stream returns. Dismiss opens Synthetic for the same UAV,
    as a pick of a map-icon UAV would (open question 1). Never a silent switch
    to another UAV.
  - The GCS learns of streams only from the camera protocol: a camera that is
    expected but never advertises shows the map icon; preflight reports it
    (`architecture_todo.md` 70).
- Deadlines: no first frame within `VIDEO_FIRST_FRAME_DEADLINE_S` (default 10 s:
  ICE setup plus one keyframe interval, which the display stream keeps at 2 s or
  less), or no new frame for `VIDEO_STALL_DEADLINE_S` (default 5 s, a few
  keyframe intervals), becomes the failed state. Measured by frame progress
  (`requestVideoFrameCallback`, fallback `getVideoPlaybackQuality`), not only the
  video element's waiting/stalled events. Both named in one frontend constants
  file, with fake-timer tests. Failed or codec not supported: a short reason and
  Reconnect video.
- Close: the camera reader closes and the synthetic widget is destroyed, since
  an unwatched stream costs radio bandwidth (v10's minimize did the same); the
  window shows the picker. v10's minimized handle and its saved state are
  removed.
- Maximize swaps the window and the map (QGroundControl, UgCS, DJI Pilot 2):
  the window takes the large box (pane width minus 12 px margins, from below
  the action bar to the bottom margin) and the map shrinks into a small box
  drawn above the large window's picture: at the picture's top right, 12 px
  under the window's last header, control or failure row and 12 px in from
  its right edge, width min(320 px, half the large box), 16:9, shrinking
  only to stay 12 px above the picture's credit line. It is never narrower
  than its own whole credit line (Cesium's ion logo, 138 px, does not wrap;
  145 px with padding and border): a pane without that room offers no
  Maximize, and a swap ends when the pane shrinks below it. It never
  covers the UAV buttons, the overflow menu, the switch, Restore, Close or a
  failure row. Maximize is offered only while
  viewing, not in the picker. The window's Maximize reads Restore while
  swapped. A tap anywhere on the small map swaps back: a transparent button
  covers it (accessible name "Show the map large"), so the small map is never
  panned, zoomed or picked, and no accidental drag becomes the saved startup
  view. While swapped the main map's `screenSpaceCameraController.enableInputs`
  is false, which also stops the inertia of a gesture made just before
  maximizing (`ScreenSpaceCameraController.js:511`). It is set back to true on
  the swap back. With no chase (v11), the only thing that can move the map
  while swapped is a coordinate click in the sidebar (`MissionStatusRow.jsx:54`,
  through `flyToRef`, which calls `camera.flyTo` directly,
  `useCesiumViewer.js:329-333`). While swapped it swaps back first, then
  flies, so the flight runs in the large map with the right aspect.
  Maximizing during any camera flight cancels it (`camera.cancelFlight`); a
  reset-view fit is rerun after the swap back (below), a coordinate flight is
  not (the operator just asked for the window).
  - CSS only: the map wrapper stays mounted and only its box changes. Never
    remount `CesiumMap` (the viewer cleanup does not destroy the viewer, so a
    remount leaks a WebGL context), never animate the size (Cesium resizes its
    canvas on its render tick, so a transition reallocates it every frame),
    never hide the map or size it to 0 (rendering would stop).
    Cesium picks up the new size on its next tick.
  - While swapped: the top-right bar stays usable (the window starts below
    it); the header's UAV buttons move the large window to another UAV, with
    the source its icon gives; reset view is hidden (a fit made in the small
    box uses its aspect ratio and is wrong after the swap back); markers keep
    their size, so a 3-UAV map is crowded: it is an overview.
  - Fits wait for the large map. v10's chase-to-Free auto-centre
    (`useFpvView.js:46-52`) goes with the chase, and reset view is hidden while
    swapped, so the one fit left to defer is a reset-view flight still running
    when the operator maximizes: it is cancelled and deferred (`useMapSwap.js`,
    which today cancels only flights it tracks in `fitFlyingRef`; v11 cancels
    any flight). A pending fit is dropped when RC takes the camera. On the
    swap back the map calls `viewer.resize()` (public, synchronous), then runs
    the pending fit once. A fit in the small box would use its aspect ratio and
    be wrong after the swap back; shrinking itself changes only the frustum, so
    a move end while small saves the unchanged pose.
  - Swap back on: Restore, a tap on the small map, Close (swap back, then the
    picker), a sidebar coordinate click (swap back, then fly), RC entry,
    leaving the monitor view, an empty fleet, a pane too short for the small
    map. A change to another UAV keeps the
    swap. Never saved: a reload starts unswapped.
- Credits: Cesium's credit line (the Cesium ion logo, which the engine treats
  as required, and the data attribution; Cesium ion terms 2.2.3) is never
  covered. Each Cesium view keeps its credits at its own bottom-left (Cesium's
  default). At normal size the window is at the top right, clear of the main
  map's credit line. Swapped, the small map sits at the top right of the large
  window's picture, clear of its bottom-left credits (maximized Synthetic) or of nothing
  (Camera video has no credit line). The small map's own credits stay visible
  inside it; its tap button blocks their "Data attribution" link until the
  swap back.
- Saved: nothing. v10's browser storage key `gcs-vehicle-window` (minimized or
  normal) is no longer read or written; a value left in a browser is ignored.
  Never the UAV, the source or the swap; a reload opens the picker.
- Automatic changes are only these: the source a pick gets from its icon, back
  to Camera on a synthetic failure, the swap ending on the events above (and
  on a pane too short for the small map), and
  the picker after an empty fleet. The window never moves to another UAV by
  itself.
- One reader per browser; the demo has one GCS, so no radio-wide arbitration in v1.
- Removed (v8): `VITE_VIDEO_WHEP_URL`, `videoConfig.js` and the single-feed
  wiring (no shim). `useWhepStream` and the vendored reader stay; `src/gcs/AGENTS.md` and
  the Jetson config notes are updated.

#### Synthetic view

Owner, 2026-10-18 (design C): a second Cesium view while the map keeps chasing;
2026-10-19: it is a source of the vehicle window; v11: the map no longer
chases, a UAV without a camera opens on it, and Synthetic maximized is the
large vehicle view.

- Selection: a pick of a map-icon UAV, Dismiss on a lost feed, or a tap on the
  switch; never on any other automatic path. Switching to Synthetic closes the
  camera reader (one reader per browser); switching back reconnects (fresh
  connection) and shows the kept camera state, a lost feed included. A stream
  appearing, failing or going away while on Synthetic never changes the
  source.
- Which UAV: the window's UAV, picked by the UAV buttons. A pick of another
  map-icon UAV moves the same widget to the new UAV's pose; a pick of a
  camera-icon UAV destroys it and opens Camera.

- Rendering: a second public `CesiumWidget` filling the picture, created on
  entering Synthetic (normal or maximized), destroyed on Camera, Close, RC,
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
  and sky box off. No entities; peers, docks and tracks stay on the main
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
  (`telemetryStore.js:69-70,102-133,146-149`: the store's per-UAV fresh flag
  is cleared on open and close and set by that UAV's next snapshot); the store replaces a vehicle's snapshot
  object on each update for it, so "replaced" needs no timer. Both checks go
  into the shared hold check, so RC FPV gets the same held reason. A UAV removed from the fleet
  has no source entry, which the source reports as "no pose", never as a throw.
  The shared source validates before it builds or advances a pose (shipped
  in v10: `poseFinite`, `fpvView.js:16`, used by `useUavMarkers.js:44`): it
  skips the update unless lat, lon, altitude, heading, roll and pitch are all
  finite numbers, so the model, RC FPV and the synthetic view keep the last
  valid pose and resume from it. One render request per
  telemetry update. The widget saves no camera, so a nose pose is never the
  startup view.
- Wrong-UAV guard: the picture shows the widget only while its pose belongs to
  the window's UAV. After a pick of another UAV, until that UAV has a valid pose, an
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
  pose unavailable** with the seconds held.
- RC: body FPV of the RC target on the main map with today's full-pane tag,
  frame and held banner (`FpvViewOverlay`, RC only); the window is hidden
  (`vehicleWindowVisible`). Today the map FPV reads the RC target through
  `followSysId` (`CesiumMap.jsx:291`), which choosing an RC vehicle sets and
  leaving RC clears (`useManualControlWiring.js:49,70`); with `followSysId`
  gone, the map FPV reads the RC target itself, and RC never changes the
  window's UAV. On RC exit the window comes back on the same UAV with the
  source its icon gives (open question 2).
- Map camera ownership: only RC takes the map camera. The saved map camera
  and the persistence suspension (`useCesiumViewer.js`) apply to RC only;
  with the chase gone, `cameraTaken` (today `followSysId != null` or RC,
  `CesiumMap.jsx:239`) is RC only.
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
| UAVs join (monitor view) | window shows the picker: "Select a UAV for streaming" and one button per UAV with its camera or map icon |
| Pick a camera-icon UAV (picker or header) | that UAV on Camera, fresh connection; the swap stays; the map does not move |
| Pick a map-icon UAV | that UAV on Synthetic, or "Waiting for pose · UAV n"; the swap stays; the map does not move |
| Close | swap back if swapped; reader closed, widget destroyed; the picker |
| Switch to Synthetic | camera reader closes; the window's UAV in the synthetic view, or "Waiting for pose · UAV n" |
| Switch to Camera | widget destroyed; that UAV's camera state ("No camera · UAV n" for a map-icon UAV), fresh connection |
| View camera on a pending confirm card | that UAV on Camera, fresh connection, also from the picker; the swap stays; the map does not move |
| Maximize (viewing only) | map and window swap; nothing reconnects or rebuilds |
| Restore, or tap on the small map | swap back; nothing reconnects or rebuilds |
| Sidebar coordinate click while swapped | swap back, then the map flies there |
| Maximize during a camera flight | the flight is cancelled; a reset-view fit reruns after the swap back, a coordinate flight does not |
| A UAV's stream appears or goes away | its icon changes; the shown source does not |
| Shown stream goes away (Camera) | that UAV's other stream, else "Camera video lost · UAV n" with Reconnect video and Dismiss; Dismiss opens Synthetic for that UAV |
| Telemetry disconnected, link lost or pose not finite (Synthetic) | the last pose holds with the held row; resumes on the next valid pose |
| Link lost (Camera) | "Vehicle link lost · UAV n", and "Camera video lost" if the stream goes too; its button reads "link lost" |
| Picked UAV removed from the fleet | button gone, selection kept; Camera: link lost and video lost; Synthetic: held; resumes if it returns |
| Widget fails, render error, WebGL context lost or pose callback throws | widget destroyed; source Camera with "Synthetic view unavailable" ("No camera · UAV n" for a map-icon UAV); the main map keeps rendering |
| RC entry | swap back; window hidden; widget destroyed; today's RC FPV of the RC target on the main map; the window's UAV is kept |
| RC exit, or back to the monitor view | RC: the map with the saved map camera; the window returns on the same UAV with the source its icon gives, normal size (open question 2) |
| The fleet becomes empty | swap back; window gone; selection cleared, so the next UAV brings the picker |
| Page reload | the picker; unswapped; the startup view is the saved map camera |

- Code impact of v10 (shipped, PR #20): `useFpvView` shrank to RC FPV and
  RC-only map camera ownership; the top-right FPV toggle went; a `noCamera`
  kind; Dismiss on a lost feed goes to it.
- Code impact of v11 (no shims):
  - The follow selector (`MonitorMapOverlay.jsx:111-136`), `followSysId`
    (`App.jsx:134`, its clear effect `App.jsx:152-159`) and the map chase go:
    `useFollowUav` keeps only the RC first-person camera, fed with the RC
    target (today through `followSysId`, `CesiumMap.jsx:291`, which RC sets
    and clears, `useManualControlWiring.js:49,70`); `useFpvView` loses the
    chase-to-Free auto-centre; `CesiumMap`'s `cameraTaken` becomes RC only
    (`CesiumMap.jsx:239`); the "Camera follow" and "Free" strings
    (`locales/en.json:404-405`, `hy.json`) go.
  - `useVehicleWindow`: no first pick (`pickFirstFeed`, `pickFirstUav`,
    `auto` go), a null selection is the picker; a pick sets the source from
    the UAV's streams; Dismiss goes to Synthetic; minimize, restore and the
    saved state (`WINDOW_STORAGE_KEY`) go; Close clears the selection.
  - `VehicleWindow.jsx`: the picker state; badges gain the camera or map icon
    and the "link lost" text; the minimized handle goes; Minimize becomes
    Close. `vehicleWindowLayout.js`: the top-right anchor and the bottom limit
    at the reset-view button; the badge room and overflow menu stay, the menu
    opening downward with Escape and outside-tap close; the swapped small-map
    box moves inside the large window's picture.
  - `useMapSwap` cancels any camera flight on maximize (not only its own
    fits), and the sidebar's fly-to swaps back before it flies.
  - `src/gcs/AGENTS.md` ("bottom-left vehicle window") is updated.
  - Tests of the removed behaviour go with it; the test plan below lists the
    new ones.
- Deferred: full screen; a camera-direction synthetic view from gimbal attitude
  + FOV (needs verified mount axes and stream-to-device association); resizing
  the window by drag; a "follow on map" option if operators ask for the chase
  back.

#### Found by the walkthrough, outside this design

Logged in `architecture_todo.md` (67-72): a pending-confirm cue and card order
by time left; the confirm card says what happens at zero (code and demo config
reject on expiry: `aas_params.lua:19`, `eval_gcs_demo_config.py:144`); E-STOP
and the right UAV's Cancel task reachable when cards stack; a per-UAV camera
row in preflight, including an expected camera that never advertises; the
still's capture time and its dock; operator status clarity (what the companion
indicator measures, mission phase, dock assignment, honest completion wording).

Unchanged and not part of this design: confirm cards and their countdown stay
over the map in RC (`App.jsx:855-886`). Deferred: an audit of who decided each
confirm (operator or timeout), recipient authorization for real delivery,
fleet separation checks (a mission-level verification), and outdoor
glare/glove acceptance (the 44 px and 14 px rules are built now).

#### v12: slim window behind a video icon (approved 2026-10-20)

Owner, 2026-10-20: "collapse the new gcs, also the design need improvement
current is very bulki optmiize it to be cool matching to our current app and
also optmized for siyi unirc10pro." Mock:
`docs/design/gcs_video_v12_mock.html` (static, frames at CSS-pixel size;
frame 1 is v11 for comparison). v12 changes only the window's form; every
v11 rule not named below stays (who picks the UAV, sources, live state and
deadlines, loss wording, swap and its 145 px small-map rule, credits, no
map chase, nothing saved, WebGL release on unmount). Codex reviewed the
first draft in 3 rounds and this one in round 4 (approve with changes, 8
points); all are folded in.

Target device (SIYI UniRC 10 Pro User Manual v1.2,
`res.siyi.biz/oss/other/2026/07/16/UniRC_10_Pro_Handheld_Ground_Station_User_Manual_v1_2_57e418e4.pdf`,
linked from `siyi.biz/en/product/hand-gcs/uni-rc-10-pro/download`; spec
page `siyi.biz/en/product/hand-gcs/uni-rc-10-pro/spec`; both refuse plain
scripted fetches with HTTP 403, a browser user agent gets them):

- 10.1 in, 1920×1200, 2000 nit, 60 fps, adaptive brightness (manual p.13,
  p.23); Android 13, 4 GB + 64 GB (p.24); IP54, 2.6 kg (p.23). Some resellers
  write "1080P"; SIYI's manual and spec page say 1920×1200.
- Pre-installed (6.1, p.130): UniGCS, SIYI QGroundControl, Ping Tools.
  Other apps install as APK files from TF card, USB drive or Type-C (6.2,
  p.130-131). The manual names no browser; whether one ships is not
  stated. Deployment assumption: a current Chromium-based browser, checked
  on the device first.
- Touch: the manual only says tap and slide on the "touchscreen" (p.111-112);
  touch type, multi-touch and glove use are not stated.
- Not published: the display density, so the CSS viewport. 1920×1200 on
  10.1 in is 224 ppi. Test scenarios: 960×600 CSS px (density 2,
  1 px = 0.23 mm; map pane 640×504 beside the 320 px sidebar, the tight
  case) and 1280×800 (density 1.5, 1 px = 0.17 mm); browser and system bars
  can make the real viewport smaller. Before the field test, measure on the
  device: see "Device check" below.


Owner, 2026-10-20, on the first v12 mock: "our panel from right always
keep design same as we have. the buttons are bulki not as good as our
current, also make collapsable on top of the icons like others and can show
as video icon, check to always have good design." So: the app's layout
stays as it is (sidebar on the right; the first mock drew it on the left);
every control in the window is the app's own 32 px map button, with no
larger touch size; and the window collapses into a video icon in the
top-right action bar, a toggle like the vision, coverage and trail icons.
The mock is drawn over screenshots of the running app (top bar, sidebar,
action-bar icons); only the map area is painted, because Cesium renders
black in a headless capture.

What changes:

1. **Video icon in the action bar.** A `MapIconBtn` with a video-camera
   glyph joins the top-right action bar at its right end, right above the
   window's corner. It toggles the window as the vision and trail icons
   toggle their layers: active style (accent glyph, accent border, 20 %
   accent fill) while the window is open, plain while it is closed. Accessible
   name "Vehicle video" (stable), `aria-pressed` for the state; the
   tooltip says "Show vehicle video" / "Hide vehicle video". With an empty
   fleet the whole action bar is not drawn (`App.jsx` mounts
   `MonitorMapOverlay` only with a fleet), so there is no icon and the
   window is closed. The window has no collapse or close button of its own.
   Closing stops the camera reader and unmounts the synthetic widget (its
   WebGL context released), as v10's minimize did (owner decision 6); the
   picked UAV and source are kept, and opening again plays them with a fresh
   connection. Closing while swapped swaps back first. View camera on a
   confirm card opens the window and plays that UAV's camera; while closed a
   card never says "Viewing". Open or closed is kept across RC and view
   changes in the session, never saved; a reload starts closed with no UAV
   picked, and so does a fleet that returns from empty (open question 1).
2. **One slim bar.** Viewing: [UAV chip ▾] … [Camera | Synthetic]
   [Maximize/Restore]. All 32 px, `MapIconBtn` style. The source switch is
   two glyph buttons (camera, map) with `aria-pressed`; the selected one has
   `MapIconBtn`'s active style (accent border, glyph and fill), the other
   its plain style, as the action bar's toggles; the picture tag names the
   source in text, so the cue is not colour alone. Every
   glyph button has an accessible name and a tooltip. Close goes: the icon
   closes the window and the chip changes UAV (open question 2). The live
   state and the stream name move onto the picture's top-left tag ("● LIVE ·
   tracking overlay", "CONNECTING", "NO FRAMES 7 s"), where v11 already put
   the stale tag and Synthetic its "SYNTHETIC · not camera" tag; the tag
   wraps (never cut), and while swapped it wraps before the small map as in
   v11. Synthetic keeps its 3 px accent frame. Warning rows keep v11's text
   under the bar: "Vehicle link lost" and "Synthetic view unavailable" sit
   alongside any picture that still plays (video keeps playing on link loss;
   a held synthetic view keeps its last picture); only no camera, video
   lost, failed and Synthetic waiting for its first pose have no picture.
   Reconnect video and Dismiss are 32 px text buttons in the plan button's
   style.
3. **UAV chip and list.** The chip is a 32 px dark button with the UAV's
   zone colour as a 10 px swatch, "UAV n" (12 px bold, as the plan button)
   and ▾; "link lost" shows after the name in `colors.error`. Text never
   sits on a zone colour (white on orange, cyan or silver is under 4.5:1).
   Chip width: content, at most 160 px. The swatch, "link lost" and ▾
   never shrink; only the name ellipsizes, and the full name is in the
   chip's accessible name and in the list. A tap opens the UAV
   list (disclosure pattern: `aria-expanded` on the chip, the rows are plain
   32 px buttons with `aria-pressed`, in tab order; no `role="menu"`): one
   row per UAV with the camera or map glyph, the swatch, "UAV n",
   "camera"/"synthetic" and "link lost"; the selected row is outlined and
   checked. It opens downward, draws above the small swapped map, closes on
   a pick, Escape or a tap outside. Its height limit is the room from the
   chip down to the window's bottom limit (normal: the margin above reset
   view; swapped: the window's bottom), not the window's rendered bottom;
   past it, it scrolls. It replaces the header's UAV buttons and the
   overflow menu, so the badge room calculation, number-only badges and
   `NARROW_PANE_PX` go. Cost: a switch is two taps instead of one (open
   question 3).
4. **Picker.** Open with no UAV picked: a line "Select a UAV for
   streaming" (12 px), then 32 px UAV buttons (glyph, swatch, "UAV n",
   "link lost"), wrapping, as v11.
5. **Size and style.** Width min(320, pane width - 24) px (v11: 432); the
   picture is 318×179 px at 320. The window uses the map overlay style
   already in the app (`MapIconBtn`, the plan button):
   `rgba(22, 33, 62, 0.85)` with `backdrop-filter: blur(4px)`, a 1 px
   `colors.border` edge, 6 px radius, 4 px gaps, white glyphs, the accent
   for the active state. No new colours or sizes. The blur sits on a
   background layer behind the content, never on the section or a list
   ancestor: a backdrop filter makes a stacking context, and the swapped
   window must not have one so its UAV list can draw above the small map
   (`VehicleWindow.jsx:18-25`).
6. **Touch on the UniRC (owner, 2026-10-20: the app's sizes).** Controls
   stay 32 CSS px with 4-6 px gaps, as the action bar. Under the assumed
   densities that is 7.3 mm at 2 and 5.4 mm at 1.5; 48 CSS px would be
   10.9 / 8.2 mm. Android recommends touch targets of at least 48 dp
   (support.google.com/accessibility/android/answer/7101858); how the
   UniRC's browser maps CSS px to dp is not published. Risk: missed taps with gloves or at
   density 1.5; the device check below measures it, and a larger size would
   be a change for the whole action bar, not this window alone.
   `TOUCH_TARGET_PX` (44) goes.
7. **Bar fit.** The viewing bar needs chip 160 + switch 64 + 32 + three
   4 px gaps + 8 px padding = 276 px; a 320 px window has 318 inside, so
   from a 302 px pane it always fits. Narrower, the bar wraps by itself
   (flex-wrap) when its contents do not fit: the chip on the first line, the
   switch and Maximize on a second; buttons never shrink.
8. **Short pane (todo 87 b).** At normal size the picture keeps its width
   and shrinks in height to the window's limit, never below 82 px, the 16:9
   height of `SMALL_MAP_MIN_PX` (145 px, the narrowest box that holds a
   whole Cesium credit line). With less room the picture area is not drawn
   and the reader or widget is unmounted (no hidden stream, WebGL released);
   the selection stays, a card does not say "Viewing", and a row reads "Too
   little room for the picture". When room returns, a fresh session starts.
   No picture mounts before the window has measured its bar (the chrome is
   unknown until then, and again after closing). A camera failure stays
   latched for its session outside the picture, so its own row taking the
   picture's room never hides it or reconnects; only Reconnect video does.
   Maximize follows its own v11 rule (the swapped layout reserves less at
   the bottom), so it can still be offered here and gives the large picture.
   Both decisions are computed from the chrome their own layout needs (the
   bar plus the warning rows that stay), never from the measured height
   with the "Too little room" row in it; otherwise that row would hide
   Maximize and delay the picture's return. Todo 87 a (GlobeDepth width 0
   at load) is the main map and stays a separate trace.
9. **Focus.** The bar sits outside the reader subtree that is keyed by
   session, so its buttons are not remounted by a switch. The video icon
   keeps focus when it opens or closes the window; a pick in the picker or
   the UAV list moves focus to the chip; a source switch keeps focus on the
   pressed source button; Restore that
   disappears (pane too short for the swap) moves focus to the chip; Close's
   focus rule goes with Close.

Device check before the field test: on the UniRC, open the GCS in the
browser and record browser and version, `devicePixelRatio`, `innerWidth`,
`innerHeight`, and whether the 32 px controls are hit reliably by tap
(bare hand and glove).

Code impact of v12 (no shims):

- `utils/vehicleWindowLayout.js`: `DOCKED_WIDTH_PX` 320;
  `PICTURE_MIN_H_PX` from `SMALL_MAP_MIN_PX` and a `pictureFits`
  flag separate from `canSwap`; the list's height limit from the window's
  bottom limit; `badgeLayout`, `FULL_BADGE_PX`, `NUMBER_BADGE_PX`,
  `MENU_BUTTON_PX`, `NARROW_PANE_PX`, `TITLE_MIN_PX`, `STATE_PX`,
  `TOUCH_TARGET_PX` and `narrow` go.
- `useVehicleWindow`: `open` (starts false; back to false on an empty
  fleet), `toggleOpen` (closing swaps back and stops); `viewCamera` opens;
  `close` goes; `viewing` is false while closed or without picture room;
  `session` changes on opening and when picture room returns.
- `MonitorMapOverlay.jsx`: the video toggle at the end of the top-right
  bar (props: open, onToggle). The bar grows by 38 px; its measured bottom
  (`onTopControlsBottom`) already places the window when it wraps.
- `VehicleWindow.jsx`: the one bar, the UAV chip and list (from
  `OverflowMenu`), the glyph switch, state on the picture tag, the blur
  layer, the focus moves above; `Header`, `ControlRow`, the number badges
  and the Close focus effect go; buttons reuse `MapIconBtn` (it gains
  `aria-label`/`aria-pressed` pass-through; its styles are unchanged). `SyntheticView.jsx`: its tag
  takes the shared tag style.
- `App.jsx`: mounts the window only while open; passes the toggle state to
  `MonitorMapOverlay` and `pictureFits` to the window; `useMapSwap.js`
  unchanged (closing calls `unswap` first).
- Locales `en.json`, `hy.json`: "Vehicle video", show/hide vehicle video,
  "Too little room for the picture"; the close string goes.
- `src/gcs/AGENTS.md`: the window's description.

Tests (vitest unless noted):

- Layout (`vehicleWindowLayout.test.js`; its badge and overflow tests are
  replaced): width 320 and pane - 24 below 344; bar fit and wrap at 302;
  picture floor 82 px at pane heights 236 (fits) and 235 (does not fit;
  swap still offered, the no-room row not counted), for one action-bar row
  and no warning rows;
  the picture returns at the same boundary growing and shrinking; list
  height limit; swap rule unchanged.
- `tests/gcs/test_vehicle_window_placement_js.py`: the locale key list
  (`close` out, the new keys in) and the wiring checks (the toggle in the
  action bar).
- Hook: starts closed; closing closes the reader and unmounts the widget,
  swaps back; opening gives the same UAV and source with a new session;
  View camera opens; empty fleet closes; RC keeps open or closed; no picture
  room makes `viewing` false and its return a new session.
- Overlay: the video toggle's stable name, tooltip, pressed state, at the
  bar's right end. Wrap chain: a ResizeObserver change from one action-bar
  row to two and back moves the window and changes swap eligibility through
  `App`, in both locales, with the plan entry shown and hidden, and on
  reopening after a resize while closed.
- App: empty fleet → no bar, window closed; a returning fleet starts
  closed with no UAV picked.
- Window: bar controls' accessible names and 32 px size; switch pressed
  state and its box; the UAV list (open, pick to another camera UAV, Escape,
  outside tap, link lost, selected checked, scrolling in no-camera and
  waiting-for-pose states); the focus moves above (the actual
  focused element after a picker pick, a list pick, a source switch and a
  lost Restore); a long UAV name ellipsizes with "link lost" intact, in
  both locales; video keeps playing with
  "Vehicle link lost"; Camera with the synthetic-unavailable notice; LIVE
  and stream name on the tag; both locales.
- Live (live-tester) at 1920×1080 and at 960×600 and 1280×800 with touch
  emulation: toggle open/close, a 20× toggle loop on Synthetic (WebGL
  contexts stay released), the UAV list over the swapped small map, the bar
  wrap on a narrow pane, the small map's credits; screenshots compared with
  the mock.

### 6. SITL vs real hardware

| Setup | Stream source | Discovery |
|---|---|---|
| Plain SITL, default | none | nothing announced -> map icons; a pick opens Synthetic ("No camera · UAV n" only after a switch to Camera) |
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

#### v12.1: the source belongs to the UAV (approved 2026-10-20)

Owner, 2026-10-20, on the built v12: "what if one uav has stream the other
doesn't how should we resolve it current global button seems swithcing for
all? come up with good design".

What the code does now: the source is part of the current selection
(`useVehicleWindow.js`, `pickOf`), not remembered per UAV; a pick of another
UAV sets it from that UAV's icon (picking the selected one is a no-op), so it
is not global. Three things make a mixed fleet confusing:

- The Camera | Synthetic switch sits at the far end of the bar, apart from
  the UAV chip, so it reads as a fleet-wide setting.
- Camera stays enabled on a UAV with no stream; a tap gives "No camera ·
  UAV n" and no picture (the switch and the list still work).
- A synthetic failure falls back to Camera, the same empty state on such a
  UAV, and an automatic source change.

Design (Codex round 1 changes folded in):

1. **One UAV group.** The chip and the switch join into one bordered
   group, [● UAV 1 ▾ | camera | map], 32 px high with 1 px dividers (fit
   below); Maximize stays
   apart at the right end. The group reads "this UAV's camera or map" and
   never splits: when group and Maximize do not fit on one line, Maximize
   wraps to a second line; when the group alone does not fit, the chip's
   name ellipsizes (the swatch, "link lost", ▾ and both source buttons keep
   their size). The switch's accessible name names the UAV ("Video source
   · UAV 1"). The list still opens from the chip and draws above the
   group's border and the small map. The joined borders are drawn by the
   group: `MapIconBtn` gains a `joined` prop (no border or radius of its
   own; the group draws the outer border and the dividers) and an
   `unavailable` look, since today it overwrites a passed `style`. No new
   colours.
   Fit (replaces v12 item 7): chip 160 + Camera and Synthetic 64 + one
   4 px gap + Maximize 32 + 8 px padding = 268 px, so from a 294 px pane
   the bar is one line. The group's border is an inset outline and the
   dividers sit inside the 32 px buttons, so the group is 32 px high and
   takes no extra width.
2. **Camera availability is the selected UAV's.** Camera is available when
   the selected UAV advertises any usable stream (the resolver's rule: its
   selected key, else its default stream; never another UAV's, never link
   health). Unavailable, Camera shows the camera glyph slashed and dimmed
   (a cue beyond colour), keeps its place (the bar does not change width
   between UAVs) and stays focusable. Activating it by mouse, touch, Enter
   or Space never changes the source; it shows the row "No camera stream on
   UAV 3" under the bar (`role="status"`, also the button's
   `aria-describedby`), which clears on the next source change or pick. Its
   tooltip says the same for mouse users. When the UAV starts advertising,
   Camera becomes available; nothing switches.
3. **Only the operator changes the source.** No automatic change remains:
   - A synthetic failure keeps Synthetic selected. The failed widget
     unmounts (WebGL released); a latched failure, held outside the widget,
     shows "Synthetic view unavailable" with Retry (plan-button style), and
     swapped the large area states it in text. Resizing, a stream appearing
     or room returning never clear it or retry; Retry does (focus then goes
     to the Synthetic button). Camera, if available, stays one tap away.
   - Return from RC or another view keeps the selected source (v12 re-picked
     it from the icon); the swap still ends.
   - Dismiss (after the last stream is withdrawn) is the operator's change
     to Synthetic; a stream coming back does not switch back.
4. **Camera loss, unchanged and made explicit.** Losing the selected
   stream moves to the same UAV's other stream; losing its last stream
   shows "Camera video lost" with Reconnect video (disabled) and Dismiss,
   and Camera stays pressed though unavailable until the operator changes
   source; a failed reader whose stream is still advertised offers
   Reconnect video only. Link loss alone never makes Camera unavailable.
5. **Entry points.** A pick of another UAV opens what its icon shows (open
   question 1). View camera is offered only for a UAV with a stream (as
   now, `App.jsx` `videoActionFor`), and the hook also refuses it without
   one. A UAV that leaves the fleet stays selected (only an empty fleet
   clears it): Camera unavailable, link and video lost shown, Synthetic
   holds; when it returns, nothing switches.
6. **"No camera · UAV n" goes** as a reachable state: Camera cannot become
   selected without a stream, and nothing falls back to it.

Code impact: `VehicleWindow.jsx` (the joined group and its wrap; Camera's
unavailable state and status row; the latched synthetic failure with Retry
and its swapped area; the "No camera" row and area go), `useVehicleWindow.js`
(`chooseSource` refuses Camera without a stream and sets the notice;
`syntheticFailed` keeps Synthetic and latches; `retrySynthetic`; the RC
return keeps the source; `viewCamera` refuses without a stream),
`MapIconBtn.jsx` (the `joined` prop and the `unavailable` look, with tests
that its other uses are unchanged), locales (`vehicleWindow.source.noCameraFor`, `source.groupFor`,
`retry`; `noCamera` goes if unused).

Tests: Camera unavailable on a map-icon UAV by click, Enter and Space (no
reader, no source change, the status row and its `aria-describedby`); it
becomes available when that UAV advertises, without switching; another
UAV's stream never makes it available; the group's name follows the UAV;
synthetic failure stays on Synthetic (widget unmounted, failure latched
across a resize and a new stream, Retry remounts and focuses Synthetic,
swapped text area); RC return keeps Synthetic on a camera UAV; Dismiss then
a returning stream does not switch; View camera refused without a stream;
the departed UAV; the group never splits at 256, 280 and 320 px panes with a
long Armenian name and "link lost" (Maximize wraps first, then the name
ellipsizes). Live: the vitest cases carry the mixed fleet, because
`simulation.video_test_source` is one switch for every UAV
(`sim_video_companion.py`); the live check starts the companions with it on,
turns it off, restarts one UAV's companion so that UAV advertises none,
turns it on again (it also sets the WHEP port in every playback URL), and
waits for that UAV's old advertisement to expire before checking. If the GCS has no per-UAV companion restart, the live mixed case is
reported as not run.

Approved by the owner 2026-10-20 with the recommended answers to both
questions below (decision 9), after the mock
(`docs/design/gcs_video_v12_1_mock.html`). The Synthetic icon stays the
folded map; other glyphs were shown and the owner chose it: "OK use the
Today for now".

Questions for v12.1 (owner):

1. **Remember the source per UAV.** Recommended: no; a pick of another UAV
   opens what its icon shows, and the list row says which (close/reopen and
   RC return keep the current source, design 3). Alternative: remember the
   last source picked for each UAV in the session (never saved), so
   UAV 1 on Synthetic, then UAV 2, then UAV 1 again gives Synthetic.
2. **Camera without a stream.** Recommended: shown but unavailable, with
   the reason in its tooltip. Alternative: hidden, so the bar is narrower
   for that UAV but changes width between UAVs.

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
- Frontend (vitest): the window at the top right, 12 px under the measured
  action bar (also when the bar wraps) and 12 px above the reset-view button,
  in monitor view whenever the fleet has a UAV, hidden in manual control and
  outside the monitor view, gone with an empty fleet; it starts on the picker
  ("Select a UAV for streaming", one button per UAV), also when UAVs have
  streams (no automatic pick), and a reload starts there; each button shows the
  camera icon when that UAV advertises a stream and the map icon when not, and
  the icon follows announcements; "link lost" text on a UAV with a lost link;
  accessible names carry the UAV and its icon meaning; a camera-icon pick opens
  Camera and a map-icon pick opens Synthetic, from the picker and from the
  header, also while swapped (the swap stays); picking the selected UAV does
  nothing; View camera always opens Camera; a stream appearing while on
  Synthetic changes only the icon, and a stream going away while on Camera
  shows "Camera video lost", never Synthetic by itself; Dismiss opens
  Synthetic for that UAV; Close closes the reader, destroys the widget and
  shows the picker with focus on that UAV's button; no follow selector in the
  bar and no chase: no pick, View camera or switch moves the map camera or
  changes the saved camera, and none changes RC; no minimized handle, nothing
  read from or written to browser storage; first-frame and stall deadlines
  (peer connected but no frames; frames then freeze; frames stop while the
  element raises no waiting/stalled event); codec-not-supported and error
  states with Reconnect video; stream removal: the same UAV's other stream,
  else "Camera video lost · UAV n", never another UAV; the picked UAV removed
  from the fleet: button gone, selection kept, link lost and held, resumes on
  return; Maximize absent in the picker; maximize and restore keep the same
  map element (no remount, no animation, never size 0) and the same reader;
  the small map at the top right of the picture, under the last header,
  control or failure row, covering no window control, also at 375 px and with
  a failure row; the overflow menu opens downward inside the window, above the
  small map, closes on pick, Escape or an outside tap, and Escape returns
  focus to its button; Close after the picked UAV left the fleet focuses the
  first UAV button; the button over the small map swaps back
  and passes no input to the map; maximizing during inertia or an active
  touch gesture saves no moved pose, and camera inputs are back on after the
  swap back; reset view hidden while swapped; a reset-view flight running
  when maximizing is cancelled and rerun once after the swap back with the
  large pane's aspect, and dropped on RC entry; a sidebar coordinate click
  while swapped swaps back, then flies, and maximizing during a coordinate
  flight cancels it without a rerun; Close, RC entry, leaving the monitor
  view and an empty fleet each swap back; every row of the transition table.
  Synthetic: entered by a map-icon pick, Dismiss or the switch, never on any
  other automatic path; SYNTHETIC state and tag, accent frame, never LIVE;
  held rows with the time held; the telemetry socket closing and reopening
  with no snapshot in between keeps holding with "telemetry disconnected", and
  the next snapshot for that UAV resumes it; "Waiting for pose · UAV n" after
  a pick of another UAV until that UAV has a valid pose. The widget (Cesium
  mocked): one per mount under StrictMode, `baseLayer: false`, its own terrain
  provider instance (never the main map's) that follows the Ion terrain
  upgrade, no imagery layer, no entities, inputs off, the shared pose helper
  gives today's RC FPV values, the shared pose source (real `useUavMarkers`
  and `lerpAnim`) skips a sample with a missing or non-finite lat, lon,
  altitude or attitude, keeps the last valid pose and resumes from it while
  the main map keeps rendering, no pose applied while the snapshot fails the
  hold check or the copy is not finite, a missing source entry reads as "no
  pose", one render per update, the same widget on a pick of another map-icon
  UAV, unsubscribed and destroyed on unmount, and a create error, render
  error, context loss or a pose-callback throw each return to Camera once with
  the notice, never touching the main camera or stored view. A real
  `CreditDisplay` (Cesium not mocked; Ion and Bing endpoint metadata mocked
  and cached) with the main map and the widget rendering at once, then the
  widget torn down: the main map keeps its Bing and terrain attributions and
  ion logo in its own container throughout, and the widget shows its own
  terrain attribution and ion logo. RC: the map FPV follows the RC target
  while the window shows another UAV; choosing the RC vehicle never changes
  the window's UAV; RC exit restores the saved map camera and brings the
  window back on the same UAV with the source its icon gives; RC FPV keeps
  its SYNTHETIC tag and the new held reason; a nose pose is never stored as
  the startup view. View camera on a pending confirm card picks that UAV on
  Camera, also from the picker, keeps the swap, shows "Viewing" only when that
  UAV's camera is shown, is absent on approved cards, when that UAV has no
  stream and during RC; tapped midway through a countdown it keeps the
  original deadline, expiry runs the existing timeout action once, the tap
  calls neither approve nor deny. LIVE dot only while frames arrive, "no
  frames for N s" after; "Camera video lost" and "Vehicle link lost" shown
  separately, both when both. `npm test`, `npm run build`.
- Layout: at desktop size and a 375 px wide map pane (a 695 x 812 touch
  viewport: the 320 px sidebar does not shrink, `styles.js:92`), with 3 UAVs, a
  UAV with two streams, the window in the picker, viewing and maximized, and
  three cards in the sidebar: the window under the wrapped bar and above the
  reset-view button, picker buttons wrapping, header on one line (buttons or
  the menu), 44 px window targets, the map buttons fit with the plan button
  showing, the RC button and Approve/Deny and E-STOP not covered, the Cesium
  credit line visible in every state, no overlap (vitest layout contract plus
  a `live-tester` screenshot check).
- Live (`live-tester`): 3-UAV SITL with the test source, from two slots at once
  (each plays its own slot's patterns): the window opens on the picker with
  camera icons; each UAV pick plays its labelled pattern and the map does not
  move; killing a path's FFmpeg shows the failed state and Reconnect video
  recovers; maximize, pick another UAV while maximized, tap the small map to
  swap back, Close back to the picker, then View camera on a confirm card
  opens that UAV's camera. With the source off: map icons on every UAV;
  picking UAV 1 opens its synthetic view, no console errors; pick other UAVs
  while they fly, link loss shows the held row, maximize Synthetic, switch to
  Camera ("No camera · UAV n"), RC entry and exit (the window comes back on
  the same UAV, Synthetic), force a WebGL context loss on the widget canvas
  (back to Camera with the notice, the map keeps rendering), then reload the
  page: the picker and the saved map view. Screenshots at desktop size and
  under 480 px. With the test source on: Synthetic by the switch replaces the
  video, Camera reconnects, View camera from Synthetic shows that UAV's
  camera. Then the load gate (Synthetic view section), on a quiet PC.
- Hardware (later, with the owner): Jetson H.264 stream over the radio from the
  laptop and from a second device (WHEP OPTIONS/POST accepted from the real GCS
  origin): glass-to-glass latency (on-screen clock),
  bitrate, and vision capture timing unchanged while the GCS watches.

## Open questions for v11 (owner; answered 2026-10-20)

Settled by the owner on 2026-10-19: the window sits at the top right under
the action bar and holds the UAV picker ("Select a UAV for streaming"); each
UAV shows a camera icon or a map icon; a map-icon pick opens Synthetic by
itself; the follow selector and the map chase go (so the map never follows a
UAV); no live preview tiles. The design above is written with each
recommended answer below; the owner approved all six on 2026-10-20.

1. **Dismiss on "Camera video lost".** Recommended: it opens Synthetic for
   the same UAV, as a pick of that UAV now would (its icon has turned into a
   map). Alternative: "No camera · UAV n" as in v10, Synthetic one tap away.
2. **Back from RC or from another view.** Recommended: the window returns on
   the same UAV with the source its icon gives, normal size. Alternative: back
   to the picker.
3. **Hiding the window.** Recommended: no minimize in v1; the picker is one
   line plus a row of buttons, and Close stops the picture. If the corner
   proves needed for the map, a collapse comes back later.
4. **Lost link on the buttons.** Recommended: a "link lost" text on that UAV's
   button, never colour alone, so the picker shows the same warning the
   window does.
5. **Picked UAV leaves the fleet.** Recommended: as in v10 (approved): its
   button goes, the window keeps the selection with "Vehicle link lost" or the
   held view, and resumes if it returns; Close gives the picker.
6. **Handheld.** Recommended: on a narrow pane the window spans the width
   minus 24 px under the wrapped bar, header buttons show icon and number with
   an overflow menu (v10 rule), and the picture shrinks to stay above the
   reset-view button.

## Open questions for v12 (owner)

Answered 2026-10-20 with each recommended answer (decision 8); the v12
section above is written that way.

1. **Start state.** Recommended: closed (the video icon plain), no UAV
   picked; the map is clear until the operator taps the icon. Alternative:
   open on the picker, as v11.
2. **Close.** Recommended: removed; the video icon closes the window and the
   UAV chip changes UAV. Alternative: keep Close (back to the picker) in the
   bar, one more 32 px button.
3. **Switching UAV.** Recommended: the UAV chip and its list, two taps.
   Alternative: v11's row of UAV buttons in the bar, one tap, which needs a
   wider window or a second row again.
4. **Where the video icon sits.** Recommended: the right end of the action
   bar, right above the window. Alternative: first icon after Edit plan.

## Owner decisions

1. Approach approved (2026-10-18): camera protocol discovery, browser WHEP
   straight to each aircraft's MediaMTX, one feed at a time, FPV toggle for
   Cesium FPV of any UAV (v10: replaced by the Synthetic source, decision 5).
2. D1 (a) (2026-10-18): the follow selector moves only the map and FPV; badges
   and View camera move the video; follow picks only the first feed (v10: the
   follow selector moves only the map; decision 5). Reversed by decision 7 (v11).
3. Display codec (2026-10-18): the Jetson publishes a software H.264 display
   stream; the ZR10 and the vision input stay untouched (Jetson CPU measured
   first).
4. v9 direction (2026-10-18): design C, FPV in the bottom-left panel while the
   map keeps chasing.
5. v10 direction (2026-10-19): one bottom-left vehicle window that the
   operator minimizes or maximizes; badges pick the UAV for both sources and
   the follow selector moves only the map; maximize swaps map and window; plain
   SITL opens on "No camera" (v11, decision 7: the follow selector goes,
   the window moves to the top right). The top-right FPV button goes; the synthetic
   source is named Synthetic, chosen by hand only and never saved; only
   minimized or normal is saved.
6. v10 approved (2026-10-19) with the recommended answers: minimizing stops
   the camera video and the synthetic view (restore reconnects); the
   synthetic view shows shaded terrain without imagery. A second Cesium view
   stays behind the load gate (fallback: a 2D attitude indicator).
7. v11 direction (owner, 2026-10-19; reverses decisions 2 and 5): one control
   picks the UAV. Reason: the follow selector and the window's badges
   duplicated each other. Final form the same day: the vehicle window sits at
   the top right under the action bar; with no UAV picked it reads "Select a
   UAV for streaming" and lists the UAVs, each with a camera icon (stream
   advertised) or a map icon (none); a pick opens Camera or Synthetic as the
   icon shows; Synthetic then Maximize gives the large synthetic view; the
   follow selector and the map chase go; no live preview tiles (one stream at
   a time, no extra radio load). Approved by the owner on 2026-10-20,
   with the recommended answers to open questions 1-6.
8. v12 approved (owner, 2026-10-20): "Approved: v12 design with the
   recommended answers to the four open questions." The window starts
   closed behind a video icon at the right end of the action bar; Close
   goes; the UAV chip and its list switch UAV in two taps. The larger touch
   size for the whole action bar is todo 94.
9. v12.1 approved (owner, 2026-10-20): "approved just have better icon
   this map I don't like much, but first check with me", then "OK use the
   Today for now". The chip and the source switch form one UAV group;
   Camera without a stream is shown but unavailable; only the operator
   changes the source; no per-UAV memory; the Synthetic icon stays the
   folded map.
