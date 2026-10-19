# GCS video streams per vehicle

Status: design v8 approved by the owner 2026-10-18 (D1 (a): follow moves only the map; display codec: software H.264 on the Jetson) and shipped as PR #18. v10 (2026-10-19: one bottom-left vehicle window with a Camera / Synthetic switch, minimize and maximize-swap; it replaces the v9 draft, design C) approved by the owner 2026-10-19 (decision 6) and shipped as PR #20. **v11 draft** (owner request, 2026-10-19): the selector at the top right opens the vehicle window for the UAV it picks, and Free closes it; this reverses D1 / decision 5. Written with the recommended answers to the open questions; awaiting owner approval (decision 7).

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
map chase is close to a synthetic view), so one control is left: the selector
picks the window's UAV and opens the window, Free closes it.

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

So the monitor view has one **vehicle window** at the bottom left, closed by
default. It shows one UAV and one source, **Camera** (first) or **Synthetic**,
never both. The map stays the main view; only RC turns the main map
first-person. One rule (v11, reverses D1 / decision 5): **the selector at the
top right picks the window's UAV: a UAV opens the window on it, Free closes
it; View camera on a confirm card sets the same selector. The selector no
longer moves the map** (open question 1). The window has no UAV badges (open
question 2).

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
| **Selector** at the top right (today's follow combo box, now a button that opens the vehicle list below) | yes (v11) | pick UAV 2: the window opens on UAV 2's camera; pick UAV 1 to come back; Free gives the map back. A UAV picked while the window is closed opens it normal on Camera; another UAV while it is open moves the window to it and keeps the source and the size; Free closes it. The selector never moves the map and never changes RC |
| **Vehicle list** opened by the selector (owner, 2026-10-19) | yes (v11) | before picking, the operator sees which UAVs have a camera and which have lost their link. A panel under the selector at the top right: a "Free" row, then one row per UAV in the fleet: colour dot, "UAV n", and a text state from telemetry and stream announcements: "Camera", "No camera" or "Vehicle link lost" (link first when both). Status only: no picture, no reader, no synthetic view (owner, 2026-10-19: live tiles would play several streams at once, against the one-stream rule, and cost radio bandwidth per UAV). The picked row shows a check mark and "Viewing", never colour alone. A row tap picks, as in the selector row above, and closes the list; Escape, a tap outside or the selector again closes it with no change |
| UAV badges in the window header | removed (v11) | they repeated the selector (open question 2) |
| **View camera** on each task confirm card | yes | a card arrives: tap, compare the live scene with the still, then Approve/Deny on the card. It sets the selector to that UAV and the source to Camera (opening the window if it is closed), so the operator does not watch the wrong UAV; it does not move the map and does not prove the camera still shows the same dock (it may have moved since the still). Neutral button beside the still, away from Approve/Deny; "Viewing" when the window already shows that UAV's camera; on tap the old picture clears and "connecting · UAV n" shows. The countdown keeps running (the card says so); the tap never approves, denies or pauses. Pending cards only; shown only when that UAV has a stream and RC is off |
| **Camera / Synthetic** switch in the window | yes | SITL demo or a dead feed: a vehicle view without RC, for the window's UAV. Synthetic only by a tap here: never automatic, never saved |
| Live state: LIVE (frames arriving), connecting, stale shown on the picture as text ("NO FRAMES 7 s"), failed, **Reconnect video** | yes | before relying on the picture at confirm or approach: a frozen frame looks like a calm scene. LIVE means frames arrive, not low latency |
| **Close** and **Maximize** / restore | yes | Close sets the selector to Free and gives the map back, without a reach to the top right; maximize swaps window and map to judge detail at confirmation (or to see Synthetic large) while the map stays visible, small. No tap-anywhere toggle on the window |
| Minimize to a handle | removed (v11) | Close replaces it; this drops the kept source and the nearby restore (open question 3) |
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

Generic vs delivery: the selector, the switch, live state, loss wording, close
and maximize serve any mission module; View camera depends on the
task-confirmation flow.

Touch and layout (handheld): the window's controls are 44 px targets with text
labels, no hover-only actions, nothing within 8 px of Approve/Deny. Header text
14 px on an opaque background. The header line holds only the "UAV n" title and
the state; a control row under it holds the switch (left) and Maximize and
Close (right); a failure or held view goes on a row under that (reason,
Reconnect video, Dismiss). Narrow map pane (under 480 px): Maximize and Close
show their standard glyphs with an accessible name. Focus: Close removes the
control that has focus, so focus moves to the selector; the same when the
window closes because its UAV left the fleet while focus was inside it. With the badges gone the
header needs no fleet-size room and no overflow menu. In monitor mode confirm
cards and E-STOP are in the sidebar, not over the map, so neither the window
nor the small swapped map can cover them. The top-right bar keeps its pre-PR
#18 controls at 32 px (owner, 2026-10-18), except the selector: it is now the
only way to open the window, so it gets the 44 px height on a touch device
(open question 9). The bar already wraps on a narrow pane
(`MonitorMapOverlay.jsx:64-78`) and reports its bottom edge, so the window
still starts below it. The vehicle list drops from the selector's bottom edge,
right-aligned to it, above the map and the window (normal or maximized); its
rows are 44 px on touch, its width at most the pane width minus 24 px, and
past the pane height it scrolls. It is a disclosure button with a list of
options (WAI-ARIA listbox pattern): arrow keys move, Enter picks, Escape
closes and returns focus to the selector, and each row's accessible name
holds the UAV and its state.

#### Vehicle window

- Where: bottom-left of the map pane, monitor view only, while the selector
  names a UAV of the fleet; hidden in manual control (RC). Closed by default
  and after a reload (the selection is not saved, as `followSysId` is not
  today, `App.jsx:134`). Two sizes: normal (v8's panel box: width
  min(432, pane - 68) px, stopping below the top-right controls) and maximized
  (the swap, below).
- Layout, top to bottom: header line; control row; a status row when needed;
  the picture (16:9, shrinking to fit the height limit). The UAV label stays at
  the top in both sources. A state with no picture (no camera, failed, lost,
  waiting for pose) collapses the picture area at normal size, so the window
  never shows a black box; maximized, the large area shows that state as text.
- Which UAV: the selector's UAV, for both sources; View camera sets the
  selector. Only the operator picks: v10's automatic first pick, its move from
  "No camera" to the first UAV that starts advertising, and its re-arm on an
  empty fleet are removed (the window is closed until a pick, so there is
  nothing to pick automatically). A UAV with no stream opens on "No camera ·
  UAV n", Synthetic one tap away.
- Source: a pick that opens the window starts on Camera; a change to another
  UAV while open keeps the source, as v10's badges did (open question 5).
- The picked UAV leaves the fleet (removed from the backend's `active_ids`,
  `telemetryStore.js:128-133`): today's effect sets the selector to Free
  (`App.jsx:152-159`), so the window closes, with the notice "UAV n left the
  fleet · vehicle window closed" (open question 7). A lost link is not a
  removal: the UAV stays in the fleet and the window shows "Vehicle link lost"
  (Camera) or holds the last pose (Synthetic), as in v10.
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
- Closed (Close in the window, or Free in the selector): the camera reader
  closes and the synthetic widget is destroyed, since an unwatched stream costs
  radio bandwidth (v10's minimize did the same); the next pick opens fresh on
  Camera. v10's minimized handle and its saved state are removed (open
  question 3).
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
  (`ScreenSpaceCameraController.js:511`). It is set back to true on the swap
  back. With no chase (v11), the only thing that can move the map while
  swapped is a coordinate click in the sidebar (`MissionStatusRow.jsx:54`,
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
    it): another UAV in the selector moves the large window to that UAV, Free
    swaps back and closes it; reset view is hidden (a fit made in the small box
    uses its aspect ratio and is wrong after the swap back); markers keep their
    size, so a 3-UAV map is crowded: it is an overview.
  - Fits wait for the large map. v10's chase-to-Free auto-centre
    (`useFpvView.js:46-52`) goes with the chase, and reset view is hidden while
    swapped, so the one fit left to defer is a reset-view flight still running
    when the operator maximizes: it is cancelled and deferred (`useMapSwap.js`,
    which today cancels only flights it tracks in `fitFlyingRef`; v11 cancels
    any flight). A pending fit is dropped when RC takes the
    camera. On the swap back the map calls `viewer.resize()` (public,
    synchronous), then runs the pending fit once. A fit in the small box would
    use its aspect ratio and be wrong after the swap back; shrinking itself
    changes only the frustum, so a move end while small saves the unchanged
    pose.
  - Swap back on: Restore, a tap on the small map, Close or Free (swap back,
    then close), a sidebar coordinate click (swap back, then fly), RC entry, leaving the monitor view, the picked UAV leaving the
    fleet. A change to another UAV keeps the swap. Never saved: a reload starts
    unswapped (and closed).
- Credits: Cesium's credit line (the Cesium ion logo, which the engine treats
  as required, and the data attribution; Cesium ion terms 2.2.3) is never
  covered. Each Cesium view keeps its credits at its own bottom-left (Cesium's
  default), and the small item (the window normally, the map when
  swapped) sits 12 px above the top of the large view's credit line, measured
  from the large viewer's credit container the way the top controls are
  measured. Camera video large has no credit line, so the margin is 12 px.
  Maximized Synthetic fills the window, so its credits sit at the window's
  bottom-left. The small map's own credits stay visible inside it; its tap
  button blocks their "Data attribution" link until the swap back.
- Saved: nothing. v10's browser storage key `gcs-vehicle-window` (minimized or
  normal) is no longer read or written; a value left in a browser is ignored.
  Never the UAV, the source or the swap.
- The window never moves to another UAV and never enters Synthetic by
  itself. Only the operator picks a UAV (selector, View camera) or a source
  (switch). The lifecycle resets in the transition table are the only
  automatic changes: back to Camera on a synthetic failure, RC entry or
  leaving the monitor view, and closed (selector Free) when the picked UAV
  leaves the fleet, the fleet empties or the monitor view is left. A closed
  window never reopens by itself, also when that UAV rejoins.
- One reader per browser; the demo has one GCS, so no radio-wide arbitration in v1.
- Removed (v8): `VITE_VIDEO_WHEP_URL`, `videoConfig.js` and the single-feed
  wiring (no shim). `useWhepStream` and the vendored reader stay; `src/gcs/AGENTS.md` and
  the Jetson config notes are updated.

#### Synthetic view

Owner, 2026-10-18 (design C): a second Cesium view while the map keeps chasing;
2026-10-19: it is a source of the vehicle window; v11: the map no longer
chases, and Synthetic maximized is the large vehicle view.

- Selection: only a tap on the switch. Switching to Synthetic closes the camera
  reader (one reader per browser); switching back reconnects (fresh connection)
  and shows the kept camera state, a lost feed included. A stream appearing,
  failing or going away while on Synthetic never changes the source.
- Which UAV: the window's UAV, picked by the selector. Another UAV in the
  selector moves the same widget to the new UAV's pose.
- Rendering: a second public `CesiumWidget` filling the picture, created on
  entering Synthetic (normal or maximized), destroyed on Camera, Close or Free, RC,
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
  the window's UAV. After the selector changes UAV, until that UAV has a valid pose, an
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
  frame and held banner (`FpvViewOverlay`, RC only); the window and the
  selector are hidden (`App.jsx:773`, `vehicleWindowVisible`). v11 separates RC
  from the selector: today choosing an RC vehicle sets the follow selection and
  leaving RC sets it to Free (`useManualControlWiring.js:49,70`), and the map
  FPV reads the RC target through `followSysId` (`CesiumMap.jsx:291`). In v11
  the map FPV reads the RC target itself, RC never writes the selector, and on
  RC exit the window comes back on the same UAV, normal, on Camera (open
  question 6).
- Map camera ownership: only RC takes the map camera. The saved map camera
  and the persistence suspension (`useCesiumViewer.js`) apply to RC only; a
  picked UAV no longer counts as taking the camera (today `cameraTaken`
  includes `followSysId != null`, `CesiumMap.jsx:239`).
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
| UAVs join (monitor view) | nothing opens; the selector lists them |
| Selector: a UAV, window closed | window opens normal on that UAV, source Camera (or "No camera · UAV n"); the map does not move |
| Selector: another UAV, window open | that UAV in the current source and size; the map does not move |
| Selector: Free, or Close in the window | swap back if swapped; reader closed, widget destroyed; window closed |
| Switch to Synthetic | camera reader closes; the window's UAV in the synthetic view, or "Waiting for pose · UAV n" |
| Switch to Camera | widget destroyed; that UAV's camera state, fresh connection |
| View camera on a pending confirm card | selector set to that UAV, source Camera, fresh connection; opens a closed window normal; the swap stays; the map does not move |
| Maximize | map and window swap; nothing reconnects or rebuilds |
| Restore, or tap on the small map | swap back; nothing reconnects or rebuilds |
| Sidebar coordinate click while swapped | swap back, then the map flies there; the window stays open |
| Maximize during a camera flight | the flight is cancelled; a reset-view fit reruns after the swap back, a coordinate flight does not |
| Telemetry disconnected, link lost or pose not finite (Synthetic) | the last pose holds with the held row; resumes on the next valid pose |
| Link lost (UAV still in the fleet), Camera | "Vehicle link lost · UAV n", and "Camera video lost" if the stream goes too |
| Picked UAV removed from the fleet | selector to Free; swap back; window closed with the notice "UAV n left the fleet · vehicle window closed" |
| Stream appears, fails or goes away | Camera: the camera states above; Synthetic: the source stays, the camera state shows after switching back |
| Widget fails, render error, WebGL context lost or pose callback throws | widget destroyed; source Camera with "Synthetic view unavailable"; the main map keeps rendering |
| RC entry | swap back; window and selector hidden; widget destroyed; source Camera; today's RC FPV of the RC target on the main map; the selector's UAV is kept |
| RC exit | Free map with the saved map camera; if a UAV is still picked, the window returns on it, normal, source Camera |
| Leaving the monitor view, or the fleet becomes empty | selector to Free (today's effect, `App.jsx:152-159`); window closed |
| Page reload | window closed (selector Free); the startup view is the saved map camera |

- Code impact of v10 (shipped, PR #20): `useFpvView` shrank to RC FPV and
  RC-only map camera ownership; the top-right FPV toggle went; a `noCamera`
  kind; Dismiss on a lost feed goes to it.
- Code impact of v11 (no shims):
  - `followSysId` becomes the window's UAV (renamed, e.g. `windowSysId`);
    `useVehicleWindow` takes it as its selection and loses `selectUav`, the
    automatic pick (`pickFirstFeed`, `pickFirstUav`, `auto`), minimize,
    restore and the saved state (`WINDOW_STORAGE_KEY`); `viewCamera` sets the
    App-level selection. `vehicleWindowVisible` also needs a picked UAV that is
    in the fleet.
  - `VehicleWindow.jsx` loses the badges and the minimized handle;
    `vehicleWindowLayout.js` loses the badge room, number badges and the
    overflow menu; Minimize becomes Close.
  - The map chase goes: `useFollowUav` keeps only the RC first-person camera,
    fed with the RC target; `useFpvView` loses the chase-to-Free auto-centre;
    `CesiumMap`'s `cameraTaken` becomes RC only; `useManualControlWiring` no
    longer writes the selection.
  - `useMapSwap` cancels any camera flight on maximize (not only its own
    fits), and the sidebar's fly-to swaps back before it flies; Close moves
    focus to the selector.
  - The native `<select>` (`MonitorMapOverlay.jsx:112-136`) becomes a button
    plus the vehicle list, a new focused component fed by `useVideoFleet`
    entries (`label`, `streams`, `linkOk`, as `VehicleWindow.jsx:382` reads
    them); its accessible name "Camera follow" (`locales/en.json:404`,
    `hy.json:404`) and its height change (open questions 8 and 9).
  - `src/gcs/AGENTS.md` ("the window shows whenever the fleet has a UAV")
    is updated.
  - Tests of the removed behaviour go with it; the test plan below lists the
    new ones.
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
over the map in RC (`App.jsx:855-886`). Deferred: an audit of who decided each
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
- Frontend (vitest): the window closed by default, after a reload and with the
  selector on Free, also when the fleet has UAVs with streams (no automatic
  pick); a UAV in the selector opens it normal on Camera, also with no stream
  ("No camera · UAV n", no Reconnect or Dismiss); another UAV while open keeps
  the source and the swap; Free and Close each close it, close the reader and
  destroy the widget, and the next pick opens fresh on Camera; no badges, no
  minimized handle, nothing read from or written to browser storage; hidden in
  manual control and outside the monitor view; the picked UAV removed from the
  fleet sets Free, closes the window and shows the notice, while a lost link
  keeps it open with "Vehicle link lost"; a UAV removed during RC and then
  rejoining leaves the window closed; the vehicle list: a Free row and one
  row per fleet UAV with its colour, label and text state ("Camera", "No
  camera", "Vehicle link lost", link first when both), updating while open,
  no video reader and no Cesium widget created by opening it, the picked row
  marked with a check and "Viewing", a row tap picks and closes, Escape, a tap
  outside and the selector close it with no change, a UAV leaving the fleet
  while open drops its row; keyboard: the selector opens the list, arrow keys
  and Enter pick and move the window, Escape returns focus to the selector; Close and a removal with focus inside move focus to the
  selector, the selector's accessible name is "Vehicle view"; a sidebar
  coordinate click while swapped swaps back, then flies, and maximizing during
  a coordinate flight cancels it without a rerun; the selector and View camera never
  move the map camera (no chase, no fit, no saved-camera change) and never
  change RC; first-frame and stall deadlines (peer connected but no frames;
  frames then freeze; frames stop while the element raises no
  waiting/stalled event); codec-not-supported and error states with Reconnect
  video; stream removal: the same UAV's other stream, else "Camera video lost
  · UAV n" with Reconnect video and Dismiss, Dismiss to "No camera · UAV n",
  never another UAV; maximize and restore keep the same map element (no
  remount, no animation, never size 0) and the same reader; the button over
  the small map swaps back and passes no input to the map; maximizing during
  inertia or an active touch gesture saves no moved pose, and camera inputs are
  back on after the swap back; reset view hidden while swapped; a reset-view
  flight running when maximizing is cancelled and rerun once after the swap
  back with the large pane's aspect, and dropped on RC entry; Close, Free, RC
  entry, leaving the monitor view and the picked UAV leaving each swap back;
  the small item's bottom offset follows the measured credit line; every row of
  the transition table. Synthetic: entered only by the switch, never on any
  automatic path; SYNTHETIC state and tag, accent frame, never LIVE; held rows
  with the time held; the telemetry socket closing and reopening with no
  snapshot in between keeps holding with "telemetry disconnected", and the
  next snapshot for that UAV resumes it; "Waiting for pose · UAV n" after the
  selector changes UAV until that UAV has a valid pose. The widget (Cesium
  mocked): one per mount under StrictMode, `baseLayer: false`, its own terrain
  provider instance (never the main map's) that follows the Ion terrain
  upgrade, no imagery layer, no entities, inputs off, the shared pose helper
  gives today's RC FPV values, the shared pose source (real `useUavMarkers`
  and `lerpAnim`) skips a sample with a missing or non-finite lat, lon,
  altitude or attitude, keeps the last valid pose and resumes from it while
  the main map keeps rendering, no pose applied while the snapshot fails the
  hold check or the copy is not finite, a missing source entry reads as "no
  pose", one render per update, the same widget when the selector changes
  UAV, unsubscribed and destroyed on unmount, and a create error, render
  error, context loss or a pose-callback throw each return to Camera once with
  the notice, never touching the main camera or stored view. A real
  `CreditDisplay` (Cesium not mocked; Ion and Bing endpoint metadata mocked
  and cached) with the main map and the widget rendering at once, then the
  widget torn down: the main map keeps its Bing and terrain attributions and
  ion logo in its own container throughout, and the widget shows its own
  terrain attribution and ion logo. RC: choosing the RC vehicle and leaving RC
  never write the selector; the map FPV follows the RC target while the
  selector names another UAV; RC exit restores the saved map camera and
  brings the window back on the selector's UAV, normal, on Camera; RC FPV
  keeps its SYNTHETIC tag and the new held reason; a nose pose is never
  stored as the startup view. View camera on a pending confirm card sets the
  selector to that UAV and Camera, opens a closed window, keeps the swap,
  shows "Viewing" only when that UAV's camera is shown, is absent on approved
  cards, when that UAV has no stream and during RC; tapped midway through a
  countdown it keeps the original deadline, expiry runs the existing timeout
  action once, the tap calls neither approve nor deny. LIVE dot only while
  frames arrive, "no frames for N s" after; "Camera video lost" and "Vehicle
  link lost" shown separately, both when both. The top-right bar has no FPV
  button. `npm test`,
  `npm run build`.
- Layout: at desktop size and a 375 px wide map pane (a 695 x 812 touch
  viewport: the 320 px sidebar does not shrink, `styles.js:92`), with 3 UAVs, a
  UAV with two streams, the window closed, normal and maximized, and three
  cards in the sidebar: header on one line, 44 px window targets, the
  selector 44 px on touch, the map buttons fit with the plan button showing,
  the window starts below the wrapped top-right bar, Approve/Deny and E-STOP
  not covered, the Cesium credit line visible in every state, no overlap
  (vitest layout contract plus a `live-tester` screenshot check).
- Live (`live-tester`): 3-UAV SITL with the test source, from two slots at once
  (each plays its own slot's patterns): the window is closed at start; each UAV
  in the selector plays its labelled pattern and the map does not move;
  killing a path's FFmpeg shows the failed state and Reconnect video recovers;
  maximize, change UAV in the selector while maximized, tap the small map to
  swap back, Close, then View camera on a confirm card opens the window on that
  UAV. With the source off: picking UAV 1 opens "No camera · UAV 1", no
  console errors; Synthetic shows the nose view, change UAVs in the selector
  while they fly, link loss shows the held row, maximize Synthetic, back to
  Camera, RC entry and exit (the window comes back on the same UAV), force a
  WebGL context loss on the widget canvas (back to Camera with the notice, the
  map keeps rendering), then reload the page: the window is closed and the
  startup map view is the map. Screenshots at desktop size and under 480 px.
  With the test source on: Synthetic replaces the video, Camera reconnects,
  View camera from Synthetic shows that UAV's camera. Then the load gate
  (Synthetic view section), on a quiet PC.
- Hardware (later, with the owner): Jetson H.264 stream over the radio from the
  laptop and from a second device (WHEP OPTIONS/POST accepted from the real GCS
  origin): glass-to-glass latency (on-screen clock),
  bitrate, and vision capture timing unchanged while the GCS watches.

## Open questions for v11 (owner)

The design above is written with each recommended answer.

1. **When a UAV is picked, should the main map still follow it?**
   Recommended: no, the map stays where it is. Reasons: View camera at a
   confirm card would otherwise pull the map off the 3-UAV overview while
   another UAV approaches (the v8 walkthrough's reason for D1); the owner's
   idea says the map needs no chase of its own; and the chase-to-Free
   auto-centre and its swap deferral go. Cost: nothing keeps a moving UAV
   centred on the map; Synthetic maximized is the large vehicle view. If
   yes: the selector moves map and window together, View camera also moves
   the map, and Minimize (question 3) keeps a use: chase without the window.
2. **Should the badges go, or stay as a quick switch while the window is
   open?** Recommended: go. The selector is one pick away, View camera covers the
   confirm-time switch, and the header loses its fleet-size room, number
   badges and overflow menu. Cost: on a wide desktop the selector is at the
   top right, away from the window at the bottom left.
3. **Minimize.** Recommended: replace it with Close (= Free) and drop the
   handle and the saved minimized state. The handle kept the source and
   offered a nearby restore; without it the next pick opens on Camera and the
   selector is at the top right. With no chase, Free does the rest of what
   minimize did. If question 1 is "yes", keep Minimize.
4. **View camera on a confirm card.** Recommended: it sets the selector to
   that UAV, so it opens a closed window (Camera); the map does not move.
5. **Source when the selector changes UAV while open.** Recommended: keep the
   source (as v10's badges did), so Synthetic maximized can step through the
   fleet; opening from closed always starts on Camera.
6. **RC.** Recommended: RC no longer reads or writes the selector (today RC
   vehicle choice sets follow and RC exit sets Free); RC exit brings the
   window back on the same UAV, normal, on Camera.
7. **The picked UAV leaves the fleet.** Recommended: the selector goes to Free
   and the window closes with a notice, because the selector cannot show a
   UAV it does not list. A lost link is not a removal and keeps the window
   open with "Vehicle link lost". Alternative: keep the window open on "Vehicle
   link lost" with the selector showing Free, which makes the two disagree.
8. **Selector wording.** (The list itself is decided: status rows, owner
   2026-10-19.) Its accessible name is "Camera follow"
   (`locales/en.json:404`), wrong once the map no longer chases. Recommended:
   rename it "Vehicle view" (Armenian string too); keep the entry "Free"
   (`en.json:405`), the owner's own word for "no vehicle shown".
9. **Handheld and narrow layout.** Recommended: the selector, now the only
   way to open the window, is 44 px on a touch device (the rest of the top bar
   stays 32 px); the window header holds only the title and state; Close in
   the window saves a reach to the top right.
10. **Reload.** Recommended: the window opens closed (the selection is not
   saved, as today's follow is not).

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
   SITL opens on "No camera" (v11: badges and the follow-only-map rule
   reversed by decision 7). The top-right FPV button goes; the synthetic
   source is named Synthetic, chosen by hand only and never saved; only
   minimized or normal is saved.
6. v10 approved (2026-10-19) with the recommended answers: minimizing stops
   the camera video and the synthetic view (restore reconnects); the
   synthetic view shows shaded terrain without imagery. A second Cesium view
   stays behind the load gate (fallback: a 2D attitude indicator).
7. v11 direction (owner, 2026-10-19; reverses decisions 2 and 5): one control
   picks the UAV. The vehicle window is closed by default; picking a UAV in
   the top-right selector opens it on that UAV, Camera first; Free closes it;
   Synthetic then Maximize gives the large synthetic view; the badges go.
   Reason: the follow selector and the badges duplicated each other. Same
   day: the selector opens a status-only vehicle list at the top right
   (no live tiles: one stream at a time, no extra radio load). Pending:
   approval of v11 and the open questions above.
