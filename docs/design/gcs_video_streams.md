# GCS video streams per vehicle

Status: design v7 (video rounds 1-4, FPV UX rounds 1-3, one view at a time), awaiting owner approval. No code yet.

Owner request (2026-10-16): "we should be able to see video streams in our GCS if
those are available."

Goal: the monitor view shows live video from any vehicle that has a camera stream,
and shows nothing extra (no empty or broken panel) for vehicles that have none.

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
- The panel checks `RTCRtpReceiver.getCapabilities('video')` against the
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

### 5. UI: one vehicle view at a time

Owner, 2026-10-16: build each feature for a purpose; in the demo any UAV may
have a stream but one plays at a time; Cesium FPV must work without video; no
need for FPV and video together. Controls below were proposed independently by
Claude and Codex and merged; each is tied to a delivery moment.

Purpose of the two views:

- **Camera (real video)**: the operator inspects the real scene where a decision
  or a watch happens: dock/task confirmation, final approach, handover area.
  The map cannot show this.
- **FPV (Cesium, synthetic)**: orientation and a stand-in vehicle view when
  there is no usable stream (SITL demo, UAV without camera, failed feed). It
  never confirms anything.

So the operator sees **map + camera panel** (default) or **FPV** (main map
switches to first person, camera panel hidden and its reader closed), never both.

#### Controls

| Control | v1 | Delivery moment and purpose |
|---|---|---|
| UAV badges in the panel header (one per UAV with a stream) | yes | pick whose camera to watch; one stream plays |
| **View camera** on each task confirm card | yes | confirmation: the card has the detector's still image (`TaskConfirmCard.jsx:168`); live video shows whether the scene still holds. One tap switches the panel to that UAV (and leaves FPV). It does not approve, deny, reset or pause the countdown or auto-approve. Shown only when that UAV has a stream and RC is off (confirm cards stay visible in RC, but the camera panel is hidden there, so the button is not offered) |
| **FPV** toggle beside the follow selector | yes | orientation; fallback when there is no video |
| **View FPV** in a failed feed's collapsed header | yes | video loss: explicit fallback, never automatic. With no stream at all there is no panel; the FPV toggle is the fallback |
| Live state in the panel: LIVE dot, connecting, "no frames for N s", Retry | yes | approach/handover: never read a frozen picture as live. "Last frame N s ago" is frame arrival, not camera latency |
| Expand / restore (exists) | yes | inspect detail at confirmation or handover |
| Separate "video lost" vs "link lost" | yes | abnormal events: losing imagery is not losing the vehicle; neither changes mission state |
| Auto-switch to a UAV that raises a confirm, with a pin | later | can pull the view off another UAV's approach; manual View camera is enough for the demo |
| Snapshot / recording as evidence | later | needs storage, timestamps, task association; an image does not prove physical docking or cargo receipt |
| Gimbal or zoom control from the panel | later | NavPy owns the gimbal during approach; zoom changes vision calibration assumptions; needs an ownership design |
| FPV and video together, tiles, mission phase in the header | no | no operator decision needs them in the demo; the phase is not in the snapshot and the sidebar shows status |
| Flight or mission actions in the panel, "handover complete" from video | no | duplicates existing controls; observation is not outcome |

All controls are touch targets of at least 32 px with text labels and no
hover-only actions (handheld use). E-STOP, confirm cards and alerts stay on top
and reachable when the panel is expanded.

Narrow screens (map pane under 480 px wide, the handheld): badges shrink to the
UAV number, the stream picker becomes one icon menu, and the header never wraps
past one line. Expanded takes the pane width minus the 12 px margins and stops
below the top-right controls and confirm cards (today's expanded cap already
keeps clear of the top bar), so Approve/Deny and E-STOP are never covered.

#### Camera panel

- Bottom-left of the map, monitor view only, under the map buttons and
  confirmation cards, same docked and expanded sizes as today. Rendered only
  when at least one UAV has a stream; hidden in FPV and in manual control.
- Header: badges (selected one outlined), then **CAMERA · UAV n** and the live
  state; a stream picker only if that UAV has more than one stream.
- Selection: the follow selector (top-right, "Free" or a UAV) choosing a UAV
  with video switches the feed to it; Free or a UAV without video keeps the
  current feed (never an empty box). A badge or View camera switches only the
  video, not the map camera. First feed: the followed UAV if it has video, else
  the first UAV with video.
- Deadlines: no first frame within `VIDEO_FIRST_FRAME_DEADLINE_S` (default 10 s:
  ICE setup plus one keyframe interval, which the display stream keeps at 2 s or
  less), or a stall longer than `VIDEO_STALL_DEADLINE_S` (default 5 s, a few
  keyframe intervals), becomes the failed state. Both named in one frontend
  constants file, with fake-timer tests.
- Failed or codec not supported: the video area collapses to the header with a
  short reason, Retry and View FPV, so a dead feed never leaves a black box.
- Streams changing: the shown stream disappears -> that UAV's other stream, else
  the followed UAV's stream, else the first stream; the last stream gone ->
  panel gone.
- One reader per browser; the demo has one GCS, so no radio-wide arbitration in
  v1. Tiles become an option only once several streams play at once.
- Removed: `VITE_VIDEO_WHEP_URL`, `videoConfig.js` and the single-feed wiring (no
  shim). `useWhepStream` and the vendored reader stay; `src/gcs/AGENTS.md` and
  the Jetson config notes are updated.

#### FPV

- The FPV toggle puts the main map camera in the followed UAV's first-person
  view through today's `useFollowUav` FPV path (body attitude), for any eligible
  UAV, with or without video. The camera panel hides and its reader closes.
- FPV pressed while following Free sets the follow selector to the panel's UAV
  if eligible, else the first eligible UAV; the selector then names it. View FPV
  in a failed feed sets the follow selector to that feed's UAV and turns FPV on.
- Eligible = the existing backend `link_ok` flag (used by the bottom bar and
  preflight) plus finite position and attitude in the snapshot; no new
  freshness threshold. Checked on every entry (toggle, View FPV, changing the
  followed UAV while in FPV); an ineligible target leaves the current view as it
  is with a short notice naming the UAV. No eligible UAV: the toggle stays off
  with a notice.
- Always labelled: a fixed top-centre tag **SYNTHETIC · UAV n · body view** and
  a distinct frame around the map. In FPV the camera holds the last valid pose
  and resumes on the next valid one; the tag adds "link lost" when `link_ok` is
  false and "pose unavailable" when linked but position or attitude is not
  finite.
- View mode is separate from control mode: FPV never enables RC; RC keeps
  today's behavior (body FPV of the RC target, panel hidden).
- Map camera ownership: one saved map camera, taken when the view first leaves
  the map (FPV on, or RC entry from the map); switching the FPV UAV does not
  overwrite it. Camera persistence (`useCesiumViewer.js` stores camera moves) is
  suspended while FPV or RC owns the camera, so a first-person pose is never
  the startup view. The viewer runs in request-render mode, so FPV requests a
  render per pose update.

| Event | Result |
|---|---|
| FPV off | chase view of the same UAV; camera panel back; saved map camera restored when follow goes to Free |
| Follow set to Free (FPV on) | FPV off, saved map camera restored, camera panel back |
| Followed UAV removed | as Free |
| View camera on a confirm card (FPV on) | FPV off as above, panel shows that UAV |
| View camera during RC | not offered (panel hidden in RC) |
| RC entry (FPV on or off) | today's RC FPV of the RC target |
| RC exit | Free map with the saved map camera (today: Free); FPV off |

- Deferred: a second Cesium viewer, swapping map and panel, and a
  camera-direction synthetic view from gimbal attitude + FOV (needs verified
  mount axes and stream-to-device association).

### 6. SITL vs real hardware

| Setup | Stream source | Discovery |
|---|---|---|
| Plain SITL, default | none | nothing announced -> no panel |
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
- Frontend (vitest): no panel without streams; first-frame and stall deadlines
  (peer connected but no frames; frames then freeze) collapse to the header; badges only for vehicles with
  streams; switching; follow selector to a UAV with video switches the feed, to
  Free or a UAV without video keeps it, a badge click leaves the map camera;
  hidden in manual control; FPV toggle for a UAV without video, from Free (picks
  the panel's UAV), off and on Free returns to the saved map camera, never
  stored as startup view, SYNTHETIC tag always shown, RC entry and
  exit while in FPV; codec-not-supported and error
  collapse to the header; stream removal picks the replacement feed (same UAV's other stream, followed
  UAV, first; last one gone -> no panel); View FPV from a failed feed's header while
  following another UAV; every row of the transition table; no eligible UAV;
  `link_ok` lost and back in FPV; non-finite position, then attitude, while
  linked (hold, "pose unavailable", resume); ineligible target on each entry
  path; FPV hides the panel and closes its reader, FPV off brings it back;
  View camera on a confirm card switches the panel (and leaves FPV), is absent
  when that UAV has no stream and during RC; tapped midway through a countdown
  it keeps the original deadline, expiry runs the existing timeout action once,
  the tap calls neither approve nor deny, and the pending card survives leaving
  FPV; LIVE dot only while frames arrive, "no frames for
  N s" after; "video lost" and "link lost" shown separately. `npm test`,
  `npm run build`.
- Layout: at desktop size and a 375 x 812 touch viewport, with 3 UAVs, a UAV
  with two streams, the panel expanded and a pending confirm card: header on
  one line, Approve/Deny, E-STOP and the map buttons visible and tappable, no
  overlap (vitest layout contract plus a `live-tester` screenshot check).
- Live (`live-tester`): 3-UAV SITL with the test source, from two slots at once
  (each plays its own slot's patterns): each badge plays its
  labelled pattern, switching works, killing a path's FFmpeg collapses to the
  header and Retry recovers; with the source off: no panel, no console errors.
  Then, with video off: FPV on, switch UAVs while they fly (view follows pose),
  follow Free (map camera restored), FPV on then RC entry and exit, then reload
  the page and check the startup map view is the map, not an FPV pose.
- Hardware (later, with the owner): Jetson H.264 stream over the radio from the
  laptop and from a second device (WHEP OPTIONS/POST accepted from the real GCS
  origin): glass-to-glass latency (on-screen clock),
  bitrate, and vision capture timing unchanged while the GCS watches.

## Open points for the owner

1. Approve the approach: camera protocol discovery, browser WHEP straight to each
   aircraft's MediaMTX, one feed at a time that follows the follow selector,
   and an FPV toggle for Cesium FPV of any UAV.
2. Display codec: the Jetson publishes a software H.264 display stream
   (recommended; leaves the ZR10 and the vision input untouched, costs Jetson CPU
   that is measured first), or switch the ZR10 to H.264
   (changes the vision input, needs its own check).
