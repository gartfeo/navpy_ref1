# GCS video streams per vehicle

Status: design v8 approved by the owner 2026-10-18 (D1 (a): follow moves only the map; display codec: software H.264 on the Jetson) and shipped as PR #18. v9 draft (2026-10-18, design C: FPV in the bottom-left panel while the map keeps chasing) awaits owner approval; its open questions are at the end.

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
need for FPV and video together. v8 (2026-10-17): every element was walked
through a 3-UAV delivery mission by a field operator, a delivery operations
adviser, a UX designer and Codex, then debated with Codex; elements without a
concrete operator use were cut.

Purpose of the two views:

- **Camera (real video)**: inspect the real scene at the two delivery moments
  the map cannot show: the dock confirmation (does the live scene still match
  the detector's still?) and the final approach (is there a reason to Cancel
  task?). Watching is optional; the map stays the main view.
- **FPV (Cesium, synthetic)**: orientation, and a vehicle view where there is no
  usable video (SITL demo, UAV without camera, failed feed), without taking RC
  authority. It shows attitude over terrain, not what the camera or detector
  sees, and never confirms anything.

So the map always stays and chases (v9), and the bottom-left panel shows
**camera video** (default) or the **FPV** nose view (camera reader closed),
never both. Only RC still turns the main map first-person.
One rule: **the follow selector moves the map (and FPV); badges and View camera
move the video** (owner decision D1 below).

#### Controls

| Control | v1 | Operator use |
|---|---|---|
| UAV badges in the panel header (one per UAV with a stream) | yes | watching UAV 1's approach, check UAV 2's camera and tap 1 to come back, without moving the map. Selected badge: outline + fill + the full "UAV n" label, never colour alone. A badge never changes follow or RC |
| **View camera** on each task confirm card | yes | a card arrives: tap, compare the live scene with the still, then Approve/Deny on the card. It opens that UAV's live stream, so the operator does not watch the wrong UAV; it does not prove the camera still shows the same dock (it may have moved since the still). Neutral button beside the still, away from Approve/Deny; "Viewing" when the panel already shows that UAV; on tap the old picture clears and "connecting · UAV n" shows. The countdown keeps running (the card says so); the tap never approves, denies or pauses. Pending cards only; shown only when that UAV has a stream and RC is off |
| **FPV** toggle (text label, clear on state, unlike the RC button) beside the follow selector | yes | SITL demo or a dead feed: a vehicle view without RC, in the bottom-left panel while the map chases the same UAV (v9). Shows the followed UAV; from Free it takes the panel's UAV if eligible, else the first eligible UAV, and the follow selector then names it |
| Live state: LIVE (frames arriving), connecting, stale shown on the picture as text ("NO FRAMES 7 s"), failed, **Reconnect video** | yes | before relying on the picture at confirm or approach: a frozen frame looks like a calm scene. LIVE means frames arrive, not low latency |
| Expand / restore, plus collapse to header for a healthy feed (labelled buttons) | yes | expand to judge detail at confirmation; collapse to give the map back without entering FPV. No tap-anywhere toggle |
| "Camera video lost" vs "Vehicle link lost" | yes | camera only: keep monitoring; link: check the vehicle, consider Cancel task or E-STOP. Both shown when both happen, link first. Video loss says nothing about NavPy or the detector; neither changes mission state |
| Header names the view from stream metadata (e.g. "UAV 2 · tracking overlay") | yes | the operator knows the picture is the Jetson overlay with detector boxes, not the raw camera |
| Stream picker (two views on one UAV) | later | no delivery decision needs it; the data model keeps all streams, v1 plays the UAV's default stream: the lowest stream id, and the companion config gives the display stream id 1 |
| View FPV in a failed feed | cut | no decision beyond the FPV toggle; it made synthetic imagery look like the answer to lost video |
| Auto-switch to a confirming UAV, with a pin | later | pulls the view off another UAV's approach; View camera is one tap |
| Snapshot / recording | later | after-action review only; an image does not prove docking or cargo receipt |
| Gimbal or zoom control | later, payload module | NavPy owns the gimbal during approach; zoom changes vision calibration; needs an ownership design |
| FPV and video together; mission phase in the header | no | no decision needs them; phase belongs in the sidebar |
| Tiles | later option (owner, 2026-10-16) | only with a new operator need once several streams play at once |
| Flight or mission actions in the panel | no | Cancel task and E-STOP already exist; observation is not outcome |

Generic vs delivery: badges, live state, loss wording, expand and FPV serve any
mission module; View camera depends on the task-confirmation flow.

Touch and layout (handheld): new and touched controls are 44 px targets with
text labels, no hover-only actions, nothing within 8 px of Approve/Deny. Header
text 14 px on an opaque background. The header line holds only badges, the
"UAV n" title and the live state; a failure goes on a row under it (reason,
Reconnect video, Dismiss). Narrow map pane (under 480 px): badges show the
number, the full "UAV n" stays in the title, and badges that do not fit move
into one menu button. Expanded takes the pane width minus 12 px margins and stops below the
top-right controls. In monitor mode confirm cards and E-STOP are in the sidebar,
not over the map, so the panel cannot cover them. Open conflict (v9 question 4):
the owner asked for the follow selector at its pre-PR #18 size and the FPV
toggle at the bar's height, which makes the top-right bar 32 px, below this
44 px rule; the panel's 44 px "FPV off" is then the touch-size exit from FPV.

#### Camera panel

- Bottom-left of the map, monitor view only. Shown while any UAV advertises a
  stream or a lost feed is still on screen; replaced by the FPV panel in FPV,
  hidden in manual control.
  The GCS learns of streams only from the camera protocol: a camera that is
  expected but never advertises is not shown here; preflight reports it
  (`architecture_todo.md` 70).
- Selection: badges and View camera switch the video only. The follow selector
  picks only the first feed (the followed UAV if it has video, else the first
  UAV with video); later follow changes leave the video alone (D1).
- Deadlines: no first frame within `VIDEO_FIRST_FRAME_DEADLINE_S` (default 10 s:
  ICE setup plus one keyframe interval, which the display stream keeps at 2 s or
  less), or no new frame for `VIDEO_STALL_DEADLINE_S` (default 5 s, a few
  keyframe intervals), becomes the failed state. Measured by frame progress
  (`requestVideoFrameCallback`, fallback `getVideoPlaybackQuality`), not only the
  video element's waiting/stalled events. Both named in one frontend constants
  file, with fake-timer tests.
- Failed or codec not supported: the video area collapses to the header with the
  UAV, a short reason and Reconnect video, so a dead feed never leaves a black box.
- Streams changing: the shown stream disappears -> that UAV's other stream if
  any; else the panel stays on "Camera video lost · UAV n" with Reconnect video
  and Dismiss until it returns. Never a silent switch to another UAV. The panel
  goes away only when no advertised stream is left and nothing failed is pending.
- One reader per browser; the demo has one GCS, so no radio-wide arbitration in v1.
- Removed: `VITE_VIDEO_WHEP_URL`, `videoConfig.js` and the single-feed wiring (no
  shim). `useWhepStream` and the vendored reader stay; `src/gcs/AGENTS.md` and
  the Jetson config notes are updated.

#### FPV (v9: in the panel, the map keeps chasing)

Owner, 2026-10-18: "FPV in the bottom-left panel while the map keeps chasing."
v8 put FPV on the main map; v9 moves it into the camera panel's place.

- Slot: the bottom-left slot shows camera video or FPV, never both. FPV on
  replaces the camera panel and closes its reader (one reader per browser); the
  video selection and any collapsed or lost state are kept and come back on FPV
  off, docked, with a fresh connection. With no streams (SITL default) FPV on
  shows the FPV panel and FPV off leaves nothing there. A stream appearing,
  failing or going away during FPV never switches the slot. The main map keeps
  chasing the followed UAV and never goes first-person outside RC. Reset view
  stays hidden in FPV, as in v8: the chase re-applies its framing on every
  frame (`useFollowUav.js:141-147`), so a fit would be overridden.
- Which UAV: as v8. FPV shows the followed UAV, so map and panel always show
  the same UAV and the follow selector names it. Eligibility (`link_ok` plus
  finite position and attitude, checked on every entry), the notice for an
  ineligible target, the reverted follow change and D1 are unchanged. The FPV
  panel has no badges.
- Rendering: a second public `CesiumWidget` filling the panel picture, created
  on FPV on, destroyed on FPV off (at most one, also under React StrictMode).
  One Scene has one camera and CesiumJS has no public multi-viewport API, so a
  second widget is Cesium's own pattern (Sandcastle "Multiple Synced Views");
  the private `SharedContext`, `View` and `Framebuffer` are not used. Options
  (Cesium 1.138.0 installed): `baseLayer` is a new `ImageryLayer` over the main
  map's current imagery provider, or `false` when it has none (never
  `imageryProvider`, ignored since 1.107, and never undefined, which adds Ion
  world imagery); `terrainProvider` is the main map's instance. The main map
  swaps in Ion imagery, then Ion terrain, asynchronously
  (`useCesiumViewer.js:226-242`), so an open panel follows both (imagery layer
  added or removed, terrain changed) and unsubscribes on teardown;
  `requestRenderMode` with `maximumRenderTimeChange`
  Infinity, the main map's target frame rate as one shared constant,
  `msaaSamples` 1, `showRenderLoopErrors` false, camera inputs off; sky
  atmosphere on (the horizon is the attitude cue), sun, moon, sky box and
  lighting off. No entities: terrain and imagery only; peers, docks and tracks
  stay on the chasing map. Credits stay in the widget.
- Pose: the same interpolated position and orientation the map's UAV model
  uses (copies through a narrow source ref from `CesiumMap`, wired like the
  map camera ref), through today's first-person math moved into one pure helper
  shared with RC FPV. The panel applies a pose only when the snapshot passes
  the hold check (`fpvHoldReason`) and the copied position and orientation are
  finite; otherwise it holds the last good pose. Prerequisite, part of v9: the
  shared source validates before it builds or advances a pose. Today it skips
  only a missing lat/lon and fills a missing altitude or attitude with 0
  (`useUavMarkers.js:37-56`), then interpolates towards that pose
  (`lerpAnim.js`); a UAV with no altitude drops to the terrain on the map. v9
  skips the update unless lat, lon, altitude, heading, roll and pitch are all
  finite numbers, so the model, the chase and FPV keep the last valid pose and
  resume from it. One render request per telemetry update. The
  panel saves no camera, so a nose pose is never the startup view.
- Failure: the widget fails to create, `scene.renderError` fires, the canvas
  loses its WebGL context, or the panel's own pose callback throws -> widget
  destroyed, FPV off, notice "FPV view unavailable". The callback needs its own
  catch: Cesium raises `preRender` outside its error wrapper
  (`Scene.js:4440`), and a throw there stops the widget's render loop with no
  event (`CesiumWidget.js:73-80`, error panel off). The main map keeps
  rendering (live-tested). Retry is a new tap; no automatic retry.
- Panel: the camera panel's layout contract (bottom-left, width
  min(432, pane - 68) px, stops below the top controls). Header: "UAV n · nose
  view" and **SYNTHETIC** in the state slot (never LIVE, never a green dot).
  While the view holds its last pose, a row under the header reads **View held ·
  vehicle link lost** or **View held · pose unavailable** with the seconds held.
  Picture 16:9, shrinking to fit the height limit, with an accent inner frame
  (video never has one) and a fixed tag **SYNTHETIC · not camera**. One 44 px
  **FPV off** button. No expand in v1.
- RC: unchanged. Body FPV of the RC target on the main map with today's
  full-pane tag, frame and held banner (`FpvViewOverlay`, now RC only); no panel.
- Map camera ownership: only RC takes the map camera now. The saved map camera
  and the persistence suspension (`useCesiumViewer.js`) apply to RC only; FPV
  then Free auto-centres like a plain chase (change from v8).
- Load gate before merge, on the demo PC with FPV off and on: main-map frame
  time, GPU process memory, tile and Ion request counts. Then toggle FPV 20
  times: no panel canvas is left in the DOM, a heap snapshot holds no panel
  widget, scene or WebGL context, and GPU and JS memory return near the
  first-FPV level (Cesium frees its resources on destroy but does not release
  the WebGL context explicitly, `Context.js:1729-1750`). Throughout the gate the
  main map renders with no context loss and no render-loop stop.
  The owner accepts the measured cost before merge; the handheld is measured
  before handheld use. If the cost is too high, the
  fallback is a 2D attitude indicator in the panel (QGroundControl and Mission
  Planner show the vehicle view this way).

| Event | Result |
|---|---|
| FPV on (followed UAV eligible) | panel shows its nose view; camera video, if shown, unmounts and its reader closes; the map keeps chasing |
| FPV on from Free | target picked as in v8, follow set to it, the map starts a plain chase, the panel shows FPV |
| FPV on, target ineligible or none eligible | nothing changes; notice names the UAV or says none is eligible |
| Follow changed (FPV on) | map and panel switch together, same widget; an ineligible UAV is refused with a notice |
| FPV off (bar toggle or panel button) | widget destroyed; camera video comes back as it was (fresh connection, docked), or nothing when there are no streams; the map keeps chasing |
| Follow set to Free, or followed UAV removed (FPV on) | FPV off as above; the map auto-centres (no saved-camera restore) |
| Link lost or pose not finite (FPV on) | picture holds its last pose with the held row; resumes on the next valid pose |
| Stream appears, fails or goes away (FPV on) | the panel stays on FPV; the video state shows after FPV off |
| View camera on a confirm card (FPV on) | FPV off; the panel shows that UAV's video; follow unchanged (D1) |
| Widget fails, render error, WebGL context lost or pose callback throws | FPV off with "FPV view unavailable"; the main map keeps rendering |
| RC entry (FPV on or off) | FPV off; today's RC FPV of the RC target on the main map; no panel |
| RC exit | Free map with the saved map camera; FPV off |
| Leaving the monitor view, or page reload | FPV off; the startup view is the saved map camera |

- Deferred: swapping map and panel, and a camera-direction synthetic view from
  gimbal attitude + FOV (needs verified mount axes and stream-to-device
  association).

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
  streams; switching; the follow selector picks only the first feed and later
  follow changes keep the video, a badge click leaves the map camera;
  hidden in manual control; FPV toggle for a UAV without video, from Free (picks
  the panel's UAV), off and on Free auto-centres the map (RC alone restores the
  saved map camera), never stored as startup view, SYNTHETIC tag always shown,
  the main camera stays in chase in view FPV (never set to a nose pose), RC entry and
  exit while in FPV; codec-not-supported and error
  collapse to the header; stream removal: same UAV's other stream, else "Camera video lost · UAV n" with
  Reconnect video and Dismiss, never another UAV; panel gone only after the last
  failed feed is dismissed and no stream is advertised; stall from frame progress (frames stop while the
  element raises no waiting/stalled event); collapse and expand of a healthy
  feed; every row of the transition table; no eligible UAV;
  `link_ok` lost and back in FPV; non-finite position, then attitude, while
  linked (hold, "View held" banner with time held, resume); SYNTHETIC tag in RC FPV; ineligible target on each entry
  path, with the follow selector reverting to the FPV UAV; FPV replaces the camera panel and closes its reader, FPV off brings it back
  (kept lost state included); the FPV panel with zero streams (header, SYNTHETIC state,
  frame, tag, held row, 44 px FPV off, no badges, never LIVE); the FPV widget (Cesium
  mocked): one per mount under StrictMode, `baseLayer` set explicitly, shared terrain
  provider, no entities, inputs off, the shared pose helper gives today's RC FPV
  values, hold then resume, the shared pose source (real `useUavMarkers` and
  `lerpAnim`) skips a sample with a missing or non-finite lat, lon, altitude or
  attitude, keeps the last valid pose and resumes from it while the main map keeps
  rendering, no pose applied while the snapshot fails the hold check
  or the copy is not finite, one render per update, same widget on a follow change,
  an imagery or terrain upgrade while open is followed and unsubscribed on unmount,
  destroyed on unmount, and a create error, render error, context loss or a throw
  in the pose callback each end FPV once with the notice, never touching the main
  camera or stored view; reset view hidden in FPV;
  View camera on a pending confirm card switches the panel (and leaves FPV),
  shows "Viewing" when already shown, is absent on approved cards, when that UAV
  has no stream and during RC; tapped midway through a countdown
  it keeps the original deadline, expiry runs the existing timeout action once,
  the tap calls neither approve nor deny, and the pending card survives leaving
  FPV; LIVE dot only while frames arrive, "no frames for
  N s" after; "Camera video lost" and "Vehicle link lost" shown separately, both when both. `npm test`,
  `npm run build`.
- Layout: at desktop size and a 375 x 812 touch viewport, with 3 UAVs, a UAV
  with two streams, the panel expanded and three cards in the sidebar: header on
  one line, 44 px targets, the map buttons and FPV toggle fit with the plan
  button showing, Approve/Deny and E-STOP not covered, no overlap (vitest layout contract plus a `live-tester` screenshot check).
- Live (`live-tester`): 3-UAV SITL with the test source, from two slots at once
  (each plays its own slot's patterns): each badge plays its
  labelled pattern, switching works, killing a path's FFmpeg collapses to the
  header and Reconnect video recovers; with the source off: no panel, no console errors.
  Then, with video off: FPV on shows the nose view in the panel while the map
  chases the same UAV, switch UAVs while they fly (both follow), link loss
  shows the held row, FPV off from the bar and from the panel, follow Free
  (map auto-centres), FPV on then RC entry and exit, force a WebGL context loss
  on the panel canvas (FPV ends with the notice, the map keeps rendering), then
  reload the page and check the startup map view is the map, not an FPV pose.
  Screenshots at desktop size and under 480 px. With the test source on: FPV
  replaces the video, FPV off reconnects, View camera ends FPV. Then the FPV
  load gate (FPV section), on a quiet PC.
- Hardware (later, with the owner): Jetson H.264 stream over the radio from the
  laptop and from a second device (WHEP OPTIONS/POST accepted from the real GCS
  origin): glass-to-glass latency (on-screen clock),
  bitrate, and vision capture timing unchanged while the GCS watches.

## Owner decisions (2026-10-18)

1. Approach approved: camera protocol discovery, browser WHEP straight to each
   aircraft's MediaMTX, one feed at a time, FPV toggle for Cesium FPV of any UAV.
2. D1 (a): the follow selector moves only the map and FPV; badges and View
   camera move the video; follow picks only the first feed.
3. Display codec: the Jetson publishes a software H.264 display stream; the
   ZR10 and the vision input stay untouched (Jetson CPU measured first).
4. v9 direction (2026-10-18): design C, FPV in the bottom-left panel while the
   map keeps chasing.

## Open for the owner (v9)

1. Second Cesium view in the panel (recommended), behind the load gate. The
   alternative is a 2D attitude indicator: cheap and proven, but no terrain
   picture.
2. Follow set to Free while FPV is on ends FPV (recommended: one selector
   always names both views). The alternative keeps the last UAV's nose view
   while the map goes free, but FPV then needs its own UAV choice.
3. The FPV picture shows terrain and imagery only (recommended); peers and
   docks stay on the chasing map. Adding them doubles each entity layer.
4. Top-right bar at 32 px (your request: follow selector at its pre-PR #18
   size, FPV at the bar's height) breaks the 44 px touch rule for that bar.
   Recommended: accept it as the one exception; the panel's 44 px FPV off is
   the touch exit. The alternative is the whole bar at 44 px.
