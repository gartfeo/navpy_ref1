# Plan/camera compatibility check (v3.1, 2026-10-20)

v2 answered Codex v1 (reject, 1-8) and the owner's ruling below; v3 answers Codex v2 (reject, 1-6); v3.1 answers Codex v3 (reject, 1-4).

## Problem
The GCS lets the operator upload and launch a plan whose sim docks the active camera can never detect. Found while
moving the hand delivery demo to a normal app run: UI plans fly the dock preset altitude plus the zone stack
(`useMissionUpload.js`: base + (n-1-i)*sep), so 363/383/403 m for `siyi_zr10`, while the fixture was proven at
220-260 m. With the ZR10 at -14 deg, the image bottom meets flat ground 757 m out at 403 m, beyond the 829 m slant
detection range (for a dock at ground level). The app says nothing. (Since todo 92(d) the ZR10 preset is 200 m, so the
stack is 240/220/200 m, and the planner uses the catalog default profile when the settings name none.)

## Owner ruling (2026-10-20)
- Search is **detection, not identification**: the ZR10 searches wide and zooms (up to x10) only after a detection
  to identify. The check uses detection optics and the detection pixel gate.
- One dock that can never be detected is enough to call the plan impossible.
- The planning panel shows the cause and the fixes. If the operator presses Upload anyway, a confirmation dialog says
  what needs to be done; the operator may proceed.

## Not checked, and why
- **Two docks never in one frame.** Dropped (Codex 1): with an active POI, `PoiSelector.select`
  (`poi_selection.py:53-64`) offers other detections even when the active one is out of frame, and e2b-1 saw two POIs
  only inside the confirmation orbit. Level-search geometry cannot prove keep-one/offer-one impossible.
- **Dive too steep.** Not a failure: NavPy sizes the confirmation orbit with `r_nav_min`, whose descent term
  h/tan(eta*|PTCH_LIM_MIN_DEG|) widens the orbit for a steep dive (`orbit_geometry.py:65-80`). Whether the
  confirmation orbit and identification zoom then work (`peer_orbit_plan.py:151-175`) is outside this detection
  check. The panel shows the dive angle at both window edges as information only.

## Model (backend, NavPy's own numbers)
New pure module `src/gcs/backend/plan_camera_check.py` and route `POST /api/plan-camera-check`. The frontend sends
what it is about to fly; the backend does the math. No camera or detector constant is copied into JS.

- **Camera**: the active profile and device from settings (resolved as `config._resolve_intrinsics` /
  `_resolve_pitch`). **Search optics = the device's first zoom** (Codex 2: `CameraIntrinsics` starts at the first
  zoom-map entry, `camera_intrinsics.py:44`, and stopping a track resets to minimum zoom,
  `gimbal_detection_lifecycle.py:74`). `camera.vision_zoom` is not used. The first-zoom lookup is shared with
  `vision_gimbal_profile._first_zoom_fy` (made public), not copied. Projection reuses NavPy's mount transform
  (`camera_pitch`, `seq`, `setup_att`, `setup_seq`) at level attitude, with fx, fy, cx, cy; no distortion.
- **Detection range R** (sim, finite FOV): the `FinitePoiProjector` gate (`finite_poi_projector.py:111-119`):
  `fy*size/MIN_DETECT_PIXELS >= d` or `d <= max_detect_distance` (`build_gimbal_data`), so R = the larger one,
  829 m for the ZR10 dock. Profile `detector.ideal_360` (Codex 8): no finite frame, checks skipped and said so.
  Real mode (Codex 3): the real detector (ChArUco) has no pixel-budget gate, so R is a nominal estimate and results
  are warnings, not errors.
- **Height above the dock** (Codex v1 4, v2 1): missions fly home-relative (`mission_items.py`,
  `MAV_FRAME_GLOBAL_RELATIVE_ALT`), so height = zone altitude (as upload sends it: `zone.altitude_m ?? base +
  (n-1-i)*sep`, one shared helper) minus the dock's home-relative height. A waypoint dock sits `AAS_TARG_ALT` above
  its own ground (`poi_provider.py:147-149`; default 150 m, `navigation_poi_args.py:21`, `aas_params.lua:13`), plus
  the ground difference to home when `AAS_USE_TRN` is on (`poi_altitude.py:105-128`). Both come from the vehicle's
  confirmed AAS params in the GCS (`aasParams`):
  - `AAS_TARG_ALT` unknown: the zone is "not checked: dock height unknown", never assumed.
  - `AAS_USE_TRN` off: exact.
  - `AAS_USE_TRN` on or unknown: the GCS has no terrain source NavPy uses (vehicle terrain, then DEM tiles), so the
    dock ground is taken as home ground and the result is labelled an estimate.
- **Route** (Codex v2 5): one frontend adapter per zone, the same indexing the vehicle uses: mission waypoints
  from one helper extracted from `dockResolver.js` (corridor + track, or the track alone for corridor search and raw
  downloaded tracks), docks = `wps[bit-1]` for each `targ_wps` bit, detection from `wps[nav_last_wp-1]` on (runtime
  detects once the next item is past `nav_last_wp`, `mission_navigation.py:44-45`; 0 means from `wps[0]`). Planning
  takes `targ_wps` / `nav_last_wp` from `buildTargWpsUpdates` / `buildNavLastWpUpdates` (what upload writes);
  monitoring takes the vehicles' values. Level flight (pitch and roll 0), heading along each segment. Not modelled:
  turns, the climb from launch, the mount offset (`setup_dist_mm`), occlusion.
- **Exact visibility per segment** (Codex v1 5): along a segment the dock's camera coordinates are linear in the
  along-track distance s, so each image-edge condition is a linear inequality in s and the range condition a
  quadratic one. Intersecting them gives the exact in-view interval; no sampling steps. h_max uses the steepest
  transformed in-image ray (nadir if in frame, else a closed-form maximum along each image edge), so it holds for
  any mount, not only the ZR10's.

## Checks
Each result is "no detection opportunity on the search route in level flight" (Codex v2 4): turns and the
confirmation orbit are not modelled, and the message says so. Per the owner, one such dock makes the plan
impossible to fly as planned, and the panel says that.
1. **No search window** (per zone with sim docks): above h_max = R*sin(d_max), d_max the steepest in-frame
   depression, no ground point in frame is within R. Message (ground-level dock): "Zone 3 flies 403 m above its dock.
   The image bottom meets the ground 757 m out; detection ends at 829 m slant. Highest workable height 389 m."
   Fixes: lower the zone altitude (preset altitude or zone stack) to at most h_max,
   a steeper camera pitch, another camera profile with a longer search range.
2. **Dock never detectable** (per sim dock): no route segment has it in view. Message names the dock (zone, WP) and
   the window (near edge to range edge). Fixes: put the dock on a lane flown after the detect-after waypoint, move
   the detect-after waypoint, lower the altitude, another camera profile.
Severity: error when exact (sim, finite FOV, `AAS_USE_TRN` off); warning when an estimate (terrain, or real mode).
Per zone the result also lists h, R, the window and the dive angle at its edges.

## UI
- Planning sidebar: a "Camera check" block under the plan summary, red for errors, amber for estimates, with cause
  and fixes. It reruns when the plan, sim docks, detect-after waypoints, altitudes, AAS params or the camera
  selection change (latest request wins). A compatible plan shows nothing; a zone not checked shows one grey line.
- Upload: the check runs on the exact assignments being uploaded (their waypoints, and the `targ_wps` /
  `nav_last_wp` upload is about to write), right before the request. Problems → `window.confirm` with the causes
  and what to do; Cancel stops the upload.
- Launch (Codex v1 7, v2 2-3, v3 1): one guard in `useVehicleCommands` wraps `handleLaunch` (container and
  per-vehicle, force included) and `handleRestart` (bungee Start and Restart), before any command or local reset.
  It keeps no snapshot: it rebuilds the input from the current state, the plan the GCS holds (set by the upload, or
  by a mission download), the vehicles' `targ_wps` / `nav_last_wp` as the GCS last wrote or read them
  (`vehicleTargWps` / `vehicleNavLastWp`), and the current confirmed `AAS_TARG_ALT` / `AAS_USE_TRN`, with the
  current camera. A vehicle without known `targ_wps` has no docks to check (the check covers the docks the GCS
  knows); without known `AAS_TARG_ALT` its zone is "not checked". A failed request shows "Camera check could not
  run: <why>" in the same confirm, never a silent pass. No sim docks: no dialog.
- Unusable answers (code review 1): a missing or malformed body, or a skip other than `ideal_360` (`no_profile`,
  `no_camera`), counts as "could not run" in the panel and in every confirm. The "at most" altitude is rounded
  down (389.58 → 389), so following it passes.
- Window numbers (Codex v3 3): the near edge is where the steepest image ray meets the ground; the far number is the
  detection range edge, named as such (the image may end sooner for other mounts). h_max itself passes (v3 4). A
  dock at or above aircraft height goes through the per-segment view test, not a height rule (v3 2).

## Tests
- Python: h_max and the window against hand numbers (ZR10 first zoom: h_max about 390 m; at 260 m the window is
  about 488-787 m); the fixture zone at 260 m passes; 403 m fails check 1; a dock only before the detect-after
  waypoint fails check 2; the exact interval catches a short in-view stretch a 2 m sampler would miss; AAS_TARG_ALT
  lowers the height and unknown is "not checked"; terrain on is a warning; ideal_360 skips; real mode gives warnings; the route. Each test is proved by breaking the rule
  once.
- Vitest: the zone-altitude helper; the route adapter (corridor, corridor search, raw download, nav_last_wp 0 and
  last); the upload gate (problem → confirm, cancel stops, compatible passes, check failure shown); the Launch and
  Start guard (cancel sends nothing and resets nothing); the sidebar block.
- Live: live-tester loads a plan at the preset altitude (problem shown, upload asks) and at 220 m (nothing shown).
