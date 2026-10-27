# GCS map: closest approach to the dock (sim only)

Status: design v6, 2026-10-25. Owner approved the 2D mark and asked for the
3D view in section 4 ("yeah something like that", after the mock). An
approach cut short, before its closest pass, shows as aborted with its reason
(owner decision on todo 108, 2026-10-23); an approach that passed the dock
without contact shows as missed (`missed_dock.md`: once it stops closing, no
contact means dock failed); what the UAV flew after NAV announced the end
of the approach never counts. v6 fixes two faults found in the 2026-10-24 live
check: every near miss read "aborted (no_detection)", and the fly-to ended
underground under Ion terrain (section 4).

Owner request (2026-10-20): in simulation mode the GCS map shows each UAV's
closest approach point to its dock after a final approach: the point on the
flight path nearest the true dock, with the miss distance, so the owner can
see how close each approach came.

Display only. Simulation never proves physical docking.

## What exists today (main, 647c40c0)

- `SimDockContactSensor` (`src/navpy/sim/world/dock_contact_sensor.py`) is armed
  per (vehicle, task, own dock uid, episode) at NAV entry and disarmed at NAV
  exit or re-arm (`dock_contact_episode.py:57, 83`). It joins consecutive
  `GLOBAL_POSITION_INT` samples into segments, filters each by a flat-frame
  distance widened by the flat-earth error bound, and runs the exact geodetic
  `closest_point_components_on_segment` (which already returns the point,
  `c_star`) only for segments that could touch the 0.5 m net.
- On contact it publishes one `DockContactEvent` (3D, h, v distance; no
  position) to the lifecycle, which may still reject it (a `NAV_FAIL` latched
  first, `test_final_approach_docked_flow.py:233`).
- The sensor keeps **no episode minimum**, so a miss leaves no distance.
- **Nothing reaches the GCS backend.** In wall-clock sim the backend starts
  every NavPy itself (`NavpyProcessManager`, from `navpy_sim_runtime`) and reads
  its stdout (`read_navpy_output`), which already carries the
  `NAVPY_RUNTIME_READY_V1` record. Lockstep refuses that manager
  (`lockstep/backend_host.py:108`) and starts NavPy through `LockstepLauncher`.
- The backend broadcasts `navpy_sim_status` (instance snapshots, every 2 s) only
  while `sim_mode` is on; the browser keeps it in React state in
  `useWsHandlers` (`navpyStatus`), not in the telemetry store.

## Design

### 1. NavPy: the sensor keeps the episode's exact minimum

While armed, the sensor keeps the episode's best segment by **exact** distance:

- Every joined segment already gets the flat distance; its exact distance
  lies within the flat-earth error bound of it (`flat_earth_error_bound_m`,
  the same bound the contact filter uses). The tracker keeps every segment
  whose lower bound is not above the smallest kept upper bound, unresolved,
  and drops the rest. So no segment that could be the exact minimum is
  skipped. The record resolves the kept candidates with the exact geodetic
  closest point at disarm, off the telemetry thread.
- The sensor samples on the telemetry reader thread, so a sample costs only
  flat-frame work there. (Until 2026-10-27 the exact closest point ran for
  every segment that could beat the kept minimum: every sample while closing,
  at any range. In NAV at 10x that pymap3d work made the reader fall behind
  SITL, todo 65.) A pass keeps one to three candidates. A path that holds its
  range within the bounds (an orbit) keeps adding them; past
  `CLOSEST_APPROACH_MAX_CANDIDATES` (16) the oldest is resolved at once, so
  the reader does at most one exact computation per sample.
- The contact segment is kept separately, as today.

**Outcome comes from the lifecycle, not the sensor.** `FinalApproachDockContact`
remembers whether its latch accepted DOCKED for the current episode and tells
the sensor at the end: `DockContactSensor.disarm(end: DockEpisodeEnd)` (how
the lifecycle saw the end; nothing flows back). `dock_episode_end` builds it
after the contact is detached, from the episode's terminal latch and the
POI's confirmation state: docked, a latched aft-dock miss, NAV's own failure
(`failed`, with the `NAV_FAIL` reason), a cancel (reason `cancelled`, when
the GCS cancelled the confirmed task), or nothing known.

**Evidence cut (v6).** NAV ends the approach itself when its episode latch
records FAILED or MISSED. The latch's existing observer
(`FinalApproachEpisodeLatch(on_terminal)`, today the HITL terminal handoff)
then also calls `DockContactSensor.end_evidence(identity)`, display only.
The latch is NAV endgame code and is not changed.
- **Boundary.** `end_evidence` only stores the identity: one attribute store,
  no lock, no wait. The composition (`app/runtime_composition.py`) makes it
  the first observer, before the HITL handoff, and wraps it so an error is
  logged and never reaches the latch's terminal retirement or the handoff.
  The store is the boundary.
- **Ingress.** The sensor reads the stored identity once at the start of each
  sample, under its lock, before the sample touches any evidence. If it names
  the armed episode, the episode minimum and the pass evidence stop: this
  sample and every later one change neither (they still feed contact
  detection). A sample that read it before the store counts, even if it
  finishes after. The boundary is on ingress, so nothing is frozen at a later
  lock time, no segment straddles it, and the disarm takes the evidence as it
  stood at the boundary. A late sample (older aircraft time) after the store
  is dropped: fail-closed.
- **Known gap (an approximation, accepted 2026-10-25 by the managing chat; todo 132).** The store runs
  one call after the latch records the terminal (`episode_latch.py`,
  `_latch`). A sample read in between counts. That needs the NAV thread to be
  paused by the scheduler inside those few instructions while a position
  sample is processed; nothing bounds how long such a pause lasts. An exact
  boundary needs one of: a hook in the latch before it records the terminal
  (endgame code, which this work must not change), or reading NAV's terminal
  on every position sample, which makes the telemetry reader thread take the
  final-approach command lock 40 times a second (held during law planning
  and command sends) and so changes NAV timing in the sim. Both are left
  out; Codex design round 5 accepts the store as the boundary only with the
  owner's acceptance of this gap.
- A stored identity other than the armed one is ignored. Each episode's
  identity is unique, so a stale store never matches and nothing is cleared.
  DOCKED does not cut.
- Contact detection is unchanged after the cut; the latch is first-wins, so
  the lifecycle refuses a later contact as today.
- The cut takes no lock, so it adds no lock order. With no sensor (the real
  build) there is no cut observer.
- Ends without a latch (cancel, RESET, re-arm, a new assignment) have no
  NAV-side time: their window ends at the NAV-exit disarm, and they are never
  missed (below).

- **docked**: the reported geometry is the contact segment's, so `d3_m` equals
  the DOCKED row's `closest_3d_m` (the contact segment, not the episode
  minimum: the contact rule includes the chord-bow allowance).
- **missed**: the aft-dock miss the lifecycle latched, or NAV's own failure
  ended an approach that had already **passed** the dock: pass evidence before
  the cut. Pass evidence is ordered and kept per continuous run of joined
  segments, by the tracker, for every joined segment (before the flat-filter
  shortcut, so no segment is skipped). Each segment uses the sensor's own
  dock-local frame (`DockFrame.local`, dock at the origin): start `p0`, end
  `p1`, direction `d = p1 - p0`. A segment with `|d|` below
  `_STILL_CLOSING_TOLERANCE_M` has no direction and sets nothing.
  1. *ahead*: `d` is within `PASS_ARM_MAX_OFF_NOSE_DEG` (60 deg, the vision
     pass gate's own cone, `visual_pass.py`, imported) of `-p0`, the line from
     the start sample to the dock.
  2. *passed*: on this or a later segment of the same run after *ahead*, the
     range is opening at the segment's end: `d . p1 / |d|` (the range rate's
     sign, per metre of travel) is above `_STILL_CLOSING_TOLERANCE_M`. This
     catches a crossing inside one segment even when its end is nearer than
     its start; a segment that ends exactly at its closest point, or a
     repeated sample, is not opening.
  A break in the run (an invalid sample, no own dock frame, a dock object
  change, a clock that went back or a gap over `max_segment_s`: every case
  where the sensor refuses to join) clears a pending *ahead*; a proven
  *passed* stays for the episode. Re-arm and disarm clear both with the
  minimum.

  With a finite camera the dock leaves the image at the pass, so a near miss
  ends as `NAV_FAIL: no_detection` before two aft frames can latch the miss
  (live 2026-10-24: 0.72 m, 1.30 m). The frame that shows the loss is
  processed after the pass (in the 2026-10-24 run 1 the navigation snapshot's
  closest point lies before the `NAV_FAIL`), and position samples come at
  40 Hz, so the pass evidence is normally in before the cut. A near miss
  whose cut comes before the sensor received a sample past the closest point
  stays aborted (fail-closed). A missed record has no `reason`; the NAV
  reason stays in the NavPy log. The geometry is the episode minimum up to
  the cut, which may be a contact segment the lifecycle rejected.
  - **Why a cut, not the armed window** (Codex design round 3): a 100 m
    radius turn about 300 m out sets *ahead* on its first segment and its
    range bottoms out at 216 m with the dock abeam; a cancel 0.05 s before
    that and a disarm 0.075 s after it would read "missed 216 m" if evidence
    ran to the disarm. With the cut only NAV's own latch can make a missed,
    and only on evidence before it.
- **aborted** (with `reason` when known): every other end without contact.
  - NAV's own failure with no *passed* evidence before the cut: the UAV never
    had the dock clearly ahead (still turning in from the waiting circle,
    where the dock is about 90 deg abeam, even if the orbit made a small range
    dip), or it was still closing at the cut. The geometry is the minimum up
    to the cut.
  - A cancel, RESET, re-arm or new assignment, whatever the path: NAV did not
    end the approach and the sensor has no cut time for it, so it is never
    missed (fail-closed). Its geometry is the armed window's minimum (to the
    NAV-exit disarm, which can include the short cleanup flight after a
    cancel); the label never depends on it.
  - `reason` is the `NAV_FAIL` reason or `cancelled`, else null (RESET,
    re-arm, new assignment, unknown).
  This replaces the v5 "closest point is the last sample" test.
- The pass test is sim truth for display only: it reads the joined position
  samples the sensor already has and nothing else, and it never reaches the
  lifecycle (section 7). The cone constant is imported, not copied.
- No joined segment in the episode: no report.

**Threads.** The outcome and the end of the episode are decided in one place,
under a lock in `FinalApproachDockContact`:

- The contact callback, under that lock, checks the event's identity is still
  the armed episode, then calls the latch and records whether it accepted.
  A callback for a detached episode is ignored, as today's latch already
  rejects a stale episode.
- `disarm()` (and the disarm inside re-arm), under that lock, detaches the
  identity and reads the accepted flag. Then, outside that lock, it calls
  `sensor.disarm(docked)`, which takes the kept geometry and clears the
  episode under the sensor lock and reports outside it.

So a contact callback captured by the sensor before the detach either latched
first (docked) or finds the episode detached (not latched, and aborted: no
terminal latched): the report and the lifecycle always agree. Contact fencing is unchanged. The sink
is wrapped: an exception is logged and never reaches NAV cleanup.

`ClosestApproach` (new, `src/navpy/sim/world/closest_approach_record.py`):
vehicle, task id, dock uid, episode, outcome, aircraft time, point (lat, lon,
AMSL alt, and relative alt = point AMSL alt − (sample AMSL alt − sample
`relative_alt`) of the segment's start sample), 3D / h / v distance, the dock centre (lat, lon, AMSL alt, and relative alt
with the same home offset), and the UAV attitude at that moment (roll, pitch,
yaw in degrees). The position sample gains `relative_alt` from the same
`GLOBAL_POSITION_INT`. The feed also subscribes to `ATTITUDE` (the autopilot's
estimated attitude, not sim truth) and keeps the newest one with its
`time_boot_ms`. A position sample gets that attitude only when it is finite and
its aircraft time is within `max_segment_s` of the sample's (the same two
position periods that join samples), on either side; a clock that went back, a
missing, stale or non-finite attitude gives none. The record takes the
attitude of the segment end nearer the closest point; without one the record
has no attitude and the map draws no ghost. Contact and distance never depend
on attitude: position samples are fed exactly as today. Yaw here is display
only, which the Pure Vision Approach Constraint allows; it never leaves the
record.

### 2. NavPy to backend (wall-clock sim)

The sim composition (`sim_detector_factory.py`) gives the sensor a report sink.
The record module owns the marker, the encoder and the strict parser (same
pattern as `runtime_ready.py`: exact prefix, JSON object, typed finite fields,
else ignored), and a replaceable publisher whose default writes one flushed
stdout line:

```
NAVPY_SIM_CLOSEST_APPROACH_V1 {"vehicle":2,"task_id":7,"dock_uid":3,"episode":1,"outcome":"missed","reason":null,"aircraft_time_s":512.3,"lat":..,"lon":..,"alt_amsl_m":..,"alt_rel_m":..,"d3_m":1.84,"h_m":1.52,"v_m":1.04}
```

Every record is also written to the NavPy log (evidence in every mode).

**Lockstep is out of scope for the map**: lockstep NavPys keep the log line,
and a lockstep publisher (through its `DONE` events, like `runtime_ready`) is
a todo in `architecture_todo.md`. The owner's demo and the normal sim app are
wall-clock.

No new MAVLink message: nothing goes on the swarm link, so peers never see it.

### 3. Backend

- `NavpyProcessManager` gives each started process a generation number
  (monotonic per manager), and the backend has a random `boot_id` per start.
- `read_navpy_output` parses the record. A record whose vehicle differs from
  the instance's `sys_id` is dropped with a log line. A valid record goes to a
  manager-level store (`SimClosestApproachStore`), not to the process
  instance, so it survives Stop, auto-restart and replacement of that process
  (the manager deletes instances, `navpy_process_manager.py:68, 103`). The
  store keeps the newest `SIM_CLOSEST_APPROACH_KEEP = 20` records per `sys_id`
  (a demo has a few approaches per UAV) for the backend's life.
- Each record gets `key = "{boot_id}:{generation}:{episode}"`, unique across
  NavPy restarts, reused PIDs and backend restarts.
- `broadcast_sim_status` adds a top-level `closest_approaches` list from the
  store next to `instances`.
- It reaches the browser in the existing sim-only `navpy_sim_status`
  broadcast. No new endpoint or WebSocket message type.

### 4. Map

`useWsHandlers` keeps the message's top-level `closest_approaches` in its own
state (`simClosestApproaches`); the existing `navpyStatus` map (built from
`instances`, indexed by vehicle id by the sidebar controls) is unchanged.
`App` passes `ws.simClosestApproaches` and `simMode` to `CesiumMap`, which passes them,
with the Trails controls (`showTrails`, `hiddenSysIds`, `historyPaused`) and
the same composite reset the trail gets (`${trackResetKey}:${historyResetKey}`:
Restart mission bumps `trackResetKey`, Clear all `historyResetKey`,
`CesiumMap.jsx`), to a new hook
`useClosestApproachMarks`, with the same `hiddenSysIds`, `historyPaused`
and `${trackResetKey}:${historyResetKey}` as `useUavTrails` (todo 107). Fleet membership comes from the telemetry store
(`storeRef`, its vehicle list), as for the trail: `navpyStatus` lists companion
processes, not the fleet.

- **Height datum (v6)**: the record's heights are over home, not over the
  WGS84 ellipsoid. v5 drew them as ellipsoid heights; with Ion World Terrain
  (ellipsoidal, home 1295 m AMSL) the whole mark and the fly-to destination
  were about 1.3 km underground, and Cesium's collision step lifted the camera
  to the ground straight above them (live check 2026-10-24). The mark is
  anchored to the dock icon instead: `base = ground(dock) - dock_alt_rel_m`,
  where `ground(dock)` is the ground the dock icon is clamped to. Every part
  is drawn at `base + alt_rel_m` (or `+ dock_alt_rel_m`), so the dock dot sits
  on the dock icon and the triangle keeps its exact h and v legs. A dock
  standing above the ground (`height_m > 0`) is deliberately moved down with
  its icon: the triangle's relative geometry is exact, its absolute height is
  the icon's, not the dock's.
  - `ground(dock)` while `terrainReady` is false is 0: the globe is the
    ellipsoid, which is the ground the icon is clamped to.
  - Once `terrainReady` is true it is the most-detailed terrain sample at the
    dock (`sampleTerrainMostDetailed`). A rejected or non-finite sample falls
    back to the rendered globe height there (`globe.getHeight`) when that is
    defined; otherwise the mark has no ground yet and is **not drawn** (never
    at 0 over real terrain); its sample is tried again at the next
    `navpy_sim_status` (every 2 s, the hook's existing input), while the mark
    is retained.
  - A sample is asked for every retained mark when terrain becomes ready and
    for every new mark while it is ready. A completion applies only if the
    same mark object is still retained under its key and the terrain
    generation (bumped when `terrainReady` changes or the hook unmounts) is
    unchanged; it then redraws that mark with the current visibility. It never
    re-adds a cleared, evicted or removed mark, never consumes a key and
    never draws while sim mode is off. Hide, pause (keeps existing marks,
    discards new records), clear, fleet removal and the cap are unchanged.
- **Marker**: a point (10 px, white outline) at (lon, lat, `base +
  alt_rel_m`). The trail (`useUavTrails.js`) still draws raw `alt_rel`, so
  under Ion terrain it is underground; that is the trail's own datum fault
  and a separate todo, so the mark no longer claims to sit on the trail.
- **Colour and text**: docked green, missed red, aborted amber with its
  reason (`aborted (source_command_stale)`), and the label says it in words,
  so colour is never the only cue.
- **Label**, two lines, dark background, above the point, distance-scaled like
  the dock labels: `UAV 2 · 1.84 m missed` / `h 1.52 m · v 1.04 m`. No click or
  tooltip is needed (the Cesium info box is off).
- **3D view** (owner, 2026-10-21, from the mock), drawn with the mark:
  - **Ghost** (only when the record has an attitude): the same `/Peron15.glb` model as the live UAV, in the UAV's
    colour, about 35% opaque, no silhouette pulse, at the closest point with the
    recorded attitude (same heading/pitch/roll to quaternion conversion as
    `useUavMarkers.js:65-71`, moved into a shared helper). Cesium cannot draw a
    model dotted; dimmed is the agreed look.
  - **Dock centre**: a small white dot at the dock's true position and height
    (`dock_alt_rel_m`).
  - **Distance triangle**: from the dock centre, a dashed horizontal leg at dock
    height to the point straight below or above the closest point (`h`), a
    dashed vertical leg to the closest point (`v`), and a thin solid line from
    the dock centre to the closest point (3D, outcome colour). Each leg carries
    its own small label (`h 1.52 m`, `v 1.04 m`).
  - At true scale a miss of a metre is a few pixels at normal zoom, so the
    two-line label stays the main readout. **Tapping the label or the point flies
    the camera to the mark** (`viewer.camera.flyTo` to an explicit oblique
    destination about 15 m behind and 6 m above the closest point along the
    recorded heading, or from the south without one, looking at the triangle;
    explicit because automatic framing clamps to the viewer's 50 m minimum
    zoom; the camera controller also lifts the camera to that minimum over
    the ground, so the fly-to lowers it to 2 m and restores it once the camera
    is back above 50 m, checked after each rendered frame: `camera.changed`
    fires only after a move of half the camera height). The target is the drawn point (`base +
    alt_rel_m`). The point may sit at or just below the ground (a docked
    UAV at dock height, or below it), and the ground behind it may rise, so
    the click reads the ground under the destination itself: the rendered
    globe height there (`globe.getHeight`, the height the collision step
    uses), synchronously. If it is not finite the camera does not fly;
    nothing else stands in for it, and a later click flies once that ground
    is rendered. If the destination is less than
    `CLOSEST_APPROACH_VIEW_UP_M` over it, it is raised to that clearance.
    The camera looks along the line from the destination to the drawn point
    (direction and up, not the fixed pitch), so the point is in the centre
    of the frame.
    - Finer tiles load during the flight and can raise the ground, and the
      collision step can then lift the camera. When the flight completes,
      the clearance is applied again over the ground now rendered under the
      camera, and the camera is aimed again at the drawn point.
      Over coarse tiles the rendered ground can be metres above the real
      ground, so the camera then ends higher than 6 m over the point, still
      aimed at it (live check 2026-10-25 at 100% CPU: 13.7 and 17.7 m; 6.0 m
      once the fine tiles had loaded).
    - That arrival step runs only if this is still the viewer's latest
      flight, the viewer is not destroyed, and the mark's entity is still in
      the viewer and showing (not cleared, removed, evicted, hidden or gone
      with sim off). A flight the operator interrupts only restores the
      minimum zoom, as today.
    - After that the camera is the operator's: the collision step keeps it at
      least 2 m over whatever ground is rendered, and nothing re-aims it,
      which would fight the operator's own moves.
    - The fly-to has no asynchronous sample, so nothing can start a flight
      after Clear, fleet removal, sim off or unmount. A new `closestApproach` pick type in `handleClick.js` works in
    monitor mode, not only while editing. Leg labels and the ghost hide when the camera is farther
    than `SIM_CLOSEST_APPROACH_DETAIL_RANGE_M` (named constant, about 200 m) so
    they do not clutter the overview.
  - All of it shows, hides and clears with the mark (section 5).
- A record without both heights over home (`alt_rel_m`, `dock_alt_rel_m`)
  is consumed and not drawn: it cannot be anchored to the dock.
- One mark per episode; a UAV with several approaches shows several. The
  browser keeps at most `SIM_CLOSEST_APPROACH_MARKS_PER_UAV = 20` per UAV,
  oldest dropped.

### 5. When it appears and when it is cleared

The marks belong to the flight trail and follow the Trails control
(`gcs_trails_marks_control.md`). The hook **consumes** each record key once,
the first time a status shows it: it is added as a mark only if, at that
moment, sim mode is on, recording is not paused, the UAV is in the fleet
(telemetry store) and this is not the first status after page load. A consumed key is never added again, so a replayed
status cannot restore cleared or discarded history. Consumed keys no longer in
the latest status are forgotten (the backend never shows them again), so the
set stays bounded.

| Event | Closest-approach marks |
|---|---|
| Episode ends | added at the next `navpy_sim_status` (within ~2 s) |
| Flight trail layer off; UAV unchecked | hidden, kept |
| Pause recording | a record first seen while paused is consumed and never shown |
| Resume; Restart mission (also resumes) | records from then on are shown |
| Clear all, Restart mission | marks removed; their keys stay consumed |
| WebSocket drop and reconnect | kept; a record made during the drop is added on reconnect (intended difference from the trail: it is a computed result, not a sampled position, so it draws no false line) |
| Stop / Start NavPy, auto-restart | kept; a record made just before the stop is still broadcast (backend store); new records have new keys |
| Backend restart | kept in the browser; the backend store starts empty |
| UAV leaves the fleet (telemetry store) | its marks removed; records seen while it is absent are consumed unshown; a returning UAV shows only new records |
| Its NavPy companion stops (UAV stays in the fleet) | kept |
| Plan change | kept |
| Browser reload | lost, as the trail: the first status's records are consumed unshown |
| Sim mode off | marks removed, nothing added; on again shows only new records |

Accepted edge: an approach that ends up to ~2 s before Clear all can still
appear after it, because the status broadcast is 2 s.

### 6. Sim only

- Only the sim composition builds the sensor and sink; a real aircraft wires
  neither (`sim_detector_factory.py:61`, simulated autopilot only).
- Only backend-started NavPy processes have their stdout read; the backend
  starts NavPy only in sim mode (`_ensure_sim_mode`, auto-start in sim mode).
- `navpy_sim_status` is broadcast in sim mode (plus one final status when sim
  is turned off, `routes/settings.py:138`); the hook draws nothing when
  `simMode` is false.

### 7. Pure Vision Approach Constraint

The record is display evidence only:

- The guard (`tests/guards/test_dock_contact_isolation.py`) gets a separate
  rule for the record: only the sensor, the record module, the sim composition
  and the backend reader may import `closest_approach_record`, and its field
  names (`d3_m`, `h_m`, `v_m`, `alt_rel_m`, `alt_amsl_m`, `dock_alt_rel_m`,
  `dock_alt_amsl_m`, `dock_lat`, `dock_lon`) are guarded in the
  command path as attributes, literal subscripts (`x["d3_m"]`) and `.get("d3_m")`
  calls. Negative fixtures prove each of these, and an import of the module,
  fails the guard.
- The flows into the sensor are the lifecycle's end (`DockEpisodeEnd`) and
  the latch's evidence cut (the identity only); nothing flows from the sensor
  or record to the lifecycle beyond today's event.
- The sink writes stdout and the log only.

## Tests

- Sensor: a miss reports once at disarm with the episode minimum; competing
  segments where the smallest flat distance is not the smallest exact distance
  report the exact minimum; a 500 m approach runs no exact geometry while
  sampling, and an orbit at most one per sample, both still reporting the
  exact minimum; docked reports the contact segment and matches the
  DOCKED distance; a contact the lifecycle rejected with no terminal latched
  reports `aborted`, and with FAILED latched first after pass evidence,
  `missed`; re-arm
  reports the old episode first; no joined segment, no report; gaps and resets
  never form a segment; a throwing sink does not break disarm; relative alt.
- Lifecycle: `disarm(docked)` is true only after an accepted latch of the
  current episode; a contact callback held between sensor sampling and
  lifecycle acceptance while disarm runs ends unlatched and aborted (no
  terminal latched), both in disarm and in re-arm; an accepted contact
  reports docked.
- Record: round trip; near-miss prefixes, bad JSON, missing or non-finite
  fields ignored.
- Backend: reader keeps valid records, bounded per `sys_id`; wrong vehicle
  dropped; keys differ across generations with the same PID and episode; a
  record followed by process stop or replacement before the next broadcast is
  still in it.
- Guard: positive and negative fixtures as above.
- Frontend (vitest, from a `navpy_sim_status` message with top-level
  `closest_approaches` through `useWsHandlers` and `App` to entity creation
  with a fake viewer; `navpyStatus` and the sidebar companion controls still
  work): label text and colour per outcome; position
  uses `alt_rel_m` (nonzero home elevation); ghost position and orientation
  from the recorded attitude; dock dot at `dock_alt_rel_m`; triangle legs end at
  (dock) → (closest lon/lat, dock height) → (closest point); leg labels match
  `h_m` and `v_m`; tap flies the camera; ghost and leg labels hidden beyond the
  detail range; every part follows hide and clear; hide with the layer and the
  checkbox; pause → resume; Clear all and Restart (through the real
  `useVehicleCommands` handlers) do not re-add; disconnect → reconnect; fleet
  removal → return; companion stop keeps marks; sim off → on; reload consumes
  the first status; per-UAV cap.
- `npm test`, `npm run build`, `python -m pytest tests`.
- Outcome (v6): a pass within 0.72 m that ends with `no_detection` one
  sample after the closest point is missed with no reason; the same path
  ending at its closest sample is aborted (`no_detection`); a cancel or a
  `NAV_FAIL` while closing is aborted with its reason; a cancel or an
  unknown end after the pass is aborted (`cancelled`, null); an episode that never had the dock within the cone (an
  abeam orbit with a tiny interior range dip, an opening range) is aborted
  with its reason; an invalid reason on an aborted record is dropped; an
  aft-latched miss stays missed; a contact the lifecycle rejected after the
  pass is aborted when nothing latched, and missed when FAILED latched after
  the pass (the minimum up to the cut).
- Outcome ordering (v6, Codex design r1): an orbit that sets the minimum,
  then an inbound leg cut while closing, is aborted; closing then a repeated
  equal sample is aborted; ahead, then a break (gap, clock reset, dock
  change, invalid sample), then an opening segment is aborted; a straight
  segment from (-2, 0.72) to (0.5, 0.72) m (closest point inside, end
  nearer than start) is a pass, and so is a symmetric one; a segment ending
  exactly at its closest point is not; a stationary segment sets nothing;
  re-arm clears the pass flags; the pass evidence is updated on segments the
  flat filter skips.
- Evidence cut (v6, Codex design r3): the 0.72 m pass with its `no_detection`
  cut one sample after the closest point is missed; cut at the closest
  sample it is aborted (`no_detection`) with the minimum at the cut, though
  later samples before the disarm pass the dock; a sample received after the
  cut with an older aircraft time is dropped; a cut for another identity is
  ignored, and a stale one never cuts the next episode; a contact after the cut still
  publishes (the lifecycle refuses it) and the record keeps the cut minimum;
  Codex's 100 m turn with a cancel at -0.05 s and the disarm at +0.075 s is
  aborted (`cancelled`), and with a `NAV_FAIL` cut at -0.05 s aborted with
  its reason; `dock_episode_end` sets `failed` for FAILED only; the
  composed observer stores the cut, then calls the HITL handoff, for FAILED
  and MISSED, and only calls the handoff for DOCKED; with no sensor nothing
  is cut.
- Cut boundary (v6, Codex design r4): with the cut stored and no sample read
  since, an opening segment and a lower minimum fed next are both excluded;
  a sample held after its ingress read while the cut is stored still counts,
  and the next one does not; in the composed latch, a HITL handoff that
  blocks while another thread feeds an opening segment and a lower minimum
  leaves both excluded; a cut that raises is logged, the handoff still runs
  and the FAILED latch still retires commands.
- Frontend datum (v6): with terrain at the dock 1310 m the point, ghost and
  corner are at `1310 - dock_alt_rel_m + alt_rel_m` and the dock dot at 1310;
  the legs keep `h_m` and `v_m`; with no terrain the dock dot is at 0; marks
  drawn before terrain is ready are redrawn once it is, keys not consumed
  again; a rejected or non-finite sample with terrain active uses the globe
  height, else leaves the mark undrawn until a later sample succeeds (never at
  0); a completion after Clear, eviction, fleet removal, sim off, a terrain
  change or unmount changes nothing; hidden marks redraw hidden; pause keeps
  existing marks and their redraw; the fly-to destination clears the ground
  under it by `CLOSEST_APPROACH_VIEW_UP_M` on flat, rising and falling ground
  and for a point below the dock, and the camera direction points at the drawn
  point; higher ground behind the mark (1330 m at the destination, 1310 m at
  the dock) raises the destination to at least 1336 m; an undefined height at
  the destination starts no flight; a completed flight over ground that rose
  to 1340 m ends at least 1346 m and aimed at the drawn point; completion
  after the mark was removed or hidden, after a newer flight, or with the
  viewer destroyed does not re-aim; a cancelled flight does not re-aim.
- Feed and sensor: attitude missing, stale (older or newer than the bound),
  non-finite and from before a clock reset gives no attitude; contact and
  distances are unchanged with or without attitude; the nearer segment end's
  attitude is used.
- Live (`live-tester`, own slot, 3-UAV sim delivery run): each UAV's mark
  appears after its approach on its trail, with the outcome and a distance
  matching its NavPy log line; layer, checkbox, pause and Clear all behave as
  in the table. 3D, with screenshots: tap the label and the point in monitor
  mode and the camera flies close and oblique; the ghost loads translucent with
  a plausible attitude; the dock dot and the triangle legs are visible with
  their labels; zooming out past the detail range hides the ghost and leg
  labels, zooming back in shows them. v6, with Ion terrain loaded: the dock
  dot sits on the dock icon; tapping a mark leaves the camera about 16 m from
  it at about -22 deg pitch, the mark in frame, and the camera tilts; a near
  miss that ends `no_detection` shows red `missed`.

## Out of scope

Lockstep transport (todo), moving docks, real-flight capture evidence, a
separate Trails checkbox for these marks.

## Review

Codex round 1 (2026-10-20, `codex exec -s read-only`): approve after fixes,
10 findings, all fixed in v2: outcome from the accepted latch, exact episode
minimum, lockstep scoped out, `alt_rel` display height, consume-once
lifecycle, boot and generation keys, frontend wiring, a real guard rule, reuse
of `c_star`, h and v in the label. Round 2: approve after fixes, 4 findings,
all fixed in v3: outcome and detach under the lifecycle lock, the composite
reset key, fleet membership from the telemetry store, dictionary reads in the
guard. Round 3: approve after fixes, 1 finding, fixed in v4: records are kept
in a manager-level store, not in the process instance, so a stop or restart
before the next broadcast does not lose them.
Round 4: one finding, fixed as Codex proposed: `useWsHandlers` keeps the
top-level `closest_approaches` in its own state.
v5 adds the owner's 3D view (ghost, dock centre, distance triangle, tap to
fly). Codex round 5: approve after two fixes, both in v5: attitude timing,
staleness and missing-data policy (no ghost, distances unaffected), and live
checks for the 3D view; also the explicit fly-to destination.
v6 (2026-10-25): the pass rule for missed vs aborted and the dock-anchored
height datum, from the 2026-10-24 live check. Codex design round 1: rework,
7 findings, all taken: ordered pass evidence with its window, no 0 ground
over real terrain, fly-to clearance and aim, stale terrain completions, and
(todo 126) three download outcomes and the AasTab entry. Round 2: rework, 4
findings: a run break clears pending *ahead* and opening is the range rate at
the segment end (taken); destination ground and re-aim after arrival
(taken); an evidence cutoff at the failure or cancel time (not taken then).
Round 3: rework, 3 findings, all taken: a turn 300 m out with a cancel just
before its closest point read missed under the armed window, so evidence now
stops at NAV's own latch and a cancel or other outside end is never missed;
the dock-ground fallback could not prove destination clearance, so the
fly-to needs a finite ground at the destination and re-applies clearance and
aim on arrival; the asynchronous fly-to sample is gone, and the arrival step
checks flight, viewer and mark. Round 4: approve after fixes, 2 findings,
both taken: freezing the tracker when the cut got the sensor lock let a
sample read after the latch count, so the cut is now a lock-free store by the
first observer and each sample reads it on ingress (the latch unchanged);
the lifecycle and sensor test requirements still expected missed with no
terminal latched and now expect aborted. Round 5: approve after fixes, 1
finding: samples between the latch's terminal record and its observer call
still count; the exact alternatives (a latch hook, or reading the terminal
under the command lock per sample) change endgame code or NAV timing, so the
gap is stated above as an approximation for the owner to accept.
Implementation (2026-10-25): Codex implementation round 1 approve, no
findings. A further adversarial review found two faults, both fixed: the
sensor module had passed the 300-line guard (the net-radius setting moved to
`sim/world/dock_net_radius.py`), and the minimum-zoom watch on
`camera.changed` restored the minimum hundreds of metres late over high
ground (now `scene.postRender`). Codex implementation round 2: approve, no
findings.
Live check (2026-10-25, `.logs/closest-approach-live/v6`, PC at 100% CPU
from other stacks): with the net radius 0, all three approaches ended missed
with red marks; a normal run gave one docked green mark and two aborted
(`source_command_stale`) amber marks. Each label matched its
`SIM_CLOSEST_APPROACH` line. Dock dots sat on the sampled ground (within
0.03 m of the rendered ground once fine tiles loaded). Flights over loaded
tiles ended 15.0 m behind and 6.0 m above the point, with the point in the
centre and the dock in frame; the minimum zoom read 50 m again after
zooming out (tester readback). No mission request answered 404. Follow-ups:
todos 129-131.
