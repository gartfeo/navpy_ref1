# GCS map: closest approach to the dock (sim only)

Status: design v5, 2026-10-21. Owner approved the 2D mark and asked for the
3D view in section 4 ("yeah something like that", after the mock). An
approach NAV ended early shows as aborted with its reason (owner decision on
todo 108, 2026-10-23); other ends without contact show as missed.

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

- Every joined segment already gets the flat distance. If the flat distance
  minus the flat-earth error bound (`flat_earth_error_bound_m`, the same bound
  the contact filter uses) is below the best exact distance so far, it runs
  the exact geodetic closest point and keeps it if smaller. So no segment that
  could beat the kept one is skipped. Near the dock this is one exact
  computation per sample (a few `pymap3d` calls), on the telemetry thread.
- The contact segment is kept separately, as today.

**Outcome comes from the lifecycle, not the sensor.** `FinalApproachDockContact`
remembers whether its latch accepted DOCKED for the current episode and tells
the sensor at the end: `DockContactSensor.disarm(end: DockEpisodeEnd)` (how
the lifecycle saw the end; nothing flows back). `dock_episode_end` builds it
after the contact is detached, from the episode's terminal latch and the
POI's confirmation state: docked, a latched aft-dock miss, an abort reason
(the `NAV_FAIL` reason, or `cancelled` when the GCS cancelled the confirmed
task), or nothing known.

- **docked**: the reported geometry is the contact segment's, so `d3_m` equals
  the DOCKED row's `closest_3d_m` (the contact segment, not the episode
  minimum: the contact rule includes the chord-bow allowance).
- **aborted** (with `reason`): NAV ended the approach early: a `NAV_FAIL`
  other than the aft-dock miss, or a cancel. With no known reason, an episode
  whose closest point is its last joined sample is aborted too (still
  closing, `reason` null). The geometry is the episode minimum.
- **missed**: every other end (the aft-dock miss, RESET, re-arm, new
  assignment). The geometry is the episode minimum, which may be a contact
  segment the lifecycle rejected.
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
first (docked) or finds the episode detached (missed, not latched): the
report and the lifecycle always agree. Contact fencing is unchanged. The sink
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

- **Marker**: a point (10 px, white outline) at (lon, lat, `alt_rel_m`): the
  trail draws `alt_rel` (`useUavTrails.js:102`), so the mark sits on the trail.
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
    is back above 50 m). A new `closestApproach` pick type in `handleClick.js` works in
    monitor mode, not only while editing. Leg labels and the ghost hide when the camera is farther
    than `SIM_CLOSEST_APPROACH_DETAIL_RANGE_M` (named constant, about 200 m) so
    they do not clutter the overview.
  - All of it shows, hides and clears with the mark (section 5).
- A record without both heights over home (`alt_rel_m`, `dock_alt_rel_m`)
  is consumed and not drawn: it cannot sit on the trail.
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
- The only new flow into the sensor is the lifecycle's `docked` flag; nothing
  flows from the sensor or record to the lifecycle beyond today's event.
- The sink writes stdout and the log only.

## Tests

- Sensor: a miss reports once at disarm with the episode minimum; competing
  segments where the smallest flat distance is not the smallest exact distance
  report the exact minimum; docked reports the contact segment and matches the
  DOCKED distance; contact rejected by the lifecycle reports `missed`; re-arm
  reports the old episode first; no joined segment, no report; gaps and resets
  never form a segment; a throwing sink does not break disarm; relative alt.
- Lifecycle: `disarm(docked)` is true only after an accepted latch of the
  current episode; a contact callback held between sensor sampling and
  lifecycle acceptance while disarm runs ends as missed and unlatched, both in
  disarm and in re-arm.
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
  labels, zooming back in shows them.

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
