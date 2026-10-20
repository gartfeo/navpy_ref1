# GCS trails and camera marks: one control

Status: design v3, approved by the owner 2026-10-20. Implemented 2026-10-20.

Owner request (2026-10-20): the monitor map has separate show and clear
buttons for the camera marks and for the UAV trail; they should become one
control, a button that opens a panel with pause, the UAVs whose history is
drawn, and clear. Open question from the owner: on deselecting a UAV, keep
its trail but stop growing it (pause), or wipe it (clear)?

## What exists today (main, c59303be)

Top-right bar, `MonitorMapOverlay.jsx:106-159`: Vision toggle, Coverage
toggle, Clear coverage, Trail toggle, Clear trail. "Camera marks" below means
the coverage layer: the ground footprints the camera has swept. The vision
toggle (live detection cones) is not history and stays as it is. Both history
layers live in refs inside hooks of the one `CesiumMap`, which stays mounted
in every view; the map swap only restyles its box (`App.jsx:620-634`).

| | Trail (`useUavTrails.js`) | Camera marks (`useCoverageLayer.js`) |
|---|---|---|
| Keyed by | `sys_id` | index in the vehicle list (`:115-116`) |
| Recorded when | always, every telemetry update, 2 m step (`trailAccum.js`) | only while the layer is shown (`:91`), mode AUTO (`:111`), every `COVERAGE_SAMPLE_DIST` |
| Hide (toggle off) | hides, keeps recording (`:33-38`) | hides and stops recording |
| Size limit | 2000 points per UAV, older part thinned | 500 polygons in all; the eviction loop (`:156-172`) always takes from the first key, so list index 0 loses its marks first |
| Clear | all UAVs (`trailResetKey`) | all UAVs (`coverageResetKey`) |
| Restart mission | cleared on the click, before the backend answers (`useVehicleCommands.js:60-66`, `CesiumMap.jsx:293-294`) | cleared, same |
| UAV leaves the fleet (`active_ids`, `telemetryStore.js:128-133`) | removed (`:111-122`) | kept under its old index, so it is drawn in the next UAV's colour and a later UAV at that index adds to it |
| Plan change | kept | kept |
| Browser reload | lost | lost |

So the two button pairs do the same job with different rules, and the marks
are tied to list position, not to the UAV.

The fleet seen by the browser: `active_ids` comes with each telemetry
broadcast, which the backend sends only when something changed
(`telemetry_loop.py:217-226`). A WebSocket drop keeps the store's vehicles
(`telemetryStore.js:146-151`). After a backend restart with no vehicles yet,
nothing is broadcast, so the old vehicles stay in the store until the next
broadcast; a UAV removed and returned while the browser was not connected is
never seen as leaving.

## Practice in established ground stations

- **QGroundControl** (commit 6133ce6d; `Vehicle.cc:261-262, 1247-1260`,
  `TrajectoryPoints.cc`, `FlyViewMap.qml:168-189`): every vehicle records its
  trajectory in the background from arming; disarming stops recording and keeps
  the points; the next arm, a mission upload or new mission items clear them.
  The map draws only the active vehicle's trajectory and camera trigger points;
  switching the active vehicle redraws that vehicle's retained trajectory. So
  deselecting is **hide, keep recording**. No length cap (points merge along
  straight legs). No clear button found in the Fly view map.
- **Mission Planner** (commit ce3cb94e; `FlightData.cs:1103-1110,
  3899-3905`): one **Clear Track** button clears the drawn track and the
  current MAV's camera trigger points together; loading a log and scrubbing
  the log slider call it too. The track keeps the newest `NUM_tracklength`
  points (default 200).

Camera trigger points are shot positions, not swept footprints, so the match
with our marks is loose. Still, both stations handle track and camera points
as one history, which supports one control. No pause control was found in
either.

## Design

### The control

The four history buttons (Coverage toggle, Clear coverage, Trail toggle, Clear
trail) are replaced by one button in the top-right bar, **Trails**, with the
trail glyph and a caret; accessible name "Trails and camera marks". It keeps
the bar's 32 px size (owner, 2026-10-18, `gcs_video_streams.md` section 5) and
is reachable by keyboard (Enter or Space opens it). It shows a text badge
**Paused** while paused, so a frozen picture is never mistaken for live (never
colour alone). The Vision toggle stays.

A tap opens a panel under the button:

```
Trails and camera marks
  [x] Flight trail      [x] Camera marks
  UAVs   [x] UAV 1   [x] UAV 2   [ ] UAV 3      All · None
  [ Pause recording ]                 [ Clear all ]
  (when paused) Not recording since 14:03:12
```

- **Layers**: Flight trail and Camera marks, each on or off for the whole
  fleet. Off hides; recording goes on.
- **UAVs**: one checkbox per UAV in the fleet, with its zone colour swatch
  and the "UAV n" label. Unchecked hides that UAV's trail and marks; recording
  goes on. All / None set every box.
- **Pause recording / Resume recording**: one switch for the whole fleet,
  including UAVs that join or return while paused; while paused no UAV adds
  trail points or marks. Resume starts a new trail run for every UAV.
- **Clear all**: wipes trail and marks for every UAV, shown or hidden, and
  resets the marks' sampling anchors, so the next point and the next mark come
  with the next eligible sample. Pause stays as it was. No confirmation: it is
  display only and touches no mission or log data.
- The live camera footprint shows when Vision is on, or when Camera marks is
  on and that UAV is checked. Pause does not affect it (it is live, not
  history).
- Recording reads only fresh telemetry: a UAV adds a trail point or a mark
  only while `telemetryStore.isTelemetryFresh(sys_id)` holds (socket open and
  that UAV's snapshot replaced since it opened, `telemetryStore.js:68-71`).
  The store notifies with cached positions while disconnected and on reopen
  (`:102-105, 146-149`); those never record.
- Unchanged: marks are taken only in AUTO, when the UAV has moved
  `COVERAGE_SAMPLE_DIST` or a gimbal's state changed (`useCoverageLayer.js:118-123`);
  trail points at the same 2 m step.

Panel rules follow the vehicle window (`gcs_video_streams.md`, section 5):
44 px targets with text labels, no hover-only actions; it opens downward, draws
above the vehicle window and the small swapped map, scrolls inside itself on a
short or narrow pane (the UAV row wraps), and closes on a tap outside or
Escape (focus back to the button). Nothing in it is saved in browser storage
(same rule as the vehicle window); a reload starts with all UAVs checked,
recording, the trail layer on and the camera marks layer off (today's
defaults; marks record while hidden, so turning the layer on shows them).

### Select and deselect: recommendation

**Deselect hides and keeps recording** (neither pause nor clear). Reselect
shows the retained history at once.

- Clear on deselect turns a viewing click into data loss: unchecking UAV 2 to
  read UAV 3's path would wipe UAV 2's flight.
- Pause on deselect makes a gap that the operator did not ask for, and it
  differs per UAV, so two trails no longer cover the same time span; joined
  across the gap it would show a straight-line flight that never happened.
- It is QGroundControl's behaviour, and the cost stays bounded: the trail
  budget per UAV and the marks cap are unchanged. "Retained" because thinning
  and the marks cap already drop detail on long flights.
- Pause is kept as its own explicit, fleet-wide switch for the owner's case
  ("keep the trail but do not grow it"), with the Paused badge.

### Trail runs

A trail is a list of runs; runs are never joined. A new run starts on Resume
and on every WebSocket reconnect (`ws.onopen`), and its first point is that
UAV's first fresh position afterwards (above), never a cached one; so neither a
pause nor a link gap draws a straight line, and a UAV that left and came back
unseen during an outage continues in a new run. The 2000-point budget covers all of a UAV's
runs together. Thinning works inside each run as today and keeps each run's
two ends; if the runs' ends alone would exceed the budget, the oldest run is
dropped whole. A run with fewer than two points is not drawn. Each run is
drawn with the existing fade over the whole trail.

### Lifecycle

| Event (as the browser sees it) | Trail and marks | Selection, layers, pause |
|---|---|---|
| UAV joins the fleet | records, unless paused | checked |
| WebSocket drop and reconnect | kept; nothing recorded during the drop; a new trail run starts | kept |
| Backend restart, browser open | as a WebSocket reconnect; UAVs absent from the first broadcast's `active_ids` are removed | kept |
| Vehicle link lost (vehicle stays in `active_ids`) | kept; no new points while no telemetry | kept |
| UAV leaves the fleet (removed from `active_ids`) | removed, trail and marks (marks fix) | its checkbox goes; its check state is remembered by `sys_id` for the browser session |
| Same `sys_id` returns after a seen removal | fresh history, as today | restored check state |
| Stop / Start NavPy (companion processes) | kept: the vehicles stay connected | kept |
| Restart mission | cleared on the click, as today, whatever the backend answers | pause off (a new mission is recorded); selection and layers kept |
| Plan change (edit or upload) | kept | kept |
| Browser reload | lost, as today | defaults |

Plan change differs from QGroundControl, which clears on mission upload. Kept
here because a plan edit in flight should not wipe what was flown; Clear all
is one tap. Restart mission is the point where a new history starts.

### Changes

- `useDisplaySettings`: `showCoverage` and `showTrails` become the two layer
  flags; add `hiddenSysIds` (a set; absent means checked) and `paused`.
- `useVehicleCommands`: one `historyResetKey` replaces `coverageResetKey` and
  `trailResetKey`; `handleRestart` also sets `paused` false.
- `App.jsx` and `CesiumMap.jsx`: pass the layer flags, `hiddenSysIds`,
  `paused` and the reset key to both history hooks and to the overlay.
- `trailAccum.js`: runs, run ends kept by thinning, oldest run dropped when
  over budget.
- `useUavTrails`: per-UAV show = layer and checked; skip `accumulate` while
  paused or not fresh; new run on resume and on WebSocket reconnect (a
  reconnect counter from the telemetry store).
- `useCoverageLayer`: key by `sys_id`; record whether or not the layer is
  shown; skip while paused or not fresh; per-UAV show; remove a UAV's marks when it leaves
  the fleet; evict the fleet's oldest mark. The eviction fix is needed here:
  with hidden UAVs now recording, the first-key rule would empty the lowest
  `sys_id`'s marks first.
- `MonitorMapOverlay`: the Trails button and panel replace four buttons; new
  strings in `en.json` and `hy.json`.

Out of scope: a per-UAV clear, per-UAV pause, saving history across reloads,
clearing on arm (QGroundControl).

## Tests

- Trail runs (Node subprocess tests beside `tests/gcs/test_uav_trails_js.py`):
  a new run on resume; no segment joins two runs; the budget covers all runs;
  many pause and resume cycles past the budget drop the oldest runs and stay
  within it; empty and one-point runs are not drawn.
- Trail hook (beside `test_uav_trails_update_js.py`, with a fake viewer): a
  hidden UAV keeps accumulating with the layer off and with its box unchecked;
  pause stops every UAV, including one that joins while paused; Clear all
  empties shown and hidden UAVs; a WebSocket reconnect starts a new run;
  position A held through a disconnect, reconnect with no telemetry, another
  UAV updates, then position B arrives: no A-B segment, B starts the new run;
  Resume while disconnected records nothing; a
  leaving UAV is removed, also while hidden or paused, and returns with fresh
  history.
- Marks hook (beside `test_track_coverage_js.py`, `test_coverage_cap_js.py`):
  keyed by `sys_id`, so a lower `sys_id` joining does not move marks between
  UAVs; a leaving UAV's marks are removed; recording goes on with the layer
  off and the box unchecked; still AUTO only; a stationary UAV whose gimbal
  state changes still gets a mark; no mark from cached telemetry; pause stops it; Clear all resets
  the sampling anchor; the cap evicts the fleet's oldest mark; the live
  footprint follows Vision, the marks layer and the checkbox, not pause.
- Wiring (vitest, the overlay through `useDisplaySettings` and
  `useVehicleCommands` into `CesiumMap`'s hooks with a fake viewer): pause,
  resume, Clear all and Restart mission reach both layers; Restart turns pause
  off; plan change keeps both.
- `MonitorMapOverlay.test.jsx`: one Trails button replaces the four; keyboard
  opens it; the panel closes on Escape with focus back on the button and on a
  tap outside; checkboxes, All / None, Pause / Resume and Clear all call their
  handlers; Paused badge text; accessible names; every new key in `en.json` and
  `hy.json`.
- `npm test` and `npm run build`.
- Live (`live-tester`, own slot, 3-UAV SITL): uncheck UAV 2 for a minute, check
  it again, its retained trail and marks with no gap; pause 30 s, resume, a gap
  with no straight line; Clear all; Restart mission clears and resumes;
  disconnect and reconnect UAV 3, its marks do not reappear in another UAV's
  colour; restart the backend with the page open, no line joins across it;
  narrow pane, the panel scrolls and stays above the vehicle window.

## Review

Codex (`codex exec -s read-only`, code inlined), 2026-10-20. Round 1
rejected v1: run ends against the trail budget, lifecycle precedence and
backend restart, the reset contract, integration tests, touch sizes, the
comparison wording; all fixed in v2. Round 2: approve after two fixes, a
reconnect run seeded from cached positions (fixed: record fresh telemetry
only) and the gimbal-change gate for marks (kept and tested); v3. Round 3:
approve. A clickable mock of the panel was shown to the owner.

## Owner decisions (2026-10-20)

Approved v3 as shown in the mock, with the recommended answers:

1. Deselect hides and keeps recording.
2. Clear all clears every UAV, shown or hidden.
3. A plan change keeps trails and marks.
