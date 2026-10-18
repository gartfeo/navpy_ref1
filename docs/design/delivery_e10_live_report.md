# Delivery demo e10: live GCS operator-view check

- Date: 2026-10-17 (run about 20:24 to 20:27, local time)
- Revision: fa855eb (branch claude/elated-khorana-d9ee41, same as origin/main). Tracked files were not edited.
- Level verified: live GCS stack with SITL at 10x, plus a replay of the recorded backend WebSocket through the real
  frontend reducers and missionStatus.js in Node. The page was not opened in a browser, and no screenshots were taken.
- Simulation never proves physical docking or cargo receipt. DOCKED here is the simulator contact sensor.

## Commands

```powershell
python scripts\gcs_launch.py --list        # before: No running GCS instances
$env:PYTHONPATH = "$PWD\src"
C:\Users\GARTF\Downloads\navpy-temp2\.venv\Scripts\python.exe scripts\eval_gcs_navigation_demo.py --scenario delivery --run-dir .logs\e10\live
python scripts\gcs_launch.py --list        # after: No running GCS instances
```

- The evaluator claimed this directory's own slot (chat 0, backend :8000, sysids 1 to 3). No port was chosen by hand.
- A passive recorder (rec.py) connected to /ws/telemetry of that backend once it was healthy, during the parameter
  download, and recorded every message for the whole run. It sent nothing.
- Replay: replay.mjs feeds the trace to copies of taskAssignmentState.js, taskConfirmationState.js and missionStatus.js
  (only the `colors` import of missionStatus.js is replaced by an inline constant). The wiring follows
  useWsHandlers.js and useTaskAssignment.js. wpOffset is taken as 0 (MonitoringSidebar computes it; only used for
  the AUTO en-route/searching split, not relevant to the states below).
- Artifacts, all under .logs/e10/ (untracked): live/ (run dir), live.out, live_ws_trace.jsonl (911 lines), and
  live_replay/ (rec.py, replay.mjs, replay_out.json, the three module copies in fe/).
- Trace index N below means line N of live_ws_trace.jsonl after the first `_meta` line (0-based). Times t are seconds
  of wall time from trace index 0, which is during parameter download. The sim runs at 10x.

## Evaluator result

- Exit code: 1. Not captured at the live run (it ran detached); `--analyze-only .logs\e10\live` returns exit 1 as well,
  and eval_gcs_demo_cli.py returns 0 only if `report.passed`. report.json has `"passed": false`.
- report.json top-level `errors`: `[]`.
- Per-vehicle `errors`:
  - UAV 1: `[]` (passed). UAV 2: `[]` (passed).
  - UAV 3: `["max roll-command step 56.90deg > 50.00deg"]` (failed).
- Cadence of UAV 3: coverage 0.121, 173 observations, gap p90 394 ms, longest gap 740 ms. UAV 1 and 2 had coverage
  0.978 and 0.968 with p90 100 ms. In r2 and r3 all three UAVs had coverage 0.93 to 0.97 and roll steps up to 43.3 deg.
  This looks like the sparse-observation problem the plan assigns to other chats, but I did not diagnose the cause.
  The cadence gate is `gate: False`; the failing gate is the roll step.
- Approvals in report.json: UAV 1, 2 and 3 each approved on round 1 after a 15.5 to 15.6 s wall delay,
  `nav_before_approval: false`. Round uids 777773228:21, 3497385264:28, 2509375492:59.
- Navigation logs (.logs/e10/live/uav_N_navigation.log): DOCKED lines UAV 1 `dock=0` (d1) 3d=0.01 m, UAV 2 `dock=1`
  (d2) 3d=0.03 m, UAV 3 `dock=4` (home-west) 3d=0.46 m. Fleet docks line: P1=d1, P2=d2, P3=home-east,
  P4=home-middle, P5=home-west.
- Backend logs (gcs/backend.log, backend.out, swarm_run.out): no error, exception or traceback lines. One NavPy
  warning: UAV 1 "Reset drops task 3 CONFIRMED to peer 2", after its own DOCKED row.

The flight verdict for this run is therefore FAIL (UAV 3 roll-step gate). Both earlier headless runs passed. The UI
checks below do not depend on that gate.

## Operator-visible timeline from the replay

States are `computeMissionStatus` ids (the label shown on the vehicle card), the pending confirm card, and the
assignment entry the card uses. Rows list only changes.

### UAV 1 (offerer, d1)

| t (s) | idx | Event | Card status | Confirm card |
|---|---|---|---|---|
| 0 | 0 | disarmed, AUTOTUNE | idle | none |
| 16.8 | 434 | armed, AUTO | searching | none |
| 42.98 | 590 | task_confirm_request task 2 (round 777773228:21); image at idx 596 | searching, then idle at idx 591 (mode GUIDED) | pending |
| 58.64 | 680 | task_confirm_response approve (delivery sent) | idle | approved |
| 61.1 | 694 | disarm_after_guided | idle (disarmed) | cleared |

Swarm badge: FREE until idx 683, then BUSY. Last telemetry: armed false, GUIDED.

### UAV 2 (helper, wins d2)

| t (s) | idx | Event | Card status | Confirm card |
|---|---|---|---|---|
| 17.6 | 442 | armed, AUTO | searching | none |
| 42.89 | 586 | available_task_request from 1: task 3 DOCK at 40.288498, 44.473705 | searching | none |
| 42.90 | 587 | task_assign_request 1 to 2, task 3 | waiting | none |
| 42.91 | 588, 589 | task_assign_response accepted, then task_assign_ack APPLIED | assigned | none |
| 43.01 | 591 | mode GUIDED | approaching | none |
| 44.45 | 601 | task_confirm_request sys 2, task 2 (round 3497385264:28); image idx 606 | confirming | pending |
| 60.10 | 689 | task_confirm_response approve | approaching | approved |
| 62.57 | 703 | disarm_after_guided | idle (disarmed) | cleared, assignment dropped |

Swarm badge: BUSY from idx 594.

### UAV 3 (home dock)

| t (s) | idx | Event | Card status | Confirm card |
|---|---|---|---|---|
| 18.4 | 450 | armed, AUTO | searching | none |
| 64.64 | 715 | mode GUIDED (flying to home-west) | idle | none |
| 80.26 | 800 | task_confirm_request task 2 (round 2509375492:59); image idx 806 | idle | pending |
| 95.91 | 881 | task_confirm_response approve | idle | approved |
| 98.29 | 893 | disarm_after_guided | idle (disarmed) | cleared |

Swarm badge: FREE until idx 883, then BUSY.

End state of the replay: assignments `{}`, available tasks `{}`, confirm cards `{}`. Last telemetry of all three:
armed false, GUIDED.

## Verdicts

| Check | Verdict | Evidence |
|---|---|---|
| Offerer d1 confirmation request appears and is approved | pass | idx 590 request, idx 596 image, idx 680 approve; matches report.json round 777773228:21 and the log "POI 2 confirmed by ground station" (UAV 1 20:25:46.429) |
| d2 offer shown, UAV 2 assigned as helper | pass | idx 586 offer is task 3 at 40.288498, 44.473705, equal to fleet_docks.json d2 (40.288499, 44.4737067). idx 587 to 589 give waiting then assigned, then approaching at idx 591. Log: UAV 1 "Task offered: task_id=3 obj_id=1", UAV 2 "Task 3 assigned by owner 1" |
| UAV 2 and UAV 3 requests appear and are approved | pass | idx 601 and 689 (UAV 2), idx 800 and 881 (UAV 3), each with an image before the decision, each matching its report.json round uid |
| Final approach and terminal state | pass with a gap | Every UAV ends disarmed (armed false) and the card goes idle after disarm_after_guided (idx 694, 703, 893). The UI has no DOCKED state, see below. Docked evidence comes from the navigation logs only |
| No home dock shown as offered | pass | The trace holds exactly one available_task_request (idx 586, d2). UAV 3's confirm request is its own task 2 with no offer |
| No UAV at the wrong dock, no request that never clears | pass | Each confirm card was cleared by disarm_after_guided; the final replay state is empty. Docks per log: d1, d2, home-west |
| No stuck, wrong or missing state | fail (UI gaps, not flight) | See findings 1 to 4 |

## Findings in the operator view

1. UAV 1 and UAV 3 show "Idle" while armed in GUIDED. `deriveStatusId` returns `confirming`, `approaching`,
   `assigned` or `confirmed` only when the UAV has an assignment entry. UAV 1 (own d1) and UAV 3 (home dock) have
   none, so with GUIDED and armed they fall through to `idle`: UAV 1 from idx 591 to 694 (about 18 s wall, including
   its pending request and the approach), UAV 3 from idx 715 to 893 (about 33 s wall). The pending card is shown
   separately, but the vehicle status never reads confirming or approaching for them.
2. "Confirmed" never appears for any UAV. UAV 2's assignment is keyed to the owner's task id 3, but its
   confirm request and response carry its own task id 2. The task-id match in `taskConfirming` and `taskConfirmed`
   (receiver and task id) therefore finds nothing. After approval UAV 2 reads "approaching" (idx 689), not
   "confirmed". The offerer and home UAV have no assignment, so they cannot reach it either.
3. The confirm card carries no location. All three requests have `lat: 0, lon: 0, alt: 0` (idx 590, 601, 800),
   so TaskConfirmCard.jsx (lines 99 and 154) renders "0.000000, 0.000000" with a maps link to 0,0. This follows the
   dock location rule (no waypoint dock info before detection), but the operator sees a null-island coordinate.
4. No docked state. Nothing on the WebSocket says docked. The only terminal signals are armed false and the
   disarm_after_guided event, which show as "Idle". By your instruction this is not a pass or fail of the flight.
5. The WAITING state is real but not visible. UAV 2 is "waiting" only between idx 587 and 589, 10 ms of wall time
   (no telemetry frame between them). It will not be seen by an operator.
6. Position reported at disarm. The disarm_after_guided positions are about 25 to 28 m east of the dock longitudes in
   fleet_docks.json (UAV 1 40.2871878, 44.4740346 vs d1 40.2871501, 44.4737067; UAV 2 40.2883132, 44.4740458 vs d2
   40.288499, 44.4737067; UAV 3 40.3165199, 44.4386979 vs home-west 40.3165, 44.4384), while the contact sensor read
   0.01 to 0.46 m. I did not find out why; it may be a frame or reference offset in the simulation. It matters only if the
   operator map is meant to show the UAV at its dock marker.

## Could not check

- The rendered page: no browser was opened (instruction), so layout, badge text, the map and the confirm card's
  look were not seen. Statuses are the ids the real function returns; the label text comes from locales/en.json and
  was not checked.
- The real React batching: the replay applies each message in order. Real renders may merge messages that arrive in
  one tick.
- The original live exit code (reconstructed via `--analyze-only`, see above).
- Why UAV 3 had sparse observations, and whether the 56.9 deg roll step follows from them.
- Whether the d2 offer arrives at the operator before the d1 card by design: offer idx 586 at 42.89 s, d1 card idx 590
  at 42.98 s. The UAV 1 log agrees (offer 20:25:31.174, confirm request sent 20:25:31.263).

## Teardown

The evaluator stopped its own stack. `python scripts\gcs_launch.py --list` afterwards: "No running GCS instances".
The recorder exited by itself when the backend went away. The run dir `.logs/e10/live` is left in place.
