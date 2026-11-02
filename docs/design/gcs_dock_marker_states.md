# GCS dock marker states (todo 72 part c)

Status: v4 approved by the owner 2026-11-01 (all (a)) and built, then corrected by
the final audit, the Codex rounds on the final diff and two live checks. Codex rejected
v1, v2 and v3; all their points are answered in "Review". Mock:
`gcs_dock_marker_states_mock.html`. Outcomes come from `gcs_mission_phase.md`.

## Problem

The map cannot show a dock moving through detected, awaiting confirmation,
assigned and handled:

- Assigned, confirming and confirmed draw the same marker
  (`useAssignmentMarkers.js`, a check mark from assigned onward).
- Disarm, reset and deny remove the marker, so a handled dock is never shown
  (`taskAssignmentState.js` `disarmAfterGuided`, `vehicleReset`).
- A UAV's own dock has no marker, and a homing UAV's diversion is a UAV's own
  dock (traced below).

## What reaches the GCS (traced)

| Dock | Messages at the GCS | Position |
|---|---|---|
| Offered to peers | `available_task_request`, then `task_assign_request`, response, ack | yes, from the offer |
| Helper flying an offered dock | those, then `task_confirm_request` under its own POI id | yes, from the assign request |
| UAV's own dock, including a homing diversion (ruling 4) | `task_confirm_request` only | **none: NaN** |

- A homing UAV selects the dock as its own (`poi_selection.py`; `home_dock.py`
  orders delivery docks first), and the notifier skips the active POI. Other
  docks it sees meanwhile can still be offered.
- The confirm request carries only `projected_poi_location`, which is unset on
  the vision path (`confirmation_round_runner.py` `_request_message`).
- `SelfAssignmentPublisher` broadcasts a self-addressed `task_assign_request`
  with a position, but none appears at the backend: no `sender=n receiver=n`
  line in the `.logs/gcs-mission-phase/live` run 2 and 4 backend logs, and none
  in `.logs/e10/live_ws_trace.jsonl`. The cause is not established. It is also
  sent once, with no resend, and falls back to 0,0,0, so this design does not
  use it.

So the brief's recommendation for decision 3 (no position, no marker) leaves the
homing diversion, the case this todo names, with no marker at all.

## Design

### States

One marker per dock, drawn by state. Icons and labels are in the mock.

| State | Source | Marker |
|---|---|---|
| Detected | `availableTasks` entry; a `waiting` slot says who it is offered to | dashed orange ring |
| Awaiting confirmation | `confirming` slot; pending own entry | amber ring with "?" |
| Assigned | `assigned` slot | pad in the UAV's colour |
| Approved | `confirmed` slot; approved own entry | pad with a check |
| Handled | `handled` entry | dimmed pad with the outcome |

### Entries

All in `assignments`, in `taskAssignmentState.js`; no new reducer field and no
new App prop.

- `handled:${uav}:${roundUid}`: status `handled`, `receiverId: null`,
  `handledBy: uav`. A null receiver keeps it out of `vehicleAssignment`,
  `currentTaskSlot` and the per-vehicle drops, so the UAV cards do not change
  and a new task on the same helper slot does not erase it.
- `own:${uav}:${roundUid}` (decision 3a): status `confirming` or `confirmed`,
  `receiverId: null`, `askedBy: uav`, `senderId: uav`, `requestUid` from the round.
  Created by a confirm round with a position
  that the UAV's helper slot does not take. A position of 0,0 counts as none
  (older traces). A deny drops a pending entry.
- A slot takes a round only at its own position, when the round has one. A round at
  another position is another dock, so it is an own entry, and the round the slot was
  asking about has been superseded: the slot goes back to assigned and that round's
  answer no longer moves it.
- A helper's round heard before APPLIED stays an own entry; it is never moved to
  the slot, because round order does not show which dock it asks about (the slot
  stays ASSIGNED, as today, todo 88). If it was the task's dock, two markers show
  there until the outcome.

### Moving to handled

An outcome word needs direct evidence from this approach, because NavPy's latch
keeps the previous outcome through the next approach and the byte carries no
approach identity. Without that evidence the marker reads "outcome unknown"; the
UAV card still shows NavPy's outcome.

`useTaskAssignment` forwards every new beat, once, as the existing
`swarm_heartbeat` action, which also keeps its FREE rule. Beats with a phase then
go to the UAV's entries; a new round has already ended the older ones. For an
approved slot or own entry with round R:

1. Beat from another boot: handled, outcome unknown. A pending own entry is dropped.
2. APPROACH beat with seq after R: the entry records `approachOutcome`, the outcome
   that beat carries (often none). This is the evidence that R's approach started. A
   later APPROACH beat with a different outcome is a later approach, because the
   outcome only changes as an approach ends, so the entry is handled, outcome unknown.
3. Beat with seq after R and a phase other than APPROACH, where CONFIRM and TO_TASK
   count only once rule 2 has run, since before the approach they belong to R: the
   approach is over, so handled. The word is the beat's outcome only if rule 2 ran
   and the outcome differs from `approachOutcome`, since only then did it change
   during this approach. The comparison includes the failure reason, so one failure
   replaced by another counts as a change. Otherwise outcome unknown, for good:
   NavPy records the outcome in NAV's exit and builds the display from the committed
   state on the same thread (`nav_transition.py`, `nav_display.py`), so the first beat
   after the approach already carries it, and a later one may be another approach's.
4. An entry moved by a disarm or E-STOP before its approach ended (`awaitingOutcome`)
   takes its word by the same test from the first such beat of the same boot, then
   stops waiting, and any phase but APPROACH counts here. A disarm often comes first: every dock in the live run did this.

A new round by that UAV ends the fence: its older approved entries become handled,
outcome unknown, its older pending own entries are dropped (NavPy can supersede
them with no answer), and `awaitingOutcome` clears. So a later approach cannot label
an earlier dock. A round already heard, or older than a round of that UAV still on the
map, is a delayed copy and changes nothing, so it cannot take a slot back. A denied round
leaves no entry (Known limits).

A WebSocket drop means rounds and their answers may have been missed. When the
store's connection goes down, or one UAV's beat goes stale or its next beat comes over
the 5 s TTL after the last (for that UAV only, the jump covering stalled snapshots),
`useTaskAssignment` dispatches `link_gap`: every open
round, approved or still pending, gets `noWord`, and an `awaitingOutcome` entry stops
waiting. An approved entry still moves to handled by the rules above, but reads
outcome unknown. A flagged pending own entry goes at the first later beat whose phase
is not CONFIRM, since its answer may have been missed and nothing ties it to a round
any more. A later round on a slot clears the flag: the gap did not hide that round.

Other moves to handled:

- `disarmAfterGuided` or `vehicleReset` (E-STOP) for a confirmed slot or own entry:
  outcome unknown, `awaitingOutcome` if rule 2 ran. Lower ranks are dropped.
- A new APPLIED that replaces a confirmed slot: outcome unknown. That includes the same
  task given to the same helper again: a request after the slot's APPLIED is a new
  assignment, so it needs a new approval.
- Cancel task on an approved entry (`task_resolved` at rank confirmed): handled,
  "canceled". Today that demotes the slot to assigned and the next FREE beat deletes
  it. A deny of a pending round still returns the slot to assigned.
- A cancel naming a round already handled as outcome unknown makes it "canceled": the
  backend broadcasts about 0.5 s after the first copy reaches NavPy, so the UAV's
  teardown beat can come first. A real word stays.

An approved entry whose UAV changes mode before disarming is covered by rule 3,
which needs no disarm event.

A delayed APPLIED cannot bring the slot back: it needs a pending request, and a delayed
request copy makes none ("One marker per dock").

### Lifetime

Until a new mission. Handled entries survive disarm and E-STOP, and stay through
planning and Upload until the launch.

- A new mission clears the whole map on every client: the fleet launch and restart
  routes broadcast `task_confirm_reset` with `reason: "new_mission"`, and `useWsHandlers`
  calls `reset`. A container launch is always the whole fleet and broadcasts once it is
  prepared, so a refused launch clears nothing. Bungee START MISSION goes through the
  restart route. RESTART MISSION is only offered in flight, so a landed container fleet
  has only the launch. A restart starts a new mission only if a UAV did restart; if none
  did, it sends the message with no reason for the listed UAVs, as main does.
- Local Restart also calls `reset` before the request returns, as it does for task
  markers today, so the client that pressed it clears its own map even if no UAV restarts.
- E-STOP's `task_confirm_reset` has no reason: the listed UAVs' approved entries move to
  handled and their lower entries go.
- A per-UAV bungee launch or relaunch continues the mission, so it clears nothing.
- A page reload loses these markers, as it loses all task markers today.

### One marker per dock

A pure selector `dockMarkerEntries(assignments, availableTasks)` decides what the
two hooks draw; `CesiumMap` calls it and passes each hook its part. One offer means
the same owner, task, position and owner boot, until a later advert or request releases
it; available entries keep their owner boot for this.

- Each UAV on an offer has one marker. A live state is a new decision, so it shows over
  that dock's handled entry (a RESET re-pick, or a restarted helper's new slot). A deny
  brings the handled entry back; the round's own handled entry shows once it ends. A
  UAV's two rounds at one dock under two task ids are two offers, so two markers.
- An offer shows as detected or offered only until a UAV holds it.
- What is a copy and what is new comes from the owner's order. It stamps a task's adverts
  and requests under one lock, adverts only while the task is free
  (`task_rebroadcast_state.py`) and request copies only while the reservation waits for
  an answer (`task_auction_confirmation.py`). Its APPLIED comes after it applies that
  answer (`task_auction_coordinator.py`), and a confirmed task is never taken back before
  a RESET. So the next request or advert after a task's APPLIED, from that owner boot,
  starts a new dispatch. `offerFloor` is the last such message an entry stands on:
  `reoffered`, else its APPLIED, else its request. A UAV's own round is no step of an
  assignment, so on an own entry the floor is its round and decides only what releases it.
- An advert or request newer than that floor releases the entry: it sets `reoffered`, that
  uid, on it, whether assigned, awaiting, approved or handled, since the message can arrive
  before the approach's end. A `reoffered` entry no longer holds its offer and has a marker
  of its own, so no state hides the new offer. An advert also drops a waiting round older
  than it. A UAV's pending own round is left alone: it and that UAV's own advert are one dock.
- A request no newer than the floor of an assigned entry on that offer is a delayed copy
  and makes no round. A request older than an own entry's round is neither a copy nor a
  release: the assignment it makes stands beside that dock. An APPLIED older than a pending
  request answered an earlier assignment and leaves that request waiting.
- A UAV's request to itself (`SelfAssignmentPublisher`, as each round starts) is outside
  that order and offers nothing, so it is neither a copy nor a release. It still makes a
  waiting round, as on main. None reached the map in the recordings; whether a radio link
  delivers one is not known.
- At one position a live state hides the handled entries until it ends, so history does
  not garble the live icon and word. What remains there gets a label row each and one
  pad, since pads at one spot cover each other in an order Cesium picks: the round the
  operator must answer, then approved, assigned, offered and detected, and among equals
  the last, which for history is the newest to end.

This is per offer, not per physical dock: a helper's own-POI round has its own
identity, as above.

### Map

- `useAssignmentMarkers` draws the icon and label for the entry's state;
  `useAvailableTaskMarkers` adds the state word.
- Icons are new functions in `constants/icons.js`, labels new
  `mapMarkers.dockState.*` keys in both locale files.
- An entry with no position draws nothing, never a guessed point.
- The state label sits below the pad. The hub label and the sim closest-approach label
  sit above the same spot, and a handled marker now stays there after the approach, so
  above the pad they covered its word (live check screenshots).
- The word shows only within 1500 m of the camera, because far out the words of near
  docks print over each other. On a full HD map, docks 150 m apart clear the longest word
  at about that range; in a narrower window they can still touch just inside it.
- A pad is about 22 px at Reset View (6814 m in the live check's window, 9.4 km on a full
  HD map, where the demo docks are about 23 px apart). The first far scale, 0.3, drew it
  at 14 px under the sim's 14 px closest-approach ring, and its badge could not be read.
  The word's text keeps 0.3, so the range above still holds. The sim marks still cover a
  pad's centre (todo 191).
- A marker's raw height is the ground rendered there when the entry changes
  (`dockMarkerPosition`), else the entry's height until a tile has rendered. Cesium 1.138
  draws a clamped label's new letters at the raw position: `LabelCollection.rebindAllGlyphs`
  gives a new or spare letter no clamped position, and the label's height callback fires
  only when a deeper tile renders, so with an idle camera nothing corrects them. With the
  message's 1316 m and a camera at 650 m those letters were above the camera: "outcome
  unknown" drew "UAV 6 > DOCK: outcome", "dock failed (sim)" drew "UAV 6 > DOCK: dock fa",
  and a new label drew no word. The same steps with the raw height on the ground drew every
  word complete (bare page on the installed 1.138.0, `.logs/dock-marker-states/glyph_exp/`).
  Upstream first copies the clamped position in tag 1.140. This leaves the Cesium bug in
  place; an upgrade changes the whole map engine, so it is todo 194.
- A word in a dark UAV colour gets a white outline: below 3:1 on the usual black outline,
  which in the fleet palette is only `#0033A0`.
- The handled pad has an opaque fill under its dimmed strokes, so a hub below it does
  not show through.

### Decision 3a in NavPy

`_request_message` fills the position as `SelfAssignmentPublisher` does: sim truth
in a simulation, else `projected_poi_location`, else NaN. Both are field reads, so
the worker thread that builds the request calls no geo code, and nothing is
written to the POI. The real-build ground estimate is left out: it would need
resolving before the worker starts, because the shared projector cache is not
thread-safe (`camera_ned_projector.py`). So on real hardware the diversion has no
marker until a position exists; that follow-up goes in the todo list.

### Constraints

- Dock location rule: every new marker comes from a peer message or a UAV's own
  round, so after a detection. The existing static sim markers (`D1...`) break the
  rule: in sim monitoring they show from waypoint metadata before any detection
  (`CesiumMap.jsx` `simDocks`, `dockResolver.js`). Decision 5.
- Pure Vision Approach Constraint: display only. The new position goes only into
  the outgoing message and the POI is not changed, so the command path reads
  nothing new.
- Wording: no string claims physical docking or cargo receipt.

### Known limits

- Auto-approve (`NAV_AUTO_CM` on) asks no round, so own docks get no marker and
  helper slots stay assigned. The demo config turns it off, and the sidebar still
  shows the phase.
- "Outcome unknown" reads more often than strictly needed: an approach with no
  heard APPROACH beat (beats are 1 Hz), and a cyclic run with two equal outcomes in
  a row, which the byte cannot tell apart. The UAV card is still right. In the second
  live check it stayed on 2 of 10 rounds: an approach that ended 0.18 s after its approval,
  before any APPROACH beat, where the card said dock failed, and an E-STOP, where no beat
  carried an outcome. The others read it for 0.72 to 1.10 s, between the disarm and the
  first beat after the approach.
- An approved round that NavPy drops without flying reads handled, outcome
  unknown.
- A late beat from a retired boot can drop a pending own entry from the new boot;
  the card stays.
- The sim position is truth, as offers already are (todo 40).
- A delayed request copy older than a re-advert, with no UAV holding that offer, draws an
  extra offered marker until the next advert repeat (about 0.5 s), as on main.
- The copy rules lean on the owner's order, which
  `test_request_copies_go_on_after_the_floor_until_confirmed` and
  `test_rebroadcast_prepared_before_reservation_is_not_sent_after_it` pin. One gap is left:
  the owner stamps an APPLIED outside its lock (`on_assign_response`), and a RESET in that
  moment can advertise the task below that uid. If the APPLIED reaches the map first, the
  advert reads as older than the assignment and the new offer stays hidden until the owner's
  next message of that task: a repeat 0.5 s later while a free peer is owed the advert, else
  a request. The old APPLIED could also assign a new reservation's request early, which
  needs the peer selection's 2 s wait (`PEER_SELECTION_DELAY_S`) to pass inside that gap.
- A denied round leaves no entry, so a round of that UAV older than it, arriving after the
  deny, reads as new: a marker, or a slot awaiting it. It matches the card: the backend
  classifies rounds per UAV and task, so that round opens a card as it does on main (todo
  200), and the card's answer or countdown ends the marker. If the operator approves it,
  it reads handled, outcome unknown. It needs one UAV's rounds reordered by more than an
  operator's answer, and they come through that UAV's one link reader (`mav_bus_reader.py`).
- A cancel naming a handled round marks it "canceled" even if the UAV had moved on from
  it, when the word was outcome unknown: the operator did cancel that round, and the GCS
  has no better word.
- A flagged pending own round whose approve reaches the map after the UAV left CONFIRM
  loses its marker. It fails closed: the card still shows the decision.
- A launch that prepares and is then aborted has already cleared the map. A client whose
  socket is down at the launch keeps its markers until a later reset or a reload.
- Spots are exact: two different projected positions of one dock are two spots. In the
  sim recordings the request and the round carry the same value. If a helper's round for
  its task ever carried another value, it would draw a second marker and the slot would
  stay assigned, which fails closed.
- A UAV asking about another dock while it holds a task: its slot stays assigned until
  its FREE beat, where main moved the slot to that round. So `currentTaskSlot` can return
  the slot in TO_TASK and CONFIRM; the card's label there is the same for assigned and
  awaiting. Rounds on a real build carry no position, so there a slot still takes any
  round after its assignment, and a stale slot can still take another dock's round. Its
  marker stays at the slot's position (todo 188).
- A restart that moved no UAV still sends the message with no reason, as main does: the
  listed UAVs' cards reset and their approved entries move to handled (todo 193).
- The dark blue pad's stroke stays low contrast on dark ground. The colour predates this
  change, so only the word is fixed.
- On terrain the ground read at a state change is the deepest tile rendered there, which
  can be coarse for a dock off screen. A new letter sits at that height until a deeper
  tile renders, so a word can show slightly off its pad until then (todo 194).
- White dashed legs and UAV labels can cross a word; they keep their order.
- A helper assigned again after its approved slot went to history shows the new task's
  label in TO_TASK and CONFIRM, where main kept the old confirmed slot and showed none.
  So a UAV card can change there; the new label is the right one, because the helper is
  on that task. It needs a RESET on both sides, in cyclic mode.
- A real outcome heard after a stale beat span or a beat jump over the TTL reads outcome unknown,
  even with the socket up, so a page that stalls over 5 s loses that word too.
- A handled dock's marker and word stay hidden while another owner's advert sits at the same
  position: the dock reads detected until that owner assigns it or disarms, or a new mission
  starts. The offer is still open and the UAV card keeps the outcome. Sim only (real-build
  positions from two UAVs would not match exactly); found by the pre-merge review, no recording
  shows it.
- A wrong word is still possible when a later approach's round is not heard (auto-approve
  turned on in mid-mission, or a timeout approval with its messages lost), and every beat
  between the approved approach and that later one's end is lost in gaps each shorter than
  the beat's 5 s TTL, with the socket up. A longer gap is a link gap for that UAV. Then either the later approach's APPROACH
  beats are lost too, whatever the outcome, or they are heard and the outcome did not
  change across the gap (the approved approach recorded none, or the same one as before,
  or no beat of it was heard). The later approach's end then labels the approved dock.
  What the GCS receives looks the same as one approach: beats are 1 Hz and carry no
  approach identity, so closing this needs an approach id in the heartbeat (todo 190).
  The marker is display only, the demo config has auto-approve off, and a socket drop
  or a stale beat is covered by `link_gap`.
- A companion that sends no phase byte (an older NavPy, or no beat yet): a later
  round also ends its approved entry, so its card's guess can go from "confirmed" to
  "searching". With a phase the card never reads a confirmed slot
  (`missionStatus.js`), so a current companion's card does not change. The companion
  and the GCS ship from this one repository, so no shim keeps the old reading.

## Tests

- Reducer (`test_task_assignment_state.js`), real module: each status; handled
  survives disarm, E-STOP and a new task on the slot, and is cleared by reset; rules
  1-4, including the latch from an earlier approach (Codex's dock A, round B sequence),
  an earlier dock not labelled by a later outcome, and disarm before the beat; cancel
  on an approved entry, and a cancel after the teardown beat; a helper's position-bearing
  round before APPLIED stays an own entry and the slot stays ASSIGNED; a homing UAV's own
  round through the same states; no position creates nothing; no entry before a
  detection or peer message.
  Also: a WebSocket drop gives no word (Codex's A, reconnect, B sequence) and ends a
  pending round; a new round drops a superseded pending own entry; a delayed request
  after handled makes no slot.
  Also: a delayed APPLIED after a dock's end makes no slot, however it ended, and a new
  request outlives that end; an APPLIED from before a request leaves it waiting; a round
  at another position is not the slot's, and each dock keeps its own word (Codex's
  sequence); a delayed round moves nothing; a task assigned again needs a new approval,
  in either order of the advert and the request; a request or advert older than the
  entry's APPLIED is a copy, a later one releases it, and a request to itself does neither;
  an assignment made before an own round still makes its slot in either order (Codex's
  sequence); the two limits on a denied round and on an APPLIED that overtakes an advert.
- Selector: one marker per UAV on an offer; a delayed request adds none; a new owner boot
  and a later advert or request are not hidden; a recycled id at another
  position, in each coordinate; a live state hides history at its spot until it ends;
  a restarted helper's new slot shows over its history; label rows, counted per spot; one
  pad per spot, by state in both orders of the entries, back on history when a live entry
  ends.
- Marker hooks: each state draws a different icon, badge and label at its position, at
  creation and in place; words keep their range, colour and outline, and follow a locale
  switch; handled stays, and an entry that comes back draws again. A marker's raw height
  is the ground rendered there, else the entry's; a pad or ring draws only where the
  selector left one; the pad keeps its far size and the word stays under it. UAV cards unchanged by
  handled and own entries. `CesiumMap` passes the selector's output to each hook, and
  static docks only while planning.
- `useTaskAssignment`: each beat is read once, per UAV; a socket drop takes the word away
  and a connect does not. `useWsHandlers`: the reset reaches the cards and, by its reason,
  the whole map or the listed UAVs.
- NavPy (decision 3a): truth in sim, projected otherwise, NaN when none; the POI
  is not changed; the manager passes its flag to the round runner.
- Backend: a container launch and a restart that moved a UAV broadcast `new_mission`; a
  refused launch, a bungee launch and a restart that moved none do not.
- Live (live-tester): 3-UAV delivery, each state seen, handled until the next launch,
  the homing diversion's states.

Existing tests changed by intent:

- `test_task_assignment_state.js`: an available entry now has `ownerBoot`. A later
  round no longer leaves a confirmed slot untouched, because the round ends that
  approach's evidence (the Codex round 1-2 fence); a copy of the same round still
  changes nothing. The "denied home dock keeps the confirmed task" case is the same
  rule: the task reads handled, outcome unknown, instead of staying confirmed.
- `useWsHandlers.test.jsx`: `handleConfirmingStatus` takes the whole message, since
  an own entry needs its position.
- `test_terrain_ready_marker_hooks_js.py`: both hooks' dependencies add
  `i18n.language`, so state labels follow a live locale switch.
- `test_task_assignment_status_js.py`: the check mark now means approved, not
  assigned (decision 2), and `handleConfirmingStatus` takes the whole message.
- `test_denied_cleanup_js.py`: its filter pin now reads the reset's real filter,
  `uavOf`, since `entry.receiverId === sysId` matched only an unrelated selector.
- `test_confirmation_manager_state.py`: the round runner has a required
  `is_simulation` argument. `test_confirmation_request_location.py`: its fixture takes
  that flag too.
- `useTaskAssignment.test.jsx`: its telemetry stub gains `isConnected`, for `link_gap`.
- `test_task_assignment_state.js`: the `applied` helper's default uid moves from seq 90 to
  25 and the requests that mean a new assignment move above it. The fixtures had the owner
  send requests after its APPLIED, which it does not do (Codex final diff, round 6).
- `test_mission_status_js.py`: the same fixture fault in its repeated "doing" case: the request
  after APPLIED is gone, so the repeat moves nothing on the slot, and the probe's "doing" reads 4,
  not 11. The approach-fence assertions are unchanged. It was caught only by the full run.

## Owner decisions

Decided 2026-11-01: (a) on all five, and build from v4 with Codex reviewing the
final diff.

1. How long a handled dock stays: (a) dimmed until mission reset, kept on disarm
   and E-STOP. (b) removed on disarm, as today.
2. How the states look: (a) one icon and label per state, the same for a homing
   UAV's diversion (mock), with approved kept apart from assigned. (b) one icon,
   label only.
3. A dock with no position in the confirm request, which is every UAV's own dock
   and every homing diversion:
   (a) NavPy fills the position as above, a small change outside the brief's file
   list. These docks get the same states in the sim, and the confirm card stops
   reading "Position unknown" there. Real hardware keeps no marker for now.
   (b) Frontend only: no marker for them; the card and sidebar cover them. The
   brief's live check for the diversion cannot pass and is dropped.
4. The handled words: (a) the sidebar's "docked (sim contact)" and "dock failed
   (sim)", as shipped by PR 45, which dropped the brief's "Approach ended", plus
   "canceled" and "outcome unknown" where the GCS cannot tie an outcome to this
   dock. The failure reason stays on the UAV card. (b) add the reason to the marker
   label.
5. The static sim markers (`D1...`), which show docks before detection in sim
   monitoring, against the dock location rule:
   (a) hide them in monitoring in this change, keeping them in planning. The map
   then shows only what the swarm knows, which is what the demo should show, but
   you lose the view of undetected docks while testing. (b) leave them and log a
   todo; the rule stays broken for them and the state marker draws on top.

## Review

### Codex round 1 (reject)

1. An outcome can attach to the wrong dock (a later approach's beat fills an
   earlier entry; the latch keeps an old outcome). Holds. `priorOutcome`, the
   cleared `awaitingOutcome`, and "outcome unknown" otherwise.
2. The static sim markers break the stated rule. Holds, and predates this work.
   The note no longer claims them; decision 5.
3. A helper's round before APPLIED loses its link. Holds. Reconciled on APPLIED by
   round order.
4. No retirement for new boots and late rounds. Holds. Rule 1; rounds from a boot
   other than the one heard create nothing; own keys carry the round.
5. Reset and disarm lose history or leave markers. Holds for E-STOP's drop, the
   restart cleanup of own entries, and the mode-change gap. `vehicleReset` moves
   approved entries to handled; rule 2 needs no disarm. Local Restart clearing the
   fleet before the request returns stays as it is today.
6. Drawing every entry is not one marker per dock. Holds. Selector.
7. Reusing the ground resolver on the worker thread is unsafe. Holds. Sim truth and
   projected only, as field reads; the ground estimate is left out.
8. Auto-approved own approaches ask no round. Holds. Stated as a limit.

Also corrected: "never offers" was too broad, and the cause of the missing
self-assignment is not established.

### Codex round 2 (reject)

1. `priorOutcome` can put an earlier dock's outcome on this one (dock A ends between
   beats, round B records the older value, B's exit carries A's). Holds. Dropped for
   `approachOutcome`: the outcome must change after an APPROACH beat of this round.
   A new round also retires older approved entries.
2. Moving a round to the slot on APPLIED can attach an own dock's approval to the
   peer dock. Holds (`poi_status_decision.py`, and the reducer's fail-closed rule,
   todo 88). Dropped: it stays an own entry.
3. Collapsing hides a new offer of the same task after an owner restart. Holds. One
   offer now needs the same owner boot and no later advert. Per offer, not per
   physical dock, as the note now says.
4. The boot filter drops a valid new-boot round heard before its first beat. Holds.
   Filter dropped; rule 1 retires old-boot entries at the next beat.
5. Cancel on an approved slot loses its history. Holds. Handled, "canceled".
6. Decision 5 cannot leave the static markers under the rule. Holds; the note says
   they break it and now recommends hiding them. It stays the owner's decision
   because it changes what the owner sees while testing.

### Codex round 3 (reject)

1. A WebSocket drop can hide round B, so B's outcome labels dock A. Holds. `link_gap`
   takes away the outcome word.
2. A superseded pending own entry can stay. Holds. A new round drops it.
3. Repeated requests have fresh uids, so "a later request is a new offer" draws
   duplicates. Holds (Codex probe with the real reducer). Only an advert starts a
   new offer, through `reoffered`.

Findings fell from 8 to 6 to 3 and narrowed. v4 had no design review of its own; the
final diff got one.

### Codex final diff, round 1 (reject)

Reproduced by Codex with the real reducer; each read against the code.

1. A repeated APPLIED rebuilt the slot and lost `noWord` and `approachOutcome`, so a
   gap's missed round could label dock A. Holds. The repeat keeps them.
2. An auto-approved approach could fill an `awaitingOutcome` entry. Holds; v4 listed it
   as a limit, but it breaks the rule that a word comes from this approach. Fixed in
   round 2.
3. An advert before the approach's end hid the new offer, the reverse of the other
   order. Holds. An approved entry now gets `reoffered` too.
4. An older request to another helper brought back a handled offer. Holds. An older
   request now counts as a delayed copy.
5. Suppression ignored position, so a reused id at another position vanished. Holds.
   It now compares position, like the offer identity.
6. A new own round on the same POI was hidden behind its handled entry. Holds. Fixed in
   round 2.
7. A later round changes a phase-less UAV's card from confirmed to searching. Holds for
   that case only, and kept, as listed under limits: with a phase the card never reads
   a confirmed slot, so current companions are unchanged, and the alternative would let
   a later approach label an earlier dock.

### Codex final diff, round 2 (reject)

Codex accepted point 7's reason under the published-phase constraint: 42 comparisons
against HEAD matched. Rounds 1, 4 and 5 hold on its tests.

1. The same owner and task id at another position inherited the approval, and A's
   outcome labelled B. Holds. Another position no longer counts as the same assignment.
2. An auto approach with no heard APPROACH beat could still fill an `awaitingOutcome`
   entry. Holds. Late filling is gone for rule 3: the trace above shows the first beat
   after the approach carries the outcome, so waiting served nothing. Only a disarm
   waits, for that first beat. What is left needs every beat lost across two
   approaches, listed under limits.
3. A repeated APPLIED lost `reoffered`, hiding a new offer. Holds. It keeps it.
4. Denying a repeat own round lost the handled history. Holds. The history is kept and
   the selector shows the own round over it.

### Codex final diff, round 3 (reject)

Round 2's fixes 1, 3 and 4 hold, and Codex confirmed that a beat with a post-approach
phase carries the committed outcome (`navigation_loop.py`, `nav_display.py`,
`task_actor.py`).

1. A's end unheard, then B's APPROACH beats carrying A's latched outcome, then B's end:
   B's outcome labelled A, and the limit understated this. Holds. An APPROACH beat with
   a changed outcome now ends the entry as unknown, and an `awaitingOutcome` wait.
   The limit now states what is left. An existing test changed by intent: it read a
   changed outcome on an APPROACH beat as the same approach.

### Codex final diff, round 4 (reject on wording only)

The round 3 fix holds: the reducer tests and 104 single-approach probes passed.

1. The limit understated the remaining case: an unchanged outcome is not needed when
   the later approach's APPROACH beats are lost too. Holds; the limit now says so.
   Codex accepts the ambiguity for this simulation-marker scope once disclosed: the
   received sequence looks the same as one approach and the marker is display only. An
   approach id stays outside this change. No code changed after this round.

### Codex final diff, round 5 (reject)

Codex found that the audit's fixes hold: the NavPy change only fills the outgoing message,
protected files are unchanged, the labels and static markers match this note, and the gap,
late-cancel and multi-helper fixes hold. Its sandbox could not start Python.

1. A delayed APPLIED brought a finished slot back, and the slot then took another dock's
   round: dock A's docked marker was hidden and dock B's word landed at A. Holds; the real
   reducer reproduced it. Two causes, both fixed. A request copy still pending for that UAV
   now goes with the dock's end, so nothing can bring the slot back. A slot no longer takes
   a round at another position, which also covers a stale slot from a lost FREE beat.
2. A re-offered task given to the same helper kept the old approval. Holds; reproduced.
   `reoffered` now holds the uid that released the entry, and a request after it is a new
   assignment: the approved slot goes to history, outcome unknown, and the new slot is
   assigned. A repeat from before the release still keeps the approval (round 2, finding 3).
3. A restart that moved no UAV cleared every client's map. Holds; a test failing on the old
   route shows it. It now sends `new_mission` only if a UAV restarted, and otherwise main's
   message with no reason.

These fixes showed two more, fixed the same way. A request copy from before a re-advert
could still start a round at a handled dock; it is now ignored, in either order with the
dock's end. And with no late copy left, "history shows over a late copy of a slot" only hid
a restarted helper's new slot, so any live state now shows over history.

Tests of this branch changed for this: rounds a slot takes now carry the slot's position,
and `reoffered` is asserted as the releasing uid instead of `true`.

### Live check, 3-UAV delivery (sim, before the review's fixes)

Awaiting, handled "docked (sim contact)" and "dock failed (sim)" seen on the map. A
40 ms sampler of the page's marker entities caught each dock's sequence: UAV 5, the
helper: offered, assigned, confirm, approved, outcome unknown, dock failed; UAVs 4 and
6: confirm, approved, outcome unknown, docked. UAV 4, which found no delivery dock,
went to its home dock through the same states, and its card showed a position.
Handled stayed through all disarms until the restart route was posted by hand, because
no button was offered in that phase (audit findings 5, 9, 17 below). No marker before
detection in monitoring; static `D1...` markers only in planning. Detected lasts about
0.1 s before the offer, so only the replay shows it. Canceled and a diversion to a dock
nobody took were not exercised live. This shows marker states and sim contact, not
physical docking or cargo receipt.

### Final audit (42 findings)

Eight agents read the final diff, and a second pass tried to refute each finding; four
fell. Numbers are the audit's; its text and evidence are in
`.logs/dock-marker-states/audit_findings.txt`.

Held, and fixed:

- 5, 9, 17: a landed container fleet has no Restart button and a launch cleared nothing,
  so handled markers carried into the next mission. A fleet launch now broadcasts
  `new_mission` once prepared. 9's "high" was too strong: mission 1 is unaffected, so
  medium. 17's per-UAV variant is declined: a per-UAV bungee launch continues the mission.
- 6, 15: bungee Restart lists armed UAVs only, so other pages kept disarmed UAVs' markers.
  `new_mission` now clears the whole map everywhere. 15's "until a reload" was too strong:
  a later START MISSION cleared them.
- 2: a cancel lost its word when the teardown beat came before the broadcast, which on the
  same send path happened in 1 of 5 recorded approvals. A late cancel now sets it, and a
  real word stays.
- 1, 4: a round pending across a socket gap stayed "awaiting" after its answer was missed,
  or took a later approach's word if approved after the gap. `link_gap` now flags it.
- 0: a re-advert while a slot was assigned or confirming hid the new offer and a second
  UAV. It is narrow: cyclic mode, the same id and the same position. The advert now
  releases slots too, and each UAV on an offer has its own marker.
- 10: a new offer or round at a handled dock overprinted its history. Its case B cannot
  occur as written, because a confirmed task is never taken back, but a RESET re-pick, a
  second launch or another UAV's round at that dock can. A live state now hides history
  at its spot, and what remains gets label rows.
- 11: words printed over each other at Reset View. They now show within 1500 m. The real
  label scale at 9411 m is 0.482, not 0.68, which only widens the range.
- 12: `#0033A0`'s word nearly vanishes on ocean or the no-imagery base; dark green ground
  stays readable. The word now gets a white outline. The colour predates this change, so
  the pad stays, as listed under limits.
- 14: a home hub's strokes showed through the handled pad. The pad fill is now opaque.

Held, and declared without code:

- 3: part (a), a re-assigned helper's card shows the new task's label where main showed
  none, is right and now listed under limits. Part (b) needs an advert lost to this
  client; its limit is reworded, because the later round does draw an own marker.

Held, and logged: 13 is todo 191 (low, sim only). 16 is todo 192 (pre-existing).

Fell:

- 7: Cancel removes nothing. Approval already removed the card's label on both sides, and
  main is the one adding a stale label, for under one beat.
- 8: that section of `delivery_demo_prep.md` is already marked replaced.
- 32: boot ids are random 32-bit per process, or seeded per ordinal in lockstep, so UAVs
  do not share one. The cheap cases went in anyway.
- 36: the round runner sends real values or NaN for all three, and adverts always carry
  numbers, so a partial position cannot reach the map. These cases went in too.

Test gaps, all closed by new tests with the audit's exact mutants: 18-23, 25-31, 33, 37,
39, 40. Corrections: most were test-only, so their "medium" was too strong, and the code
was right. 24's per-status ranks no longer exist, since the selector was rewritten; its
new order has tests. 34(a): history now hides while the new boot's own round is live, and
`requestUid` is pinned on the own entry. 35(b) is inverted by intent: a later advert marks
the slot reoffered and both show. 38: only (a) can occur, since the backend drops an
old-boot beat after a new one. 41: only (a) and (b) can occur.

Mutation runs on the final code:

- Reducer, after round 10's fix: 251 killed, 6 survived, all six equivalent. `outcomeCode`
  without its null case compares the same and `heardApproach` tests `undefined`; `uavOf`
  preferring `handledBy` changes nothing, because the fields are exclusive; a flat rank in
  the selector changes nothing we could reach, because within one UAV's offer a live entry
  is inserted after its history; `hidePad` looking through every spot changes nothing,
  because an entry is the pad only of its own spot; `roundHeard` with no uid changes
  nothing, because a round with no uid moves nothing anyway; ranking an entry with no status
  as waiting changes nothing, because waiting entries come after available ones and history
  never shares a spot with a live entry. 102 older mutants no longer apply, because their
  text was rewritten.
- Backend: 11 of 11 killed.
- Marker hooks, label helper, `useTaskAssignment`, the reset message and the map wiring:
  83 of 83 killed. The first pass left 11 survivors and one mutant that never ran, because
  its text was not found. Tests added for them kill all twelve. One was a real gap: a
  handled entry hidden by a live state comes back under the same key, so the hook must
  forget its marker.
- Icons: 12 of 12 killed, by the node logic test.

The three real recordings, two from the first live check and one from the second, replayed
through the reducer after round 6's fixes read the same, line for line, as through the
reducer before round 5's fixes. The replay prints the drawn entries and their words, not
which pad draws. An
earlier comparison, before the audit, differed in pass 2 only at the restart, which now
clears the whole map in one step.

### Second live check (sim, after the audit's fixes)

A live-tester agent ran four missions on three stacks in 102 minutes. Two more agents
checked its report against the logs and read its pictures. All of it is in
`.logs/dock-marker-states/live2/`. This shows marker states and sim contact, not physical
docking or cargo receipt.

Held:

- Each dock's labels, a helper's and a UAV's own: detected for 0.05 to 0.08 s, offered,
  assigned, confirm?, approved, outcome unknown, then the word. Approved to unknown took
  0.29 to 3.40 s.
- Deny and a timeout: the marker goes and no handled marker appears. Of 15 rounds, 1 got no
  answer and was ended by an E-STOP.
- Cancel: approved to canceled with no unknown first, and the UAV can ask again.
- Handled stays through disarms, Edit Plan, Upload and a refused launch (HTTP 409) until the
  next launch, which clears it in 152 ms. A marker stayed through a Stop ALL.
- No marker before its message: 37 to 256 ms after it.
- Static `D1...` markers only in planning, and the dark blue word has a white outline.
- E-STOP on one UAV: its approved entry reads outcome unknown, its pending entry goes, and
  another UAV's handled marker stays.

Found, and fixed, from the pictures: all three are under "Map" and "One marker per dock".

1. Words cut or missing at a low camera right after a state change (Cesium 1.138).
2. An older badge drawn over a newer one: UAV 4's canceled and failed rounds at its home
   dock, under two task ids, for 13 minutes. One pad per spot now.
3. Pads of 14 px under the sim's ring at Reset View, with the badge unreadable.

Not exercised: a homing diversion to a dock nobody took; an E-STOP after an APPROACH beat;
the clear on a launch other than a forced one. Detected, offered, assigned, "?" and an
approved pad with its check have no picture in this run. Tester shortcuts: UAV 5's label was
set by hand with "++++" for 19 s, so that picture cannot credit its complete word to leaving
range; a second tab was open, so duplicate denies cannot be told apart; a replacement
approve timer fired twice.

Out of scope, logged: todos 194 to 199, and the hand demo's CPU share ending mid-session,
already todo 152.

### Live check, final code (sim)

Simulation only: nothing here proves physical docking or cargo receipt. Two 3-UAV hand demos,
evidence in `.logs/dock-marker-states/live3`, each claim then rechecked by two independent
readers of the files and pictures. It ran on the code before round 7's change to `assignRequest`,
which touches no drawing and leaves the three earlier recordings' replays unchanged.

The tester could not use the page's auto-approve timer, because a permission check refused it.
It clicked by hand, 8 to 25 s a step against a 30 s card, so UAV 5's first three rounds at dock
A timed out on flat ground. Other shortcuts: a stubbed `confirm` for the forced launch, synthetic
hold events, the plan fed to the file input, and E-STOP of UAV 5 to get all UAVs disarmed.

Flat ground, no token (the case where words were cut):

- A1 pass for what was pictured: confirm, docked and canceled words whole, and in the logs every
  word's letters full and no raw height above 0, where the old fault read about 1316. Only the
  confirm word was caught soon after its change. Assigned, approved and outcome unknown are logs
  only, and outcome unknown (0.79 and 0.90 s) ended before its letter read. No dock failed.
- A2: the awaiting pad is pictured; approved (1.4 to 3 s) was missed.
- A3 not exercised: no UAV asked twice at one dock and no two entries shared a spot in either
  stack. The one-pad rule and label rows rest on the reducer and hook tests. Approve then cancel on
  UAV 5 went to "canceled" in 0.53 s with no outcome unknown between.
- A4: Reset View draws pads at about 23.5 px and no state words. No badge can be read there: the
  sim's marks cover the centres, and one UAV's sim label box covers most of a pad (todo 191).
- A5 pass: a restart naming UAV 99 returned `not_connected`, sent no `new_mission`, and all markers
  stayed. A6 pass: the launch's `new_mission` came 112 ms after the confirm and the page was
  empty 102 ms later.

Ion terrain, ground 1332 to 1338 m:

- B1: confirm and docked letters full, raw heights within rounding at the delivery docks and 3.35 m
  under the ground at the home dock, whose tile was not yet deep. At delivery docks 150 m apart, in
  this 1024 px window at 650 m, the two docked words print over each other and "UAV 6" cannot be
  read, which is the limit under Map. It is more than the note's "can touch", so todo 201 records it.
- B2: a word that changed off screen was whole at the 10 s picture. The 1 s picture is stale, so it
  settles nothing, and the tester's later letter reads are not in the saved files.
- B3: as A4. Delivery pads overlap by 2 to 3 px here, so one can cover part of the other's badge.
- B4: in a tilted view the pads sit 5 to 8 capture px below the sim rings at the delivery docks and
  about 1 px at the home dock. Likely cause, not proven: pads clamp to the tile rendered now, as they
  did on main, and rings use the most detailed sampled height. Nothing else on the map moved.

Open, not explained: each flight had two page-like socket clients beside the recorder although the
brief asks for one page, and flat-ground denies came in pairs about 100 ms apart, so two pages' cards
may have fired. Backend and NavPy logs held no error. After the cancel, UAV 5 stayed in HOMING
with no new round for nearly 8 minutes, where earlier cycles took about 16 s; cause not established
(todo 202). No marker's state or word went unsupported by its messages: 36 page records match a
replay through the reducer, every outcome word matches its heartbeats, and no marker came before its
message. Hard fixes this run checks: words whole at a state change and larger pads. Not pictured
live: approved pad, dark blue word outline, assigned, offered, outcome unknown and dock failed.

### Codex final diff, round 6 (reject)

Codex found round 5's sequences fixed, the restart route and the disclosed limits right, and
the Cesium explanation matching the installed 1.138 and the upstream fix. Its sandbox could
not start Python. It gave four new sequences, each reproduced with the real reducer; all
four failed here too (`.logs/dock-marker-states/round6_probe_old.txt`).

1. A delayed round took the slot back and gave dock A dock B's outcome. Holds: with round
   3:70 approved at B, a delayed 3:60 at A made the slot approved again, and the approach's
   end labelled both. A round already heard, or older than one that UAV asked since, now
   changes nothing.
2. A dock's end dropped a valid new request when its advert came later, so the helper's
   assignment was lost. Holds. The end no longer drops anything.
3. An APPLIED from before a release used up the new request and retired the approval, and
   the real APPLIED then found nothing. Holds. An APPLIED older than the pending request now
   leaves it alone.
4. An older detected ring hid a newer approved pad at one spot. Holds. The pad now goes by
   state, not by arrival. Two tests of this branch changed: a waiting offer beside an
   assigned slot took the pad for being newer.

Findings 2 and 3 were the third pass at one question, what a late request or APPLIED means,
so this round read the owner instead of patching the reducer again. The answer is under "One
marker per dock", and the three recordings agree with it: no later request or advert after a
task's first APPLIED, and no APPLIED before its request (few messages, 1 to 3 of each per
recording). It replaces what round 5 added. A delayed copy makes no round, so a dock's end has
nothing to drop. A later request releases what holds the offer, as an advert does, so an
APPLIED with a pending request beside a slot is always a new assignment; the branch that kept
an approval through a repeat is gone. Two limits went with it: a restarted helper assigned
again with its advert lost, and a request to another helper showing only from its APPLIED. A
third narrowed to offers no UAV holds.

Found while doing this: the UAV's request to itself, which the rule must not read as a release.

Tests changed for this, with the reason each is the owner's real order: the `applied` helper's
default seq; a restarted helper's new assignment and a helper given the task again (22 to 30);
a request to another helper after a release (24 to 30); a request after an own round (20 to
61, since 20 is older than the round). A delayed retry beside an approved slot now makes no
round at all, so it asserts no change. Older cases that sent a copy and a repeat APPLIED still
hold, now with nothing to do.

### Codex final diff, round 7 (reject)

Codex found round 6's four sequences fixed, and the recordings, the protected files, the
null receivers, the static docks and the wording right. It ran the reducer suite and the
billboard test; its sandbox ran no Python, React or live check. Three new sequences, each
reproduced with the real reducer, and each needing messages delivered out of the order they
were sent.

1. An owner's own round heard first made its older peer request read as a copy, so the
   helper got no slot. Holds, and it was this branch's mistake in round 6: the owner's order
   covers a task's adverts, requests and APPLIED, and a UAV's own round is no step of it. The
   round 6 test had been moved from 20 to 61 to fit; it is back at 20, with 61 beside it.
   A round now makes no request a copy, and a request releases an own entry only if it is
   newer than the round. One limit went with it.
2. A round older than a denied round reads as new, since a denial leaves no entry. Holds as
   a sequence, and round 6's "an older round changes nothing" was too broad, so that text is
   corrected. No code change: the backend opens a card for that round, as on main, so a
   marker agrees with what the operator is asked, and the card's answer or countdown ends
   it. Remembering denied rounds would add a state to hide a marker whose card still shows.
   It is a Known limit, with the card's own behaviour logged as todo 200.
3. An APPLIED stamped after a RESET's advert, and delivered before it, hides the new offer.
   Holds, and the Known limit was wrong that it needs a new reservation and the 2 s wait. No
   code change: the owner's next message of that task shows it, while the slot it hides
   behind is still flown. A floor of the request instead would let a delayed advert leave
   an offered marker for good, which is the worse way to fail. The limit is corrected.

### Codex final diff, round 8

Reject, one finding, reproduced by its author on the real reducer and read here against the
code. It also found round 7's finding 1 closed, the reasons for findings 2 and 3 sound (the
0.5 s repeat is only while a peer is owed the advert, as Known limits says), and the live
check section supported by its files.

1. Holds, fixed. An approved UAV was heard in APPROACH, its next beat heard was CONFIRM with a
   new failed outcome, and the later round's messages and APPROACH beats were lost. CONFIRM kept
   the entry open, so the next HOLD/docked labelled the first dock "docked". The state machine only
   reaches CONFIRM or TO_TASK (DETECT with a peer task) by leaving NAV, so after a heard approach
   those beats now close the entry by rule 3, and the disarmed path likewise. The sequence now
   reads "dock failed". Limit: this reads delivery order. A CONFIRM beat from before the
   approach, delivered after an APPROACH beat, would give outcome unknown; one link reader and
   1 Hz beats make that unlikely, and the wrong word is never "docked". The Known limit was too
   narrow and now names a timeout approval.

### Codex final diff, round 9

Reject, one finding, the Known limit itself: A's APPROACH heard, then everything lost up to B's
HOLD/docked, labels A "docked" until mission reset, with no reordering or configuration change.

1. Holds, fixed for the case it names. The backend already marks a beat stale once it outlives its own TTL
   (`companion_swarm_state.py`), so `useTaskAssignment` now dispatches `link_gap` for that UAV at
   the change to stale, and an end after that gap reads "outcome unknown". No threshold is
   added; the sender's TTL is the limit. Cost: a real outcome
   after a stale gap also reads "outcome unknown", while the UAV card keeps NavPy's word. The
   recordings show this is rare but real: stale spans 0, 33, 1 and 0, one of them starting in
   APPROACH. A later approach whose beats are all lost in gaps each under the TTL stays a Known
   limit; closing it needs an approach id (todo 190). Also dropped `APPROACH` from
   `OPEN_ROUND_PHASES`, dead since round 8's change (the mutant that removed it survived).

### Codex final diff, round 10

Reject, two findings.

1. Holds as a sequence, not closed, taken to the owner. A's APPROACH heard, then under 5 s later a
   HOLD/docked from a whole later approach, with every beat and message between lost. The note's
   "a whole cycle cannot pass inside one TTL" was unsupported and is withdrawn. Codex: "never
   another approach's outcome" is the constraint, and timeout approval has no minimum time. Here:
   it needs auto or timeout approval (the demo has none), an approach ended and another detected,
   confirmed and flown to contact in under 5 s, with 1 Hz beats all lost; none was seen in any
   recording. Only an approach id in the heartbeat closes it (todo 190), which changes the wire
   format, so it is outside this step.
2. Holds, fixed. A gap hid the recorded approach, so CONFIRM or TO_TASK kept the marker approved
   for the next wait. The open-phase test now reads `approachOutcome`, so it goes to handled with
   outcome unknown, and with no approach heard the round stays open.

### Codex final diff, round 11

Approve, for its narrow scope only: round 10's fix, its tests, the changed fixture, and that the
note states finding 1 fairly. Round 10's finding 1 went to the owner, who accepted the limit on 2026-11-02
and kept todo 190 as its fix.
