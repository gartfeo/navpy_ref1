# GCS mission phase from NavPy (todos 60, 61, 63, 72 b/d, 76)

Status: v5 approved by the owner 2026-10-28 with the six recommended
decisions; a fifth design review was skipped (the final diff gets one).
Codex rejected v1 to v4; their points are answered in "Review". Mock:
`gcs_mission_phase_mock.html`.

## Problem

The sidebar status row is guessed in the frontend
(`utils/missionStatus.js` `deriveStatusId`) from mode, armed state, mission
progress and the GCS's task-assignment entry:

- With no assignment entry, an armed UAV in GUIDED reads IDLE: the owner on
  its own dock and the home UAV through review and approach (todo 60).
- A homing UAV reads SEARCHING; there is no HOMING state (72b).
- Every UAV reads IDLE after disarm: no docked state (63), no dock-failed
  state with its reason (76).
- NavPy reports its end states only as STATUSTEXT (`STATE DOCKED`,
  `STATE DOCK_FAILED`, `nav_transition.py:116-140`; `DOCKED: task=...`,
  `dock_contact_episode.py:160`). STATUSTEXT is cut to 50 bytes
  (`vehicle_messaging.py:68`) and the HUD shows it for about 5 s.
- Sim-only outcomes carry no sim tag (72d). The only contact sensor is the
  sim's (`SimDockContactSensor`), and only an armed (sim) episode can end in
  DOCKED or DOCK_FAILED (`nav_transition.py:79`, `missed_dock.md`).

Todo 61 (helper never CONFIRMED): the reducer was fixed on main (`5e178c55`
and four later commits, 2026-10-25). With this design the badge comes from
NavPy's phase, so 61 is closed by the phase; the slot keeps the task label.

## Design

NavPy publishes two display values: the **current phase** and the **last
outcome** of a final approach. Both go in the existing SWARM_HEARTBEAT state
byte. The GCS shows the phase, and the outcome where the phase does not say
more.

### 1. Built on the navigation thread after commit

`StateActionDispatcher.act` (`navigation_loop.py:87-103`) commits each
transition and then calls `publish_phase(committed)`; a raising transition
publishes nothing. Today that hook calls
`NavNetwork.publish_participation(approaching, docked)`
(`nav_runtime_composition.py:197`), which sets the BUSY flags the heartbeat
reads.

New: the same hook builds one immutable `NavDisplay(phase, outcome)` from
the committed state and flags read on the navigation thread, and hands it to
the network with the participation flags in one update. The heartbeat reads
only that stored value. So a beat never sees an uncommitted state, a
half-run teardown, or a phase from one cycle with an outcome from another.

Display only: the builder (`core/flow/nav_display.py`, new,
mission-agnostic) is called only by that hook; no navigation, decision or
command code reads the value. A guard test checks that only the composition
and the heartbeat path import it (Pure Vision Approach Constraint).

### 2. Current phase

First match wins.

| # | Committed state + flags | Phase |
|---|---|---|
| 1 | CONFIRM | CONFIRM |
| 2 | NAV (entered only from a CONFIRMED POI, `poi_status_decision.py:75-78`) | APPROACH |
| 3 | DETECT with `peer_navigation` (flying to an assigned peer task) | TO_TASK |
| 4 | DETECT with the module's return flag (delivery: DDH active) | HOMING |
| 5 | DETECT | SEARCH |
| 6 | RESET or RECOVERY (after an approach: reset, climb back if low) | RECOVERING |
| 7 | ONHOLD or DOCKED (not started, before the detect waypoint, mode not active, one-shot end) | HOLD |

- Peer travel outranks the DDH return (row 3 before 4), as
  `work_source.py:35-41` does. Both are set up in DETECT
  (`work_source.py:34-46`), so the hub orbit is HOMING, then CONFIRM and
  APPROACH once the home dock is confirmed.
- The return flag is a port, `returning: Callable[[], bool]`; the delivery
  composition passes `poi.mission.default_delivery_hub_active`, as for
  `HomeDockRules`. Core names no product.

### 3. Last outcome

- `NavTransition.exit` already decides docked vs failed (with the failure
  reason). It writes that into a display latch the task reset does not
  touch. Only the next outcome replaces it; a new NavPy boot starts empty.
- So the outcome stays through the DOCKED entry's reset
  (`navigation_transition_handler.py:129`), the post-dock climb, a cyclic
  return to DETECT in the same tick (`recovery.py:99-109`), disarm,
  manual-mode ONHOLD (`navigation_decision.py:80`) and the next approach.
  Each beat carries it, so no sequence between two beats can lose it; only a
  second outcome inside one beat interval replaces the first unseen (logged
  in NavPy as today).

### 4. Wire: the SWARM_HEARTBEAT state byte

| Bits | Meaning |
|---|---|
| 0 | BUSY (1) / FREE (0), as today |
| 1-3 | phase: 0 none, 1 HOLD, 2 SEARCH, 3 TO_TASK, 4 HOMING, 5 CONFIRM, 6 APPROACH, 7 RECOVERING |
| 4-7 | outcome: 0 none, 1 docked, 2 + r failed with reason index r (r = 0..13) |

Reason table `NavFailReason` (core, numbered, append-only; a test pins each
number):

| r | Reason | Producer |
|---|---|---|
| 0 | unknown (any reason not below) | |
| 1 | nav_fail (no reason named) | `command_liveness.py:61`, `episode_latch.py:38` |
| 2 | no_detection | `final_approach_command_dispatch.py:95` |
| 3 | missed_dock | `final_approach_command_dispatch.py:105`, `episode_latch.py:35` |
| 4 | navigation_rejected | `final_approach_command_dispatch.py:112` |
| 5 | source_command_stale | `final_approach_command_dispatch.py:125` |
| 6 | discontinuity_source_missing | `final_approach_publication_admission.py:99` |
| 7 | final_approach_record_unavailable | `final_approach_record_deadline.py:44` |
| 8 | source_event_active_poi_missing | `final_approach_source_event_handler.py:69` |
| 9 | source_event_pump | `nav_final_approach_composition.py:239` |

- The producers pass these names as constants from the table module, and a
  test checks every latching call site uses one. An unknown string encodes
  outcome 2 (failed, unknown), never 0. 10-13 are spare.
- `SwarmNodeState.from_code` reads bit 0 only. Every decoder goes through it
  or is updated: `swarm_presence.py:97`, `companion_swarm_state.py:61`, the
  direct enum uses in `test_nav_controller.py:7974` and
  `test_swarm_presence.py:147`, and the raw-byte checks in
  `test_auction_baton.py:181,227`, which get cases with bits 1-7 set. No Lua
  or script decodes the byte.
- Fleet rule: all UAVs run one build. An older peer reads any non-zero byte
  as BUSY and skips this UAV in auctions; stated in the deployment notes.
- Air time: MAVLink 2 trims trailing zeros and `state` is last, so a FREE
  beat with a phase is 21 instead of 18 payload bytes (+3 B/s per UAV);
  BUSY beats are already full. Within radio_budget.md's 0.9 kB/s margin.
- Latency: nominally the next beat (1 Hz); a lost beat is repeated by the
  next. No new message, no extra send.

### 5. Backend

- `CompanionSwarmState.snapshot()` adds `phase`, `outcome` and
  `fail_reason` (names or null) to `swarm`; `swarm` is already in
  `_has_changed`.
- Retired boots: the record keeps the boot ids it has replaced in this
  vehicle session; a beat from one of them is dropped (by identity, not
  numeric order). So A, then B, then a late A keeps B.
- `approach_seq`: the seq of the current boot's newest APPROACH beat (null
  until one, reset by a new boot), for the label fence in section 6.

### 6. Frontend

`computeMissionStatus` uses `swarm.phase` when present:

| Phase | Badge | Extra |
|---|---|---|
| HOLD | the outcome if any, else EN ROUTE (armed in AUTO) or IDLE | |
| SEARCH | SEARCHING in any mode (EN ROUTE before the search waypoints) | progress bar |
| TO_TASK | APPROACHING | task label and distance (ASSIGNED slot, same boot) |
| HOMING | HOMING | |
| CONFIRM | CONFIRMING | task label from an ASSIGNED / CONFIRMING slot |
| APPROACH | CONFIRMED | none |
| RECOVERING | the outcome if any, else RECOVERING | |

Outcome badges: DOCKED (SIM CONTACT), DOCK FAILED (SIM) with a "Reason: ..."
row (unknown: "Reason: other, see the NavPy log"). In every other phase a
held outcome shows as a dim "Last: ..." line under the badge.

- CONFIRMED is NavPy's confirmation decision, by the operator or the
  auto-approve policy. NAV is entered only after it, for owner, home UAV and
  helper alike. The badge reads neither the assignment slot nor the approved
  card.
- Task label, fail-closed: only from an ASSIGNED or CONFIRMING slot whose
  helper boot (`doingUid.boot_id`, the helper's own "doing") equals the boot
  of the current heartbeat (`swarm.boot`). A WAITING slot carries no helper
  boot yet, so it gives no label while a phase is present (the helper's
  phase is still SEARCH). Within one boot such a slot is the task the
  helper holds: it stays BUSY while it holds it and the next FREE beat
  retires it (`taskAssignmentState.js` `swarmHeartbeat`). A slot from
  another boot is never shown, even when the next beats heard are BUSY
  (`useTaskAssignment.js:25` forwards FREE beats only). A CONFIRMED slot
  never retires there and cannot be tied to the current episode or boot, so
  when the phase is present the card shows no label or coordinates from it.
  Within one boot a slot also needs an `assignedUid` (the "doing" it was
  first assigned by, kept when the handshake repeats) newer than
  `approach_seq`: the helper stamps its answers and its beats from one
  counter, so a slot assigned before an approach the GCS heard belongs to an
  earlier task, even when no FREE beat was heard between the two tasks. A
  slot with no `assignedUid` gives no label. No reducer change;
  `taskAssignmentState.js` is untouched. With no phase (old trace) the card
  works as today.
- With a phase present the badge never reads the assignment slot or the
  approved card, including HOLD: HOLD's guess uses only mode, armed state
  and mission progress (IDLE / EN ROUTE).
- The no-phase fallback (`deriveStatusId`) is used only when `swarm.phase` is
  absent: an old trace or companion, or no beat yet. Replaying
  `.logs/e10/live_ws_trace.jsonl` checks it.
- Stale beats: the last phase and outcome are shown dimmed with
  "(last heard)"; a stale phase with no outcome falls back to the mode guess
  (IDLE / EN ROUTE / SEARCHING), never to the assignment. A new boot
  replaces both.

### 7. Wording (72d)

- Real build: with no contact sensor no episode is armed, so a real vision
  approach never reaches DOCKED or DOCK_FAILED; a NAV_FAIL ends it into
  RESET, then straight to DETECT (`recovery.py:99-102`). The card shows
  RECOVERING for that tick, then SEARCHING. No docked or "approach ended"
  label exists for hardware, and no completion rule is added for display.
- Sim tags: "DOCKED (SIM CONTACT)", "DOCK FAILED (SIM)"; closest-approach
  labels "docked (sim)", "missed (sim)", "aborted (sim)" (en.json, hy.json; the
  map hook is unchanged); NavPy lines `STATE DOCKED (sim): 3d=...`,
  `STATE DOCK_FAILED (sim): reason=...`, `DOCKED (sim): task=...`.
  `test_docked_state.py` and `test_missed_dock.py` follow.
- No string claims physical docking or cargo receipt.

## Tests

- NavPy: each phase row from committed state and flags (TO_TASK over HOMING
  for a homing UAV diverted to a peer task; hub orbit HOMING, CONFIRM,
  APPROACH; pre-search HOLD; RECOVERING; one-shot DOCKED as HOLD + docked);
  the outcome through DOCKED's reset, the post-dock climb, disarm, and
  DOCKED, DETECT, CONFIRM, NAV all between two beats (still carried), replaced
  only by the next outcome; a raising
  transition publishes nothing new; fixed byte values with both BUSY values,
  every reason index, an unknown reason (outcome 2); `from_code` ignores bits
  1-7; reason call sites use the table; the import guard.
- Backend: phase, outcome and reason reach `swarm`; other senders and older
  seqs ignored; A, B, late A keeps B; `_has_changed` broadcasts a change.
- Frontend: owner and home UAV CONFIRMING then CONFIRMED with no assignment
  (60); HOMING; helper TO_TASK then CONFIRMED (61); DOCKED; DOCK FAILED with
  reason; SEARCHING with "Last: ..."; end state after disarm and when stale;
  a CONFIRMED slot (same boot or an older one) gives no label, coordinates or
  badge in any phase, including HOLD; an ASSIGNED or CONFIRMING slot from boot
  A gives no label or coordinates once boot B's BUSY CONFIRM or TO_TASK beat
  arrives with no FREE beat between; no phase falls back to today's guess.
- Replay of the e10 trace; live (live-tester): 3-UAV delivery, and
  `--sim-dock-net-radius-m 0` for DOCK FAILED with its reason.

## Out of scope

The handled-dock marker's outcome (dock-marker-states), the task card's
failure reason (the confirm files belong to the confirm-cue session),
re-offering a failed dock's task (todo 75).

## Owner decisions

1. Phase source: (a) NavPy publishes phase and outcome in the heartbeat byte.
   Recommended. (b) the backend parses STATUSTEXT: misses homing, and the
   50-byte cut drops the end of the failure line. (c) keep guessing.
2. New states: HOMING, RECOVERING, DOCKED (SIM CONTACT), DOCK FAILED (SIM)
   with reason; a helper flying to its task reads APPROACHING; CONFIRMED for
   every UAV from NavPy's approach state.
3. Real build: no docked or "approach ended" state exists to show, since no
   physical contact signal exists; sim outcomes carry the sim tag everywhere.
4. Failure reason on the UAV card only; the dock marker comes later.
5. After disarm: the last outcome stays until the next outcome or a new
   companion boot; during later phases it shows as "Last: ...".
6. A helper's CONFIRMED card shows no task label or coordinates (the GCS
   cannot tie a confirmed slot to the current approach); ASSIGNED and
   CONFIRMING slots from the current boot keep them. With a phase present the
   WAITING badge is gone: the helper reads SEARCHING until it flies to the task.

## Review

### Codex round 1 (reject)

1. DETECT is not always search; ONHOLD covers pre-search transit; RECOVERY
   and RESET unmapped. Holds. Full table, section 2.
2. APPROACH_ENDED unreachable on the real path. Holds
   (`mission_navigation.py:54-56`). Dropped; section 7.
3. Clearing the outcome on task reset erases it. Holds. Separate latch,
   cleared at the next NAV entry; section 3.
4. The heartbeat needs a committed snapshot. Holds; post-commit hook, section 1.
5. The 50-byte STATUSTEXT cut breaks the parser. Holds. Reason index in the byte.
6. Text and beat cannot be matched by boot or episode. Holds; gone with 5.
7. Old assignment or approved card. Holds; badge from the phase, label rules
   in section 6.
8. Direct decoders and older peers. Holds; section 4.
9. Byte count, latency, extra-beat contradiction. Holds; section 4.
10. DOCK_FAILED and `DOCKED: task=` untagged. Holds; section 7.

### Codex round 2 (reject)

1. A cyclic outcome can vanish between beats. Holds (`recovery.py:109`).
   Outcome is now its own field, carried until the next approach; section 3.
2. Post-disarm lifetime held only for one-shot. Holds. Same fix; disarm and
   manual ONHOLD keep the outcome.
3. Old confirmed slot still gives the label. Holds. A non-APPROACH beat
   retires a confirmed slot; section 6.
4. A late old-boot beat restores a retired state. Holds
   (`companion_swarm_state.py:54-60`). Retired boots dropped; section 5.
5. Reason table not defined. Holds. Numbered table, section 4; unknown is
   outcome 2.
6. Mock promised a climb on the real path. Holds. Mock fixed.

### Codex round 3 (reject)

1. Clearing at the next NAV entry can lose an outcome between beats. Holds
   (25 Hz loop, auto-confirm, `recovery.py:117`). Kept until the next outcome
   or boot; section 3.
2. A later CONFIRM beat would retire a live, approved slot. Holds
   (`confirmed_poi_release.py:43`; the hook forwards FREE beats only,
   `useTaskAssignment.js:25`). Retirement rule dropped.
3. A confirmed slot survives a reboot and two back-to-back approaches.
   Holds (`taskAssignmentState.js:230,238`). Fail-closed: no label or badge
   from a CONFIRMED slot when a phase is present, HOLD included; section 6.

### Codex round 4 (reject)

1. A below-CONFIRMED slot from an earlier boot survives when only BUSY beats
   follow a restart. Holds; Codex reproduced it with the real reducer. The
   card shows slot metadata only when the slot's helper boot equals the
   current heartbeat boot; section 6.

### Codex final diff round 1 (reject)

1. SEARCH and HOLD fell through to the full mode guess (IDLE in GUIDED or
   LOITER, SEARCHING for HOLD in AUTO). Holds. SEARCH reads SEARCHING in any
   mode, HOLD reads EN ROUTE (armed in AUTO) or IDLE; section 6.
2. A stale phase with no outcome fell back to the assignment-based guess.
   Holds. It falls back to the mode guess only; section 6.
3. A same-boot ASSIGNED slot from an earlier task kept its label when no FREE
   beat was heard between two tasks. Holds (Codex probe with the real
   reducer). Label fence `approach_seq`; sections 5 and 6. Residual: if every
   beat of an approach is lost, the fence does not move.

### Codex final diff round 2 (reject)

1. A repeated handshake during NAV refreshes the slot's `doingUid` past
   `approach_seq`, so a later task showed the old label. Holds (Codex probe
   with the real reducer; `assignAck` keeps `assignedUid` on a same-boot
   repeat). The fence compares the immutable `assignedUid` and fails closed
   without it; regression with the real reducer; section 6.

### Codex final diff round 3 (approve)

No material findings. Checked: npm test 668, build, full pytest (only the 4 OpenCV-5 `test_board_detector` failures, todo 78(a)), e10 replay 0 differences. Live (live-tester, `.logs/gcs-mission-phase/live`): SEARCHING, EN ROUTE, HOMING, APPROACHING, CONFIRMING, CONFIRMED and DOCK FAILED (SIM) with its reason seen; DOCKED (SIM CONTACT) not reached, as every approach failed under about 99% CPU (todo 65).
