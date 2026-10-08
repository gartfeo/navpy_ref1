# Swarm task assignment with acknowledgments

Status: **design agreed 2026-10-07; steps 1–7 implemented and the live
check (8) run 2026-10-08.** In the 3-UAV demo: uid lines, one APPLIED per
peer, assigned before the peer approach, busy peer skipped, WAITING →
ASSIGNED. The demo evaluator's full pass is blocked by failures that
`origin/main` shows too (a second confirmation round after SNAP).
Based on `origin/main` 1bd24d1 (includes 1b4bbf3 assign resend/release and
d8605f6 adverts fenced against assign requests); "today" below means that
base. Owner = UAV that advertises the task; helper = peer that bids and
flies it.

Implementation notes:
- Nav publishes "approaching" every cycle, but only a change reaches the
  actor's lock; a reject repeat's first copy is sent from its timer thread,
  so the nav thread never sends.
- The peer task also preempts an own POI seen in DETECT and not yet
  reviewed: it is dropped like one under CONFIRM.
- Owners stop advertising to a released peer once the task is reserved
  again; that peer then learns from the RECEIVED answer to its next step 4,
  or its 18 s expiry.
- GCS: a step-4 copy and the APPLIED naming it arrive on different vehicle
  links (threads), so an APPLIED heard first waits, one per helper, for its
  copy. `swarm` is a telemetry change key, so a new beat is broadcast.
- Frontend: a FREE beat retires a round only once the helper's "doing" (or
  the APPLIED's ref) orders it; a round without one ends by advert, reject
  or APPLIED. Confirm events match entries by (UAV, task id), and available
  tasks are keyed `${owner}:${task}` like rounds. A helper's confirm carries
  its own POI id, which can differ from the owner's task id; the card then
  never shows CONFIRMED, as before this change (follow-up).
- Audit: the backend log's format also switches the peer gate, so recorded
  legacy runs need no `assigned by owner` line. The gate orders the
  assignment against `Peer navigation started`: a free peer's hub return
  may loiter first.

## Problem

Today the helper fills its slot and starts the peer approach at step 3
("ok do"). If every step-4 "doing" is lost, the owner releases after ~7 s,
re-advertises and may assign another UAV, while the first keeps flying:
two UAVs to one dock. Aborting the approach on the nav side was rejected;
the fix is to never fly before the assignment is final. Separately, a busy
UAV still bids, and a peer that does not bid stalls the auction.

## Decisions (2026-10-07)

1. Keep the four steps: task available → I can do → ok do → doing.
2. Every message is acknowledged; the sender repeats until acknowledged.
   Steps 3–4 use `SWARM_ACK` (acks a message by its UID
   `(sender, boot_id, msg_seq)`, status RECEIVED / PROCESSING / APPLIED).
   Steps 1–2: the bid is the ack of the advert, and the owner re-advertises
   until it has the bid (no extra radio traffic).
3. The helper does not fly until **ASSIGNED** = the owner received its
   "doing" and answered APPLIED. Between step 3 and APPLIED the task is
   **WAITING**, invisible to nav.
4. The owner sends APPLIED only once CONFIRMED to that helper and never takes
   a CONFIRMED task back: a flying helper is never revoked, nav needs no abort.
5. If the owner gives up first, it re-advertises; a WAITING helper drops the
   task and re-bids. A re-advertisement never drops an ASSIGNED task.
6. **Busy UAVs do not take part in auctions**: no bid, reject step 3, drop a
   WAITING task. Busy = holds another UAV's task (WAITING/ASSIGNED) or is
   flying a final approach. Searching, a found own POI not yet approached,
   and the DDH return are free. Owners skip busy or silent peers.
7. **Fuel is per task**: a UAV that cannot fly a task answers that advert
   with "can't do"; the owner leaves it out of that task instead of waiting.
8. **A peer task has priority** over a found own POI (not yet approached) and
   the DDH return. One final approach per flight: the own POI is dropped, not
   offered to the other UAVs (owner ruling 2026-10-08: it may be the assigned
   dock itself, and task ids are numbered per UAV); the DDH return comes
   after the task.
9. GCS shows WAITING vs ASSIGNED (owner APPLIED is the truth) and busy; the
   demo audit accepts repeated copies.

Invariant: at most one ASSIGNED helper per owner task per owner generation.
Losses can delay or rarely strand a task, never duplicate it.

## Message flow

| # | Message | Repeats | Ack | Stops when |
|---|---|---|---|---|
| 1 | `AVAILABLE_TASK_REQUEST` (owner → all) | every 0.5 s while a FREE, non-silent peer (or the released peer) has no reply | the reply | reply recorded |
| 2 | `AVAILABLE_TASK_RESPONSE` (bid or "can't do") | once per advert copy, only if FREE | owner keeps advertising | owner has the reply |
| 3 | `TASK_ASSIGN_REQUEST` | owner: copies at 0, 2, 4 s, fresh UID each, recorded | helper: RECEIVED for every copy | first step-3 ack, or a valid step 4; release at 9 s |
| 4 | `TASK_ASSIGN_RESPONSE` | helper while WAITING: 0, 2, 4, 6, 8 s, fresh UID each | owner: verdict table | APPLIED → ASSIGNED; RECEIVED → drop; newer advert; expiry 18 s |
| – | `SWARM_ACK` | never repeated, never acked | – | – |
| – | `SWARM_HEARTBEAT` (1 Hz) | existing | – | carries the node state (see Busy UAVs) |

Each copy has a fresh UID, so the dedup filter does not stop a receiver
acting twice; the state machines make every handler idempotent.

**Fence.** Step 4 does not say which step-3 round it answers. The owner
records the first step-3 ack from the reserved helper as the floor
`(boot, seq)` and trusts only a step 4 with the same boot and a higher seq.
The helper stamps its step-3 ack before any step-4 copy answering it, and
`msg_seq` is monotonic per process, so a stale step 4 (earlier round or
recycled task id) cannot confirm a new round.

### Owner verdict for a step-4 copy (under the store lock)

| Owner state | Passes fence | accepted | Change | Ack |
|---|---|---|---|---|
| missing / AVAILABLE / reserved to another peer | – | – | none | RECEIVED ("not yours") |
| CONFIRMING or CONFIRMED to sender | no | – | none | PROCESSING ("keep repeating") |
| CONFIRMING to sender | yes | True | → CONFIRMED (log "Task N accepted by M" kept) | APPLIED |
| CONFIRMED to sender | yes | True | none (repeat) | APPLIED |
| CONFIRMING to sender | yes | False | reject (retry/exhausted) | RECEIVED |
| CONFIRMED to sender | yes | False | none + warning (never taken back) | RECEIVED |

Every accepted step-4 copy also counts as a BUSY report for its sender.

### Helper slot (`SelectedTaskSlot`, shared lock)

| State | Event | Result |
|---|---|---|
| EMPTY | step 3 with valid meta, not approaching, task flyable | WAITING; ack; start step-4 repeat; cancel any reject repeat for that key |
| EMPTY | step 3 while approaching, or task no longer flyable | ack + reject repeat; stays EMPTY |
| WAITING | same owner, boot, task: equal copy | ack + one immediate step 4 |
| WAITING | same owner, other boot | replace WAITING (owner restarted) |
| WAITING | any other step 3 | ack + reject repeat for that key |
| WAITING | APPLIED from owner, owner's boot, naming one of this WAITING's step-4 copies | **ASSIGNED**; log "Task T assigned by owner O" |
| WAITING | RECEIVED naming one of its copies | EMPTY + warning |
| WAITING | advert from same owner boot, newer than the accepted request, listing the task | EMPTY, then bid |
| WAITING | own final approach starts | EMPTY + reject repeat (owner retries) |
| WAITING | expiry 18 s | EMPTY + ERROR log |
| WAITING | `TaskActor.reset()` / `clear_selected_poi()` | unchanged |
| ASSIGNED | any advert or ack | unchanged |
| ASSIGNED | other step 3 | ack + reject |
| ASSIGNED | `TaskActor.reset()` / `clear_selected_poi()` (nav ended it) | EMPTY |

`selected()` / `has_selected_pois()` return the task only when ASSIGNED.
Reject repeats stop only on RECEIVED or APPLIED.

## Busy UAVs, fuel and priority

**Node state** (`SwarmNodeState` in `swarm_heartbeat_msg.py`, sent in the
existing `SWARM_HEARTBEAT.state` byte, always 0 today; no dialect change):
`FREE=0`, `BUSY=1`. BUSY = slot WAITING/ASSIGNED, or nav is **approaching**
(committed phase NAV: flying a final approach). Everything else is FREE:
searching, CONFIRM of an own POI, DDH return, ONHOLD/RESET/RECOVERY.
Receivers treat any non-zero code as busy.

**Approaching** is published by the nav thread after each cycle commits the
phase (`NavNetworkRuntime.publish_approaching` →
`TaskActor.set_approaching`, under the actor lock); a raising cycle or loop
exit leaves the last value until the next commit. `TaskActor` defaults to not
approaching (tests, scripts).

**Fuel ("can't do").** `FlyEstimator.time_to_fly` already returns -1 when the
battery cannot reach the POI. Today the owner ignores a -1 handle, so that
peer's missing bid stalls the matrix. Now the helper sends it as an explicit
decline and the owner records the peer as **declined for that task** (out of
that task's matrix, still eligible for others); a later valid bid replaces it.
At step 3 the helper re-checks; if no longer flyable it rejects.

**Priority (nav).** When the slot becomes ASSIGNED while nav is not
approaching, the peer task preempts:
- DETECT with DDH return: `setup()` replaces the hub approach (DDH comes
  after the task, as today after any task).
- CONFIRM of an own POI: the own POI is dropped (active POI cleared, status
  DROPPED, never offered); then DETECT starts the peer approach. DROPPED is
  not final: if it is the assigned dock, the peer approach takes it up again.
  Open: a confirmation round already in flight keeps asking the operator and,
  at its deadline, overwrites DROPPED (follow-up: cancel it on the drop).
- NAV (approaching): cannot happen, the UAV is BUSY and rejects step 3.

**Heartbeat.** `SwarmPresence.heartbeat()` reads the state and stamps meta
under the actor lock, sends outside it; the 1 Hz loop and the CHECK_IN reply
share it. State changes and step-4 stamps share the lock and one seq
counter, so a report is exactly ordered against step-4 copies.

**Owner peer status** (`PeerRoster`, shared lock): per admitted peer
`state`, `order` (last applied `(boot, seq)`), `busy_order`, `last_heard_s`,
`silent`. Busy or silent peers stay in the roster (it routes step 4 and acks).
- Evidence, applied only if newer than `order` (another boot replaces the
  record): heartbeat → its state; bid or decline → FREE; accepted step 4 →
  BUSY. Rejects are not evidence.
- `busy_peers(store)` = this owner's CONFIRMING peers ∪ BUSY peers ∪ silent
  peers. CONFIRMED alone no longer makes a peer busy forever; the helper's
  BUSY reports keep it busy until it reports FREE.
- A task's matrix is complete when every free peer has bid or declined it.
- Stale bids: handles store their bid order. A BUSY report withdraws the
  peer's older handles on AVAILABLE tasks; `record_offer` drops a handle not
  newer than `busy_order`. A plan whose free set changed is aborted.
- Triggers: when a peer enters/leaves the busy set, the owner restarts
  rebroadcast and runs assignment planning; a release and an exhausted reject
  also plan (the exhausted path no longer excludes its task).
- Released peer: a release records `released_to`; rebroadcast keeps
  advertising to that peer until it replies, reports FREE, or the task is
  reserved/reset (otherwise a lost release advert keeps it WAITING 18 s).
- Silence: a peer with no heartbeat/check-in for TTL(`SWARM_HEARTBEAT`) =
  5 s is silent, checked at the owner's own heartbeat tick (no new timers;
  an error there is logged and never stops the heartbeat);
  its next heartbeat revives it; CHECK_OUT makes it silent at once.
- `presence.stop()` never runs under the actor lock (its join can raise).

## GCS

**Backend** (`task_assign_listener.py`): dispatch table instead of the
if/elif chain; import `SwarmAckMsg` (not registered in the GCS today); add
`uid {boot_id, msg_seq}` to the three payloads and ` uid=B:S` to the request
and response log lines (audit regexes use `search`, still match). Handle
SWARM_ACK APPLIED for step 4: resolve `(helper, ref)` against that helper's
current round, emit WS `task_assign_ack` and log
`Task assign ack: owner=O helper=H task_id=T status=APPLIED ref=B:S uid=B:S`.
`vehicle_entry.py` subscribes to the own companion's `SWARM_HEARTBEAT`, keeps
the newest by `(boot, seq)`, and adds `swarm {state, boot, seq, stale}` to the
snapshot (stale after the beat's `ttl_ms`).

**Frontend**: new pure reducer `utils/taskAssignmentState.js` (dispatch table
by action; `useTaskAssignment` becomes a thin wrapper).
- Keys: pending rounds `${owner}:${task}`; once assigned, `helper:${id}` (one
  slot per helper), so recycled task ids never collide.
- Ranks waiting < assigned < confirming < confirmed, never lowered in a round.
- Step 3 → waiting (ignore older seqs); advert newer than the step 3 → drop
  the pending round (released); step-4 reject deletes only that receiver's
  round; `task_assign_ack` → assigned; heartbeat FREE retires stale rounds
  below confirmed.
- UI: new `waiting` status before the GUIDED check in `missionStatus.js`;
  BUSY / UNKNOWN (stale) badge; markers keyed by entry key, ✓ at assigned+.

**Audit and gate**: logs without `uid=` keep today's rule (recorded runs stay
readable); uid-format logs require exactly one (owner, task) per peer whose
APPLIED `ref` names an accepted step-4 line, tolerate copies, released rounds
and rejects; mixed formats are an error. Peer gate: `Task T assigned by
owner O` must appear before `Peer navigation started`.

## Timing (derived from `ttl_defaults.py`)

| Name | Value | Derivation |
|---|---|---|
| resend interval | 2.0 s | new `SWARM_RESEND_INTERVAL_MS`; the cadence `ttl_defaults.py` documents and `ConfirmationManager` uses |
| ack TTL | TTL of the acked message (5 s) | a shorter ack could be dropped while its message is admitted |
| owner step-3 copies / release | 3 (0, 2, 4 s) / 9 s | `ceil(TTL(REQ)/interval)` / last copy + TTL(REQ) |
| helper step-4 copies | 5 (0–8 s) | answer while the owner may still confirm |
| WAITING expiry | 18 s | last copy + TTL(RSP) + ack TTL |
| busy visible | ≤ 1 s | heartbeat interval |
| peer silence | 5 s | TTL(`SWARM_HEARTBEAT`) |

Timers bound liveness only; no safety property depends on them. Tests pin
the relationships and recompute under a monkeypatched `TTL_DEFAULTS`.

## Loss and busy cases

| Case | Outcome |
|---|---|
| advert or bid lost | re-advertised 0.5 s later |
| step-3 copies lost / all lost | next copy 2 s later / release at 9 s, nobody waiting |
| step-3 ack lost | step 4 answered PROCESSING; next step-3 copy acked, next step 4 passes |
| every step 4 lost (original defect) | helper WAITING, not flying; release at 9 s; advert drops WAITING |
| one / every APPLIED lost | next copy re-acked / expiry 18 s with ERROR: stranded, not duplicated |
| stale step 4 | fenced: PROCESSING |
| owner restart (new boot) | new-boot step 3 replaces WAITING; new-boot APPLIED cannot assign it |
| peer busy (other task, or approaching) | no bid; owners skip it within 1 s and assign among free peers |
| peer lacks fuel for one task | "can't do" for that task; matrix completes without it |
| peer becomes busy after bidding / starts its own approach while WAITING | its bids withdrawn / it rejects, owner retries |
| peer in CONFIRM or DDH return wins a task | own POI dropped / hub after the task; flies the peer task |
| all peers busy or silent | no adverts until the first peer reports FREE |
| heartbeat lost / peer silent 5 s | next beat 1 s later / skipped, revived by its next beat |
| release advert lost while helper WAITING | re-advertised to it until it bids or stops WAITING |
| helper nav RESET while ASSIGNED | released, as today |

## Unchanged and out of scope

- Comm hot path, `MessageFilter`, navlink dialect, nav readers,
  final-approach path, Pure Vision constraint.
- Owner RESET forgets tasks it handed out (accepted corner case, owner ruling
  2026-10-07); it now logs dropped CONFIRMING/CONFIRMED dispatches.
- Residual strand: the own final approach starts after the owner is
  CONFIRMED but before APPLIED arrives.
- Free peers with no location, and the full-roster rule: separate tasks.
- Message authentication: deferred, see
  [swarm-messaging-security-backlog.md](swarm-messaging-security-backlog.md).
- Mixed versions unsupported: a step 3 without meta gets one reject.

## Open decision

- Armenian labels (`hy.json`): the current idle label "ՍՊԱՍՈՒՄ" means
  "waiting". Proposal: give it to the new waiting status, rename idle to
  "ԱՆԳՈՐԾ", badge "ԶԲԱՂՎԱԾ" (busy) and "ԱՆՀԱՅՏ" (unknown, stale heartbeat).
  Applied as proposed in 6b, pending the owner's OK.

## Implementation plan

Rebase on `origin/main` first. Test command:
`PYTHONPATH="$PWD/src:$PWD" .venv/Scripts/python.exe -m pytest -q <targets>`.
Commits: 1, 2 separately; 3+4+4a–4c together (either side alone breaks the
handshake, and a helper that stops bidding before owners read its state would
stall them); 4d separately; GCS 6a–6c after 4c.

1. **Refs and timing.** Leaf `swarm/task_msg_refs.py` (`MsgRef`, `OwnerRef`,
   `msg_ref()`); `SWARM_RESEND_INTERVAL_MS`; `swarm/task_ack_timing.py` with
   derived policies. Tests: `test_task_ack_timing.py`, `test_task_msg_refs.py`.
2. **Sending and routing acks.** `TaskMessageSender` pre-stamps meta and
   returns each copy's UID; `ack(message, status)` with the acked message's
   TTL; import `swarm_ack_msg`; `SWARM_ACK` in the router's peer types;
   `TaskAckRouter` dispatch table by `ref_msg_type`. Tests:
   `test_task_messaging.py`, `test_task_ack_routing.py`; rewrite
   `test_swarm_ack_unregistered.py` → `test_swarm_ack_registration.py`
   (production now registers it on purpose); SWARM_ACK in the length test.
3. **Owner.** `AssignAttempt(number, sends, request_refs, floor)` replaces
   `TaskDispatch.assign_request_sends`; `TaskReservation.attempt`;
   `TaskAssignConfirmation` records request UIDs, `on_request_ack` (floor),
   `answer_response` (verdict table), attempt check in `_current`; reset and
   shutdown log dropped dispatches.
4. **Helper.** Rewrite `SelectedTaskSlot` to the table (replaces
   `try_accept`/`release_if_held`/`clear`, keeps class name and exports); new
   `swarm/task_assign_reply.py` `AssignReplies` (accept repeat bound to a slot
   token, reject repeats keyed by (owner, task), shared lock); participation
   acks step 3 and dispatches on the offer kind; `TaskActor` wires the route,
   `reset()`/`clear_selected_poi()` release only ASSIGNED, `shutdown()`
   cancels repeats. Tests: `test_selected_task_slot.py` (one case per row),
   `test_task_assign_reply.py`; existing tests that seed `try_accept` or expect
   a visible task at step 3 change their Arrange (WAITING is invisible by
   design; assertions keep their meaning).
   - 4a. **Node state, fuel and gates**: `SwarmNodeState` FREE/BUSY;
     heartbeat build/send split, presence emits it under the actor lock; slot
     `node_state()`, approach edge, reject cancel; bid gate (FREE only),
     -1 ETA sent as "can't do", step-3 gate (approaching or not flyable →
     reject); `TaskActor.set_approaching` and a `monotonic_s` port. Tests:
     state per (slot, approaching) pair in heartbeat and CHECK_IN reply, lock
     held while building, gates, decline sent, round-trip 0–1.
   - 4b. **Nav publisher and priority**: publish `approaching` (committed
     phase NAV) after each cycle; `publish_approaching()` in `nav_network.py`;
     wire in `nav_runtime_composition.py`. ASSIGNED preempts: DDH return
     replaced by `setup()`; CONFIRM drops the own POI (clear active POI,
     DROPPED) and returns to DETECT **without
     RESET** (RESET releases ASSIGNED). Tests: new
     `test_nav_task_availability.py` (approaching per phase); nav-controller:
     CONFIRM + ASSIGNED → own POI dropped, peer approach starts; DDH +
     ASSIGNED → peer approach; NAV → BUSY, step 3 rejected.
   - 4c. **Owner peer status**: roster status and `report()`; `busy_peers(store)`
     at all call sites; per-task declines (matrix complete on bid or decline);
     ordered handles and `withdraw_handles`; `observe_peer` flips; step-4
     BUSY evidence and `released_to`; flip handler (restart, plan). Tests: new
     `test_peer_roster_status.py`; busy-set, decline, matrix, abort and
     stale-bid cases in `test_task_actor_state.py`; actor-level busy flows
     in `test_task_actor.py`; bus fixtures send heartbeats explicitly (the real
     1 Hz thread would inject wall-clock state; assertions unchanged).
   - 4d. **Silence**: `PEER_SILENCE_EXPIRY_S`, checked at the owner's tick,
     revive and CHECK_OUT. Tests with a fake clock.
5. **Integration loss matrix** (real `NetworkMavlink` + `MessageFilter`, three
   `TaskActor`s, virtual timers): loss of each message and ack; busy cases
   (busy peer skipped, peer freed, own approach starts while WAITING, fuel
   decline, lost release advert,
   silent peer). Invariant after every step: at most one ASSIGNED helper and it
   equals the owner's CONFIRMED peer; no step 3 to a peer known busy.
6. **Nav level** (`test_nav_controller.py`): no approach while WAITING; starts
   after APPLIED; reset releases ASSIGNED, keeps WAITING.
   - 6a. **GCS backend**: listener and `vehicle_entry` changes. Tests:
     uid in payloads/logs, every copy forwarded, ack resolution, other acks
     ignored, real log lines parse with the audit regexes, heartbeat filtering,
     ordering and staleness with a fake clock.
   - 6b. **Frontend**: reducer, hooks, `missionStatus.js`, status row/card,
     markers, locales. Tests: new `test_task_assignment_state_js.py` on the real
     reducer (one case per action, copies never lower a rank, same task id from
     two owners, recycled id, lost APPLIED); copied-logic JS tests replaced by
     real-reducer tests; source-grep tests re-pointed with the same intent;
     `test_mission_status_js.py` expects `waiting` for step 3 (rule 3);
     distinct labels per locale.
   - 6c. **Audit and gate**: version switch and uid rule; peer gate. Legacy
     fixtures unchanged; new uid-format cases (copies and released rounds,
     no APPLIED, unknown ref, task APPLIED to two peers, mixed format, gate).
7. **Docs**: update this file; `swarm_ack_msg.py`, `types.py` and
   `swarm_heartbeat_msg.py` comments (SWARM_ACK used, heartbeat state codes).
8. **Verify**: `tests/modules/swarm tests/modules/comm tests/modules/nav
   tests/gcs tests/scripts`, then the full suite; guards; `cd
   src/gcs/frontend && npm ci && npm test && npm run build`; live 3-UAV run via
   the isolated launcher (`scripts/eval_gcs_demo_cli.py`, then
   `scripts/gcs_stop.py`): busy peer skipped, UI WAITING → ASSIGNED, owner logs
   "Task N accepted by M", helper "Task N assigned by owner O" before its
   approach, audit and gates pass in uid format.
