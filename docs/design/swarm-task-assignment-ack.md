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
  or its no-progress drop (5 s).
- GCS: a step-4 copy and the APPLIED naming it arrive on different vehicle
  links (threads), so an APPLIED heard first waits, one per helper, for its
  copy. `swarm` is a telemetry change key, so a new beat is broadcast.
- Frontend: a FREE beat retires a round only once the helper's "doing" (or
  the APPLIED's ref) orders it; a round without one ends by advert, reject
  or APPLIED. Available tasks are keyed `${owner}:${task}` like rounds. A
  helper's confirm carries its own POI id, not the owner's task id, so
  confirm events are matched by UAV, to its helper slot: a request raises
  the slot to confirming, and only then does an approve raise it to
  confirmed or a deny drop it. A WAITING round is never raised, and an
  own-POI round asked before the assignment does not answer for the slot
  (fixed 2026-10-25; before, the card never showed CONFIRMED).
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
| 3 | `TASK_ASSIGN_REQUEST` | owner: a copy every 0.5 s until CONFIRMED, fresh UID each, recorded | helper: RECEIVED for every copy | CONFIRMED or rejected; release after 5 s without progress |
| 4 | `TASK_ASSIGN_RESPONSE` | helper while WAITING: a copy every 0.5 s, fresh UID each, each after a re-ack of the held step 3 | owner: verdict table | APPLIED → ASSIGNED; RECEIVED → drop; newer advert; drop after 5 s without owner progress |
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
| WAITING | 5 s without owner progress (PROCESSING of its copies) | EMPTY + ERROR log |
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
  reserved/reset (otherwise a lost release advert keeps it WAITING up to 5 s).
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
| resend interval | 0.5 s | `SWARM_RESEND_INTERVAL_MS`; the advert cadence (todo 114) |
| ack TTL | TTL of the acked message (5 s) | a shorter ack could be dropped while its message is admitted |
| no-progress silence (owner release, helper drop) | 5 s | TTL(`SWARM_HEARTBEAT`), as peer silence (todo 114) |
| sent-UID retention | 10 s | TTL of the copy + its ack TTL |
| busy visible | ≤ 1 s | heartbeat interval |
| peer silence | 5 s | TTL(`SWARM_HEARTBEAT`) |

Timers bound liveness only; no safety property depends on them. Tests pin
the relationships and recompute under a monkeypatched `TTL_DEFAULTS`.

**Clocks (2026-10-16, owner choice).** A message's age is the receiver's clock
minus the sender's `time_ms` (`message_filter.py`), from the first message on.
Peers must therefore run synchronised clocks: one host clock in wall SITL, the
run epoch plus the shared tick under lockstep, GPS or chrony time sync on real
companions. A stamp more than `CLOCK_SYNC_TOLERANCE_MS` (500 ms) ahead marks an
unsynchronised peer: it is dropped and warned about. A peer clock behind makes
its messages look old, so TTL drops them. The earlier per-peer offset, averaged
from heartbeats, took queue delay for clock skew and admitted stale bids
(`.logs/base-wall-1`).

## Loss and busy cases

| Case | Outcome |
|---|---|
| advert or bid lost | re-advertised 0.5 s later |
| step-3 copies lost / all lost | next copy 0.5 s later / release at 5 s, nobody waiting |
| step-3 ack lost | step 4 answered PROCESSING; next step-3 copy acked, next step 4 passes |
| every step 4 lost (original defect) | helper WAITING, not flying; release at 5 s; helper drops at 5 s |
| one / every APPLIED lost | next copy re-acked / helper drop at 5 s with ERROR: stranded, not duplicated |
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

## Lossy link (todo 114, 2026-10-24, owner approved both fixes)

**Cause.** In the `stress` sim about half of all swarm messages are lost.
A NAVLINK message crosses two lossy writes: ArduPilot i's radio port
(serial1) and ArduPilot j's port to NavPy j (serial0). `SIM_UART_LOSS`
drops the last byte of a write with probability loss % x write size,
which breaks that message; at seed 1 (0.303 %) about 18 % are lost on the
radio and 28 % on the serial0 wire. The handover then needs a chain to
survive: a step-3 copy and its ack (about 1 in 5), then a step 4 and its
APPLIED (about 1 in 4). The code sends 3 step-3 copies and 5 step-4
copies, 2 s apart, and gives up, so about 70 % of rounds fail. Seeds 8
(0.42 %) and 4 (0.56 %) fail at the swarm assignment too
(`.logs/swarm-112-seeds`). Two causes, two fixes.

### Fix 2: repeat until answered (protocol)

Decision 2 says the sender repeats until acknowledged; the code stopped
after a fixed count. Now each side repeats until it has its answer, and
gives up only when the other side stops answering this handshake.

- **Interval:** `SWARM_RESEND_INTERVAL_MS` = 500 ms (the advert cadence;
  about twice the median `stress` round trip, 0.25 s).
- **Owner:** while CONFIRMING, a step-3 copy every interval until
  CONFIRMED or rejected; it keeps sending after the floor, so every
  delivered copy brings an ack and a step 4 (more chances, no new state).
- **Helper:** while WAITING, a step-4 copy every interval; each scheduled
  copy is preceded by a fresh `SWARM_ACK` (RECEIVED) of the held newest
  step-3 copy (the first copy already follows the step-3 ack). It stops on
  APPLIED, RECEIVED, a newer advert or its own approach, as today. Reject
  repeats: every interval until RECEIVED or APPLIED.
- **Giving up (no progress):** `HANDSHAKE_SILENCE_S` = the heartbeat TTL
  (5 s, as `PEER_SILENCE_EXPIRY_S`); heartbeats play no part. Progress is
  the answer each side waits for, from the peer boot it is talking to:
  - Owner: the floor (the first ack naming one of its requests), then a
    step 4 from the floor's boot. It releases a CONFIRMING reservation 5 s
    after the first copy without a floor, or 5 s after the floor or the
    last such step 4 (a fenced one gives a verdict at once). Acks after the
    floor do not count: a helper that acks but never answers still ends.
    Release does what the 9 s release did (drop the bid, count a retry,
    re-advertise).
  - Helper: an ack naming one of its step-4 copies from the held request's
    owner boot (PROCESSING, APPLIED). It drops WAITING (ERROR log kept) 5 s
    after entering it or after the last such ack. A reject repeat ends the
    same way, or on RECEIVED / APPLIED.
  This covers a dead peer, a restarted one (its new boot never counts for
  the old round) and one that receives but does not answer. A peer whose
  handlers fail in other ways is out of scope. False alarm at the `stress`
  top: about 10 exchanges in a row must all fail, under 1e-4 with fix 1's
  single lossy hop. Replaces the copy counts, the 9 s release and the 18 s
  expiry.
- **Fence:** unchanged. The floor is the first step-3 ack of the attempt,
  boots are not ordered, and an ack or step 4 from any other boot never
  passes. A helper that restarts mid-round stops answering it, so the
  owner releases on no progress and the next round fences on the new boot.
- **Bounded memory (retention policy):** the owner keeps the request UIDs
  sent in the last copy TTL + ack TTL (10 s, at most 20); the helper keeps
  its step-4 and reject UIDs the same way (TTL(RSP) + ack TTL). An ordinary
  ack arrives well inside that; a late re-ack naming a pruned copy neither
  sets the floor nor counts as progress. The floor is kept apart. Reject
  repeat keys end with their repeat.
- **Residual strand (unchanged, stated):** the owner CONFIRMED, every
  APPLIED then lost until the helper's no-progress drop, or the helper
  restarts after CONFIRMED: the task stays with that helper, unflown. Never
  duplicated; recovering it needs a reconciliation rule (todo 115).

### Fix 1: loss on the radio only (sim)

The `stress` and `hw` profiles put the radio's byte loss on the serial0
wire between ArduPilot and NavPy too; on an aircraft that is a short
cable. ArduPilot sets `SIM_UART_LOSS` for every port at once, so the
lockstep coordinator applies the loss where bytes enter the radio:

- `uart_loss` no longer sets `SIM_UART_LOSS`; its draw (`loss_pct`, same
  ranges) is a per-byte loss rate b = loss_pct / 100, passed as the
  coordinator's `radio_loss_pct` argument (built: `Manifest` is at the
  field limit; the input manifest's `run_options` records it) and recorded
  in the journal's grid with the model name `radio_byte_loss`.
- Every byte a radio transmits is lost with probability b before the
  router frames it: each SITL's serial1 egress and the ground station's
  egress into the router. A frame with a lost byte fails the router's
  CRC check, as a frame broken by `SIM_UART_LOSS` does today, so todo
  102's boundary (broken frames before route learning) stays exercised.
  One transmission reaches every receiver alike (losses are not drawn per
  receiver).
- Draws: per radio, `model_stream(seed, "uart_loss", "radio.<endpoint>")`
  (`sitl1`, ..., `backend`),
  as geometric gaps between lost bytes, so the result depends only on the
  seed and the bytes; no draws at zero loss.
- Router input: the backend endpoint gets the SITL endpoints' input
  filter (`allow_msg_id_in`: only ids whose CRC the router checks; every
  dialect id is in the table), so a broken uplink frame never teaches the
  router a source (todo 102 at the new lossy ingress). The SITL output
  filters stay SITL-only.
- Journal: a broken frame shows as skipped bytes, as today. The
  coordinator does the deletion, so it keeps, per device, each surviving
  byte's clean-stream offset, and also frames the clean stream (an
  observer framer, same table). Matching is by position, not content: a
  lossy emission is genuine when its bytes map to one contiguous clean
  range that is exactly a clean packet's range; any other emission (an
  embedded or stitched frame) is false. A clean packet with no genuine
  emission is broken, directly or by the parser's resync. These go in a
  separate `radio` list of the journal, not the packet list:
  `["broken", device, tick, [start, end], sha256, msgid, src, target,
  fields]` (clean offsets, the tick the packet was written) and
  `["false", device, packet id]`; `device` is the physical stream (a
  SITL instance's serial1 or the backend). Clean packets still open at a
  tick's end wait for later ticks. At parking or close every one still
  open is broken: it is complete in the clean stream, so only the loss
  kept it from the router (also one held behind a frame the loss started).
  The packet list and its exactly-one-terminal check are unchanged.
- This is a different model from ArduPilot's (independent byte loss
  instead of the last byte of each write), so old and new results at the
  same seed are not comparable.
- Effects: NAVLINK crosses one lossy hop instead of two; ground-station
  uplinks (PARAM_SET, mission count/items, requests) become lossy; NavPy's
  own transfers on serial0 are clean. Todos 103 and 105 already retry lost
  requests the same as lost replies; their tests get uplink-loss cases.
- Gates (`eval_gcs_demo_fleet_packets`, `eval_gcs_demo_fleet_gates`,
  `eval_gcs_demo_cli`, `eval_gcs_demo_air_load`) use that evidence, not a
  loss flag: `navlink_trace` gives a packet the radio broke the terminal
  hop `radio_broken` only from a `broken` record on the sender's own
  serial1 device whose bytes and tick match its emission; loss on another
  stream never excuses it. An advert may end only there, and every other
  stop fails, including a missing re-emission at the receiver. The source
  check exempts only an emission recorded `false`. Air load counts
  emissions after radio loss, labelled as such.

### Checks

- Fix 2: owner resends until CONFIRMED, after the floor too; releases
  only after 5 s without progress; helper re-acks before each scheduled
  copy and drops WAITING after 5 s without owner progress; an owner whose
  step-3 acks were all lost confirms from a re-ack; a step 4 or ack from
  another boot never passes; request and reply UIDs pruned after 10 s.
  Mutations for each.
- Fix 2 also: a restarted helper answering the old round never refreshes
  its progress, so the owner releases and the next round confirms on the
  new boot; a late re-ack after pruning neither sets the floor nor counts.
- Fix 2 also: a helper that acks every request but never answers is
  released 5 s after the floor; a helper whose step 4s get no ack drops
  WAITING after 5 s.
- Fix 1: byte loss rate near b over many bytes; the same stream split
  across different chunks and ticks loses the same bytes; a lost byte
  breaks its frame and the router neither routes nor learns from it, from
  a SITL or the backend; serial0 is not lossy; zero loss draws nothing; no
  `SIM_UART_LOSS` set. Matching: a byte dropped in frame A's CRC or
  header with an untouched frame B after it (B lost by resync, recorded
  `broken`), recovery at C, a valid frame F embedded in A's payload with
  A's start marker dropped (the embedded F is `false`, the real B and F
  genuine), byte-identical repeats, and the same traffic on two streams
  with loss on only one, also split across chunks and ticks; an advert trace
  ending without `radio_broken` fails, and so does a foreign source not
  recorded `false`. Todo 102's malformed-input and filter tests
  stay.
- Interleavings: reject-repeat end on no progress and key cleanup; UID
  pruning under scheduled and immediate replies; false silence before
  CONFIRMED (safe retry) and after (documented strand, never reassigned);
  a recycled task id with delayed accepted and rejecting step 4s.
- `none`, `hw`, `stress` matrices; stress seeds 8 and 4 as evidence (not
  gates); full pytest.

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

1. **Refs and timing.** Leaf `core/swarm/task_msg_refs.py` (`MsgRef`, `OwnerRef`,
   `msg_ref()`); `SWARM_RESEND_INTERVAL_MS`; `core/swarm/task_ack_timing.py` with
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
   `core/swarm/task_assign_reply.py` `AssignReplies` (accept repeat bound to a slot
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
