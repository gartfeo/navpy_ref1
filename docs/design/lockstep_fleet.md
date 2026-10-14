# Lockstep 3-UAV fleet (v4)

v4 answers Codex design review 3 (1 P2); v3 answered review 2 (3 P2); v2 answered review 1 (11 P2, 1 P3).

Step 10 of `deterministic_demo_eval.md`. It builds on `lockstep_slice.md` (v7) and `lockstep_contracts.md`, and reuses their mechanisms unless **Supersedes** says otherwise: `NVPART01`, the baton, `LockstepBus`, the participant stop, drain and parking, the journal and the operator ports. Some facts cannot be checked from this repo (the deployed router binary, the WSL script). For those, the design names the test that pins them and does not claim them.

## Goal
Three serial-step SITLs, three NavPy participants, the backend with three vehicles and the operator run the certified scenario (`scripts/eval_gcs_demo_scenario.json`) in lockstep. The acceptance is the main doc's step 10 acceptance plus NAVLINK forwarding evidence.
- Repeatability is reported separately from mission success. Step 10 is not accepted while (0) is unmet (todo 46).
- Wall mode is unchanged.

## Today (d11c5c4)
- **Coordinator:** one SITL link. The reader is built at `coordinator_run.py:88`; the devices are fixed to `SERIAL_DEVICES` (`coordinator_types.py:17`).
- **Hooks:** the participant hooks hard-code their devices: `navpy/app/lockstep/participant.py:40-47` (`lockstep:serial0`) and `gcs/backend/lockstep/participant.py:37-44` (`lockstep:serial1`).
- **No router.** Serial1 egress goes straight to the backend, and backend egress straight to serial1 ingress (`coordinator_tick.py:62-88`). `router_model.py` is only used by tests.
- **Boot:** all egress is discarded (`coordinator_run.py:157-162`).
- **Terminal tick:** k_T is the first SNAP from any companion (`coordinator_tick.py:94-97`).
- **WSL:** one SITL per run. The constants are at `lockstep_wsl_sitl.py:34-38`, instance and relay selection at `:66,76`, ownership at `:94-98`. `prepare_root` refuses an existing root (`lockstep_wsl.py:91`).
- **Slice wiring:** devices and ordinals at `lockstep_slice_actor.py:39-41`, context at `:65-71`, companion config at `:148-151`.
- **Firmware:** boot ids are already distinct per vehicle, since `boot[1] = sysid` (`NavPySerialStep.cpp:152-153`). The firmware needs no change.
- **Eval:** `--lockstep` takes only the slice (`eval_gcs_demo_lockstep.py:95-101`).
- **Analysis:** `owner_only_assignment_evidence` requires no peers (`eval_gcs_demo_analysis.py:116`). That is an owner-only evidence precondition, not a general lockstep refusal. No `(certified, lockstep)` run-gate entry exists (`:149-153,165-170`).

## Wall topology to match
These facts were read from the deployed `~/ardupilot/Tools/autotest/run_swarm.sh` at review time. That file is already hashed into the wall input manifest (`eval_gcs_demo_sitl_root.py:39`). The lockstep manifest records the same hash, and the topology facts below are pinned by a test against that hash: a changed script fails the test.

Launch: `run_swarm.sh pi` with `COMPANION_UDP=1` (`swarm_run_wsl.py:219-224,270-274`).
- **serial0 per SITL:** a UDP client to its own NavPy (`run_swarm.sh:224-227`); point to point.
- **serial1 per SITL:** a UDP client to a router feed (`:239`). The feeds are the router's sources (`:291`).
- **serial2:** a loopback sink (`swarm_run_wsl.py:236-241`); lockstep uses `none` (`lockstep_wsl.py:201`).
- **Router:** `mavlink-routerd -s 255` (`run_swarm.sh:274`). Its endpoints are the 3 feeds, the backend monitor port, the verify port and the loopback sinks (`swarm_run_runner.py:67-70`).
- **Backend:** one merged link; every connect uses one `monitor_device` (`eval_gcs_demo_process.py:172`, `eval_gcs_demo_lifecycle.py:22-25`).
- **Peer traffic:** `-nt mav` (`navpy_process_launch.py:129-130`) selects `NetworkMavlink` on the vehicle bus (`network_mavlink.py:62-71`). The path is NavPy i → serial0 → AP i → serial1 → router → AP j → serial0 → NavPy j. Image transfer is `ENCAPSULATED_DATA`, which has no target (`network_mavlink.py:119-129`).
- **Endpoints lockstep does not reproduce:**
  - **Mission Planner:** filtered out of the loopback-only eval's exports (`swarm_run_runner.py:68-69`); absent in both modes.
  - **Loopback sinks:** UDP discard targets that never send, so they never teach the router anything.
  - **Verifier (an intentional experiment difference).** In wall mode it sends `PARAM_REQUEST_READ SIM_SPEEDUP` and `COMMAND_LONG` asking for `SYSTEM_TIME` (`swarm_run_verifier.py:141-148,195-209`). That teaches the router identities and shifts AP sequence numbers. Lockstep verifies by handshake and frames instead (step 7). The difference is listed under Not in scope with the other wall-only host effects.

## Links and names
| Link | Device | Owner |
|---|---|---|
| SITL i serial0 ↔ NavPy i | `lockstep:serial0.<sysid>` | NavPy ordinal i |
| Router endpoint `backend` | `lockstep:router` | backend |
| SITL i serial1 | router endpoint `sitl<i>` | coordinator |

Devices come from the role config, never from constants: the hooks above, `RoleSpec.devices`, NavPy's `-c` and the operator's connect device. The slice uses the same scheme with N=1 (Q1).

## Fleet barrier
**Launch** (replaces `WslSitl`):
1. `create_root` once, then `prepare_instance` per SITL. `prepare_root` is split because its `test ! -e root` (`lockstep_wsl.py:91`) forbids a second call. Every instance is prepared before any SITL runs.
2. Spawn relays 1..3 (`lockstep_relay_port(slot, i-1)`), then SITLs 1..3 back to back. Each is recorded and admitted under the slot as today, with instance `3*slot + i-1`. Then read frame 1 from each.
3. **Deadlines: one shared exchange budget per frame.**
   - **The waits add up.** The firmware arms its reply deadline before sending each frame (`NavPySerialStep.cpp:222-225`). Within one reply wait the coordinator's waits are cumulative:
     - it reads the other SITLs' frames;
     - it sends the `TICK`s;
     - it receives and applies each `DONE` in order (`coordinator_tick.py:72`). Applying one can hold for a spawn's registration or a retirement's exit while later participants are still outstanding (`coordinator_members.py:151,180,203`);
     - the operator runs;
     - the replies go out.
   - **Rule.** Tick k has one absolute wall deadline, `D_k = a_k + STEP_DEADLINE_S − EXCHANGE_MARGIN_S`.
     - `a_k` is the moment reply k−1 was sent to SITL 1, the first reply of the previous tick. Every SITL arms its frame-k deadline only after it receives reply k−1, so each firmware deadline is ≥ `a_k + STEP_DEADLINE_S`. For k = 1, `a_k` is SITL 1's `go`.
     - `EXCHANGE_MARGIN_S` covers a reply's transit to the firmware: the relay's maximum injected delay (`lockstep_relay.py` `MAX_DELAY_S`) plus a named transit allowance. It is documented and tested.
     - Every sub-wait in the tick (frame reads, `TICK` sends, `DONE` receives, registration and exit holds, the operator baton's wall budget, reply sends) draws on `D_k`. Each per-operation budget becomes `min(own, D_k − now)`.
     - Running out is `HostFault("exchange_budget")`, journaled with the tick and the sub-wait. The run stops by ownership before the firmware deadline can turn the fault into a SITL death.
   - **Initial participants** (the backend) spawn and register **before** any SITL is launched; they are admitted only after the barrier anyway. So frame 1's budget covers only the remaining SITL launches. This also removes today's mismatch: `register_s = 120` (`eval_gcs_demo_lockstep.py:72`) inside a 30 s firmware wait (`lockstep_wsl_sitl.py:38`).
   - **Tests (10a, fake clock and link):**
     - three individually legal waits in one tick (a registration, a `DONE` and an exit, each under its own cap) whose sum exceeds the old max-rule bound: `exchange_budget` fires before the simulated firmware deadline;
     - a frame-1 launch that overruns;
     - a constant test that `EXCHANGE_MARGIN_S` exceeds the relay's maximum delay.
4. **Partial launch:** any relay or SITL failure stops every recorded process by ownership. Tests: SITL 2 fails to start; relay 3 fails; the coordinator dies between launches.

**Homes.** A pure function reproduces `home_coords` (`run_swarm.sh:253-271`: `DIST_M` from `-d`, `PER_ROW` 3) with the eval's `-d`. The launcher default is 50 (`swarm_run_cli.py:67`), passed at `swarm_run_runner.py:130`. The function's test fixture is tied to the script hash above. The homes go into the input manifest.

**Tick k, step 1.** Read frame k from SITLs 1, 2 and 3 in order.
- Per-SITL checks run first, in SITL order: tick echo, out-of-loop counter and grid (`coordinator_frames.py:52-62`; Δ_max = 20825 us, `lockstep_wsl_sitl.py:34`).
- Then the **fleet clock check:** every `source_us` must be equal, and t_k is that value. A mismatch is `CoordinatorFault("fleet_clock")`, journaled with the three values.
- The grid recording is re-recorded from a 3-SITL run.

**Boot barrier.**
- `helpers_quiet` is latched per SITL. A late latch only holds admission. The initial participant (the backend) is admitted at the tick after the last latch.
- A barrier that never completes is `CoordinatorFault("boot_barrier")` after the manifest's `boot_budget_us`. That value is named, set from the 10d recording of the latest latch tick plus a documented margin, and tested.

**Boot traffic.**
- The router runs from tick 1, so SITL-to-SITL serial1 traffic flows during boot, as in wall mode (Q2).
- Bytes for an owner not yet admitted (serial0, the backend endpoint) are discarded and counted, by the slice's Joining rule.

**Parking.**
1. After reply k_F to every SITL, read each SITL's parked frame k_F+1 in order, with all checks.
2. Journal and tally each parked frame; never route or reply to it.
3. Journal any pending queue bytes as `undelivered` (per port: count and sha256), then drop them.
4. Snapshot, then stop every SITL by ownership.

## Router and delivery
`ReferenceRouter` moves into the tick path (`coordinator_router.py`).
- One persistent decoder per inbound endpoint stream; original packet bytes are forwarded.
- Endpoint order: `sitl1, sitl2, sitl3, backend`.
- Learning, sniffer, broadcast and targeted rules are as in the model, except the parity rules pinned below.

**Router parity (pinned, not claimed).** Upstream mavlink-router treats unknown message ids (forwarded as having no known target) differently from unparseable bytes. It reads target fields from its own message-entry table, not from NavPy's dialect. Whether d59ef94 does the same is unverified. So:
- The model takes targets from a **router entry table**: the id set the deployed binary was built with, extracted once into a pinned fixture. A NAVLINK id outside that table is no-target, so it is broadcast.
- Unparseable bytes and unknown-target drops are counted separately (`router_skipped_bytes`, `router_dropped`).
- **Gate before 10b closes:** `test_router_parity` runs the real `/usr/bin/mavlink-routerd -s 255` in WSL, offline and never inside a run. It replays recorded streams: targeted, broadcast, sniffer, an unknown target, a targeted NAVLINK, boot text, a split packet and an AP echo. The model's destinations and bytes must equal the binary's. The binary's sha256 goes in the fixture.

**Order within tick k:**
1. Route SITL serial1 egress from frames k, in endpoint order, then byte order.
2. SITL i's serial0 egress goes into NavPy i's `TICK k`.
3. Send `TICK k` to NavPy 1..3, then the backend; collect `DONE k` in the same order.
4. Append NavPy i's egress to SITL i's serial0 queue. Route the backend's egress as source `backend`; its output joins the SITL serial1 queues after step 1's bytes.
5. Run the operator, send reply k to SITLs 1..3, then journal.

**Queues** (no drop, bounded by the firmware). The coordinator owns one FIFO per destination port (SITL i serial0, SITL i serial1). Routing appends to the tail.
- **Admission:** reply k takes `n = min(len(queue), 0xFFFF, FIFO_CAP − fifo_bytes(frame k))` from the head, so old bytes always go before new ones.
  - `0xFFFF` is `step_wire.py:33`, which equals the firmware `MAX_BLOCK` (`NavPySerialStep.cpp:39`).
  - `FIFO_CAP` = 2^20 (`SIM_NavPyStepPort.h:30`, `BUFFER_BYTES = 1U << 20`; the FIFO is `ByteBuffer{BUFFER_BYTES + 1}`, `SIM_NavPyStepPort.cpp:25`, so it holds exactly 2^20 bytes; pinned by `firmware_fifo_source.json`, step 10b).
  - `fifo_bytes(frame k)` comes from frame k's header. Nothing else enters the firmware FIFO before reply k, so the push can never hit `ingress_fifo_full` (`SIM_NavPyStepPort.cpp:44`). A contract test checks `FIFO_CAP` against the firmware source fixture.
- **`pack_reply`** stays a strict serializer.
- **Tally and journal:** counts derive from the packet records (Evidence). `to_sitl` counts bytes actually admitted to a reply; each tick record carries the pending bytes per port.
- **Tests:** sustained overload over many ticks; a packet split across replies; no overtaking (SITL-routed bytes against backend bytes for the same port); parking and drain with pending bytes.
- **Wall difference:** the wall router's UDP fan-in drops at `UARTDriver.cpp:950` (Not in scope). Lockstep never drops; a backlog shows as pending bytes and `fifo_bytes`.

**Timing, by stage:**
- **Eligibility:** a byte produced at tick k is eligible for reply k. Backlog in the queue defers it to a later reply, and the journal shows which.
- **Firmware admission and parse** (`lockstep_contracts.md:84-89`): reply k enters the FIFO in loop k. That loop's io tick admits `min(fifo, space, 10000)`, and `GCS_MAVLINK::update` parses within its budget in the same loop.
- **Observed response:** a `COMMAND_ACK` in frame k+2 for a `COMMAND_LONG` in reply k (step 7). This is a measurement, not a parse-tick rule.
- **NAVLINK latency:** per packet, from its **emission** (the sender's `DONE` tick, recorded before queuing) to its **delivery** in the receiver's `TICK`. The path is chained through the AP hops by UID (each AP re-emission is a new SITL emission). The contract is reproducibility for the same traffic trace, with no fixed hop count: source-side carry-over, backlog and AP parse budgets change it.

**Tally:** keyed by `(link, direction)`.
- Links: `serial0.<sysid>`, `sitl<i>.serial1` and `router.backend`.
- Directions: today's (`coordinator_mavlink.py:35-38`), plus `routed_out`, `router_dropped` and `router_skipped_bytes`.

## Participants
**Ordinals:** the operator is 0, NavPy i is sysid i, and the backend is N+1 (4 here; the slice keeps 2). `TICK` order is ordinal order (`coordinator_members.py:117-120`).

**Spawn stays sequential, as in wall mode.**
- The start route awaits each instance in turn (`routes/navpy_sim.py:122-131`). The launcher port publishes one spawn request and waits for its lifecycle result before returning (`lockstep/launcher_port.py:136-137`).
- So NavPy 1's request is in `DONE k` and it is admitted at k+1; its result reaches the backend in `TICK k+1`; NavPy 2's request follows in a later `DONE`.
- Admission ticks are staggered and deterministic, and journaled. Fixtures and gates expect this sequence; there is no batch launch.
- Readiness is one `ready` record per ordinal.

**Stop is sequential too.** `stop_all` iterates sysids in order and waits for each (`launcher_port.py:168-179`). Each NavPy retires through its ending `DONE`. `_check_companions_stopped` covers all of them (`coordinator_run.py:181-190`).

**Backend:**
- One `LockstepBus` on `lockstep:router`, with reservations for sysids 1-3.
- The launcher's `companions` map has 3 entries, keyed by sysid; the telemetry loop caches by sysid.
- **ESP32:** one shared in-process simulator, keyed by channel (`esp32_port.py:62-80`). The channel is `_channel_map.get(sys_id, sys_id)` (`launch_execution.py:52`), so sysids 1-3 map to channels 1-3. Tests trigger channels 1-3, including overlapping holds.

**Operator:**
- `StackContext.sys_ids = (1, 2, 3)`.
- `navigation_tail(sys_id)` returns that sysid's tail; today it ignores the argument (`lockstep_slice_actor.py:70`).
- One lines file per UAV, `lockstep_navpy_lines_uav<sys>.jsonl`; today there is one (`eval_gcs_demo_lockstep.py:139-144`).

**Terminal tick.**
- The expected fleet is {1, 2, 3}, taken from the scenario, not from the live set.
- The coordinator latches each expected companion's first `SNAP(VISION-NAV` as `(ordinal, generation, tick)`.
- **Success:** k_T is the maximum of the three first-SNAP ticks, with outcome `ok`. `timeout` and `operator_returned` stay unsuccessful.
- **Same tick:** if the last first-SNAP lands in the tick where the budget or the operator's return also hits, completion wins. This is today's check order (`coordinator_run.py:168-175`).
- An expected companion that was never spawned, or retired without a SNAP, can never satisfy the rule.

## Auction: timing and ids
Product semantics are preserved; nothing changes in `core/swarm`. Inputs arrive on ticks under the baton, so outputs repeat for the same trace. The historic wall assignment shows plausibility; it is not a prediction.

| Dependency | Anchor | Note |
|---|---|---|
| Roster: the first `MAX_REMOTE_PEERS = 2` heard by swarm heartbeat (every 1.0 s); expiry from the TTL | `task_peer_roster.py:16,19,69`; `swarm_heartbeat_runtime.py:12,128` | `snapshot()` returns a set (`:101-103`); the planner sorts peers (`task_assignment_planner.py:29`) |
| Selection timer, 2.0 s | `task_auction_coordinator.py:33,149,176-177` | It re-runs `_run_complete_assignment`, which still needs the complete matrix (`task_auction_planning.py:108-110`). There is no incomplete-matrix fallback |
| Offer rows | `task_auction_planning.py:165-181` | Dispatch insertion order = task registration (detection) order |
| Ack/resend (2000 ms), rebroadcast (0.5 s), dispatch and reply timers | `ttl_defaults.py:20`; `task_rebroadcast.py:18`; `task_dispatch.py:117,180,213`; `task_assign_reply.py:180` | Sim time; on a deadline tick, input runs before the timer |
| Message identity `(sender, boot_id, msg_seq)` | `task_msg_refs.py:26-56` | `boot_id` comes from `Random(seed, ordinal)` |
| TTL and dedup | `message_filter.py:155-167,212-224`; `message_admission_state.py:137-173` | Not raw epoch ages. A heartbeat receipt updates a clock-offset estimator; TTL is bypassed before the first heartbeat; later ages use adjusted timestamps. Deterministic, and preserved |
| Plan | `task_assignment_planner.py:62-63` (Hungarian, float32) | Float32 ties stay Not in scope |
| Bids (ETA) | `fly_estimator.py:56-68` | Uses ground speed and wind; this is not the final-approach command path |

**Tests under the baton (10f):**
- an incomplete matrix never selects, even after the 2 s timer;
- a final bid or ACK lands on its deadline tick;
- stale BUSY evidence (`follows`);
- a retry and a release;
- task registration order;
- offset-adjusted TTL before and after the first heartbeat.

**Expected assignment (Q3):** P1 → UAV 1 (the owner keeps it), P2 → UAV 2, P3 → UAV 3. Wall evidence: `.logs/step5-output/step5-d` backend log, lines 115 and 119.

## Peer camera and image transfer
- **Peer camera:** the examined paths use runtime or injected clocks and managed primitives. Keep pre-NAV geo pointing, the visual handoff and the NAV geo-session fence. 10f adds baton tests for each transition.
- **Image pacing:** the header waits 10 ms and each chunk 5 ms (`network_abc.py:79-91`). Both round up to ticks, so a transfer takes about one tick per chunk. This changes the transfer duration compared with wall mode; it is recorded, not gated.
- **Reconstruction on the merged link:** tests send three simultaneous confirmation images on `lockstep:router` and check reassembly per sender (sender isolation) and listener fencing at shutdown.

## Evidence the gates need (journal additions)
**Packet records.** Every MAVLink packet that crosses the coordinator gets a packet id and a journaled life. Counts and gates derive from these records.
- **Emission, recorded before any queue.**
  - A participant's egress is decoded per `(ordinal, generation, device)` with a persistent decoder when its `DONE k` is applied. A SITL's egress is decoded per `(sitl, port)` when frame k is read.
  - Each complete packet gets an emission record: id `(origin, generation, device, emission seq)`, the emitting tick, its byte range in the origin stream, sha256, msg id, source, target, the NavPy UID `(sender, boot_id, msg_seq)`, and `task_id` and ACK ref/status where present.
  - Backend emissions are included: e5 and the resends are backend NAVLINK emissions, which the approval gate reads.
- **Routing.** Each router copy is a new id with `parent` = the source id, recorded before queuing. `router_dropped` and `router_skipped_bytes` are dispositions of the source.
- **Byte progress.** Queues hold byte ranges tagged with packet ids, and each reply records the bytes admitted per packet. Delivery to a SITL is complete at the reply that admits the last byte, so a split packet shows each fragment's tick.
- **Delivery to a participant** is a separate record: `(id, receiver ordinal, generation, TICK k)`. It is written only when `TICK k` goes to an admitted, live member. Bytes for an unadmitted or retired receiver get `discarded{reason: unadmitted | retired | drain}`, never `delivered`.
- **Terminal disposition.** Every id ends in exactly one of `delivered`, `discarded{reason}`, `router_dropped`, or `undelivered{bytes_admitted, bytes_pending}` at parking (a pending fragment).
- **Correlation:** hops chain by `parent` (router) and UID (AP re-emission). The router hop keeps the sha256. Whether the AP hop is byte-identical is measured in 10f, not assumed.
- **Tests (10b):**
  - latency with source-side carry-over: emitted in `DONE 10` behind a backlog and completed in reply 15, it counts from 10;
  - a split packet;
  - a pending fragment at parking and its disposition;
  - bytes for an unadmitted and for a retired receiver are recorded `discarded`, and the gate rejects a fixture that claims their delivery.

**Operator approvals.** Extend `sent` (today method and target only, `operator_baton.py:82-84`) and `receipt` (delivery tick and event type, `:77-80`) with `(sys_id, local_task_id, round_uid, request_id)`. Global assignment ids stay distinct from peers' local confirmation ids (`eval_gcs_demo_poi_binding.py:118-155`).

**Operator polling schedule.** Each pass of the confirmation loop journals:
- the receive's start and return ticks, and whether an event or the timeout ended it (`eval_gcs_demo_confirmation.py:146`);
- the due-check tick, its `now` and the due list (`:173-179`);
- each POST's send and answer ticks (`:183-213`);
- the sleep's start and wake ticks (`:216`).

**TICK/DONE payloads** stay hashed (`coordinator_journal.py:68-69`). Gates read only the explicit records above.

## Execution profile and gates
**Resolved execution profile.** `execution_profile(gate_profile, mode)` gives `certified_lockstep` for `(certified, lockstep)` and leaves every other pair unchanged. Every profile-keyed table uses it:
- `PROFILE_SIM_SPEEDUP` (1.0);
- the override refusal (no `-gsu`, `eval_gcs_demo_analysis.py:196-197`);
- `_ASSIGNMENT_EVIDENCE` (profile-only today, `:173-177,222`);
- `_RUN_GATES`, role gates and vehicle gates.

Wall `certified` resolves to itself and is unchanged.

**Retained, on sim time:**
- R1, R2 (the backend log is complete under lockstep; cross-checked with the journal), R3.
- V1, V2, V3 (owner: task 2/3 dispatch, `eval_gcs_demo_role_gates.py:73-78`, and the orbit; peer: no POI, GUIDED_LOITER, approach order, `:54-64,83-88`).
- V4, V5, V7, and V8 (bounds, roll quality).
- **W1 on sim time:** observation count, median and max gap, and the speedup ratio (`eval_gcs_demo_final_approach_gates.py:17-41`). Column 0 of the debug CSV is `rt.now()`, which under lockstep is epoch + sim time (`navigation_clocks.py:13`, `lockstep_runtime.py:61-63`).
- **W2 is replaced** by the approval gate below.

**Docking dependence, narrowed.**
- Today the episode parser rejects SNAP without DOCKED (`eval_gcs_demo_evidence.py:161-166`), and both vehicle scoring and R3 call it. So one miss hides every other check.
- **Split:** the parser returns episodes with a `docked` flag, and "DOCKED before SNAP" becomes its own gate. Wall `certified` keeps the same pass/fail set.
- V9's finite and cross-file SNAP integrity (`eval_gcs_demo_vehicle.py:44-87`) runs independently.
- **Expected failures:** exactly two named checks per UAV, DOCKED and the coordinate-pass reset, as a strict xfail on todo 46. (0) stays unmet.

**Journal gates:**

| Gate | Pass |
|---|---|
| Run | Outcome `ok`. Zero faults, including zero `fleet_clock`, `boot_barrier` and `exchange_budget`; zero out-of-loop advances. Every packet id has exactly one terminal disposition. No NAVLINK packet is `undelivered` |
| Assignment and forwarding | For P2 and P3, each correlated by UID and `task_id`:<br>- `TASK_ASSIGN_REQUEST`, owner → helper: `serial0.1` → `sitl1.serial1` → `routed_out` to `sitl<h>` → `serial0.<h>`;<br>- the helper's `SWARM_ACK` of the request and its `TASK_ASSIGN_RESPONSE`, helper → owner;<br>- the owner's APPLIED `SWARM_ACK`, owner → helper, on the request path.<br>The router hop keeps the sha256. Adverts reach every peer. A source outside {1,2,3,255} fails. Zero `router_dropped` for NAVLINK |
| Approval timing | **Replay of the operator's polling schedule** (`CONFIRMATION_POLL_S = 0.05`, `eval_gcs_demo_confirmation.py:23`). Every gap must be explained; an unexplained gap fails.<br>**Receive (FIFO replay):** the port queues deliveries and a receive consumes the oldest queued event (`operator_baton.py:159`, `operator_ports.py:118`). Delivery and consumption are journaled separately. A receive that starts with a queued event returns at its start tick. With an empty queue it returns at the next delivery tick, or at the first tick with t ≥ start + w, where w = min(0.05, deadline − now) (`:146-147`); a delivery on the timeout tick wins. k3 is the consumption tick, and the 15 s delay counts from it (`:167`).<br>**Due check:** at the receive's return tick (`:173-179`). An approval goes out in the first pass whose due check has t − t(k3) ≥ 15.00 s; the previous pass's check had < 15.00 s.<br>**Sends in a pass:** the first POST at the due-check tick; each later POST at the previous one's answer tick, in pending order. Each answer tick is the tick of the backend `DONE` carrying that response. The backend's three emissions (packet records) come at k4+1, then at resend intervals in [0.25, 0.25 + Δ_max] (`routes/task_confirm.py:91-94`).<br>**Sleep:** starts at the last answer tick (or the due-check tick if nothing was sent) and wakes at the first tick with t ≥ start + w (`:216`). The next receive starts at that tick.<br>So a request that becomes due during a blocking POST in pass n is sent in pass n+1, after explained waits only.<br>**Also kept:** k1 ≤ k2 ≤ k3, e6 completion, and the audited elapsed time equals the journal's.<br>**Fixtures:** an unexplained late pass (a wake past its bound) fails; an unexplained late send (a POST after its predecessor's answer tick) fails; the cross-pass blocking case passes; an event queued during a sleep and one queued during another approval's blocking POST, each consumed by the next receive, pass; an unexplained delayed consumption (a queued event not returned by the next receive) fails |
| Workflow ticks | Per vehicle, the role's marker order in `(tick, stream position)` |
| Pose cadence | `navpy` and `gcs` files for sysids 1-3, each flushed whole |
| SNAP | Each expected UAV's first SNAP ≤ k_T, and k_T equals their maximum |

**Slice-only checks not applied:**
- `SLICE_FORBIDDEN_MESSAGES` and "no peer source" (`eval_gcs_demo_slice_gates.py:46-59,147-155`);
- exactly one approval;
- owner-only pose files (`:200-202`);
- `owner_only_assignment_evidence`;
- `plan.pois[0]` (`:226`).

## Remaining packages to migrate (each owned by a sub-step)
1. **Coordinator (10a-10b):** N links, readers, fleet clock, boot latch and budget, parking, devices as data, router, queues, tally, first-SNAP latch, packet and operator evidence.
2. **Participant hooks (10c):** devices come from the role config (`navpy/app/lockstep/participant.py:40-47`, `gcs/backend/lockstep/participant.py:37-44`).
3. **WSL (10d):** `create_root`/`prepare_instance`, `WslFleet`, homes, the deadline rule, partial-launch cleanup.
4. **Eval wiring (10e-10g):** a fleet actor module with per-UAV role and config mapping. It replaces `lockstep_slice_actor.py:39-41,65-71,148-151` and `eval_gcs_demo_lockstep.py:95-144`.
5. **Input manifest (10d):** topology inputs: the script hash, homes, the router entry table and binary hash, `FIFO_CAP`.
6. **Analysis (10g):** the execution profile, the gate split, journal schemas, the confirmation-timing replay, and profile shapes (`eval_gcs_demo_gate_profiles.py:9-47`).
7. **Swarm paths new under the baton (10f):**
   - the `NetworkMavlink` receive path (`network_mavlink.py:77-104`);
   - the heartbeat thread (`swarm_heartbeat_runtime.py:52`);
   - `PeerPoiDispatch` (`peer_poi_dispatch.py:156,207-222`);
   - the auction timers;
   - `task_assign_listener` with more than one vehicle (`task_assign_listener.py:82`), and its fence entry.
8. **Transports (10f):** `NetworkWifi` and `NetworkSerial` stay wall-only. Under lockstep the NavPy bootstrap refuses any `-nt` except `mav` or `none`. The reviewed import inventory (`tests/guards/runtime_time_lint.py:86-101`) does not prove that no socket is reachable, so the refusal gets bootstrap tests.
9. **Fleet failure cleanup (10d, 10h):** a fault at any tick stops all 3 SITLs, all 3 relays and every participant by ownership. Tests: a fault before the barrier, mid-mission, and in drain.

## Acceptance (certified 3-UAV scenario)
Run `eval_gcs_navigation_demo.py --lockstep --seed 1 --run-dir <dir>`.
- **Setup:** two runs end at the same k_T and k_F, with zero faults and zero out-of-loop advances.
- **(a), (b):** the same digest; byte-equal artifacts (the slice inventory per UAV, plus the per-UAV lines files).
- **(c):** the digest stays equal:
  - under seeded delays in all 3 relays, the coordinator and every participant;
  - with helper jitter on every SITL, under CPU load;
  - on another slot, run concurrently.
- **Negative controls:**
  - Seed 2 changes the digest.
  - One moved waypoint changes it, from the upload tick on.
  - An endpoint-order swap changes it (unit test on recorded frames).
  - The slice's controls still hold.
  - **Fleet clock:** SITL 3 runs with `SCHED_LOOP_RATE 100`. Its frame gap of about 10 ms passes its own grid check (≤ Δ_max), so the expected fault is `fleet_clock` at the first tick where the `source_us` values differ (per-SITL checks run first). A coordinator unit test on recorded frames, with one SITL offset inside the grid, pins the same fault.
- **NAVLINK forwarding:** the assignment gate's per-hop, UID-correlated records, and the per-packet latency of each `TASK_ASSIGN_REQUEST`, identical across runs.
- **(0) Mission success:** the expected assignment, three replayed approvals, and SNAP for every expected UAV. **Docking success depends on todo 46.** Until it lands, (0) is reported unmet and step 10 is not accepted.
- **Wall mode:** pytest is green and the wall certified eval is unchanged.

## Sub-steps (each a commit with its tests)
| # | What | Proves |
|---|---|---|
| 10a | Coordinator for N SITLs: readers, fleet clock, shared exchange deadline, boot latch and budget, per-SITL parking, devices as data, backend = N+1, the first-SNAP latch, operator evidence fields | A scripted 3-link toy on recorded frames is deterministic under delays. `fleet_clock` fires; a late latch holds admission; `boot_barrier` fires; `exchange_budget` fires on cumulative legal waits |
| 10b | Router with its entry table, queues, packet records (emission, routing, byte progress, delivery, disposition); `test_router_parity` against the real binary | The contract and parity cases; overload, a split packet, no overtaking, pending bytes at parking; latency with carry-over; no delivery to an unadmitted or retired receiver |
| 10c | Hooks take their devices from role config; the slice runs on the router path with N=1 | Slice regression and repeatability on the new path; then its digest is re-baselined (Q1). Built 2026-10-14: the router path is the only path. The direct path and its toy pin are removed, and the one-SITL journal has the fleet schema. Digest `545f05d1…` (`deterministic_demo_eval.md`, Today) |
| 10d | `create_root`/`prepare_instance`, `WslFleet`, homes, the deadline rule, partial-launch and fault cleanup, the topology manifest | 3 SITLs boot under the barrier with equal `source_us`, using step 7's scripted operator (AUTO ×3); journals are identical under delays and across slots. Built 2026-10-14: `WslFleet` (`lockstep_wsl_sitl.py`) is the coordinator's launch hook, after the initial participants registered; SITL 1's `go` anchors frame 1's deadline (`spawn(on_go=...)`) and every wait inside SITLs 2..N's spawns (pid line, identity probe) draws on it (`spawn(wait_budget=...)`); a child joins the owner's held list before any of its threads starts, and a failed or interrupted spawn only closes its stdin and raises at once; `WslFleet.stop` stops by ownership first, then reaps held launchers with bounded waits and keeps one that does not finish for a retry; every lockstep run's roster takes `exchange_deadline()`, and the scripted operator's baton, startup included, runs on it through `ExchangeDeadline.bounded`, the coordinator's operator path. Homes and the topology section: `lockstep_topology.py`. `BOOT_BUDGET_US` = 0 (latest recorded latch: frame 1 in every run) + `BOOT_BUDGET_MARGIN_US` 5 s (`fleet_boot_latches.json`), in every serial-step manifest, the slice's included. Acceptance: `scripts/eval_lockstep_fleet_auto.py` (`ScriptedFleetRun`, step 7's operator per SITL on its own serial1, no router yet), digest `c7230d95…` on slots 40, 41 and 42 (42 with host delays and helper jitter); seed 2 differs from tick 1. The slice's grid now carries the boot budget, so its digest is re-baselined to `b09e79f8…` (`deterministic_demo_eval.md`, Today). This acceptance does not exercise the router from tick 1 with 3 SITLs (each operator is on its own serial1); that is 10e's |
| 10e | Backend with 3 vehicles on `lockstep:router`; ESP32 channels 1-3 | The scripted actor for 3 sysids repeats byte for byte under two slots. It is the first run with the router on 3 SITLs from tick 1 (10d's AUTO ×3 run has no router). Built 2026-10-15: the fleet actor module `scripts/lockstep_fleet_actor.py` (`FleetLayout`: sysid i ↔ SITL i, NavPy ordinal i on `lockstep:serial0.<i>`, backend N+1 on `lockstep:router`; `stack_context` with a per-sysid NavPy tail; `backend_role`; `setup_script` and `fleet_script` for every sysid); the slice is `FleetLayout(1)` with unchanged role configs (digest still `b09e79f8…`). Acceptance: `scripts/eval_lockstep_fleet_backend.py` on the frozen fixture `scripts/eval_lockstep_fleet_scenario.json` (the certified scenario at 2495cf6, its canonical sha256 pinned with that revision in `eval_lockstep_fleet_scenario.lock.json`, enforced and recorded in the run's inputs): connect 1-3, named bounds, AAS, 3 mission uploads with readback, the in-process ESP32 simulator; outcome `operator_returned` (no NavPy). A run passes only with outcome `operator_returned` and no fault; acceptance needs two or more runs with equal artifacts (a single run makes no repeat claim). Digest `ce778e74…` on slots 40 and 41, and again on 43 and 44 after Codex round 1 (the second of each pair with host delay seed 7); journal, actor result, backend lines and `gcs/` files byte-equal; the packet records show each targeted packet copied only to its target's serial1 and peer packets admitted by reply 1; seed 2 differs from tick 1. ESP32: sysids 1-3 resolve to channels 1-3, overlapping holds tested in process. Left: per-UAV companion roles and the expected fleet (10f), `eval_gcs_demo_lockstep.py`'s certified wiring and per-UAV lines files (10g/10h) |
| 10f | 3 NavPy participants (sequential spawn and stop), swarm paths, auction tests, peer camera transitions, image reconstruction, the `-nt` refusal | A NAVLINK packet traced hop by hop with reproducible latency; the journal repeats |
| 10g | Execution profile, gate split, journal gates, polling-schedule replay, fleet actor | Each gate passes and fails on fixtures; wall certified is unchanged |
| 10h | `--lockstep` for the certified scenario; acceptance runs; cost | The acceptance above |

## Supersedes
- **Main doc, Tick order row:** "at k+1" becomes the staged timing above. Serial0 is direct, backend egress is routed, and delivery goes through coordinator queues.
- **`lockstep_slice.md`:** the device names change (Q1). k_T becomes the latched first-SNAP maximum over the expected fleet. Boot routes SITL serial1 (Q2).
- **`lockstep_contracts.md`:**
  - **Router:** targets come from the router entry table; skipped bytes are counted apart from unknown-target drops; parity with the binary is pinned by test.
  - **UART:** adds the coordinator queue and the `FIFO_CAP` admission rule.
- **Slice constants:** per-operation wall budgets draw on one exchange deadline per frame. Initial participants register before any SITL launches.
- **Not in scope** gains the wall verifier's traffic.

## Risks
- **Cost:** about 3x the slice's work per tick. The slice runs 4.7-5.2 ms per tick, 14.5 ms with delays (`lockstep_contracts.md`, Execution model). Estimate: 1.3-1.5x real time, below 1x with delays. 10h measures it.
- **Router parity:** if the binary disagrees with the model on NAVLINK targets, the model follows the binary. The wall-parity claim rests on `test_router_parity`.
- **Docking:** all three approaches share todo 46. Assignments happen before NAV and stay gated.
- **Backlog divergence:** wall drops serial1 bytes and lockstep queues them. Scores may differ; that is not a regression.
- **Swarm code new to the baton** (item 7): a wall wait or an unmanaged thread surfaces only at 10f.

## Owner questions (default in bold)
1. Rename the devices to `lockstep:serial0.<sysid>` and `lockstep:router`, and re-baseline the slice digest? **Yes, after the one-SITL router path passes slice regression and repeatability (10c).**
2. Route SITL serial1 traffic during boot? **Yes, as the wall router does. The omitted verifier traffic is a stated experiment difference.**
3. Pin the expected assignment {UAV1: P1, UAV2: P2, UAV3: P3}? **Yes, as an acceptance requirement. A different deterministic assignment needs investigation and owner review, never an automatic re-baseline.**
4. Routed bytes beyond one reply? **Carry them in coordinator queues, admitted within the wire cap and the firmware FIFO capacity. Journal pending and undelivered bytes; never drop silently.**
5. Mission terminal tick? **The maximum over every expected UAV's latched first SNAP. Operator return and budget expiry stay unsuccessful.**
