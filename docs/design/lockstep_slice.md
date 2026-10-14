# Lockstep 1-UAV slice (v7)

Step 9 of `deterministic_demo_eval.md`. It builds on:
- the contracts in `lockstep_contracts.md` (baton, UART, wire, bus, ASGI, ownership), with the changes listed under **Supersedes**;
- the firmware serial-step mode (step 7);
- the RuntimeTime port (step 8).

Codex approved v7 after seven design review rounds.

## Goal
One SITL, one NavPy, the GCS backend and the operator run in lockstep from boot through confirmation and SNAP. The acceptance is the main doc's, applied to the 1-UAV fixture:
- two same-seed runs give the same journal digest and byte-equal artifacts;
- the digest is robust to host delays;
- a different seed changes it, and the negative controls fire.

Wall mode and the certified 3-UAV scenario are unchanged.

## Fixture, scenario and gate profile
`scripts/eval_lockstep_slice_scenario.json` is a schema-2 scenario with one slot, the owner:
- **Mission:** the certified owner track and the DDH.
- **POI:** one, at owner track ordinal 2.
- **Gate profile:** the scenario names it, `owner_only_slice`.
- **Why the auction never runs:** with one POI, the UAV's only detection is its own (`PoiSelector`, `core/skills/search/poi_selection.py`). The auction path needs a full peer roster (`MAX_REMOTE_PEERS = 2`).

**9a makes the scenario an explicit value** (`DemoScenario`, loaded once from `--scenario`; the default is the certified file):
- Every consumer takes it: configuration, upload, input hashing, plan persistence, command bounds, confirmation, gates and analysis.
- This replaces the hard-coded assumptions: three slots and ordinals `(2, 3, 6)` (`eval_gcs_demo_scenario.py:103,125`); three vehicles and POIs (`:255`); manifest reloads (`eval_gcs_demo_config.py:146`, `eval_gcs_demo_scenario.py:221`); two peers in assignment analysis (`:76`).
- `ThreeUavIds` becomes a count-checked id tuple.

**Gate profiles:**
- **`certified`:** today's gates, unchanged.
- **`owner_only_slice`:** every `certified` check for the owner role, minus only the checks that need peers, plus the slice checks below. Retained as today:
  - sensor configuration: the resolved vision profile, detector rate and mount, and rejection of `ideal_360` (`eval_gcs_demo_gates.py:95`);
  - the ordered workflow: manual request, confirming, ground-station confirmation, gimbal tracking lock, NAV, coordinate-pass reset, SNAP, and tracking kept through review (`:137`);
  - the owner's self-detect orbit and the forbidden confirmation paths (`:194`);
  - selection, approval and command identity bound to the scenario POI coordinate (`eval_gcs_demo_poi_binding.py:94,151,172`), with the global assignment `{owner: P}` from the scenario;
  - the vehicle gates: confirmation images, finite and cross-file SNAP integrity, observation count and cadence limits, roll quality (`eval_gcs_demo_vehicle.py:137,204,235`), on sim time.
  - Dropped: only the task 2/3 dispatch check (`eval_gcs_demo_gates.py:195`) and the peer-role checks.
  - Failing fixtures: a wrong vision profile and `ideal_360`, a forbidden review marker, a wrong command identity, a wrong POI coordinate, and each slice check below.

Slice checks:

| Gate | Evidence | Pass |
|---|---|---|
| Sensor | NavPy log lines | `POI: P` from the owner |
| Workflow ticks | NavPy log lines, as `(tick, stream position)` | the retained order holds in stream position. Equal ticks are allowed: input runs before timers, so a response and the NAV release can share a tick (`confirmation_inbound.py:73`, `poi_status_decision.py:60`) |
| Assignment | the journal's decoded MAVLink message counts per link and direction, and NavPy log lines | zero task-dispatch, advert and peer-relay messages on both links, and no auction or peer log marker; the profile names the message ids and markers. An empty backend event journal is not evidence: the listener suppresses adverts with one vehicle (`task_assign_listener.py:82`). Replaces the task 2/3 dispatch gates (`eval_gcs_demo_gates.py:195`). Named in `SLICE_FORBIDDEN_MESSAGES` (`eval_gcs_demo_slice_gates.py`, 9g): task dispatch `TASK_ASSIGN_REQUEST`, `TASK_ASSIGN_RESPONSE`, `SWARM_ACK` (only the auction and an assigned peer send it); advert `AVAILABLE_TASK_REQUEST`, `AVAILABLE_TASK_RESPONSE`; counts are keyed by id and identity, `<id>:<src_sys>.<src_comp>><target>` (`coordinator_mavlink.py`; a message outside the dialect by its raw frame's id and source), per link and direction (`from_sitl` including the parked frame, `to_sitl`, and `drain_to_sitl`, drain egress that is never delivered). The one exemption is the owner companion's `TASK_ASSIGN_REQUEST` to its own system (`self_assignment_publisher.py`, its task's publication); any other forbidden id fails, self-addressed or not. **Peer relay** is defined by source, not by id: swarm traffic reaches a vehicle only by MAVLink routing, which keeps the original source (mavlink-router between SITLs, the autopilot between its serial ports, `router_model.py`), so any message whose source system is neither the owner (autopilot and companion share its id) nor the ground station (255) fails |
| Pose cadence | `source-time/*.csv` (both participants record, 9g) | producer-named files (`navpy`, `gcs`), the owner's NavPy arrival streams, every file flushed whole |
| Approval | `operator_approvals.json` and the journal | exactly one approval, for P, with the timing below |
| Command | NavPy debug `EVENT:FINAL_APPROACH_CMD` rows (`eval_gcs_demo_evidence.py:106`) and the named-read bounds | every command within bounds until SNAP |
| Cadence | NavPy debug CSV | the sim-time speedup is 1 (the lockstep factor, not the wall launch speedup default at `eval_gcs_demo_vehicle.py:157`) |
| SNAP | NavPy log lines | present, at the mission terminal tick |

The retained checks and the slice checks are the named sim-time scoring gates the main acceptance requires. Counts that assume three vehicles take the scenario's count. Every gate in each profile is tested on passing and on failing evidence.

**Invocation** (the main doc's step 9 command gains `--scenario`):
`eval_gcs_navigation_demo.py --lockstep --scenario scripts/eval_lockstep_slice_scenario.json --seed 1 --run-dir <dir>`.

## Processes and ownership
**Coordinator** (the eval process):
- **Slot:** it claims the slot in-process through `gcs_launch`'s lockstep claim (owner ruling 4: the eval band, no frontend), so the owner is the coordinator from the start and there is no handoff.
  - The registry entry records the full owner identity `{pid, process start time, run token, claim nonce}`. The nonce is host entropy, fresh per claim: the run token repeats per seed, so it cannot tell two claims apart. The registry is host-side and never in the journal.
  - Release is compare-and-release on the full identity, atomically under the registry lock (a new `release_if_owner`).
  - **Children are recorded under the registry lock.** A spawn creates the child (it waits for `go`), then records it under the lock only if the entry is `active`, then sends `go` (`lockstep_wsl.py:118,129-135`). If the entry is `stopping`, the coordinator kills that child before `go`. A child that never got `go` exits on stdin EOF when its coordinator dies.
  - **Pruning keeps any lockstep entry with a live recorded child**, whatever its phase or the owner's liveness (today liveness reads only supervisor fields and ports, `instance_registry_entries.py:67,85`). A claim cannot take such a slot.
  - **Stop:** `gcs_stop` marks the entry `stopping` under the lock, which freezes the child inventory. It stops the recorded children, then the coordinator, then compare-releases once every frozen child is confirmed gone.
  - **Owner death:** the entry stays with its children; `gcs_stop` on it is the recovery, as above.
  - Tests: a delayed release from an old claim against a replacement claim with the same run token leaves the replacement intact; after the coordinator is killed, the children stay discoverable and the slot is not claimable until `gcs_stop` cleans it; a stop racing the record-and-`go` boundary leaves no child.
- **Ticks:** it owns the ticks; the SITL link is step 7's relay.
- **Participants:** it spawns and records them. Each spawn record carries the hermetic environment, the input manifest and a per-spawn capability (below). Real PIDs and ports go only to `*_host` sidecars.
- **Operator:** hosts it on its own baton, and journals every stream.

**Logical devices.** The two links are named `lockstep:serial0` (NavPy) and `lockstep:serial1` (backend monitor). These names appear everywhere a device string does:
- NavPy's `-c` (`navpy_process_launch.py:119`), and so its configuration log (`runtime_composition.py:85`);
- the operator's connect query (`eval_gcs_demo_lifecycle.py:25`, `eval_gcs_demo_process.py:155`);
- bus identity, and companion status `connection` (`navpy_process_instance.py:179`).

Only the coordinator maps them to transports. The coordinator address reaches a participant through its environment record. Payload tests run under two slot numbers and require identical bytes.

**Participant bootstrap** (NavPy and backend alike):
1. The entry imports only the standard library, `navpy.core.kernel.runtime_time` and `navpy.sim.lockstep`.
2. It binds the `LockstepRuntime` and installs the logging stamp and the thread guard.
3. It connects and sends `REGISTER`, meaning "transport and runtime ready to tick".
4. At its **admission tick** k_a (its first `TICK`), a managed bootstrap thread imports the product and constructs it under the baton.
5. Product readiness is a later event in a `DONE` (NavPy's `runtime_ready`; backend lifespan startup complete). Its round trips (heartbeat wait, mission download, parameter reads; `vehicle_build_transaction.py:144`, `runtime_composition.py:119`) run on ticks.

**Joining.** The tick number k is global, and t_k is the SITL's `source_us`. Nothing resets them for a participant.
- **Backend:** admitted at the first tick after step 7's boot barrier (`helpers_quiet`).
- **NavPy:** spawned from a spawn request in `DONE k`. The coordinator holds the clock until it registers (a wall budget), and admits it at k + 1.
- **Earlier bytes:** egress on a link with no admitted participant is discarded. The journal counts it per link and tick, as a UART with no reader would lose it.
- **Tick checks:** `DONE` and the missing-tick checks start at k_a.

## Wire `NVPART01`
TCP loopback; length-prefixed, versioned, with golden vectors.

**Participant to coordinator:**
- `REGISTER{capability, role, ordinal, spawn id, generation}`;
- per tick, `DONE{k, port bytes, responses[], events[], spawn requests[], stop requests[], log lines[], faults[], ended: null | {flushed: bool}}`.
- A `DONE` with `ended` set is the participant's last message. Retirement is part of the tick result, so there is no separate message whose arrival time could matter.

**Coordinator to participant:** `TICK{k, t_k us, port bytes, requests[], lifecycle[], cancels[], end: bool}`.

**Rules:**
- **Capability:** derived from the seed and the spawn id (deterministic), and passed only in that child's environment. `REGISTER` must match a recorded spawn's capability, spawn id and generation, or it is a host fault. The PID and start-time check stays host-side, in the sidecar.
- **Ids:** an id is `(origin ordinal, origin sequence)`: the operator is ordinal 0, and each participant numbers its own requests. The ids are deterministic, and the coordinator keeps them.
  - A response carries its request id and may come in a later `DONE`.
  - A request can stay pending over many ticks.
- **Lifecycle:** spawn and stop results, readiness and membership arrive in `lifecycle[]`, carrying the request id, ordinal and generation. Never a PID.
  - **Ids:** each record's `id` is its commit id, drawn when the coordinator commits it from the operator side's sequence (origin 0, shared with the coordinator's own spawns and the operator's requests). So commit order is id order, and `lifecycle[]` is in id order like every other list. The request a record answers or carries (a spawn or stop request; for `ready` and `exited`, the spawn id) is its `request`.
  - **Readiness:** a participant's `runtime_ready` event in `DONE k` commits a `ready{ordinal, generation, payload}` record at k. The next backend `TICK` carries it; the launcher port stores it as today's `ready_payload` and wakes `wait_ready` (`navpy_process_instance.py:158`, `navpy_process_manager.py:159`). `REGISTER` alone never satisfies it.
  - **Startup failure:** an ending `DONE` or a fault before readiness commits `exited{generation, failed}`, which `wait_ready` reports as today's exit before ready.
  - Test: registration, then several bootstrap ticks, then readiness; status is not ready until the record arrives.
  - A duplicate stop is answered with the same result.
  - A stale generation is an error result.
  - A stop requested during drain is answered `missed`, a fault: the operator stops companions in finalization, before drain.
- **Retirement is logical.** A participant retires at the tick k of its ending `DONE k`. The coordinator commits it in tick step 4, before the operator's baton runs and before `TICK k+1`. That tick is its lifecycle record. OS process exit never makes a lifecycle event:
  - after the ending `DONE`, the coordinator holds the clock and waits for the process to exit (a wall budget); the host exit code goes to the `*_host` sidecar;
  - a nonzero host exit after a flushed end, or a missed exit budget, is a host fault;
  - an exit or EOF without an ending `DONE` is a coordinator fault, never a membership change;
  - status under lockstep reads the lifecycle records (`running` until retired, `exit_code` 0 after a flushed end), not `poll()` (`navpy_process_instance.py:175`).
  - Test: a host delay after the ending `DONE` (a stop, and a plain drain end) leaves the journal unchanged.

**Participant stop.** One transaction, for a companion stop in mission or finalization:
1. The operator calls stop-all as today (`eval_gcs_demo_lifecycle.py:88`). The backend's launcher port puts a stop request in `DONE k`.
2. `TICK k+1` to NavPy carries `stop{id, generation}` in `lifecycle[]`.
3. Under lockstep the launcher passes `--stop-on-lifecycle`, not `--stop-on-stdin`. No thread reads real stdin; today's reader would block holding the baton (`runtime_stop.py:40`). A `LifecycleStopRequest` with the same `requested()` surface latches from the lifecycle entry, so the run loop's stop path is unchanged.
4. NavPy's run loop ends, and the existing cleanup joins every producer and drains the log worker, on ticks. `acknowledge_stop` runs its strict flush; its ack is a `stopped{id}` event, not a stdout marker.
5. NavPy sends `DONE` with that event and `ended{flushed}`. The coordinator retires it at that tick.
6. The next backend `TICK` carries the result: `acked` for a flushed end, else `missed`. The API returns it as today (`STOP_ACKED`), so the operator's check is unchanged.
- Budget: 10 s of sim time from the request, else `missed`.
- Tests: a real stop in finalization before drain; a cleanup failure gives `missed` and a host fault; a duplicate and a stale stop.
- **Ordering:** every list is in id order; streams are in delivery order.
- **Golden vectors:** each message; a duplicate tick; a missing tick; a pending multi-tick request; a cancel; a failed or mismatched registration; a stale stop.

## Run phases
1. **Boot:** step 7's firmware boot barrier.
2. **Mission:** SITL ticks, as below.
   - The **mission terminal tick** k_T is the first tick whose NavPy log lines contain `SNAP(VISION-NAV`. If the mission's sim-time budget (the scenario's) passes first, the run ends as a timeout, with the reason journaled.
   - **9g:** an operator whose run returns before k_T (its own sim-time workflow ended without SNAP, e.g. a confirmation timeout; it still stops its companion) ends the mission at that tick with the journaled outcome `operator_returned`, never `ok`, and the run drains and snapshots as usual. 9d made that a coordinator fault, which killed the run by ownership before its evidence was flushed. The eval's mission budget is a 3600 s backstop (`MISSION_BUDGET_S`); the operator's own waits (each at most `--timeout-s`) end a stalled mission first.
   - Gates are completed-evidence checks: none runs during flight. They run after the snapshot on the final files, as in wall mode (`eval_gcs_demo_vehicle.py:192`); where they need a SNAP cut they already make one (`mission_navigation_segment`, the selection window in `eval_gcs_demo_poi_binding.py:94`). Only faults (coordinator, host, participant) end a run early.
3. **Finalization:** SITL ticks continue after k_T until the operator's run returns (approvals written, cleanup calls answered).
   - The operator wakes at the next due wait: its waits are baton waits of at most 0.05 s of sim time.
   - Its companion-stop calls go through the API as usual (`eval_gcs_demo_process.py:239`) and complete through the participant stop, so NavPy has retired before drain.
   - Budget: 30 s of sim time, else a fault. The last SITL tick is k_F.
   - Ticks after k_T are journaled like any other.
4. **Drain:** after the reply to SITL frame k_F, the SITL runs loop k_F and sends frame k_F+1 through the relay. The coordinator reads that frame and never replies. That is the acknowledged parking boundary: the SITL is blocked reading reply k_F+1 (`lockstep_contracts.md:72`, `lockstep_relay.py:67`).
   - The parked frame is journaled as such (tick, length, hash) and routed nowhere.
   - SITL artifacts are snapshotted up to and including the parked frame's CSV row. The SITL is stopped by ownership after the snapshot, so no later row exists.
   - Test: host delays between the final reply, the parked frame and the stop give the same journal and artifacts.
   - Drain ticks continue k = k_F + 1, ...; t_k = t_{k_F} + (k - k_F) x Δ_drain, where Δ_drain is the manifest's `drain_period_us` (the scheduler period). It is synthetic and separate from the observed mission cadence.
   - The first drain `TICK` carries `end: true`. Drain `TICK`s carry no port bytes. Lifecycle, cancels and responses still travel.
   - Port egress during drain is journaled and discarded.
   - A participant leaves the barrier with its ending `DONE`. Drain records are in the digest.
   - Budget: 10 s of sim time, else a fault, a stop by ownership and no clean end.
5. **Snapshot:** artifacts are taken only after every participant has ended with `flushed: true` and the processes have exited (behind the clock hold, see Retirement).

## Tick k (mission and finalization)
1. Read SITL frame k. Relay EOF, an empty or malformed frame, a frame whose tick is not k, or a reply the coordinator cannot build: a coordinator fault. Never skip or repeat a tick.
   - **Grid check:** `source_us` must rise strictly, by at most the manifest's `max_frame_gap_us` (Δ_max) per frame; else a coordinator fault. Today the decoder accepts any header (`step_wire.py:79`). Δ_max is set from recorded serial-step firmware frames, and a test runs the check on such a recording.
     - **9g:** a frame advances 24 or 25 whole SITL physics steps of 833 us, 19992 or 20825 us, and the recording (`tests/fixtures/lockstep/serial_step_frames.csv`, 1200 frames of a seed-1 slice run) shows only those two. Δ_max = 20825 us (`lockstep_wsl_sitl.MAX_FRAME_GAP_US`); `test_grid_check_recording.py` passes the recording and fails a tighter bound, a stall and a skipped physics frame.
2. Route serial0 egress to NavPy and serial1 egress to the backend (or discard them, see Joining).
3. Send `TICK k` to the admitted participants, NavPy first, then the backend. It carries the operator requests made at tick k-1 and the lifecycle results. The participants run concurrently.
4. Collect `DONE k` in the same order:
   - NavPy egress goes to serial0 ingress, and backend egress to serial1;
   - responses, events and log lines go to the operator's ports;
   - an ending `DONE` retires its participant now.
5. Run the operator's baton to quiescence at t_k. Timers due at t_k fire after the deliveries. New requests are queued for `TICK k+1`.
6. Reply k to the SITL and hash the tick.

Each participant's work at tick k depends only on `TICK k`, so concurrency cannot change the result.

**Confirmation timing.** Named events:

| Event | Where | Tick |
|---|---|---|
| e1 | NavPy logs `CONFIRMING: P` (`confirmation_operator_review.py:39`) | k1 |
| e2 | The request reaches the backend over serial1; it broadcasts `task_confirm_request` (`task_confirm_listener.py:99`), delivered to the event port in step 4 | k2 ≥ k1 |
| e3 | The operator consumes e2: its loop's receive returns it (`eval_gcs_demo_confirmation.py:140-159`), after any sleep in progress and any events queued ahead | k3 ≥ k2 |
| e4 | The operator submits the approval: the first loop pass with elapsed ≥ 15 s since e3 | k4 |
| e5 | The backend's first response send | k4 + 1 |
| e6 | The response completes after three sends 0.25 s apart (`routes/task_confirm.py:95`) | k6 |

- The owner's 15 s delay starts at consumption, as in wall mode (`:159`). Between two approval checks the loop makes at most two waits, a receive of ≤ 0.05 s and a sleep of ≤ 0.05 s. Each wakes at the first tick with t ≥ its float deadline (`baton.py:197`), so each overshoots by at most Δ_max. So 15.00 ≤ t(k4) - t(k3) ≤ 15.10 + 2Δ_max, with Δ_max the validated frame gap above.
- A test checks the bound from the journal on recorded firmware frames, and on a deadline that falls exactly on a tick (9g: `test_confirmation_timing.py`, the real workflow on the operator's baton; the gate is `eval_gcs_demo_confirmation_timing.py`, in integer microseconds).
- **Journal evidence (9g):** the operator's observations in each tick record carry `received` (delivery tick and event type, e2 and e3), `sent` (e4) and `answered` (e6); tick records carry the NAVLINK messages decoded on each link and direction (e5 and the resends); NavPy's lines with their ticks are `lockstep_navpy_lines.jsonl`.
- k3 is journaled: the event port records each receive's tick. t(k3) - t(k2) has no fixed bound; it depends on the loop's phase and the queue ahead, both deterministic.
- Tests: delivery while the loop sleeps, and delivery behind queued telemetry events.
- e6 is about 0.5 s after e5, on the tick grid. Each resend waits its own 0.25 s after the previous send (the route sleeps after every send but the last), so the gate checks every consecutive interval, gap ≤ t(s_{i+1}) - t(s_i) ≤ gap + Δ_max, not only the span (two sends in one tick fail).
- The audited `actual_delay_s` ends at e6, as in wall mode.
- A test checks every relation from the journal. The 15 s owner ruling is unchanged.

## Participant runtime: `LockstepRuntime`
| Facet | Behaviour |
|---|---|
| Clock | `monotonic` = t_k; `time` = epoch + t_k; `perf_counter` = `monotonic`; `now` from epoch + t_k; `sleep` parks to a tick |
| Threads | the baton primitives, plus a managed `Thread` (the `threading.Thread` surface the product uses), `Timer`, `Semaphore`, `Queue`, `current_thread`, and `get_ident` from the creation sequence |
| Entropy | `Random(seed, ordinal)`; `uuid4` and `token_urlsafe` drawn from it |

- **Logging:**
  - A record factory sets `created`, `msecs` and `relativeCreated` from `rt.time()`.
  - `Formatter.converter` is pinned to `time.gmtime`.
  - Both are installed at bootstrap step 2, before the first record.
- **Thread guard:** at bootstrap step 2, `threading.Thread.start` and `_thread.start_new_thread` are wrapped. Starting any Python thread that the runtime did not create is a fault at creation, even one that would finish before quiescence.
  - Native threads inside extension modules are a documented boundary.
  - The lockstep defaults set `cv2.setNumThreads(0)`.
  - The per-tick `threading.enumerate()` audit stays as a second check.
  - Negative controls: a thread that starts and finishes within one tick, and one that outlives it.
- **Process identity:** `runtime_ready` and the launcher report `{ordinal, generation}`; `process_pid` is the ordinal under lockstep (a seam in `runtime_ready.py`). Captured stdout is not a payload.
- **Paths:** producers write run-relative paths under lockstep: `report.json` `log_dir`, and the backend's startup log line (`main.py:61`). Host paths and ports go to `*_host` sidecars.
- **Rates:** three cadences are distinct.
  - The scheduler cadence is the manifest's scheduler period (Δ_drain in drain).
  - The requested pose cadence is `resolve_pose_stream_rate_hz`, 0.8 x `SCHED_LOOP_RATE` read from the vehicle. The dock contact sensor uses it in both modes.
  - The observed pose cadence is message timestamps.
  - Nothing assumes 50 Hz, and nothing compares the cadences as a fault.

## Bus adapter: `LockstepBus`
- **Surface:** the one `VehicleMav` composition uses:
  - `conn` (the encoder plus `mode_mapping`), `send_lock` and `device`;
  - `reserve`, `attach`, `close` and `wait_heartbeat`;
  - heartbeats and membership.
- **Inbound:** `TICK` bytes go through `StreamDecoder` to `_dispatch`, on a managed reader that yields after each packet as the wall reader does.
- **Wall parity:** the bus admits heartbeats as `MavBusHeartbeats` does today, and `mode_mapping` follows `mavfile` (fixed-wing before any heartbeat, then the latest vehicle heartbeat's type).
- **Outbound:** drained into `DONE`.
- **Reservations:** `reserve` returns a reservation with `attach` and `release`, as real composition uses (`vehicle_build_transaction.py:70,117`, `vehicle_lifecycle.py:55,93`). Message subscriptions stay the `MavBusMembership` receivers. Releasing the last reservation closes the link: the coordinator stops routing its egress and journals the close.
- **Contract test:** a NavPy-style and a backend-style `VehicleMav` run against a reference autopilot byte stream. The stream covers heartbeat admission, a mode change, a parameter set, a named parameter read, a mission download and close; and construction failing before attach, failing after attach, and cancelled.

## Backend participant
**Requests:** `asgi_request` is extended first:
- the URL splits into path and query string;
- headers, with JSON bodies sent as `application/json`;
- status and error bodies pass through;
- a sim-time deadline, cancellation and the request id.

Its test uses `/connect?device=lockstep:serial1&sys_id=...` (`routes/vehicles.py:26`).

**Event loop:** `BatonEventLoop.time()` is the runtime clock, so the backend's `asyncio.sleep` waits are sim waits.

**Tick input order** (backend): lifecycle, then cancels, then requests in id order (each starts its task), then the serial1 bytes; then run to quiescence, with timers due at t_k after the deliveries. A subscribe and a trigger queued together therefore install the subscription first. Tests: broadcasts just before and just after the installation; a subscribe and a trigger in one tick.

**Threads:** AnyIO and the default executor bind to `BatonExecutor`.

**WS:** broadcasts go to a per-tick event sink. The sink keeps two things apart:
- **The event journal** records every broadcast, subscribed or not.
- **Subscriber delivery** follows WebSocket semantics. `EventStreamPort.__enter__` sends a subscribe request and parks until its acknowledgement, as the WS connect returns today before `trigger()` (`eval_gcs_demo_confirmation.py:136`). The backend installs the membership when the request's handler runs, and sends its initial snapshot first (`routes/telemetry.py:92`). A broadcast is delivered if the membership exists when the broadcast executes (`broadcast.py:33`), so delivery starts exactly after installation. Nothing earlier is replayed.
- `__exit__` queues an unsubscribe; events queued but not received are dropped, and the journal counts them.

**Ports:**
- **Vehicle bus:** a `LockstepBus` for `lockstep:serial1`.
- **Companion launcher:** spawn and stop requests in `DONE`. Start-all and status answer from the coordinator's lifecycle records, with ordinal and generation (`routes/navpy_sim.py:132`, `navpy_process_instance.py:175`, the telemetry broadcast at `telemetry_loop.py:226`).
- **ESP32:** an in-process simulator port covers start, status, health (including `launch_session.py:84`), trigger results and stop. It creates no `HTTPServer` (`esp32_simulator.py:234`) and reports no port.
- **Named parameter reads:** `GET /vehicles/{sys_id}/params/named?names=A,B`.
  - Fresh reads through the existing fresh-read API (`vehicle_public_parameters.py:40`, `parameter_reader.py:42`), never the cached `get_parameter`.
  - A missing or unanswered name is an error that lists it.
  - The operator's command bounds use it under lockstep (owner ruling 7). Wall mode keeps MAVFTP (`eval_gcs_demo_stack_ops.py:119`).
- **Parameter writes:** the blocking `set_parameter` on the loop thread (`routes/missions.py:247`) moves to the executor. A wall regression test covers it.

**Shutdown in drain:**
1. **Fence every producer.** The backend keeps an ownership inventory of everything that can make work or events, and the fence stops each:
   - HTTP routes: new requests get 503;
   - the telemetry loop: stopped first, with its existing stop behaviour (it submits executor work and broadcasts, `telemetry_loop.py:226`);
   - autoconnect and the launch tasks: cancelled;
   - the mission probe (`vehicle_mission_probe.py:55`): now a tracked, cancellable `rt.Thread`;
   - every listener (task confirm, `task_confirm_listener.py:53`; task assign, `task_assign_listener.py:57`; compass calibration, `compass_cal_listener.py:62`): registration keeps its subscription handle and tracks the broadcast futures it schedules (`task_confirm_listener.py:148`); the fence unsubscribes them;
   - route-created broadcast futures are tracked too (the mission progress bridge, `routes/missions.py:195`).
   - **Admission versus admitted work:** the fence closes admission only. Work admitted before it (a scheduled broadcast coroutine, a running job) runs to completion or its cancellation and is journaled. Shutdown's own operations (`main.py:190`) run under a shutdown owner that the fence admits.
   - After step 3 the event sink closes. From then on a broadcast is a fault, and so is an executor submission by anything but the shutdown owner. The shutdown owner may submit until step 7 shuts the executors down (companion cleanup goes through `asyncio.to_thread`, `main.py:190`).
   - Tests: the fence lands between scheduling a broadcast coroutine and running it (the broadcast is delivered and journaled); the inventory is compared with the producers the code registers, so a new producer fails until it is fenced.
2. **Cancel.** Each unit of work has a cancellation scope:
   - **Ownership:** a scope is a context variable, not thread state, because every coroutine shares the loop thread (`baton_asyncio.py:132`). Each `asgi_request` runs its task in its own scope, keyed by its wire request id. The background producers (telemetry, autoconnect, launch tasks, the probe) each own one.
   - **Propagation:** `BatonExecutor` runs each job in the submitter's copied context, and a managed `rt.Thread` copies its creator's context, so a route's executor job and its child work share the route's scope. A `cancels[]` entry cancels exactly the scope of its request id.
   - **Every runtime wait observes it.** The `rt` lock, condition, event and queue waits check the current token, and a cancel wakes them. That covers `MessageStore.wait_after` (`message_store.py:119`), the mission inbox (`mission_inbox.py:60`) and the mission lock (`mav_mission.py:100`, `waypoint_builder.py:128`) without threading a token through each call.
   - **A cancelled wait raises `Cancelled`, a `BaseException`.** No `except Exception` retry branch catches it, so it is not an upload failure: no retry (`waypoint_builder.py:148`), no failure cached, no further send.
   - **Unwinding:** a primitive checks cancellation after its park, from its own state like any wake (`lockstep_contracts.md:39`). Every unshielded wait is a cancellation checkpoint, and a cancelled scope never returns from one, so no product side effect runs after its cancel (the mission lock at `waypoint_builder.py:128` is followed at once by a progress broadcast, `missions.py:195`). If a lock was already handed to a cancelled waiter, it takes ownership only to pass it on: it releases (handing to the next waiter) and raises. Otherwise the waiter removes itself in a `finally` and raises; today removal runs only after a normal park (`baton_sync.py:41,167,203,219`). A condition's reacquire in `_restore` (`:68`) is shielded, and so are rollback and cleanup paths (`vehicle_build_transaction.py:99`, `vehicle_lifecycle.py:81`), through an `rt.shielded()` scope.
   - **Consumption:** a `Cancelled` raised in a scope that was cancelled ends its unit normally. The managed-thread entry retires the thread (not a scheduler error, `baton.py:127`); the executor future ends cancelled, not failed (`baton_asyncio.py:109`); the request answers `cancelled` with its id. A `Cancelled` from a scope nobody cancelled, or any other escape, is still a fault.
   - Under lockstep a cancel arrives in `cancels[]` and is applied before the tick's input, so a parked job returns within that tick, not at its 10 s deadline.
3. **Retire work.** Await the jobs, the probe, the cancelled tasks and the tracked broadcast futures, on drain ticks. A job does not stop because its awaiting coroutine was cancelled (`baton_asyncio.py:106`); the token stops it.
4. **Companions:** stop them through the port; an error is a fault.
5. **Vehicles:** close them strictly; a close failure is a fault under lockstep.
6. **Recording:** stop the pose flusher with a strict flush, then finalize the diagnostics.
7. **Executors:** shut them down and join the managed threads.
8. End with a `DONE` carrying `ended{flushed}`. Any failure or a blown budget means no clean end.

Tests: a full lifespan shutdown succeeds, also with no companions; two simultaneous requests with one cancelled leave the other intact; a waiter cancelled and then the same primitive used by another thread; a cancel racing a handoff in one tick (the cancelled waiter passes the lock on, and no progress is submitted); a probe cancelled in flight ends cleanly. Tests also cover shutdown during a pending parameter read, during a mission upload (the cancel arrives just after the job parks, and later UART replies are withheld), while a job waits for the mission lock, and with a probe in flight, not only after success. Each asserts no further sends or progress submissions after the cancel.

## NavPy participant
- **Catalog transaction.** `PoiProvider` separates mutation from reading:
  - **Mutation lock:** one lock linearizes `refresh` and `set_sim_poi`, held across capture, resolve and publish. Only mutations take it, so it may park on round trips (parameter reads, `navigation_poi_args.py:34`; terrain, `poi_altitude.py:108`). Two refreshes cannot interleave, so there is no generation retry and no retry fault.
  - **Catalog lock:** guards only the published tuple. Readers take it and never wait on a round trip (`sim_poi_catalog.py:30` no longer resolves under it).
  - **Steps:** capture the inputs (the argument values, home, mission items, the altitude settings, the sim-POI overrides); resolve them with pure functions; publish a tuple of frozen POI records (frozen copies of `SimulationObject` and `Location`) under the catalog lock.
  - **Override lifetime:** as today, a sim-POI override lives until the next refresh rebuilds the list (`poi_provider.py:46`).
  - **Wall mode:** the same locks; a mutation waits for the one in progress.
  - Tests: wall-mode concurrency (refresh against `set_sim_poi`, two refreshes); a reader during a parked parameter read is not blocked; a terrain wait under the baton.
- **Cadence:** `IdentitySchedulerCadence` replaces the speedup, and nothing writes `SIM_SPEEDUP`.
- **Observation:** navigation log lines go into `DONE` (`log lines[]`) as well as the file.
- **Drain** (only if NavPy has not already stopped through the participant stop):
  1. On `end`, stop the run loop. Workers sleeping to deadlines wake on drain ticks (`runtime_resources.py:215`, `high_resolution_deadline.py:35`, `navigation_loop.py:161`).
  2. Flush the pose-cadence buffers and the log worker.
  3. End with a `DONE` carrying `ended{flushed}`.
- **Terrain:** the lockstep defaults set `TERRAIN_ENABLE 0`, so terrain comes from the DEM after the 1 s report wait, which is in sim time.

## Operator
The operator's modules run on the coordinator's baton. Their I/O becomes explicit ports on `StackContext`, and `run_mission` passes them instead of building defaults:

| Port | Lockstep | Wall |
|---|---|---|
| `JsonApiPort` | queues the request and parks until its response arrives in a `DONE` | the HTTP API |
| `EventStreamPort` | the WS event sink | a real WebSocket |
| `TailPort` | backend and NavPy log lines from `DONE` | the existing file tails |
| `StackContext.clock` | the baton clock | `SystemClock` |

## Supersedes
In `deterministic_demo_eval.md`, edited with this design (also the Python-participants row: an ending `DONE` replaces `END(k_end)`, and readiness is a lifecycle record; the Operator row: `EventStreamPort` delivers subscribed events, and the backend event journal records all; the thread audit rows add the creation-time guard):
- **Clock row** (`:75`), "sleeps round up to whole ticks in integers": a wait wakes at the first tick with t ≥ its float deadline, as the prototype does (`baton.py:197`) and the product's float deadlines need (step 6).
- **Operator row** (`:82`), "approval happens at the request tick + 15 s": the 15 s run from the operator's consumption of the request, as in wall mode; see Confirmation timing. The owner ruling is unchanged.

In `lockstep_contracts.md`, edited in the sub-step that implements each:
- **"There are no leases"** (`:104`): the bus has reservations (9c).
- **Lifespan startup before `REGISTER`** (`:64`): it now runs at the admission tick (9e).
- **`REGISTER` with pid and start time** (`:117`): it now carries the capability, spawn id and generation. The PID check is host-side (9d).
- **`END` flush and ack** (`:119`): it is now the participant stop or the drain phase, both ending in a `DONE` with `ended` (9d).

## Sub-steps (each a commit with its tests)
| # | What | Proves |
|---|---|---|
| 9a | `DemoScenario` threaded through the operator; gate profiles; slice fixture; operator I/O ports | Wall mode unchanged; both profiles pass and fail on their evidence |
| 9b | `LockstepRuntime`: facets, bootstrap, logging stamp, thread guard, process identity, drain | Real components give byte-identical traces under host delays (`MessageStore`, `CacheLogger`, `TaskDispatch` timers, `ThreadLaunchGate`); fresh-process import with pose recording on; the thread-guard negative controls |
| 9c | `LockstepBus` and its contract test | `VehicleMav` composes and runs on bytes alone |
| 9d | `NVPART01`, the participant loop, the coordinator (phases, joining, drain), capability spawn, slot claim and compare-release, `gcs_stop` by ownership | Golden vectors; a two-participant toy run with a late joiner, a participant stop and the SITL parking boundary is deterministic; retirement (with a delay after the ending `DONE`) and exit faults; ownership tests (coordinator exit, partial spawn failure, concurrent stop and reclaim, a delayed release against a same-token replacement, owner death with live children, a stop racing record-and-`go`) |
| 9e | Backend participant: `asgi_request` extension, lifespan, ports, named reads, producer fence, cancellation-aware waits, event subscription, strict shutdown | Through a scripted API actor (the operator's setup calls), connect, parameters and mission upload of the slice fixture repeat byte for byte, under two slot numbers |
| 9f | NavPy participant: bootstrap, lifecycle stop, catalog transaction, cadence, log lines, drain, run-relative paths | Spawned through the launcher port after 9e's setup, NavPy boots on a serial-step SITL, is stopped through the API, and its journal repeats |
| 9g | Operator on the baton, `--lockstep` on the eval, the confirmation-timing test, acceptance runs, baton cost re-measured | The acceptance above. Built 2026-10-14; repeatability accepted, mission success blocked by todo 46 (see `deterministic_demo_eval.md`, Today) |

Step 10 then adds the fleet barrier, the router and three NavPy participants for the certified 3-UAV scenario. The swarm chat's d4 acceptance (two 3-UAV runs, all three dock) waits on it.
