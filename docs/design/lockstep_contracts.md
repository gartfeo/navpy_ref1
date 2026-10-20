# Lockstep contracts

Step 6 of `deterministic_demo_eval.md`. These are the contracts the lockstep build (steps 7-10) implements. Each is pinned by a test now, or by a named acceptance test in the step that builds it.
- Prototype code: `src/navpy/sim/lockstep/`.
- Tests: `tests/modules/lockstep/`.
- Benchmark: `scripts/lockstep_baton_benchmark.py`.

## Execution model (chosen; re-measured at step 9g)
**The baton.** Existing product threads run unchanged as managed OS threads, one at a time, on the sim clock (`baton.py`, `baton_sync.py`, `baton_queue.py`, `baton_asyncio.py`).

**Benchmark** (Windows, Python 3.11.9, 7 samples, wall µs per tick, median with range; `scripts/lockstep_baton_benchmark.py`):

| Case | Baton, step 6 | Baton, 2026-10-14 (step 9g) | Focused synchronous, step 6 / 9g |
|---|---|---|---|
| Mission upload, 200 items | 183 (169-276) | 291 (287-307) | 102 (91-119) / 162 (160-171) |
| 10 threads woken every tick (wake count asserted) | 183 (155-193) | 162 (95-273) | n/a |
| 40 threads woken every tick | 788 (691-810) | 609 (574-651) | n/a |

The 9g re-run was on a host busy with other sessions; both columns of the upload case slowed by the same factor (1.6x), so the baton's relative cost is unchanged.

**The real 1-UAV slice (step 9g, 2026-10-14)**, measured end to end: wall seconds of `execute_lockstep_run` (the WSL serial-step SITL and relay, the coordinator with its journal and MAVLink decode, the backend and NavPy participants with NavPy's simulated camera, and the operator) over the run's 5194 SITL frames (about 104 s of sim time, boot to the parked frame):

| Run | Wall s | Wall ms per tick | Real-time factor |
|---|---|---|---|
| seed 1 (`.logs/s9g/acc-a`) | 27.2 | 5.2 | 3.8x |
| seed 1 (`acc-b`, concurrent with `acc-c`) | 24.3 | 4.7 | 4.3x |
| seed 1, host delays (relay, coordinator, participants up to 4 ms each, helper jitter) under CPU load (`acc-c`) | 75.5 | 14.5 | 1.4x |

With pose-cadence recording on in both participants (after Codex round 1, `r3-a`, `-b`, `-c`): 25.1, 25.7 and 75.8 wall s, the same costs. So the slice runs faster than 1x real time; the fallback below is not needed.

**Why the baton:**
- **Budget.** A lockstep tick is one ArduPlane main loop, 50 Hz by default (`SCHEDULER_DEFAULT_LOOP_RATE`), so about 20 ms of sim time. 10x real time leaves 2 ms of wall time per tick; 1x leaves 20 ms. The baton's overhead fits even with 40 threads waking every tick.
- **Integration cost.** The synchronous model is faster per tick, but each blocking workflow becomes a hand-written state machine. The prototype shows such a rewrite must also copy the product's float deadline arithmetic: with integer deadlines a retry fired one tick early, because `0.2 + 0.1` is `0.30000000000000004`. There are about 200 NavPy and 85 backend wait sites. The baton runs the real `MissionUploader`, `MissionInbox` and a FastAPI app unchanged; only their clock and primitives are bound (step 8).
- **Re-check at step 9.** The 1-UAV slice is re-measured. If it cannot reach 1x real time, the fallback is synchronous seams at the measured hot paths only, not a general rewrite. Done at 9g (above): about 4x real time.

## Scheduling (pinned: `test_baton.py`, `test_mission_upload_lockstep.py`)
- **Coordinator.** The thread that creates the runtime. Only it advances time, schedules (`run_until_quiescent`, never nested) and delivers inputs (`BatonQueue.deliver`, `BatonEvent.deliver_set`, `BatonEventLoop.deliver`). Product threads use primitives only while they hold the baton. Anything else is a `LockstepFault`.
- **States.** A managed thread is runnable, parked or done. It parks on a lock, condition, event, queue, sleep or join, with an optional timer. Primitive state needs no OS lock.
- **Tick order** (`BatonRuntime.tick`):
  1. Set `t_k`.
  2. Deliver inputs in byte order.
  3. Run to quiescence.
  4. Fire due timers and run to quiescence; repeat while anything is due at `t_k`.

  So an input that arrives at a waiter's deadline tick wins over its timeout.
- **Order.** Runnable threads run first-in first-out. Timers fire in `(due, park seq)` order. Every wake-up order follows, so the order is deterministic under host delays (pinned with injected sleeps).
- **Deadlines.** A wait of `d` seconds started at `t0` is due at the first tick with `t_k >= t0 + d`, evaluated in float seconds: the comparison the product makes when it re-checks.
  - A plain sleep never wakes early: `sleep(0.0200004)` wakes at tick 2.
  - A deadline that float drift puts just after `t_k` waits for the next tick instead of re-arming forever: `0.2 + 0.1` fires at tick 16, not 15.
  - Only a zero wait is due at the same time. More than `MAX_TIMER_ROUNDS_PER_TICK` timer rounds in one tick is a busy wait, and a fault.
- **Wake-ups.** Release hands a lock directly to the first waiter. After a park, each primitive decides from its own state, never from how the park ended, because a handoff, notify, set or put can land after the timer fired but before the thread ran.
  - An event set and cleared again still releases the waiters it found.
  - A queue get keeps one absolute deadline across contention.
- **Quiescence** means no runnable thread. `DONE` may leave threads parked on future input: a mission upload spans ticks.
- **End of run.** `check_no_deadlock` fails if any thread waits without a timer on a lock or a join. `abort` unwinds every parked thread with `LockstepAborted`.
- **Latched faults.** A fault is latched in the runtime (`fail`), so the next scheduling call raises it even if product code, or asyncio's callback handler, swallowed the exception.
- **Poisoning.** After a wall-budget breach the holder may still be running, so the runtime refuses every further tick, handoff and abort. Only stopping by ownership is left.
- **Host faults, never outcomes:**
  - a primitive used off the baton;
  - re-acquiring a plain lock, or releasing an unowned one;
  - a self-join;
  - time going back;
  - a baton holder that does not give the baton back within `BATON_WALL_BUDGET_S` (it blocked on a host primitive);
  - an unmanaged thread in `threading.enumerate()`. Finished managed threads are reaped (joined) as they end, so the audit compares membership exactly.
- **Creation seq.** Every managed thread and park has a unique sequence number. Product code must not order work by `id()` or hash (step 8's lint).

## ASGI (pinned: `test_baton_asyncio.py`; full backend in step 9)
- **The loop.** The backend loop runs in one managed thread (`BatonEventLoop`).
  - `loop.time()` is the runtime clock. asyncio timers become due only in the timer phase, and exactly at `when <= t_k`: the selector sets asyncio's clock-resolution slack to one ulp in the timer phase and to minus infinity while inputs run. So an input at a timer's tick runs first, and float drift waits a tick (pinned).
  - `select` never touches the host. It returns immediately when asyncio passes timeout 0 (ready callbacks exist). Otherwise it parks until the next asyncio timer or a wake-up.
  - `call_soon_threadsafe` (the self-pipe write) wakes it through the runtime.
- **Joint quiescence.** The loop thread parks only once asyncio has run every ready callback and due timer. So "no runnable managed thread" covers callbacks, tasks and timers together. asyncio keeps its own order: ready callbacks first-in first-out, timers by `when`, with heap ties resolved by insertion history.
- **Workers.** The default executor runs each job in a managed thread (`BatonExecutor`); its completion is a `call_soon_threadsafe`, so it lands in the same tick's quiescence.
  - Shutdown refuses new jobs and joins outstanding ones. `loop.shutdown_default_executor` waits for them on the loop, instead of starting asyncio's helper thread (pinned).
  - AnyIO worker threads (sync routes, `run_in_threadpool`) must be managed too: step 9 binds `anyio.to_thread` to the managed executor. A stray AnyIO thread is caught by the audit.
- **Lifespan** runs explicitly: startup at the admission tick, then readiness (`runtime_ready`) in a `DONE` (step 9e); shutdown in the drain phase, ending in a `DONE` with `ended` (step 9d).
- **Requests** run through an in-process ASGI call (`asgi_request`): no server, no sockets. A request delivered at tick k runs from tick k. Its response returns in the `DONE` of the tick its handler finishes in.
- **Cancellation** is an input. A cancel delivered at tick k cancels a handler still running then, because inputs come first. A response already in `DONE` k-1 stands.
- **No blocking MAVLink call on the loop thread:** such work goes to the managed executor (pinned with a request that sleeps in the executor).

## UART (pinned on the model: `test_reference_models.py`; firmware matches `uart_model.py` in step 7)
- **Ingress.** Bytes the coordinator sends for tick k enter a per-port FIFO that never drops. Each io tick admits `min(fifo, read-buffer space, 10000)` bytes, as the socket path does (`UARTDriver.cpp` `max_bytes = 10000`) while `SIM_BAUDLIMIT_EN` is 0, the firmware default. The sim profile's `baud_limit` (`sitl_models.py`) sets it to 1; then each io tick also admits at most the line's baud/10 bytes per second, metered on sim time (`DataRateLimit`), and egress drains at that rate. The usable buffer is 16383 bytes, because a `ByteBuffer` keeps one free. A burst larger than the buffer arrives whole and in order over several io ticks.
- **Egress.** A write takes at most `txspace()` bytes; the writer drops the rest and the drop is counted. At the hook, everything written since the last frame is drained into the frame.
- **Coordinator queue** (a routed fleet; pinned: `test_coordinator_queues.py`). The coordinator keeps one FIFO per SITL port and never drops. Reply k takes `min(queued, 0xFFFF, FIFO_CAP - fifo_bytes(frame k))` from its head, so the firmware's `ingress_fifo_full` cannot happen. `FIFO_CAP` is 2^20: the firmware FIFO is a `ByteBuffer{BUFFER_BYTES + 1}` with `BUFFER_BYTES = 1U << 20`, and a `ByteBuffer` keeps one byte free. Bytes still queued at parking are journaled `undelivered`. Each byte keeps the phase it was written in (admitted by reply j, undelivered at parking, or written in drain tick k) until framing resolves it; the closing `stream_bytes` counts every byte once per port, phase and class (`packet`, `skipped`, `held`), so each phase adds up to the bytes it carried (pinned: `test_coordinator_streams.py`).
- **Phases within firmware loop k.**
  1. The FAST_TASK hook sends frame k: the header plus the egress written since frame k-1.
  2. The hook blocks for reply k, the ingress per port, which enters the FIFOs. The clock is frozen across the exchange, as today.
  3. The SITL io tick admits from the FIFOs.
  4. `GCS_MAVLINK::update` parses what was admitted and applies it within its per-loop budget.
  5. Everything written in loop k goes out in frame k+1.

  Measured at step 7: a `COMMAND_LONG` in reply k yields its `COMMAND_ACK` in frame k+2, in every run. A message split across ticks parses once complete (pinned).
- **Pacing.** In serial-step the coordinator paces the run: `sync_frame_time` skips its wall sleep. The achieved rate it measures only feeds the SIM2 BIN record, which is off.
- **Wire** (`NVSTEP02`, version 2, little-endian, no implicit padding; separate from `NVGUID03`). The exact layout and its golden vectors are in `step_wire.py` and `test_step_wire.py`, and the firmware must produce the same bytes:
  - **Frame header** (52 bytes): magic, version, port count, boot id, tick k, `source_us`, out-of-loop advances, `helpers_quiet` (0 or 1), 3 reserved zero bytes.
  - **Frame, per port** (16 bytes): FIFO length, read-buffer fill, egress drop count, egress length; then the egress bytes.
  - **Reply header** (20 bytes): magic, version, port count, tick echo k.
  - **Reply, per port:** ingress length as `u16`, at most 65535; then the bytes.
  - A wrong magic, version, port count or echo, an oversize or truncated block, trailing bytes, or a missing or duplicate tick is a protocol error.

## Router (pinned: `test_router_parity.py` against the binary, `test_reference_models.py`)
Emulates mavlink-router `d59ef94` (`/usr/bin/mavlink-routerd`), `Endpoint::read_msg`, `accept_msg` and `route_msg`, in the configuration `run_swarm.sh` gives it: `-s 255` (sniffer sysid 255, the GCS), `BlockSrcCompOut = 1,100` and `BlockMsgIdOut = 130,131` on each SITL endpoint (`radio_budget.md` item 2; none on the backend), no other filters or endpoint groups, and no dedup period, so no dedup. Model: `router_framer.py` and `router_model.py`; in the tick path of every run, the one-SITL slice included since 10c (`coordinator_router.py`, `lockstep_fleet.md`).
- **Targets** come from the router entry table (`router_entries.json`), extracted from the deployed binary, whose sha256 is pinned beside it; never from NavPy's dialect. An id outside the table has no target (a broadcast) and no CRC check; a known id must pass its CRC. A target field beyond a zero-trimmed payload is 0.
- **Parity** is pinned, not claimed: `test_router_parity` replays recorded datagrams (targeted, broadcast, sniffer, unknown target, targeted NAVLINK, boot text, split packet, AP echo, and five cases with the SITL filter: broadcast, targeted message classes, unknown id, mixed loop, filter and target rejection, the image ids from each companion) and requires the binary's bytes at the binary's endpoints and its `msg_to_unknown` count (from its trace log); `NAVPY_ROUTER_PARITY=1` re-runs the binary in WSL (`scripts/router_parity.py`).
- **Learning.** Each endpoint learns every `(sysid, compid)` it receives from.
- **Loops.** No endpoint gets a message whose source it has seen. This excludes the sender and stops loops, including ArduPilot's serial-to-serial forwarding echoes.
- **Sniffer.** An endpoint that has seen sysid 255 gets every other message, targeted or not. So the backend sees vehicle-to-vehicle traffic, as today.
- **Broadcast.** Target system 0, or no target field: every other endpoint.
- **Targeted.** Only endpoints that have seen the target system, and the component when it is nonzero. An unknown target is dropped and counted (`router_dropped`), apart from bytes that are no packet or fail their CRC (`router_skipped_bytes`).
- **Filtered.** After the loop check and before the target and sniffer rules, a SITL endpoint refuses packets from components 1 and 100, for any message id. A packet no endpoint accepted and at least one filtered ends `router_filtered`, not `router_dropped` (mavlink-router does not count it as `msg_to_unknown`).
- **Forwarding.** Original packet bytes are forwarded unchanged, in fixed endpoint order: SITL 1-3, then the backend.
- **Decoder state** is kept per inbound stream across tick fragments (`router_framer.RouterFramer`).

## Bus adapter (wire pinned: `mavlink_wire.py`; the adapter in step 9)
One narrow bus replaces `mavutil.mavfile` for NavPy and the backend in lockstep.
- **Inbound.** Per tick bytes, decoded with persistent parser state and dispatched through the existing `_dispatch` fan-out (`MavBusReader._dispatch`) in byte order, on the managed reader thread. After each packet the reader yields: it goes to the back of the runnable queue at the same t_k (`baton_sync.yield_now`, no timer round), so a woken consumer runs before the next packet. The wall reader differs: it yields once per batch of already-decodable packets, bounded by one scheduler wall slot (`reader_batch.md`), so wall and lockstep reader scheduling are not the same.
- **Outbound.** A `MAVLink(file=ByteSink)` encoder per source keeps `seq` and MAVLink 2 framing as a real link would. Each participant's `(sysid, compid)` is its product value.
- **`mode_mapping()`** is `mavfile`'s: the fixed-wing table before any heartbeat, then the table for the latest probable vehicle heartbeat's type (`mavfile.probably_vehicle_heartbeat`), or PX4's map for a PX4 autopilot.
- **Heartbeat admission.** The bus admits as `MavBusHeartbeats` does today: any non-GCS, non-companion heartbeat from an eligible sysid counts. The router's compid-1 filter governs the vehicle's heartbeat state, not bus admission.
- **Subscriptions** are the existing `MavBusMembership` receivers.
- **Reservations.** `reserve(sysid)` returns the existing `MavBusLease`: `attach` adds the receiver, `release` drops the reservation. Opens share one bus per logical device (`lockstep:serial0.<sysid>` NavPy, `lockstep:router` backend; the role config names it, 10c), as `MavBus.get_or_create` does.
- **Close.** Releasing the last reservation closes the bus, following step 4: the reader stops first, then the bus counts as closed, and the link reports `closed`. A reopen is a new bus; bytes delivered while none is open are lost and counted (`lockstep_link.py`).

## Firmware helpers (specified; acceptance in step 7)
- **Barrier.** The frame's `helpers_quiet` flag is the init barrier. Only SITLs tick until every SITL reports it, and participants join only after. The firmware sets it once all three hold:
  - the scripting run list is empty: `aas_params.lua` is one-shot and has finished; `chute_deploy.lua` and `tune_pitch_roll.lua` are not loaded in lockstep;
  - `AAS_NAV_CGT` exists;
  - the logger backend is none (`LOG_BACKEND_TYPE 0`, BIN off per owner Q6) and FTP is refused.
- **Evidence.** Step 7's firmware review gives source lines for each helper's remaining loop: the scripting thread's idle wait, the logger's IO thread and the FTP worker. It shows that none can change vehicle state while the flag holds.
- **Terrain is not a helper (step 9h, todo 46).** `TERRAIN_ENABLE 1` as in the wall eval, with the pinned tiles seeded into each instance's `terrain/` and hashed in the input manifest. In SITL the terrain IO is no thread, so the barrier needs no terrain condition (firmware `2dbd7e38dd`):
  - `AP_Terrain::io_timer` is an IO proc (`TerrainIO.cpp:68-71`). SITL's `register_io_process` only appends it to a list (`AP_HAL_SITL/Scheduler.cpp:176-188`). `_run_io_procs` (`:266-296`) runs that list on the main thread only: after each loop (`HAL_SITL_Class.cpp:289-290`), in `stop_clock` on sim time (`Scheduler.cpp:302-308`, from `SITL_State.cpp:294`), and in `timer_event` (`Scheduler.h:45-48`) from `_fdm_input_step`. `wait_clock` steps the FDM only on the main thread; a helper only sleeps (`SITL_State.cpp:152-194`).
  - The handoff is a state machine on that thread. `find_grid_cache` marks a missing block `DISKWAIT` (`TerrainUtil.cpp:107-143`). `schedule_disk_io`, called from the 10 Hz `AP_Terrain::update` task (`ArduPlane.cpp:133`, `AP_Terrain.cpp:363-367`), queues the read (`TerrainIO.cpp:73-81`). The next IO proc run does a blocking `read` (`:290-335`, `:370-377`), and the following `schedule_disk_io` makes the block valid (`:83-95`). Each step happens at a fixed loop, and the only clock read is `AP_HAL::millis()`, which is sim time. Wall time cannot move a load to another tick.
  - Consumers: the SITL ground is `ground_height_difference()` from `height_amsl` (`SIM_Aircraft.cpp:117-131`; `SIM_TERRAIN` defaults to 1, `SITL.cpp:192`), applied by the ground clamp (`:775`) on the main thread. Until the home block loads, it is the flat home level. `TERRAIN_REPORT` replies to NavPy's `TERRAIN_CHECK` come from the same cache.
  - GCS traffic: `request_missing` sends no `TERRAIN_REQUEST` while a block waits for disk or is fully populated (`TerrainGCS.cpp:52-62`). With complete tiles covering the flight area, it sends none. A request would still be a deterministic main-thread send, and no lockstep participant answers it with `TERRAIN_DATA`, so nothing is written to disk. Measured in step 9h with a decoder probe: no `TERRAIN_REQUEST`, and `TERRAIN_REPORT` shows `pending` 112 while the first blocks load, then 0.
- **Out-of-loop advances.** The header counts clock advances not caused by the loop's own wait (`Semaphores.cpp:41-63`); nonzero is a host fault. The counter cannot see a helper that changes state without advancing time. That is why the barrier exists, and the helper-thread jitter hook (acceptance (c)) checks the remainder.

## Ownership (specified; implemented in step 7)
- **Spawn record.** Each child is recorded by pid and process start time the moment it is spawned, before it registers, in `<run>/lockstep_processes_host.json` (write, then rename). A WSL child is also recorded by its `/proc` cwd, which must lie under the run's root.
- **Registration.** Every participant connects to the coordinator and sends `REGISTER{capability, role, ordinal, spawn id, generation}` (`NVPART01`, `part_wire.py`). The capability is derived from the run seed and the spawn id and reaches only that child, in its environment record. `REGISTER` must match the recorded spawn, else a host fault. The PID and start-time check stays host-side, in the `*_host` sidecar (step 9d).
- **Partial launch.** If any participant fails to register within its wall budget, the coordinator stops every spawned process, registered or not, by ownership. The run is a host fault.
- **End.** There is no `END` message. A participant ends through the participant stop (a lifecycle `stop` in mission or finalization) or the drain phase (`TICK` with `end: true`); either way it flushes (log worker, pose-cadence buffers, backend log) and sends a `DONE` with `ended{flushed}` as its last message, which retires it at that tick. The coordinator then holds the clock until the process exits (a wall budget). An unflushed end, a nonzero exit after a flushed end or a missed exit budget is a host fault, followed by a stop by ownership; an exit or EOF without an ending `DONE` is a coordinator fault (step 9d, `lockstep_slice.md`).
- **Stopping.** By those records only, never by sysid pattern, slot or name.
- **WSL output.** Each WSL child's `wsl.exe` handle is held from right after its start to its close by a `lockstep_wsl_spawn.WslChild` in the owner's `held` list. A failed spawn after `go` was attempted stops the child by its record. Its stdout and stderr go to `<run>/lockstep_wsl_<role>_host.log`. Dropping the handle closes the pipe, and the next writes kill the child with `SIGPIPE` (step 9h).
- **Launcher rule.** The rule in `src/gcs/AGENTS.md:60-63` (no `gcs_launch` for eval-only runs) gets its lockstep exception in the same change (owner Q4).
