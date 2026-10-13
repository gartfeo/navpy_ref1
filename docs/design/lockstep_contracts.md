# Lockstep contracts

Step 6 of `deterministic_demo_eval.md`. These are the contracts the lockstep build (steps 7-10) implements. Each is pinned by a test now, or by a named acceptance test in the step that builds it.
- Prototype code: `src/navpy/sim/lockstep/`.
- Tests: `tests/modules/lockstep/`.
- Benchmark: `scripts/lockstep_baton_benchmark.py`.

## Execution model (chosen; re-measured at step 9)
**The baton.** Existing product threads run unchanged as managed OS threads, one at a time, on the sim clock (`baton.py`, `baton_sync.py`, `baton_queue.py`, `baton_asyncio.py`).

**Benchmark** (Windows, Python 3.11.9, 7 samples, wall µs per tick, median with range):

| Case | Baton | Focused synchronous |
|---|---|---|
| Mission upload, 200 items | 183 (169-276) | 102 (91-119) |
| 10 threads woken every tick (wake count asserted) | 183 (155-193) | n/a |
| 40 threads woken every tick | 788 (691-810) | n/a |

**Why the baton:**
- **Budget.** A lockstep tick is one ArduPlane main loop, 50 Hz by default (`SCHEDULER_DEFAULT_LOOP_RATE`), so about 20 ms of sim time. 10x real time leaves 2 ms of wall time per tick; 1x leaves 20 ms. The baton's overhead fits even with 40 threads waking every tick.
- **Integration cost.** The synchronous model is faster per tick, but each blocking workflow becomes a hand-written state machine. The prototype shows such a rewrite must also copy the product's float deadline arithmetic: with integer deadlines a retry fired one tick early, because `0.2 + 0.1` is `0.30000000000000004`. There are about 200 NavPy and 85 backend wait sites. The baton runs the real `MissionUploader`, `MissionInbox` and a FastAPI app unchanged; only their clock and primitives are bound (step 8).
- **Re-check at step 9.** The 1-UAV slice is re-measured. If it cannot reach 1x real time, the fallback is synchronous seams at the measured hot paths only, not a general rewrite.

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
- **Lifespan** runs explicitly: startup before the backend registers, shutdown in `END`.
- **Requests** run through an in-process ASGI call (`asgi_request`): no server, no sockets. A request delivered at tick k runs from tick k. Its response returns in the `DONE` of the tick its handler finishes in.
- **Cancellation** is an input. A cancel delivered at tick k cancels a handler still running then, because inputs come first. A response already in `DONE` k-1 stands.
- **No blocking MAVLink call on the loop thread:** such work goes to the managed executor (pinned with a request that sleeps in the executor).

## UART (pinned on the model: `test_reference_models.py`; firmware matches `uart_model.py` in step 7)
- **Ingress.** Bytes the coordinator sends for tick k enter a per-port FIFO that never drops. Each io tick admits `min(fifo, read-buffer space, 10000)` bytes, as the socket path does (`UARTDriver.cpp` `max_bytes = 10000`; `SIM_BAUDLIMIT_EN` stays 0, which is asserted in the defaults). The usable buffer is 16383 bytes, because a `ByteBuffer` keeps one free. A burst larger than the buffer arrives whole and in order over several io ticks.
- **Egress.** A write takes at most `txspace()` bytes; the writer drops the rest and the drop is counted. At the hook, everything written since the last frame is drained into the frame.
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

## Router (pinned: `test_reference_models.py`)
Emulates mavlink-router `d59ef94` (`/usr/bin/mavlink-routerd`), `Endpoint::accept_msg`, in the configuration `run_swarm.sh` gives it: `-s 255` (sniffer sysid 255, the GCS), no filters or endpoint groups, and no dedup period, so no dedup. Model: `router_model.py`.
- **Learning.** Each endpoint learns every `(sysid, compid)` it receives from.
- **Loops.** No endpoint gets a message whose source it has seen. This excludes the sender and stops loops, including ArduPilot's serial-to-serial forwarding echoes.
- **Sniffer.** An endpoint that has seen sysid 255 gets every other message, targeted or not. So the backend sees vehicle-to-vehicle traffic, as today.
- **Broadcast.** Target system 0, or no target field: every other endpoint.
- **Targeted.** Only endpoints that have seen the target system, and the component when it is nonzero. An unknown target is dropped and counted.
- **Forwarding.** Original packet bytes are forwarded unchanged, in fixed endpoint order: SITL 1-3, then the backend.
- **Decoder state** is kept per inbound stream across tick fragments (`mavlink_wire.StreamDecoder`).

## Bus adapter (wire pinned: `mavlink_wire.py`; the adapter in step 9)
One narrow bus replaces `mavutil.mavfile` for NavPy and the backend in lockstep.
- **Inbound.** Per tick bytes, decoded with persistent parser state and dispatched through the existing `_dispatch` fan-out (`mav_bus_reader.py:85-88`) in byte order, on the managed reader thread. After each packet the reader yields, as the wall reader does: it goes to the back of the runnable queue at the same t_k (`baton_sync.yield_now`, no timer round), so a woken consumer runs before the next packet.
- **Outbound.** A `MAVLink(file=ByteSink)` encoder per source keeps `seq` and MAVLink 2 framing as a real link would. Each participant's `(sysid, compid)` is its product value.
- **`mode_mapping()`** is `mavfile`'s: the fixed-wing table before any heartbeat, then the table for the latest probable vehicle heartbeat's type (`mavfile.probably_vehicle_heartbeat`), or PX4's map for a PX4 autopilot.
- **Heartbeat admission.** The bus admits as `MavBusHeartbeats` does today: any non-GCS, non-companion heartbeat from an eligible sysid counts. The router's compid-1 filter governs the vehicle's heartbeat state, not bus admission.
- **Subscriptions** are the existing `MavBusMembership` receivers.
- **Reservations.** `reserve(sysid)` returns the existing `MavBusLease`: `attach` adds the receiver, `release` drops the reservation. Opens share one bus per logical device (`lockstep:serial0` NavPy, `lockstep:serial1` backend), as `MavBus.get_or_create` does.
- **Close.** Releasing the last reservation closes the bus, following step 4: the reader stops first, then the bus counts as closed, and the link reports `closed`. A reopen is a new bus; bytes delivered while none is open are lost and counted (`lockstep_link.py`).

## Firmware helpers (specified; acceptance in step 7)
- **Barrier.** The frame's `helpers_quiet` flag is the init barrier. Only SITLs tick until every SITL reports it, and participants join only after. The firmware sets it once all three hold:
  - the scripting run list is empty: `aas_params.lua` is one-shot and has finished; `chute_deploy.lua` and `tune_pitch_roll.lua` are not loaded in lockstep;
  - `AAS_NAV_CGT` exists;
  - the logger backend is none (`LOG_BACKEND_TYPE 0`, BIN off per owner Q6) and FTP is refused.
- **Evidence.** Step 7's firmware review gives source lines for each helper's remaining loop: the scripting thread's idle wait, the logger's IO thread and the FTP worker. It shows that none can change vehicle state while the flag holds.
- **Out-of-loop advances.** The header counts clock advances not caused by the loop's own wait (`Semaphores.cpp:41-63`); nonzero is a host fault. The counter cannot see a helper that changes state without advancing time. That is why the barrier exists, and the helper-thread jitter hook (acceptance (c)) checks the remainder.

## Ownership (specified; implemented in step 7)
- **Spawn record.** Each child is recorded by pid and process start time the moment it is spawned, before it registers, in `<run>/lockstep_processes_host.json` (write, then rename). A WSL child is also recorded by its `/proc` cwd, which must lie under the run's root.
- **Registration.** Every participant connects to the coordinator and sends `REGISTER{run token, role, ordinal, pid, process start time}`. The coordinator validates it against the spawn record.
- **Partial launch.** If any participant fails to register within its wall budget, the coordinator stops every spawned process, registered or not, by ownership. The run is a host fault.
- **`END(k_end)`.** Each participant flushes (log worker, pose-cadence buffers, backend log) and acks. Processes are stopped only after all acks. A missing ack is a host fault, followed by a stop by ownership.
- **Stopping.** By those records only, never by sysid pattern, slot or name.
- **Launcher rule.** The rule in `src/gcs/AGENTS.md:60-63` (no `gcs_launch` for eval-only runs) gets its lockstep exception in the same change (owner Q4).
