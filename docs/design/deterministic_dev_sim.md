# One deterministic sim for development

Status: design, awaiting the owner. No code until it is approved.

## Goal
Owner, 2026-10-18: "we should have one deterministic sim for development. otherwise if we have bug in wall mode it will be very hard to reproduce in locked step mode."

So lockstep (`deterministic_demo_eval.md`, `lockstep_fleet.md`, `lockstep_contracts.md`) becomes the sim developers use every day. Two properties must hold together:
- **Realistic:** the timing, ordering, loss and noise effects that cause bugs in wall mode, and on hardware, also happen in lockstep.
- **Repeatable:** the same code, manifest and seed give the same digest, as the 5-run matrix proves today (`65aae9e1…`).

The way to get both is the one established deterministic simulation testing uses: every source of chaos is a model that draws from the run seed, in sim time.

## Established practice (checked first)
- **FoundationDB** runs the whole cluster single-threaded in simulation, with machine, disk and network faults injected on purpose, at short failure intervals and heavy load, and tens of thousands of runs each night. A failure is rerun by its seed ([testing](https://apple.github.io/foundationdb/testing.html)). Its `BUGGIFY` macro also makes product code take rare paths (slow, early timeout) when the seed says so (Will Wilson, Strange Loop 2014).
- **TigerBeetle's VOPR** runs the real code with a seed that also picks the fault parameters (packet loss, reordering, partitions, storage faults). Seed plus git commit reproduces a failure exactly. It runs all the time on 1024 cores ([safety](https://docs.tigerbeetle.com/concepts/safety/), [VOPR](https://github.com/tigerbeetle/tigerbeetle/blob/main/docs/internals/vopr.md)).
- **Antithesis** runs unchanged software in a deterministic hypervisor and injects network faults (latency, partitions, clogs), node faults (pause, kill), clock jumps and thread pauses throughout the run, interleaved with the workload ([fault injection](https://antithesis.com/docs/concepts/fault_injection.md)).
- **Swarm testing** (Groce et al., ISSTA 2012): each run turns a random subset of fault kinds on, instead of all of them every time. It finds more bugs, because some bugs need a fault to be absent.

What we take: (1) every effect is seeded and in sim time; (2) the seed also picks which effects are on and how strong (swarm); (3) faults run during the whole mission, then heal, and the mission must still finish (safety and liveness); (4) a failure is its seed plus commit; (5) many seeds, not one long run. What we do not copy: a hypervisor. Our baton already makes the threads deterministic; the models plug in around it.

## 1. The gap: what wall mode has that lockstep lacks
Lockstep today, from the code:
- **Links are perfect.** SITL to NavPy arrives in the same tick; NavPy to SITL is admitted by the same tick's reply (`COMMAND_ACK` in frame k+2); NavPy to NavPy goes NavPy, autopilot, router, autopilot, NavPy; it measured 4 ticks for every pair at step 10f-1 (`deterministic_demo_eval.md:28`), but backlog, carry-over and the autopilot's parse budget can change it, so the contract is reproducibility, not a hop count (`lockstep_fleet.md:124`). Queues never drop; there is no jitter, loss or bandwidth model (`lockstep_fleet.md:110,118`; `coordinator_router.py`). Only the caps 0xFFFF bytes per port per reply and `min(fifo, space, 10000)` per io tick exist (`lockstep_contracts.md:82`).
- **Compute takes zero sim time.** `monotonic`, `perf_counter` and `thread_time` all return `t_k` (`lockstep_runtime.py:52-64`; `deterministic_demo_eval.md:164`). A process can never fall behind, stall or starve.
- **Thread order is FIFO and switches only at primitives.** Runnable threads run first-in first-out; timers in `(due, park seq)` order (`lockstep_contracts.md:46`). Wall threads are preempted by the OS and the GIL every 5 ms.
- **Phase order is fixed.** The lockstep reader yields after every packet, so a woken consumer always runs before the next packet (`lockstep_bus.py:109-112`); the wall reader yields once per batch (`reader_batch.md`). An input on a waiter's deadline tick always beats its timeout (`lockstep_contracts.md:39-45`). Wall mode has neither guarantee.
- **The camera has no latency.** NavPy renders the frame from the pose it just received (`deterministic_demo_eval.md:100`).
- **SITL noise is one fixed sequence, not the seed.** Checked in the locked firmware (`2dbd7e38dd`): noise comes from glibc `random()`/`rand()` (`AP_Math.cpp:343-346`, `SIM_Aircraft.cpp:356-375`), and nothing calls `srandom` (only Lua's `math.randomseed`, `lmathlib.c:282`), so every SITL start uses glibc's default seed 1. `get_random16` has fixed seeds 1234/76542 (`AP_Math.cpp:332-338`). No `SIM_` seed parameter exists; the branch does not touch noise. Default ArduPlane noise is on (`SIM_BARO_RND` 0.2, `SIM_ARSPD_RND` 2.0, `SIM_DYN_GYR_RND` 0.1, `SIM_DYN_ACC_RND` 0.3); GPS noise, mag noise and wind are 0. Lockstep sets no `SIM_WIND_*` or noise parameter (`lockstep_wsl.py:43-49`). No deployed Lua script calls `math.random` (grep of `scripts/lua`), so the main-thread draw order is fixed.
- **Missing features.** Lockstep refuses MAVFTP, has no ESP32 HTTP simulator, and has no frontend (`deterministic_demo_eval.md:63-77`; `lockstep_slice.md:61`).

Effects that can cause a bug in wall mode but not in lockstep today, with the wall evidence:

| Effect | Wall evidence | Lockstep today |
|---|---|---|
| Process freeze (GC, host stall) | 90-220 ms per NavPy, 40-48 ms after the fix; a lone 90-117 ms gap 0.1-0.9 s into NAV (`message_history.md:4,65`; todo 31) | none |
| CPU starvation under load | Todo 65: coverage 12%, 740 ms gap, `NAV_FAIL: no_detection` while pytest ran; todo 33 (sim detector and gimbal fall behind) | none |
| Reader lag, queue build-up | 354 ms before the batch fix, 2.7 s behind at 3000 pkt/s (`reader_batch.md:5-14`); todo 38, 54 | reader always keeps up |
| A slow SITL backs up commands | SITL at 6.7x: command delay 11 s sim (`final_approach_delivery_fix.md:73-77`; todo 34) | all SITLs equal, by the barrier |
| Link delay and jitter | NavPy sends <13 ms; SITL reads UDP every 10 ms sim; budget 0.5 s jitter (`final_approach_delivery_fix.md:25,56,65`) | fixed hops |
| Packet loss | Todo 32: one vehicle lost all 12 `PARAM_SET` echoes; router fan-in drops (`deterministic_demo_eval.md:49,153`) | never |
| Thread interleaving | Which peer fails first changes per run (todo 31); logger close race (todo 45) | one fixed FIFO order |
| Reader batching, input versus timeout | A burst evicts the reply a waiter needed; a reply lands just after its timeout (`reader_batch.md`; todo 32, 38) | per-packet yield; input always first |
| Camera and gimbal latency | Real ZR10: median 99.8 ms, p99 121.5 ms, frame gap median 41.8 ms (`docs/measurements/zr10-latency-20260830/REPORT.md`) | 0 |
| Wall-sampled telemetry | GCS frames every 0.2 s wall = 2 s sim at 10x (todo 64) | sim-time |
| `SIM_SPEEDUP` 10 | NavPy cadences divided by it (`deterministic_demo_eval.md:65`); todo 22 (SNAP misses at 10x) | refused, 1x only |
| Wind and noise | Fixed in both modes, not seeded | fixed |

Two kinds of wall bugs need different answers:
- **Real-world bugs:** freezes, lag, loss, jitter, latency, ordering. Hardware has them too. Lockstep must model them.
- **10x artifacts:** a SITL slowed to 6.7x, cadences divided by 10, the GC pause being 10x longer in sim time. Hardware runs at 1x and does not have them. The model reaches them only through a stress profile (below), not the default.

One limit stays: the baton switches threads only at primitives (locks, queues, sleeps), so a race between two bytecodes with no primitive between them cannot happen in lockstep. Seeded thread order (section 2) covers races at primitives, which is where our documented races are (todo 31, 45). A bytecode-level race needs a code review or a wall run; it is listed as a known gap.

## 2. Seeded realism models
All models live on the coordinator side or in the participant loop, never in product code. Each draws from its own stream, `Random(run seed, model name, link or ordinal)`, so turning one model on or off does not shift another's draws. Delays are counted in ticks (about 20 ms; `t_k` is the SITL's `source_us`). Every delayed, dropped or reordered packet keeps a journaled disposition, as the gates require today (`lockstep_fleet.md:196`). With every model off, the digest must equal today's `65aae9e1…`.

A **profile** file (part of the input manifest) gives each model's range. Per seed, the swarm step draws which models are on (each on with probability 0.5 by default) and a value inside each range.

| Model | Where it plugs in | `hw` profile range (default) | Basis |
|---|---|---|---|
| Serial link NavPy↔autopilot: delay, jitter, bandwidth | `RoutedPorts.frames`, `outbound`, `ReplyQueues.admit` (`coordinator_router.py`, `coordinator_queues.py`) | delay 0-2 ticks, jitter 0-1 tick, 11.5 kB/s, loss 0 | **assumed:** 11.5 kB/s is the configured 115200 baud (`conn_args.py:15`); HITL uses USB CDC, so real throughput is not measured |
| Radio leg (router path, NAVLINK and GCS): delay, jitter, loss, bandwidth, outage | `RoutedPorts._route` | delay 2-8 ticks, jitter 0-5 ticks, packet loss 0-10%, 8 kB/s | **assumed.** Which radio carries this path on the aircraft is not settled. The RFD900 constants in code (340 B, 50-150 ms jitter, `serial_interface_rfd900.py:10-14`) belong to a separate delimiter-framed transport, so they are only a hint |
| Process freeze (GC, host stall) | participant loop: from the freeze tick the participant runs no thread (the holder finishes to its next park or yield, a baton boundary; runnable threads stay runnable), its inputs queue and sim time goes on; on resume, inputs go first as today and overdue timers fire in `(due, park seq)` order, deadlines unchanged | per NavPy, 2-6 ticks (40-120 ms), one every 5-60 s sim | lengths from `message_history.md:4,65` and todo 31; **the frequency is assumed** |
| CPU budget per tick | participant loop: at most N baton handoffs per tick, the rest carry over | off in `hw`; in `stress` 50-200 handoffs per tick | todo 65, 33; counts are deterministic, never wall time |
| Camera latency and frame drops | sim camera in the NavPy participant: capture the whole frame at tick k as today (pose, gimbal, optics, POIs, capture time; `finite_poi_projector.py:89,153`), publish it L ticks later with its capture time kept and a separate receipt time; skip frames | latency 4-6 ticks with a tail to 8 (80-160 ms), frame gap 2-3 ticks, drop 0-2% | approximation: the ZR10 report measures LED event to publication (median 99.8 ms, max 154.3 ms, gap median 41.8 ms), not exposure, and cannot see exposure drops (`REPORT.md:12,50`); drop rate assumed |
| Thread order | baton runnable queue: seeded order instead of FIFO among threads runnable in the same tick | on in half the seeds | FDB, Antithesis (scheduling is varied) |
| Reader service budget | lockstep reader: decode at most B packets per tick and yield per batch of b, the rest carry over to the next tick | off in `hw`; `stress` B 20-200, b 1-16 | wall reader handled 400-540 of 3000 pkt/s (`reader_batch.md:5-14`) |
| Input versus timeout at the same tick | participant loop: per seed, let due timers fire before that tick's inputs | on in a quarter of the seeds | the wall race behind a just-late reply |
| Wind and turbulence | SITL parameters `SIM_WIND_SPD`, `_DIR`, `_TURB`, `_T` per run | speed 0-8 m/s, any direction, turbulence 0-2 | 8 m/s used in tests (`test_vision_static_point_mass.py:47`); no real-flight data |
| Sensor noise | SITL `srandom(seed)` (firmware patch, step 2) plus the default `SIM_*_RND`; optional `SIM_GPS_NOISE` 0-1 m | default ArduPlane values | `SITL.cpp`; the patch is new |

The **`quiet` profile** is `hw` without loss, outages or faults; it is the one calibrated against quiet wall runs (section 4). The **`stress` profile** widens every range (freezes 5-12 ticks, radio loss up to 30%, the CPU budget on, one SITL's link held back to mimic the 6.7x case). The fleet clock stays one clock: the barrier needs frame k from every SITL with the same `source_us` (`coordinator_frames.py:92-97`). So "a slow SITL" is modeled as its serial bytes, both ways, being held d ticks at the queue seams, not as a vehicle clock that lags. It is the profile used to reach 10x artifacts.

**Faults** are scheduled events in the same profile, drawn per seed, and healed before the mission's last phase (liveness):
- link drop: one link at 100% loss for 1-10 s;
- late heartbeat: a freeze long enough to pass a liveness timeout (1-3 s);
- NavPy restart: the coordinator stops a participant and respawns it with `generation + 1`, which `REGISTER` already carries (`lockstep_contracts.md`, Ownership);
- radio partition: one UAV hears no peer for 2-20 s.

The firmware RNG patch: an env var `NAVPY_STEP_RNG_SEED`, set per instance from the run seed and sysid, and `srandom(seed)`/`srand(seed)` once in SITL start, before any sensor draw; also re-seed `get_random16`. An env var, not a `SIM_` parameter, because parameters load after the first draws and would persist in `eeprom.bin`. It is a firmware branch change and a new `lockstep_firmware.lock.json`.

## 3. Reproducing a wall bug in lockstep
Options:
- **A. Record and replay a wall run's schedule.** Record each packet's measured delay and the thread switches, then force them in lockstep. Thread switches cannot be recorded from Python without heavy tracing, and the wall run is at 10x, so its delays do not map to 1x sim time. Large and incomplete.
- **B. Fit a profile, then search seeds.** A tool reads the wall run's logs (cadence gaps, freezes, reader lag, which messages were lost or late), sets the profile's ranges to match, and runs seeds until the failure's signature appears (the same gate fails, for example todo 65's coverage drop or `NAV_FAIL: no_detection` on any UAV). The found seed is then a permanent regression seed.
- **C. Write the fault by hand.** From the wall log, write a fault event at the time and place seen (for example a 740 ms freeze of UAV 3 at NAV + 0.5 s). Fast when the log names the cause.

**Recommended: B, with C as the first try when the log already shows the cause.** Both end in a seed (plus, for C, a fault file) that fails the same way every run. If neither reproduces the bug within the search budget (say 200 seeds), it is logged as an **unreproduced bug** in `architecture_todo.md`, with the possible causes: a profile that does not fit, too few seeds, a different mechanism, or a missing model. The sim changes only once mechanism evidence (from the wall logs or a wall rerun) shows which model is missing. That loop keeps lockstep realistic over time.

## 4. Gates
- **Determinism gate (unchanged):** the 5-run matrix (`eval_gcs_demo_lockstep_matrix.py`: plain x2, host delays x2, CPU load) must give one digest, passed reports and byte-equal compared artifacts (`eval_gcs_demo_lockstep_matrix.py:94-100`), now with the `hw` profile at a fixed seed as well as with every model off.
- **Development gate (new): a seed sweep.** One run is about 72-75 s wall (plain matrix runs, `.logs/s11/matrix2/*/lockstep_run_host.json`), 8.5 ms per 20 ms tick. Models add some cost (to be measured in each step).
  - per change: the regression seeds plus 10 new seeds of `hw`, about 15 min run one at a time;
  - nightly: 100 seeds of `hw` and 50 of `stress`, about 3 h;
  - parallel runs on one host are not measured yet; the matrix runs one at a time.
- **Outcomes**, each journaled:
  - *injected:* a fault the profile scheduled, with its id and recovery deadline. It is expected; the run fails only if the mission does not recover by that deadline.
  - *product failure:* any scoring gate (mission success, assignments, approvals, SNAP, docking contact, separation, coverage), a product fault, a missed recovery deadline, or a participant exit nobody scheduled.
  - *host fault:* only with harness evidence (wall budget, coordinator fault, WSL process lost with its record). Reported apart and rerun.
  - Each failing seed is rerun once. The same digest confirms it; a different digest is a determinism bug, which is worse. A rerun adds to the record and never replaces the first result.
- **Realism check (each model step).** Two checks, kept apart:
  - each model's own output matches its range or measured distribution (a unit test on the drawn delays, freezes or frame gaps);
  - a `quiet` profile (delays, jitter, camera latency and measured-length freezes; no loss, outages or faults) must land inside the quiet wall runs' cadence and coverage (`.logs/e10/r2`: gap median 60 ms, longest 140-181 ms, coverage 0.93-0.95). The fault-bearing `hw` and `stress` profiles are judged by the outcome and recovery gates instead, and `stress` must reproduce todo 65.

## 5. Watching it live
So developers use it day to day, lockstep needs the live app. What it needs:
- **Pacing to 1x.** The coordinator sleeps after each tick until `wall_start + t_k`. It sits where host delays already sleep (`coordinator_run.py:242,256`) and stays out of the manifest, as host delays do, so the digest does not change. If the run is slower than 1x it simply runs slower. Pause is not in v1: the SITL waits at most 30 s for a reply (`STEP_DEADLINE_US`, `lockstep_wsl_sitl.py:63`).
- **A read-only live relay.** The backend runs in-process with no socket (`lockstep_asgi.py:1`), and a subscriber inside it would change the digest. The coordinator already journals every backend broadcast in each `DONE` (`event_sink.py:5-6`). A relay process outside the baton gets those events from the coordinator and serves `/ws/telemetry` (and the read-only REST the frontend needs) on the eval slot's ports. The frontend connects to it unchanged.
- **Frontend fixes for sim time.** The armed token is checked against `Date.now()` (`useFullParams.js:454,463`) and would always look expired, because sim time starts 2026-01-01; it must use the backend's `ts`. The confirm card auto-approves on a wall-time countdown (`TaskConfirmCard.jsx:51-88`); in a lockstep view it must count in sim time or be off. Map animations on wall time are display only and stay.
- **Operator input.** v1 keeps the scripted operator (`eval_gcs_demo_confirmation.py`). v2: a button press goes to the relay, the coordinator delivers it in the next tick (`deliver_operator_requests`) and journals it as `(tick, request)`; the run's input file then replays it exactly.
- **Launcher.** A frontend for the eval band needs ports (the eval slot's own 3000+N and 8000+N, or a new band), a change to owner ruling 4 ("the eval band, no frontend", `lockstep_slice.md:61`), and todo 55 fixed so `gcs_stop.py --eval` can stop it.

## 6. Steps
Each step is small and has its proof. Every step keeps the digest with all models off (`65aae9e1…`, or a documented re-baseline).

| Step | Change | Proof |
|---|---|---|
| 1 | Profile file, per-model seed streams, swarm on/off draw; no model yet | unit tests; matrix digest unchanged |
| 2 | Firmware: `NAVPY_STEP_RNG_SEED`; lock bump; wind and noise parameters from the profile | same seed: equal baro and airspeed streams; seeds 1 and 2 differ from frame 1; matrix passes |
| 3 | Link model: delay, jitter, loss, bandwidth, outage at the router and queue seams, every disposition journaled | per-link unit tests with mutations; matrix passes with `hw` |
| 4 | Process freeze and CPU budget in the participant loop | a regression test where work is already runnable when the freeze starts and timers come due during it; `stress` reproduces todo 65's mode; matrix passes |
| 5 | Camera latency and frame drops | cadence and SNAP inside the quiet wall range |
| 6 | Seeded thread order in the baton; reader service budget; input-versus-timeout order | baton tests: order differs by seed, equal for one seed; regression tests where a burst evicts a reply and where a reply lands on its timeout tick; matrix passes |
| 7 | Faults: link drop, late heartbeat, NavPy restart, partition; heal before the last phase; journaled with id and recovery deadline | a fault enters the default sweep only with an executable recovery assertion that passes; a fault without one stays experimental (off by default) with a named todo; an unscheduled exit is a product failure |
| 8 | Seed sweep runner, regression seed list, failure classes | sweep of 10 seeds; a planted bug is found and reruns with one digest |
| 9 | Wall-bug tool: fit a profile from wall logs, seed search | todo 65 and todo 31 reproduced from their wall logs |
| 10 | 1x pacing, live relay, frontend sim-time fixes, launcher ports | `live-tester` watches a lockstep run in the app; digest equal to an unpaced run |
| 11 | Recorded operator input (v2) | a button press replays to the same digest |
| 12 | Make lockstep the default dev sim in the docs and `src/gcs/AGENTS.md` | owner sign-off |

## Wall mode after this
Wall mode stays, but not for development:
- the investor demo and live operator sessions with real people;
- HITL and hardware (real time cannot be stepped);
- a smoke check before hardware, run now and then.

A bug seen only in wall mode must first be reproduced in lockstep (section 3). If it cannot be, it stays an unreproduced bug in `architecture_todo.md`; it is recorded as a model gap only when mechanism evidence shows one.

## Open decisions for the owner
1. Approve the firmware change (seeded SITL noise) and a new firmware lock.
2. Lift ruling 4 ("no frontend" in the eval band) for a read-only live relay, and choose its ports.
3. Default target: the `hw` profile (hardware at 1x) as the default and `stress` for 10x-style bugs, as proposed; or tune the default to the wall eval's numbers.
4. Gate cost: 10 seeds per change (about 15 min) and 150 nightly (about 3 h) on this PC.
5. Ranges without data: which radio carries NAVLINK on the aircraft, its loss and air rate, camera drops, wind. Use the assumed ranges above until a field test measures them?
6. Interactive operator input in lockstep (step 11): now or later.

## Known gaps
- Races between bytecodes with no primitive between them (section 1).
- Delays are whole ticks (about 20 ms). `SCHED_LOOP_RATE 100` would give 10 ms ticks at about twice the cost (`lockstep_fleet.md:283`).
- MAVFTP and the ESP32 HTTP simulator are wall-only.
- Detector compute time on the Orin Nano is not measured; the freeze and budget models stand in for it until HITL measures it.
