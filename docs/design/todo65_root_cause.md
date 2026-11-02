# Todo 65 at its root: what starves `stress` seed 40

Status: built 2026-11-01 on main `5179baab` (results at the end). Design rev 3.2: rev 3.1 approved by Codex round 4; rev 3.2 only adds what main's PR #44 changed (below, "Overlap with PR #44"). Rev 1 proposed not charging a yield's resumption; Codex round 1 blocked it (it removes the tick's work bound). Rev 2 drew `batch` from 8-16; Codex round 2 found that range uncalibrated, so rev 3 drops the count for the product's drain rule. Brief: `.logs/load-root-cause/brief.md` (main checkout). Evidence: `.logs/lrc` in the main checkout, with the hook patch and each variant's flags (`VARIANTS.md`).

## Question
`stress` seed 40 (CPU budget 4 handoffs per tick per NavPy) reproduces todo 65: no UAV docks, median observation gaps 0.40-0.42 s, longest 0.88-1.48 s. The candidate causes were the NavPy reader at about 3000 packets/s (todo 66), sim parts on wall slots (todo 33), work on the reader thread (todo 38) and SITL/WSL starvation (todo 65 (a)).

## Method
A temporary counter (removed before commit) wrote, for every tick of every participant, the cap, the thread of each baton handoff, the packets the lockstep reader dispatched by type, and what was left runnable or due. The counter does not change the run: seed 40 kept main's digest `6777f210…`. Each variant below is one seed-40 run with one change, all else equal.

## Evidence
**Demand, uncapped** (`nocap40`, passes, all dock; 8163 ticks of one NavPy): 9.21 handoffs per tick (median 9, p90 13). By thread: MAVLink reader 4.19, command session 1.84, main thread 1.01, sim camera 0.75, nav loop 0.49, sim gimbal 0.49, the rest under 0.2. The reader dispatches 3.20 packets per tick (about 160/s per NavPy) and takes one handoff per packet plus about one per tick for the delivery: 34214 reader handoffs for 26089 packets. Packets: ATTITUDE and GLOBAL_POSITION_INT 39 Hz each, and 17 types at 2-4 Hz that no NavPy or GCS source names (AHRS2, SIMSTATE, AOA_SSA, ESC_TELEMETRY_1_TO_4, AHRS, POWER_STATUS, MEMINFO, SERVO_OUTPUT_RAW, RAW_IMU, SCALED_IMU2/3, SCALED_PRESSURE/2, SYSTEM_TIME, VIBRATION, POSITION_TARGET_GLOBAL_INT, id 295), 26 % of packets (28 % in `diag40`).

**Cap 4** (`diag40`): the cap binds on 99 % of ticks, and due timers are left waiting on 92 %. Share of each thread's own handoff demand that it gets:

| Thread | Uncapped | Cap 4 | Served |
|---|---|---|---|
| MAVLink reader | 4.19 | 3.23 | 77 % |
| nav loop | 0.49 | 0.11 | 22 % |
| sim gimbal | 0.49 | 0.10 | 20 % |
| main thread | 1.01 | 0.16 | 16 % |
| sim camera | 0.75 | 0.09 | 12 % |
| command session | 1.84 | 0.10 | 5 % |

The reader keeps most of its demand; the timer-driven observation path (camera, nav loop, command session) gets 5-22 %.

**Mechanism.** Before this change the lockstep reader yielded after every packet (`ReaderService.after_packet`; it still does without a reader budget) so that a woken consumer runs before the next packet, and the CPU budget charges each resumption as a handoff (`BatonRuntime.run_until_quiescent`). So each packet costs a turn, and a tick's 3.2 packets take most of a 4-turn cap. A tick runs its inputs to quiescence before its timers (`BatonRuntime.tick`), so a capped tick seldom reaches its due timers. This is the throttle the wall reader had until `reader_batch.md`: its per-packet `rt.sleep(0)` held NAV reader lag up to 354 ms; draining every decodable packet and then yielding once brought it to 6-16 ms (about 300 yields/s for 3000 packets/s, so about 10 packets per yield). The product reader has batched since; lockstep kept the per-packet yield on purpose (`reader_batch.md`, "Lockstep is unaffected"), a parity gap the step-4 CPU budget then turned into a throttle. The cap range 4-12 was chosen so that 4 reproduces todo 65 (`deterministic_dev_sim.md` section 2), not measured against wall CPU, and its unit (handoffs) is not CPU time; this design makes no CPU-share claim.

**Variants at cap 4, seed 40:**

| Run | Change | Result |
|---|---|---|
| `diag40` | none | fails: no dock, median gaps 0.40-0.42 s, longest 0.88-1.48 s |
| `drop40` | the 17 unread types dropped before the reader | fails narrowly: median 0.20 s, longest 0.50-0.60 s, 1 docks |
| `exdrop40` | that, and sim camera and gimbal turns not counted | fails: median 0.16-0.22 s, longest 0.30-0.52 s, 2 dock |
| `batch40` | reader yields once per delivery (wall-like) | all 3 fly and dock; observation gates pass (median 0.14-0.16 s, longest 0.30-0.38 s); fails only todo 121 (a)'s router-filtered NAVLINK frame |
| `free40` | reader's per-packet resumption not charged (rev 1's fix) | UAVs 1-2 dock; UAV 3 never launches (backend `EKF status stale`; see todos) |
| `all40` | drop, batch and exempt | passes every gate |
| `fair40` | due timers woken at the end of a capped tick | fails earlier: NAV 1 sends a second confirm request for P1 (see todos) |

`batch17` and `batch24` (the other todo-65 seeds) no longer collapse (median 0.14-0.16 s); they fail on one 0.54 s gap, todo 121 (a) frames, two forwarding errors from another branch of that check (packet `['sitl1', 1, 'lockstep:serial0.1', 20112]` from system 27 and its SITL 3 twin) and a SNAP of 0.70 m without `DOCKED` on `batch17`, and on seed 24 a swarm assignment-evidence failure with a 51.08 deg roll step (it matches open todo 115 by its log lines, not traced).

**The listed candidates on seed 40:**
- Todo 66 (packets): real, second. Dropping the unread 26-28 % moves the median from 0.42 to 0.20 s but does not pass. In wall runs the cut saves real CPU; in lockstep compute takes no sim time, so it helps only through the turns.
- Todo 33 (sim camera and gimbal on the NavPy's budget): small (`exdrop40` against `drop40`).
- Todo 38 (work on the reader thread): cannot act in lockstep, where a callback takes no sim time.
- Todo 65 (a) (SITL/WSL starvation): absent; the SITLs are not capped and the barrier holds them equal.

## Cause
On seed 40 the observation path starved because the lockstep reader still yielded after every packet, which the product reader stopped doing in `reader_batch.md`, and under a CPU cap each of those yields cost a turn. With inputs ahead of timers, the camera, nav loop and command session got what was left. Product code is not the main cause on seed 40.

Wall todo 65 has its own causes outside lockstep: OS scheduling (`bounded_cpu_share.md`, merged in PR #44), the reader's real per-packet CPU (todo 66, closed by PR #44's stream cut), and the closest-approach maths on the reader thread (fixed on main 2026-10-27, `fc57372c`; todo 65 (b) in NAV). It stays open until the wall fixes pass under load.

## Fix: a capped NavPy's reader batches as the product does
Dev sim step 6 (on main) added the mechanism: with the baton's `reader_budget` the lockstep reader dispatched at most `packets_per_tick` per tick and yielded once per `batch` packets. Each yield was still one charged handoff, so the tick's work stayed bounded (Codex round 1, finding 1). What was wrong was `batch`, a packet count the product reader does not have. The product batch (`mav_bus_reader.py:101`) ends when no packet is decodable or when it has run one scheduler wall slot. In lockstep compute takes no sim time, so the time bound never ends a batch; only exhausted input does, plus step 6's `packets_per_tick`. A drawn `batch` below that put back the per-packet throttle the product dropped (seed 17 draws 1). The "about 10 packets per yield" of `reader_batch.md` came from an unbounded test and is not used.

This change, on top of step 6:
1. **The reader's turn ends as the product's batch does:** when its delivered packets are drained (it then waits on input, the handoff that ends the turn) or when `packets_per_tick` is reached. The `batch` parameter goes from `reader_budget`, `ReaderService` and `stress`. No count is drawn, so no range needs calibrating. `packets_per_tick` (4-16 then, 6-16 since the floor below) stays: a free ceiling on dispatches per tick, chosen from NavPy's demand and not fitted to the wall reader (todo 199), which keeps the reader's work per tick finite.
2. A profile check refuses `cpu_budget` on without `reader_budget` on: a capped NavPy must not use the per-packet reader. Only `stress` has `cpu_budget`, so no other profile changes.
3. Docs: `deterministic_dev_sim.md` section 2 (the CPU budget row: a reader turn serves its delivery up to `packets_per_tick`; the calibration note: 4-12 was set with the per-packet reader) and `reader_batch.md`'s lockstep line.

Evidence that this is the drain rule and not a tuned count: a `batch` above the drawn `packets_per_tick` never reaches a batch yield, so it behaves as the drain. At equality it is not the same: `ReaderService.after_packet` then still yielded after the last allowed packet, and a reader picked again in that tick takes one more charged handoff (Codex round 3). Seed 24 (`packets_per_tick` 10) gives the same digest, `be39b8e3…`, with `batch` 12 and 16, both above. Seed 40's run below drew `packets_per_tick` 16 with `batch` 16, so it supports batching but not the exact drain schedule; the acceptance runs are on the implementation.

Not changed: the per-packet yield without a reader budget (`none`, `quiet`, `hw`; no cap there, so it costs no turn, and their digests stay). Lockstep and wall reader scheduling still differ there; that parity gap goes to a todo, since closing it moves every baseline digest.

**Joint accounting with step 6** (Codex round 1, finding 4). Under a cap C a tick runs at most C handoffs. A reader turn serves what is queued, and the reader at most `packets_per_tick` per tick; both are upper bounds. The reader is not guaranteed its share: other runnable threads (seeded order) and, with `timers_first`, due timers can take the cap first, and the unserved packets wait. A reader past `packets_per_tick` waits on a timer that a delivery does not wake, so it has no turn of its own and competes for picks under the cap (todo 200). The combination tests below pin a used-up cap keeping a reader past B parked (also with a delivery while it is parked), a reader that competes once its timer fires, and `timers_first` firing it first.

## Not in this change (todos)
- **Todo 33.** The sim camera and gimbal are world, not NavPy CPU, but stand in for the detector's compute. Small effect; no change.
- **Todo 196: lockstep reader parity without a budget.** Lockstep without `reader_budget` yields per packet; the product drains and yields once. Closing it moves the `none`, `quiet` and `hw` digests.
- **Todo 197: launch refused on a stale EKF report.** `free40`: the backend's EKF gate (`ekf_status_gate.py`, stale after 4 report periods) refused V3's arm at 00:02:05, and the session aborted. Not traced: whether the loss streak on the 2 Hz report or the free resumptions' order caused it. It goes to a todo with the run named; `free40` is not the fix.
- **Todo 198: a second confirmation round under load.** `.logs/lrc/fair40`: NavPy 1 sent "Sending confirm request for P1" twice, and the operator refused a second round. That run has no `timers_first` model. Reader lag is the better-supported reading, not proven: the approval went out about 14 s before the vehicle's timeout, with NavPy lag p50 about 55 s over the run and about 107 s on UAV 1 at that moment, and `.logs/lrc-review/cc/s30b16`, with `timers_first` on and no lag, has one round per UAV. A timers race is only a possibility.

## Overlap with PR #44 (merged to main as `f47d10ea`, 2026-10-28)
- No lockstep code: PR #44 changes wall runs only (CPU share, process priority, spawn); nothing under `src/navpy/sim`. The fix above does not touch its files.
- Todo 66's stream cut (`1b74159e`) sets `SR0_EXTRA1` and `SR0_RAW_SENS` to 0 in `eval_gcs_demo_sitl_root.py`, which lockstep uses too: about 28 of 149 packets per sim second go (19 %, 10 of the 17 unread types). Every number and digest above, and the capacity digest, predates it and is not comparable with later runs. With the cut alone seed 40 still fails, median gaps 0.26-0.36 s (`base40`, below); `drop40`'s 0.20 s dropped all 17 types, more than the cut.
- Todo 65's text on main now has PR #44's bounded-share paragraph; this change's todo-65 entry sits before it.

## Checks
- Baseline on main after step 6 (with the cut), before the fix: `stress` seeds 40, 17 and 24 (Results below).
- Step-6 head unchanged (`aee5a1fc`, `.logs/lrc/s6`), all at cap 4:
  - seed 40, batch 7: all 3 fly and dock, median gaps 0.14-0.16 s, longest 0.30-0.38 s; fails only todo 121 (a)'s router-filtered NAVLINK frame;
  - seed 24, batch 4: the same (median 0.16 s, longest 0.34-0.36 s), only todo 121 (a);
  - seed 17, batch 1: collapses as on main (median gaps 0.44-1.08 s, longest 0.90-1.08 s, no UAV docks, roll step 67.8 deg). The per-packet throttle, through step 6's own draw.
- With `batch` 8-16 on step 6's head (`.logs/lrc/s6/b*`, patch `batch8_16.patch`; seeds 40, 17, 24 draw 15, 8, 12), all at cap 4:
  - seed 40: **passes every gate**; all 3 dock, median gaps 0.14-0.16 s, longest 0.30-0.36 s;
  - seed 24: all 3 dock, observation gates pass (longest 0.30-0.40 s); fails only todo 121 (a) frames;
  - seed 17: all 3 pass the observation gates (median 0.14-0.16 s, longest 0.32-0.44 s; was 0.44-1.08 s and 0.90-1.08 s). UAV 3 docks; UAVs 1 and 2 end in `NAV_FAIL: no_detection` at SNAP 0.8 and 0.6 m, plus todo 121 (a). Class: depends on the cap (with no cap it passes, in Results), cause not traced; one run is one schedule.
- Near-drain on step 6's head (`batch` fixed at 16, the top of `packets_per_tick`, so exact drain only where `packets_per_tick` is below 16: seeds 17 and 24, not 40; `.logs/lrc/s6/d*`, `drain.patch`), all at cap 4:
  - seed 40: **passes every gate**; all 3 dock, median gaps 0.14-0.16 s, longest 0.32-0.36 s;
  - seed 24: same digest as with `batch` 12 (above); only todo 121 (a) frames;
  - seed 17: all 3 pass the observation gates (median 0.16 s, longest 0.30-0.42 s). UAV 1 docks but fails the roll-step gate (54.8 deg); UAVs 2 and 3 end in `NAV_FAIL: no_detection` at SNAP 1.3 and 2.3 m. Class: depends on the cap (with no cap it passes, in Results), cause not traced; one run is one schedule.
- Profile test: `cpu_budget` without `reader_budget` is refused; `reader_budget` has no `batch`.
- Lockstep bus regression (step 6's tests changed): under cap 4, a reader given a 3-packet delivery takes one handoff for it, and a due timer's thread runs in the same tick; a delivery of exactly `packets_per_tick` takes one handoff and no extra yield; a backlog several times `packets_per_tick` carries the rest, in order, over the next ticks.
- Combinations, as mechanism tests in `test_scheduling_models.py` (no loss, order kept, no hang; one result per seed): reader budget with `timers_first` at B 4 and 16 with no cap, and at B 4 under a used-up cap; with seeded order; with a cap other threads use up, at B 4 (pins todo 200, with a delivery each tick) and with a reader below B; a reader past B competing with timer threads; with freeze and resume. Separately, a close while the reader waits out its budget ends the reader in that tick.
- On the no-`batch` implementation: `stress` seed 40, all three UAVs fly and pass the observation gates; the rerun gives the same digest. Seeds 17 and 24 reported. Any other failure is classed with its todo.
- `none`, `quiet`, `hw` digests unchanged from step 6's; `stress` matrix one digest, result reported.
- Full pytest; known failures named.
- Capacity recheck on the final branch: one lone `hw` seed-1 run and N = 8, digests equal; section 4 gets the new digest.

## Parallel capacity (brief item 2)
The lockstep demo eval takes no evaluator lock (`eval_gcs_demo_cli.py:175-176`); each run claims its own eval-band slot (40-84), WSL root and run folder, so N runs start from one worktree on one commit. Result (`hw` seed 1, N = 1, 2, 4, 8, 12): in `deterministic_dev_sim.md` section 4.

## Results on the implementation (2026-11-01, on main `5179baab`; `.logs/lrc2` in the main checkout, cap runs `.logs/lrc2/q4`, overlap `.logs/lrc2/overlap`)
Pre-fix baseline on that main (`base*`; seed 1's from `lrc2/overlap/real/mx_stress`), all with the stream cut. The before runs of seeds 40, 17 and 1 had the reader budget off; seed 24 had it on with batch 4:
| Seed (cap, B) | Before | After |
|---|---|---|
| 40 (4, 16) | `19d8d304…`: no UAV ends docked by the gates, median gaps 0.26-0.36 s, longest 0.72-0.98 s, coverage 0.01 or less, roll step 70 deg | `297878ae…`, rerun equal: **passes every gate**, all dock, median 0.14-0.16 s, longest 0.28 s, coverage 0.50-0.66 |
| 17 (4, 14) | `152f19c6…`: collapses, median 0.30-0.33 s, longest 0.56-1.06 s, no dock | `70b50252…`: observation gates pass (median 0.16 s, longest 0.30-0.48 s). UAV 1 docks; UAVs 2 and 3 end in `NAV_FAIL: no_detection` at SNAP 1.6-1.7 m without `DOCKED`. With no cap the seed passes every gate (below): it depends on the cap, cause not traced; one run is one schedule |
| 24 (4, 10) | `689019bb…`: median 0.14-0.16 s; fails swarm assignment evidence, a swarm task drop that moves with the schedule, not the per-packet yield (batch 4); it matches open todo 115 by its log lines, not traced. The fix did not address it; the schedule moved | `85ac3822…`: all dock; fails only todo 121 (a)'s router-filtered NAVLINK frame |
| 1 (6, 10) | `21988d91…` (matrix fails) | `7359cec3…`: **passes**, all dock; 5-run matrix one digest |

Matrices with the fix: `none` `b9bb2e63…`, `hw` and `quiet` seed 1 `eb263f68…`, all equal to main's (the code path without a reader budget is unchanged). The first solo `hw` matrix failed its artifact comparison on `input_manifest.json`: tracked-diff and untracked hashes changed between runs (`.logs/lrc2/fail1/mx_hw_docedit`), which I attribute to my doc edit during the run (the edit and its timing are my observation, no retained log); the final matrix ran with the tree frozen; `stress` seed 1 `7359cec3…`, one digest, passes. Full pytest: 10178 passed, 8 failed, all known: 4 `test_board_detector.py`, and 4 `test_gen_codex_agents.py` from stray untracked TOML files in the fix's worktree.

**Does `stress` still bind? Yes at the caps counted (2, 3, 4 and 6), weakly at 8; 230 of 400 seeds draw from 8 up. Whether its range still reaches failures is known only for the seeds run so far, as single runs: 1, 17, 24, 30, 32, 38, 40, 86, 180, 212, 267, 317, 324** (todo 162). Each NavPy's handoffs per tick, with temporary counters in a helper checkout (seed 40's digest unchanged by them). All rows exclude freeze ticks (cap 0):
| Run | Ticks at the cap | Ticks ending with work still runnable | Handoffs per tick |
|---|---|---|---|
| no cap, seed 40 and 1 (freeze ticks excluded: 0.42-0.51 % of ticks end runnable, all in freezes, for example 35 of 8,281 on `q4/s40none` UAV 1) | - | 0 | mean 5.8-6.1, p90 8, p99 10-11, max 36-41 |
| cap 8, seed 180 at forced B 6 (`cfw180_b6_t1`) | 13 % | 2.5-3.3 % | mean 5.9-6.0 |
| cap 6, seed 1 | 61-65 % | 10-14 % | mean 5.4 |
| cap 4, seeds 40, 17 | 97 % | 78-79 % | mean 3.9 |
| cap 3, seed 40 | 98 % | 76-77 % | mean 3.0 |
| cap 2, seeds 40, 17 | 99 % | 80 % | mean 2.0 |

Effect on seed 40 by cap: none: coverage 0.86-0.92, median 0.12 s, passes. Cap 4: coverage 0.50-0.66, median 0.14-0.16 s, passes. Cap 3: coverage 0.33-0.37, median 0.16-0.18 s, one roll step 50.84 deg over the 50 deg gate. Cap 2: coverage 0.02-0.06, median 0.18-0.23 s, roll steps 53-70 deg, 2 of 3 fail to dock: todo 65's collapse. Seed 17 at cap 2: coverage 0.03-0.06, median 0.20-0.22 s, roll steps 60-70 deg, 3 of 3 fail to dock, and a confirmation-timing error on vehicle 1 (k 11245, 11244, 11247). Cap 1: no scored result. The run ends `HostFault: generation 1 missed its stop budget`, a sim-time limit (500 ticks after the stop, `STOP_BUDGET_US`), with the same digest `c121c3fe…` on all three runs (`.logs/lrc2/q4/s40c1`, `.logs/lrc3/c1_40_t1`, `t2`). So it is deterministic and the host label is wrong (todo 202); why NavPy 1 misses the stop is not traced beyond one handoff per tick. So the cap still costs work and lowers coverage (a diagnostic, not a gate). On seed 40 the gates pass at 4, first fail at 3 and collapse at 2, below the drawn 4-12. Seeds 1 (cap 6), 24 (cap 4) and 40 pass the observation gates as drawn, and seed 17 (cap 4) passes them but two UAVs end in `NAV_FAIL: no_detection`. Runs inside the drawn range that miss a dock: seed 17 cap 4 misses, uncapped docks; seed 180 cap 8 misses (`.logs/lrc-review/cc/s180`, `s180b16`), uncapped docks (`s180nc`); seed 30 (wind 0.76 m/s, turbulence 1.92 m/s) misses at cap 5 and with no cap (`s30b16`, `s30b16nc`) and docks only with wind off (`s30b16nw`); seed 212 at cap 4, B 4, in "Stress seed 212" below. These are single runs, and docking can flip on small schedule changes, so this is classification only, with no accuracy work: they depend on the cap or the wind, causes not traced except seed 212's. None is seed 40's collapse. Other seeds are unknown; the seed sweep (step 8) measures them. The packets themselves stay free, as todo 162 says. Options for the owner, in todo 162.

**Plan rule (owner, 2026-11-01: "Yes, change the rule"):** `stress` need not reproduce todo 65. Seed 40 is a regression seed that must keep passing its gates; todo 65 stays open for the wall, and step 9 reproduces its remaining causes from wall logs (`deterministic_dev_sim.md` section 4).

**Step 6's half-on reader budget:** gone. `stress` always draws `cpu_budget`, the new check refuses it without `reader_budget` at p_on 1, so `stress` has it on in every seed. Step 6 kept it at half only so that seeds 1 and 40 would keep reproducing todo 65; that reproduction was the artifact this change removes. `packets_per_tick` draws 6-16 since the floor below; `hw` and `quiet` have neither model. Mechanism coverage of the old throttle stays in `test_a_per_packet_reader_spends_the_cap_and_the_due_timer_waits`.

**Host fault in overlapping matrices.** Four matrices started at once (mine, by mistake) gave `hw` and `quiet` exchange-deadline faults in their CPU-load cells; see `deterministic_dev_sim.md` section 4. Class: host fault. The fix cannot cause it: the journal is byte-equal to `plain-a` up to the fault, those profiles have no reader budget, and the same overlap on main without the fix faults at the same tick.

Capacity recheck on the fix: 8 `hw` seed-1 runs at once, all `eb263f68…`; 196-198 s in the lockstep run (`wall_s`), 214 s for the whole command with setup and scoring (`.logs/lrc2/cap8/summary.txt`).

**Later runs on `87f94c5f`** (`.logs/lrc3` in the main checkout):
- Seed 32 (cap 5, B 4, `timers_first` on), `s32_t2`, `d36c6d55…`: **passes every gate**, all dock, median 0.14-0.16 s, longest 0.32-0.46 s, coverage 0.38-0.58, SNAP 9.0, 1.6, 2.0 m, NavPy lag max 360 ms. The first run of the reader budget with `timers_first` to pass; `s30` and `s30r` at the same cap and B finished earlier and fail on a second confirmation round. Backend lag max about 13 s, which no gate sees (todo 199).
- Seed 17 with no cap (B 14), `s17none_t1`, `d8116c4f…`, the review's digest: **passes every gate**, all dock, median 0.12 s, longest 0.26-0.42 s, coverage 0.81-0.93. So seed 17's misses at cap 4 (`70b50252…`, on the pre-commit tree; no finished cap 4, B 14 run on `87f94c5f`) depend on the cap as far as these runs show; the cause is not traced, and one run is one schedule.

## Stress seed 212: a reader past its budget (2026-11-02; `.logs/lrc3` in the main checkout)
Seed 212 drew cap 4, B 4 under the range 4-16 (it draws B 6 since the floor below), `timers_first` off, no wind, loss .486. On `87f94c5f` it ends `HostFault: ended without a flushed end` at tick 18257, digest `a23e29a3…`, with no dock, on all 8 runs that got that far, on three source trees (other tries stopped on wall-clock waits at ticks 1, 2, 9619 and 9620, one at tick 2 as relay closed, and one left no journal; why each wall deadline ran out is not logged). `d212f_t1` and `t2` have byte-identical journals. Seed 267 (same cap and B, loss .320) repeats it: `967e70e5…` at tick 16164, journals byte-identical. Deterministic in both, so not a host fault (todo 202). Replay is what repeats; the fault itself needs the stop to land in a slow exit (below). Ticks below are `d212f_t1`'s, which carries temporary counters, since removed.

**The chain.** Measured, except where a link says otherwise.
1. *The reader falls behind.* A reader below B is woken by a delivery in the input phase. A reader past B waits on an ordinary timer, so it has no turn of its own and competes for picks under the cap. Demand is above cap 4 long before NAV: uncapped 5.9-6.1 handoffs per non-freeze tick before NAV, 6.4-7.1 in NAV. By state at tick start, the reader gets a handoff on .99 or more of ticks below B; past B, on .73-.75 before NAV and .61-.66 in NAV (seeds 212, 267), against .93-.96 in NAV at B 16 (`cf_b16_t1`). Over all ticks that is .83-.84 before NAV (about 3.1 packets per handoff) and .62-.66 over all NAV (4 per handoff). From k 16403 to UAV 1's NAV_FAIL at k 17855 it is .57-.65, so its capacity there is 2.28-2.60 packets per tick against an arrival of about 2.72 (it needs .68-.69). Outside freezes it starts 5680, 5402 and 5348 ticks parked and serves nothing on 1688, 1500 and 1482, always with the whole cap used. By rules over the counters, not a direct count, pooled over the three UAVs: woken but not picked on 84.7-86.7 % (per UAV 81.4-87.9 %, at most 89.5 %), its timer not fired on the rest. No stall is long (3 ticks at most outside freezes); the lag comes from the rate. Undecoded bytes at k 17855: 29874, 9490, 8279. Seed 267: share .84-.85, then .61-.65; UAV 1 again holds the most bytes (33057, 15926, 15777 at k 15955), about 2 times the others, not 3.
2. *Lag.* NavPy attitude lag on UAVs 1, 2, 3: p90 4540, 1281, 1241 ms; max 5841, 2060, 1921 ms. Backend max 181-220 ms. On rows more than 100 ms above the 20 ms floor, 99.3-99.6 % of that lag is dispatch minus delivery (98.8-99.5 % over all rows). UAV 1 falls furthest, on a lower reader share from NAV entry to its NAV_FAIL (.57 against .64-.65 on k 17000-17855). Its `nav-peer-poi-dispatch` thread takes .35 handoffs per tick over k 16060-17855 and stops at k 17686: `NavPeerSubmission.submit` runs on every NAV tick and the worker then skips a POI already handled (`nav_network.py:195-204`, `peer_poi_dispatch.py:80-87`). All three are over capacity without it, so it is an amplifier: likely, not proven, since no run removes it.
3. *No detection.* All three end `NAV_FAIL: no_detection` (k 17855, 17970, 18042; SNAP 16.2, 8.5, 6.3 m) before any stop. Sim frames are clocked by the ATTITUDE messages the reader delivers (`pose_frame_clock.py`), so frame age follows the lag: 5.90, 2.04, 2.04 s at the end, against 0.18-0.26 s in `cf_b16_t1`. Strong inference, not isolated: the decision that first dropped each POI is not logged, and no run separates lag from frame cadence. This is a sim path and says nothing about hardware.
4. *A slow exit.* UAV 1's exit sends `WP_LOITER_RAD` at k 17859, 17959, 18059. SITL echoes each within 2 ticks, but the reader is 260-290 ticks behind, so sends 1 and 2 log `no echo`, and the wait ends at k 18122 on send 1's echo (the reader's total reaches it at k 18121 and send 2's at k 18202). The POI refresh then asks terrain three times at 1 s each (only UAV 1 has waypoint POIs), with the same unread answers.
5. *The stop lands in that exit.* The last SNAP (UAV 3, k 18045) meets the eval's `stop_when`; stop-all follows at k 18049, 191 ticks after UAV 1's SNAP (2 on seed 267), and ordinal 1 is asked first, at k 18050 (one at a time, so UAVs 2 and 3 are never asked). An operator's Stop-All on that route would do the same (inference). `cf_tf_t1` also had 3.7 s of lag, but its stop came 17.56 s after UAV 1's SNAP, and it ended clean. Seed 196 (cap 4, B 4, wind, loss .437; `sw_196_t2`, `20c35ca9…`) has lag max 6881, 2981, 2901 ms and no dock (SNAP 25.8, 14.0, 17.0 m), and it too ends clean: its stop came 380 ticks (7.6 s) after UAV 1's SNAP, inside the last 72 ticks of a 9.06 s exit (`sw_196_t1` has the same journal up to a `wsl.exe` timeout stopping a relay at the end, a real host fault).
6. *The fault.* `NavigationLoop.stop` joins for 2 s, twice; both time out (k 18151, 18252). The exit needed at least 413 ticks and had 393. NavPy 1 fails, its stop ack never runs, and at k 18257 `retire` labels the unflushed end `HostFault`, with 4.14 s of the 10 s stop budget used. UAVs 2 and 3 are never stopped.

So the lag is the trigger; the exit's blocking is a separate fragility (todo 203), and the label is todo 202.

**Counterfactuals on seed 212** (single runs; docking can flip on small schedule changes):

| Run | Change | Leaves the base at | Digest | Result | NavPy lag max, ms | UAV 1 parked / served nothing |
|---|---|---|---|---|---|---|
| `d212f_t1` | none | - | `a23e29a3…` | unclean end at k 18257, no dock | 5841, 2060, 1921 | 5680 / 1688 |
| `cf_nocap_t2` | no cap | k 9842 | `9ba82745…` | all dock, median 0.12 s, longest 0.28-0.34 s, coverage 0.69-0.72; fails only todo 121 (a) | 201, 301, 320 | 2383 / 0 |
| `cf_b16_t1` | B 16 | k 6 | `cb8fceec…` | **passes**, all dock, median 0.14-0.15 s, longest 0.28-0.42 s, coverage 0.55-0.70 | 201, 241, 241 | 18 / 10 |
| `cf_b8_t4` | B 8 | k 6 | `3a26beff…` | **passes**, all dock, median 0.14-0.16 s, longest 0.40-0.44 s, coverage 0.55-0.66; reader share in NAV .91-.94 | 241, 261, 241 | 155 / 69 |
| `cfw_b6_t1` | B 6 | k 6 | `9d9b0a21…` | **passes**, all dock, median 0.14-0.16 s, longest 0.38-0.44 s, coverage 0.39-0.53; reader share in NAV .76-.83 | 361, 280, 241 | 2358 / 739 |
| `cf_tf_t1` | `timers_first` on (forced: no seed in 0-399 has it at cap 4, B 4 or 5) | k 6 | `5c3988f5…` | clean end; UAV 3 docks, UAVs 1, 2 reach SNAP with no dock (18.0, 0.8 m), observation gaps 0.70, 0.54 s over the gate; reader share in NAV .68-.74 | 3681, 641, 681 | 6313 / 1584 |
| `m212_t1` | main `5179baab` (per-packet reader, budget off; not a one-change arm) | k 5 | `2697a434…` | clean end, no dock: UAV 1 SNAP 523 m, `source_command_stale`, UAVs 2, 3 `no_detection`; median 0.32-0.34 s. Not a passing control | 221, 320, 300 | n/a |

No arm is the same run plus one change at the failure: journals split at k 5-6 and the NavPys start 16.8-35.0 s later (without the cap the journal splits at k 9842, the NavPys start on the same ticks and NAV entries move by -204 to +239 ticks), so seed 212 alone cannot exclude a re-roll. Seed 267 supports the chain: same cap, B and `timers_first`, same chain to the fault, except that UAV 1 is the last SNAP and both joins time out in the first `WP_LOITER_RAD` wait, before any terrain ask; its backend lag also reaches 2.1-2.2 s. So the cause is supported at mechanism level; single-seed dock outcomes in these tables are not evidence for a fix, and no arm perturbs the schedule without changing capacity.

**Why low B fails: service against arrival.** A saturated reader serves B × (share of ticks it runs) packets per tick. NavPy, cap 4, arrival 2.72-2.97: share in NAV .57-.66 at B 4 (2.3-2.6, backlog grows), .73-.90 at B 6, .91-.94 at B 8 (backlog flat). Backend, no cap, so capacity is B: arrival falls as link loss rises and peaks at 5.47-5.58 per tick over 2000 ticks (seeds 38, 180, 91, loss .022-.001). Below B there is no lag (seed 212 at B 4, arrival 2.68-3.58; `s38b16`); at B 4-5 with arrival 4.0-5.5 it reaches 3-27 s (seeds 32, 86, 30, 38, 180). The calibration chose 4 from NavPy's mean alone, with neither the router nor the share under a cap in view.

**Prototype "form 4"** (`.logs/lrc3/proto4.patch`, 16 insertions, 4 deletions, in `baton.py`, `lockstep_bus.py`, `participant_loop.py`; tests `.logs/lrc3/test_proto_freeze.py`; not in the commit). With `timers_first` off, a reader past B parks on a second timer heap that `tick` pops in the input phase after the deliveries, so it is runnable there like a reader below B; the cap and seeded order can still defer it. A freeze pops nothing. With `timers_first` on the wait stays an ordinary timer: the earlier form demoted it there (40 packets drained at tick 18-20 against 11). It leaves the backend's capacity at B. Lockstep module tests on it: 6 failed, 630 passed, 13 skipped under load; the expected pin, and 5 in `test_backend_participant.py` (4 spawn registration timeouts, 1 tick fault) that pass when that file runs alone (14 passed).

**Same deliveries and demand, in process** (arrival 3 per tick, 300 ticks, seeded inputs first; 3 seeds; a model of the load, not a mission). Reader backlog of 900 at the end:

| Load | Base, B 4 | Base, B 6 | Form 4, B 4 |
|---|---|---|---|
| Cap 4, 8 timer threads | 377-425 | 0 | 0-3 |
| Cap 5, 8 timer threads | 267-285 | 0 | 0 |
| Cap 4, 2 threads that never block, 3 timers (no timer ever wakes, on either tree) | 878-890 | 771-849 | 5-6 |

With `timers_first` on, the trees are identical: 5 timers at B 4 136-231, at B 6 3-18; 8 timers at B 4 388-412, at B 6 117-171, at B 8 15-38. So with `timers_first` off the base reader at B 6 keeps up against timer threads, the real runs' kind of load; with it on (25 % of `stress` seeds) 8 timer threads leave a backlog of 117-171 at B 6, so a sweep must still check reader lag at B 6 and above. A never-blocking thread still collapses the reader at any B. No real run has such a thread (longest starved streak 3 ticks outside freezes).

**Three arms, by seed.** A is `87f94c5f`, B is form 4, C draws B from 6-16. Compare arms on docks and lag, not on pass: a pass can flip on one todo 121 (a) frame, and no gate sees lag. Wall budgets were raised in temporary copies; that does not change digests (`p4w_32_t1`, `p4w_hw_t1`). Single runs.

| Seed (cap, B, loss) | A | B: form 4 | C |
|---|---|---|---|
| 212 (4, 4, .486) | `a23e29a3…` unclean end, no dock, NavPy lag max 5841 ms | `da0bd3ef…` passes, 300 ms | B 6, `9d9b0a21…` passes, 361 ms |
| 267 (4, 4, .320) | `967e70e5…` unclean end, no dock, 6421 ms | `808fecc7…` passes, 321 ms; backend 2.1-2.2 s | B 6, `acf5b221…` all dock, 121 (a), 300 ms; backend 241 ms |
| 196 (4, 4, .437, wind) | `20c35ca9…` clean end, no dock, 6881 ms | `dd1b5c40…` launch aborted on a stale EKF status (todo 197), so not comparable | B 6, `52ab6440…` UAVs 1, 2 dock, UAV 3 SNAP 1.58 m no dock, 121 (a), a 0.58 s gap, roll 52-61 deg, 261 ms |
| 324 (5, 4, .381) | `32166957…` passes, 360 ms; backend 1.4-1.5 s | `3ccc6646…` all dock, 121 (a), 321 ms; backend 541-681 ms | not run |
| 32 (5, 4, .213, `timers_first`) | `d36c6d55…` passes; backend 13.1 s | same digest | B 6, `54aca45a…` all dock, 121 (a); backend 221 ms |
| 86 (10, 4, .224) | `lowB86b` all dock, 121 (a); backend p99 about 13 s | `bd2dcc76…` all dock, 121 (a); backend unchanged | B 6, `a28eae96…` all dock, 121 (a); backend 260 ms |
| 38 (6, 4, .022) | `9818e906…` mission download failed; backend about 3 s | `23b67fa8…` same | B 6, `b87514ab…` all dock, 121 (a); backend 220 ms |
| 30 (5, 4, .120, `timers_first`, wind) | `49660948…` second confirm round; backend 27 s | same by construction | B 6, `95ae87d2…` one round per UAV; UAV 1 docks, 2, 3 SNAP 1.15, 0.97 m no dock; backend 260 ms |
| 45 (4, 4, .440, wind 5.8 m/s) | `5202259b…` mission download failed, no lag | `4004ff16…` 121 (a), UAV 3 docks, roll 70 deg, 260 ms | B 6, `1b29d8a6…` no dock, SNAP 3.1-5.8 m, roll 67-70 deg, 240 ms |
| 317 (4, 4, .563) | `871c0bca…` mission download failed, no lag | `59decf37…` the same | B 6, `1b7961ab…` all dock, 121 (a), 241 ms |
| 180 (8, 5, .008, wind) | UAV 2 misses; backend 10.5 s | not run | B 7, `538f261d…` UAV 1 docks, UAVs 2, 3 SNAP 1.04, 2.46 m no dock; backend 221 ms |
| 40 (4, 16, .279) | `297878ae…` passes | `0a1d7648…` passes | B 16, `297878ae…` the same |
| 1 (6, 10, .303) | `7359cec3…` passes | `ae2b5f36…` passes | B 11, `7e07a5ac…` passes |
| 17 (4, 14, .355, wind) | `70b50252…` (pre-commit tree) UAVs 2, 3 no dock | `3ffc6dea…` 121 (a) × 2, UAV 2 no dock | B 15, `554bee51…` 121 (a), UAVs 1, 2 no dock |
| 24 (4, 10, .553) | `85ac3822…` all dock, 121 (a) | `d095a79e…` passes | B 11, `c5ed25d7…` all dock, 121 (a) × 2 |
| `none`, `hw` | `b9bb2e63…`, `eb263f68…` | the same | unaffected |

Both B and C move many digests: form 4 can move all 311 of 400 seeds with `timers_first` off that reach B (those run all moved), and the C range changes B in 297 of 400, 52 of them below 6 today. Docking on wind seeds stays schedule-sensitive in every arm (seeds 196, 45, 30, 17, 180, and `sw_211_t2` at natural cap 4, B 6, lag 320 ms, UAVs 2, 3 no dock): an open class, classification only, not evidence for or against either arm.

**Floor for the range.** The backend's arrival peaks at about 5.6 per tick at zero loss for this 3-UAV mission (5.58 seed 180, 5.55 seed 91), so B 6 leaves about 7 %, modest: no seed is known to overrun it, and none is proven not to; more vehicles or streams need a recheck. A saturated NavPy reader at B 6 needs service on about .46-.50 of ticks and got .73-.90 in NAV. Forms and floor fix different things: the floor addresses todo 199 and hides todo 200 on the measured seeds, form 4 addresses todo 200 only with `timers_first` off and not the backend. The floor is a calibration: the parked mechanism stays, and the only drawn route to multi-second lag leaves `stress`, so the failures keep their recipe (a copy of `stress` with `packets_per_tick` [4, 4], todos 202, 203). **Decided 2026-11-02: `stress` draws B from 6-16; form 4 stays a saved patch under todo 200.** All results above with B 4 or 5 are from the old range.

**Final runs on the commit with the floor** (`.logs/lrc3/final`; unmodified wall budgets, each digest equal to its row above):

| Run | Digest | Result | NavPy lag max, ms |
|---|---|---|---|
| `none` | `b9bb2e63…` | passes | 1 |
| `hw` | `eb263f68…` | passes | 1 |
| seed 40 (B 16) | `297878ae…` | passes | 241 |
| seed 1 (B 11) | `7e07a5ac…` | passes | 240 |
| seed 212 (B 6) | `9d9b0a21…` | passes | 361 |
| seed 32 (B 6) | `54aca45a…` | all dock, observation gates pass; fails only todo 121 (a) | 321 |
| seed 24 (B 11) | `c5ed25d7…` | all dock, observation gates pass; fails only todo 121 (a), 2 frames | 240 |
| seed 17 (B 15) | `554bee51…` | observation gates pass; UAV 3 docks, UAVs 1, 2 SNAP 0.50, 0.63 m no dock; todo 121 (a). Depends on the cap as above, cause not traced | 241 |
