# Final approach: late commands and the stale-command check (v4, 2026-10-13)

**Status.** Approved 2026-10-13: the owner said yes to question 2 ("at least for now; our current goal is the accuracy") and "fix all". Earlier: the owner said yes to both directions (fix the delay at its source; judge staleness in aircraft time) and
asked for a Codex review. Codex reviewed v1 ("rework", 8 findings) and v2 ("rework", 5 findings on precision); v3
applies all of them. Codex on v3: "good enough to start a1"; its one remaining point (prove the override stayed active
from the aircraft side) is applied. Nothing is built. Evidence: runs
`.logs/self-orbit-c` and `-d` (main `54df009`), SITL dataflash logs in WSL `~/navpy-eval-roots/self-orbit-{c,d}-*/
{5,6}/logs/00000001.BIN`. Sim only; nothing here shows physical docking.

## What fails
After approval all three approaches fail near the dock:
- **Peers miss by 7-20 m** and lose the dock in the last metres (`no_detection`).
- **The owner stops with `source_command_stale`** partway in.

## Cause 1 (leading hypothesis): commands reach the plane late
**What we know:**
- ArduPlane follows its own pitch demand to within about 2° (`ATT.DesPitch` vs `ATT.Pitch`), with no limit,
  saturation or TECS involvement.
- The best-fit delay of NavPy's `cmd_pitch/cmd_roll` against `DesPitch/DesRoll` grows to about 11 s of sim time.
- **This is provisional.** `DesPitch` is the controller's demand, not packet arrival, and the fit is ambiguous while
  commands are saturated or change slowly. NavPy also updates its diagnostics before taking the send lock
  (`attitude_command.py:81`), so "after NavPy" is not proven either.

**Where the queue might be:**
- NavPy's own sends take under 13 ms, so seconds of blocking inside NavPy is unlikely.
- About 63-65% of sends are held repeats (`command_hold.py:122`).
- SITL's UDP input admits one datagram per IO tick (`UARTDriver.cpp:831-924`, Plane-4.6.3).
- The mavlink-router's fan-in queues are the other candidate.
- Gimbal telemetry is only 5 Hz wall (`gimbal_telemetry_publisher.py:29`).

**a1, the measurement baseline (before any behaviour change; logs only):**
- **Step 0, map the route.** `swarm_run_wsl.py` enables `COMPANION_UDP=1`, and the router also announces endpoints.
  Find which socket NavPy's commands actually go to and whether they bypass the router.
- **Per-packet timing.** Tag each command: NavPy writes a running command id into `SET_ATTITUDE_TARGET.time_boot_ms`,
  which ArduPlane ignores for attitude targets (UNVERIFIED; else use the payload as the key). A packet capture on the
  SITL-facing UDP port (tcpdump in WSL) gives arrival at SITL's socket. The NavPy-side send time and the capture time
  give the residence time up to SITL. The `DesPitch/DesRoll` change that matches that payload gives application.
  Router statistics are used only if the router exposes them; the capture is the fallback.
- **Per sent `SET_ATTITUDE_TARGET`:** the command id and payload, lock wait, socket-write completion time, and the
  aircraft boot time at send. Today NavPy's `time_boot_ms` field is process wall uptime (`attitude_command.py:90`), not
  aircraft time.
- **Per message type:** uplink bytes, datagram counts and rates.
- **Router:** endpoint ingress and egress counts, and drops if it exposes them.
- **Firmware:** when the demand changes in `ATT.DesPitch/DesRoll`.
- **Fit method:** written down (clock alignment, windows, residuals, uncertainty); only roll and pitch transitions are
  matched, never saturated stretches.
- **Speedup 1:** one run, as a load-sensitivity check only. It does not locate the hop.

**a2, fix the hop a1 finds.** ArduPlane 4.6 drops a roll/pitch/throttle override after a hardcoded 3000 ms and then
resumes normal GUIDED navigation (`mode_guided.cpp:35-87`, Plane-4.6.3). So:
- If it is volume:
  - send a target for each new command;
  - between new commands, re-send only the newest fresh target, once per `ATTITUDE_REFRESH_S = 0.5 s` of aircraft
    time, instead of every tick;
  - the deadline resets after every successful send, and there are no catch-up bursts;
  - the margin holds: refresh 0.5 s + scheduling jitter (at most 0.5 s) + delivery (p95 at most 0.1 s) = 1.1 s, well
    under ArduPlane's 3.0 s override expiry;
  - freshness is still checked every command tick, whether or not a refresh is due.
- If it is the router: fix its endpoint.
- Either way: verify the deployed firmware's SHA, change no ArduPlane parameters, and do not lower the speed.

## a1 result (2026-10-13, run `.logs/a1-measure-b`, temporary uplink counter, not committed)
- **Route:** each NavPy talks to its own SITL over a dedicated UDP link (`COMPANION_UDP=1`, `run_swarm.sh` serial0
  `udpclient`). There is no router on the command path.
- **SITL input:** IO, including one `recv` per UDP serial port, runs when sim time has moved over 10 ms
  (`Scheduler.cpp:298-307`).
- **NavPy uplink during an approach:** about 52 messages per aircraft-second, peak 64 in any second:
  - `SET_ATTITUDE_TARGET` 44;
  - `STATUSTEXT` 6.5;
  - gimbal and camera status 1.5.
- **With SITL holding 10.1x, there is no queue.** The best-fit delay stays at 0.1-0.8 s without growth. Both peers
  closed to a 0.0 m miss.
- **In the failed runs SITL ran slower** (about 6.7x: 30 aircraft-s in 4.5 wall-s). Those runs overlapped with other
  heavy work on the host. NavPy's command cadence follows the *nominal* speedup in wall time, so per aircraft-second
  it sent proportionally more than SITL reads, and the queue grew.
- **So the hop is SITL's input, overrun when SITL falls behind nominal speed.** The a2 send policy (a target per new
  command plus a 0.5 s aircraft-time refresh) ties the rate to aircraft time and removes the overrun. Cutting
  `STATUSTEXT` during the approach is a further margin.

## New: the pass detector fires at the start of an approach from the circle
- In `.logs/a1-measure-b` the owner was approved and entered NAV. Within 70 ms it logged `RESET: PASSED POI`, 630 m
  from the dock.
- `VisualPassDetector` (`visual_pass.py:42-59`) arms on any frame with the dock even slightly forward (`body_x > 0`).
  It then declares a pass after two frames with `body_x <= 0`.
- From the waiting circle the dock starts almost exactly abeam (about 90° off the nose), so one small turn reads as
  "ahead, then behind".
- **Fix (a6):** arm only once the dock is clearly ahead: `body_x >= cos(PASS_ARM_MAX_OFF_NOSE_DEG)`, with the angle
  named and justified (proposal 60°). `body_x` is the frame-local line of sight, which the Pure Vision constraint
  allows.
- **Test:** an abeam start does not pass; an ahead-then-behind sequence does.

## New: what ends an approach at the dock with a gimballed camera
- Both peers closed to 0.0 m, but the run ended as `NAV_FAIL: no_detection`, not as a pass. The eval then reports
  "SNAP without passed=True".
- The pass detector needs two frames with the dock behind the camera. A gimbal camera cannot look behind: pitch stops
  at -90°, so the sim projector reports `behind_cam` with no detection.
- **a7 (owner question 2):** define completion for a gimballed narrow-field camera from camera state only (gimbal
  readback is allowed): the dock last seen near the gimbal's down limit, then lost, counts as passed.

## Cause 2: the stale check measures the PC clock
- `command_freshness.py:30-47` checks the frame's producer age in aircraft time (limit 1.0 s), and the receipt age in
  PC time with a limit of 1.0 s ÷ speedup (0.1 s at 10x).
- In all four failures the PC stalled for 100-130 ms while aircraft time moved 20-40 ms. Silence does not prove SITL
  paused, but the plane barely moved.

**a3, one clock contract:**
- **Aircraft clock port.**
  - Timestamp only, from `ATTITUDE.time_boot_ms` (`pose_telemetry.py:27`), with an epoch (reset/reboot) and a
    "last update" health stamp.
  - It exposes no yaw, position, velocity, altitude or SIM_STATE, so the Pure Vision Approach Constraint holds.
  - It is snapshotted when a frame is genuinely received.
- **Navigation rule.** The receipt age is the aircraft time now minus the aircraft time at receipt; above 1.0 s, the
  approach fails (`source_command_stale`). A frozen camera with the plane still flying fails within 1.0 s of aircraft
  time.
- **Host/link guard (fail closed).**
  - If the aircraft clock itself has not advanced for 1.0 s of wall time (telemetry frozen or link dead), the approach
    stops with a distinct reason, `host_link_stall`.
  - The eval reports it as an infrastructure fault and the run fails. It never excludes the approach from acceptance.
- **Limits, stated plainly.**
  - With telemetry silent at 10x, detection takes up to about 10 s of aircraft time; in real flight (speedup 1) it is
    1.0 s, as today.
  - A telemetry backlog whose timestamps keep advancing can keep resetting the guard while running behind the
    aircraft. The producer-age check stays, and a2's delay gate catches that case in the eval.
  - A blocked host cannot send its stop. Once commands stop, ArduPlane drops the override after 3.0 s and resumes
    normal GUIDED flight around its guided target (`mode_guided.cpp:35-87`). That is the aircraft-side behaviour, and
    it is ArduPlane's own safety (owner ruling: ArduPlane safety is enough until after the demo).
  - The lockstep contract (`deterministic_demo_eval.md:74`) covers lockstep runs only, not today's wall runs.
- **Failure semantics.** Each of these fails closed as `source_command_stale`:
  - a missing or invalid stamp;
  - an epoch mismatch between receipt and now;
  - aircraft time going back (a reboot; there is no wrap within a flight, since `time_boot_ms` is 32-bit ms, about
    49 days).
  Out-of-order telemetry never moves the clock back: the port keeps the maximum per epoch. The receipt snapshot rides
  with the frame through queued publication, and dispatch or reset never renews it.
- **One reason set.** The command hold and the pump watchdog return the same reasons (`source_command_stale`,
  `host_link_stall`). Today the hold carries only a boolean and the pump has its own timeout reason; both change.
- **Both paths.** The command hold and the pump watchdog (`final_approach_source_watchdog.py`, wired at
  `nav_final_approach_composition.py:181`; its age starts at activation and dispatch, `detection_dispatch.py:54`) use
  the same contract.
- **Tests.**
  - `test_navigation.py:420` keeps its intent: a frozen camera with the aircraft advancing must expire.
  - New cases: a PC stall with the aircraft paused passes; frozen telemetry fails as `host_link_stall`; aircraft clock
    reset or wrap; queued publication; no first frame.

## Not fixed here: the retry after a failed approach
A failed approach clears the POI and requests RESET (`navigation_decision.py:159`). The UAV then finds the same dock
and asks for a second confirmation (`.logs/self-orbit-d/uav_4_navigation.log:6256`), which the eval forbids. What a
UAV should do after a failed approach (retry without asking again, ask again, or go home) is a separate owner decision
and design item, a5.

## Order and acceptance
| # | What | Done when |
|---|---|---|
| a1 | Measurement baseline and the written fit method (logs only) | The hop is named with evidence; tail delays (p95, max) per run |
| a2 | Fix that hop (bounded refresh, or router) | A mechanism test for the send policy. One SITL run, per UAV, with delay measured from NavPy's socket write to the matching demand change in `DesPitch/DesRoll`, both in aircraft time: p95 at most 0.1 s, max at most 0.2 s, and a fitted delay slope over the approach of at most 0.01 s/s. Aircraft-side evidence that the override stayed active throughout, including held stretches: `DesPitch/DesRoll` stay on the last sent payload and never revert to GUIDED navigation demand. Gaps between NavPy's sends are kept as a separate diagnostic |
| a3 | Aircraft clock port, receipt age in aircraft time, `host_link_stall` | The tests above; both watchdog paths |
| a4 | Two 3-UAV runs | Every approach ends in its normal terminal state (contact or visual pass), not a failure; zero infrastructure faults across both runs; the a2 delay gate holds. SNAP is reported against the 0.5 m gate (todo 22) and is not claimed to pass it until it does |

The point-mass bench's under-2 cm miss shows the law converges when commands arrive. It does not promise centimetre
SITL results.

Added after a1: **a6** (pass arming) and **a7** (completion with a gimbal camera) run before a4.

## Questions for the owner (recommended answer in bold)
1. **After a failed approach, retry the same dock without asking you again (until a pass budget runs out)?** Or ask
   again, or go home? **Decide at a5; a1-a3 do not depend on it.**
2. **A dock last seen near the gimbal's straight-down limit and then lost counts as reached (passed), not as a lost
   dock?** **Yes**, with the angle named and tested.
