# JSBSim-only lockstep vs ArduPilot SITL (research, 2026-10-16)

Question: can the deterministic demo eval, and other evals, run JSBSim directly in lockstep with no ArduPilot SITL?

## Answer
**Keep SITL for every eval that certifies ArduPilot integration: the certified 3-UAV demo, mission and parameter protocols, peer forwarding through the autopilot, and docking. Do not build a JSBSim-only lockstep path.** SITL is the only part of the eval that runs the flight code we fly; that alone decides it. Speed is no reason either: the estimate below puts SITL at a small part of each tick, though that is not yet measured on the fleet workload. A JSBSim-only path would cost an autopilot stand-in, about L-XL. Swarm logic that does not depend on the autopilot already runs without firmware (baton tests). For fast work on the vision law alone, the repo already has the cheaper tier: the point-mass bench and the pixel/camera loop tests.

## The premise: 10h is faster than the estimate, not slower
The two docs use "x real time" in opposite senses:
- The slice table (`lockstep_contracts.md`, Execution model) uses speed: 5.2 ms per 20 ms tick is "3.8x".
- The fleet estimate, "1.3-1.5x real time", is in the same sense: three slices of about 5 ms is 15 ms per 20 ms tick.
- The 10h figure, "0.52-0.62x real time", is wall time over sim time. `.logs/s10h/r1-a/lockstep_run_host.json` has 108.8 wall s for 8786 ticks of 20 ms (175.7 sim s): 12.4 ms per tick, 0.62 wall/sim. That is **1.6x faster than real time**. 10.3 ms per tick is 1.9x.

So the fleet runs at 1.6-1.9x real time, which beats the 1.3-1.5x estimate. The "20.4 ms (1.02x)" with host delays is about 1x, in line with the estimate's "below 1x with delays". `lockstep_fleet.md` (Risks and the 10h row) and `deterministic_demo_eval.md` (step 10h) now state the unit.

## Where the tick time goes
A tick is one ArduPlane loop at 50 Hz, 20 ms of sim time. The SITL physics steps 24 frames of 833 us per tick (`SIM_Aircraft.h:248`, `rate_hz = 1200`).
- **The SITL side looks small (estimate).** Step 7 ran one serial-step SITL, its relay, the coordinator and a scripted operator at 16x real time: about 1.25 ms per tick for all of that (`deterministic_demo_eval.md`, step 7). That is a different workload; it does not isolate a SITL's cost inside the 3-UAV run.
- **Participants run concurrently** (`coordinator_tick.py:5-7,63-73`), and the SITLs step concurrently: `FleetFrames.reply` sends reply k to SITL 1..N in a loop with no wait between them (`coordinator_frames.py:107-115`), and only the next tick's `read` waits for each frame (`:92-95`; called at `coordinator_run.py:175,191`). The operator runs after the participants (`:189-190`). So the critical path per tick is about: one SITL step, the slowest participant, the operator, plus coordinator work. It is not a sum over vehicles.
- **Inference, not measured:** most of the 10-12 ms is the Python participants and the coordinator. The likely critical path is the backend, one process serving 3 vehicles, or a NavPy with its simulated camera. The host has 20 logical CPUs (WSL has 10), so plain CPU starvation is unlikely at idle load. **To settle it:** journal per tick the wall time spent waiting for frames and for each `DONE` (host-only fields, outside the digest).

**JSBSim cost, measured** (`jsbsim` 1.3.2 from `.venv`, in-process, 20 s of sim, 4 control inputs set every step as `SIM_JSBSim` does; script in this session's scratchpad, not in the repo):

| Model | Rate | Wall per 20 ms tick, 1 vehicle | Two runs equal |
|---|---|---|---|
| Rascal110 (ArduPilot's `Tools/autotest/aircraft/Rascal`) | 1200 Hz | 0.43 ms | yes |
| c172x | 1200 Hz | 0.99 ms | yes |
| c172x | 120 Hz | 0.11 ms | yes |

(Rascal at 120 Hz went to NaN open loop; physics rate is a fidelity choice, not free.)

This times the physics alone, with no controller or MAVLink shim. **Estimate, not measured:** if a SITL step costs about what step 7 suggests, removing SITL saves about 1-2 ms of the 10-12 ms per tick, and a JSBSim plus an autopilot stand-in puts some of that back: a gain of roughly 10-20%. The gain stays unknown until the per-tick timing above is measured. Either way, speed does not decide the question; section 1 does.

## 1. What SITL gives that JSBSim alone does not
SITL runs the ArduPlane code we fly. The physics in our SITL is the least of it: we run `--model plane` (`scripts/lockstep_wsl.py:179`), whose source says it is "very simple ... Not aerodynamically accurate" (`SIM_Plane.cpp:16`).
- **Flight modes and guidance.** The eval flies AUTO missions (takeoff, waypoints, `LOITER_TO_ALT`, `DO_JUMP`, ROI), LOITER, GUIDED and `DO_REPOSITION`, with L1 and TECS doing the work.
- **The final-approach command path.** NavPy sends `SET_ATTITUDE_TARGET` (`attitude_command.py`). ArduPlane decodes it (`GCS_Mavlink.cpp:1312-1376`), clamps roll and pitch to `ROLL_LIMIT_DEG` and the pitch limits (`mode_guided.cpp:42,80`), and controls throttle from the message for 3 s, else TECS takes over (`:90-97`). The command bounds gate reads these same parameters. A stand-in would have to copy each rule.
- **EKF and arming.** Todo 46's missed dock had two causes, both outside NavPy: launch before the EKF was ready (origin set in flight, +46 m altitude bias) and terrain off (`architecture_todo.md`, 46). A truth-state stand-in hides both. Catching firmware and setup behaviour like this is what the eval is for.
- **The MAVLink interface.** This covers the mission protocol with readback, the parameter protocol, `COMMAND_ACK` timing (frame k+2), stream rates (`SR0_*`, `SR1_*`) and STATUSTEXT.
- **Swarm forwarding.** Peer NAVLINK goes NavPy i → serial0 → AP i → serial1 → router → AP j → serial0 → NavPy j (`lockstep_fleet.md`, Wall topology). ArduPilot's own routing does both AP hops. The certified Assignment and forwarding gate checks exactly these hops.
- **Lua.** `aas_params.lua` creates the `AAS_*` table NavPy reads (`AAS_DEL_CTRL`, `AAS_NAV_MIN_ALT`, ...). `chute_deploy.lua` and `tune_pitch_roll.lua` are out of lockstep scope today (`deterministic_demo_eval.md`, Not in scope).
- **Parameters and failsafes:** stock `plane.parm` plus the pinned overrides; geofence, battery and link failsafes as the firmware does them.
- **Certification value.** Owner ruling: ArduPilot fixed-wing first, stock firmware. The serial-step build is stock ArduPlane flight code plus an eval-only SITL/HAL hook (`2dbd7e38dd` on `bf0dd8f`). The hook gates I/O and the clock, not the flight code. Known gap: `bf0dd8f` is 7 commits past the demo's `4c23c0a1` (the hooks, pose precision and the pitot EAS fix). A JSBSim stand-in tests our model of ArduPilot, not ArduPilot.

## 2. What a JSBSim-only lockstep path would need
| Piece | Work | Size |
|---|---|---|
| Vehicle model | A JSBSim model of our airframe (elevon). Rascal110 is not it, and neither is `SIM_Plane` | M, plus identification flights |
| Autopilot stand-in | AUTO mission executor, L1, TECS, roll/pitch controllers, GUIDED attitude-target rules, LOITER, REPOSITION, arming, an EKF-like estimate with its readiness and biases | XL, and it drifts from ArduPilot with every release |
| MAVLink shim | Heartbeat and mode, telemetry streams and rates, mission and parameter protocols with the `AAS_*` table, `COMMAND_LONG`/`ACK` timing, STATUSTEXT, serial0↔serial1 routing | L |
| Wind | JSBSim has wind and MIL-SPEC turbulence (`SIM_JSBSim.cpp` sets `atmosphere/wind-mag-fps`, `psiw-rad`, turbulence). Turbulence RNG seeding: not checked | S |
| Terrain | JSBSim ground is its own; matching the pinned SITL tiles needs a terrain callback | M |
| Gates | Re-derive which gates still mean something (section 4) | M |

- **Speed:** a gain of roughly 10-20% (estimate, above); unknown until profiled.
- **Determinism:** JSBSim is a single-threaded integrator, and two runs of each open-loop case above gave equal digests of 7 state fields (no turbulence, controller, shim, terrain or host delays). That is all that was shown. The integrated path's determinism and turbulence seeding are unverified. The serial-step SITL is already byte-exact under delays, jitter and CPU load (10h), so determinism is no reason to switch.
- **Pure Vision Approach Constraint:** a stand-in must publish the same fields as ArduPilot. Its telemetry must not be cleaner than ArduPilot's either: truth yaw instead of compass yaw with bias, or truth altitude. Otherwise a leak into the command path would show no behaviour difference.

## 3. Middle options
- **SITL with the JSBSim backend (`--model jsbsim:Rascal`).** It is already stepped per frame: `send_servos` writes `iterate 1` on JSBSim's TCP console (`SIM_JSBSim.cpp:339-371`), and JSBSim runs `--suspend` (`:219-226`). Three problems in lockstep:
  - **(a) Not deterministic under host delay.** `recv_fdm` waits 100 ms of wall time for the FDM packet, and on a timeout it resends the servos, including another `iterate 1` (`:420-424`; the loop is at `:422`). A slow host can step the physics twice. The serial-step firmware would need a patch there.
  - **(b) Slower.** It adds 24 socket round trips per tick per vehicle, and a JSBSim binary pinned in `firmware.lock`. JSBSim is not installed in our WSL today.
  - **(c) Terrain.** Altitude comes from JSBSim's AGL plus home (`:440`), not our pinned SITL terrain.
  It buys aero fidelity only once we have our own airframe model. Worth it later for fidelity, not for speed.
- **SITL speedup.** It gives nothing here. Serial-step has no wall pacing (step 7), so the SITL already runs as fast as the coordinator lets it. `SIM_SPEEDUP` != 1 is refused, because NavPy's cadences divide by it.
- **Faster physics backend.** Not measured in isolation. The step 7 bound (about 1.25 ms per tick for a SITL, its relay, the coordinator and an operator together) suggests the physics is a small part. Hypothesis until profiled.
- **A fast tier for the vision law, plus SITL for certified runs.** Yes, and it exists: `scripts/vision_static_point_mass_plant.py` (the point-mass bench) and `tests/modules/navigation/nav/vision_nav_camera_loop.py` (finite-SIYI pixel/camera loop). A JSBSim plant could replace the point mass there for better aero. That is a small, separate change (S-M), with no MAVLink or autopilot.
- **To make lockstep faster**, measure the per-tick split first (section "Where the tick time goes"). The hypothesis is that the participants hold most of the 10-12 ms.

## 4. Which evals could move, which must stay
| Eval or gate | Tier | Why |
|---|---|---|
| Vision/approach law convergence, LOS and pixel math, command bounds as numbers | JSBSim or point mass (exists) | Certifies the law in isolation; no autopilot behaviour claimed |
| Gimbal, zoom and tracking plants | In-process (exists: `navpy/sim/gimbal`) | No vehicle dynamics involved |
| Auction, ack and roster logic | Baton tests (exist: `test_auction_baton.py` and the others) | Already off SITL |
| Certified 3-UAV demo (`certified_lockstep`): assignment and forwarding, approval timing, workflow ticks, SNAP, DOCKED | **SITL** | Forwarding goes through AP routing; DOCKED depends on EKF, terrain, the GUIDED clamps and TECS |
| Mission upload and readback, parameter and AAS checks, command-bounds readback | **SITL** | They certify ArduPilot's protocols and Lua |
| `ideal_360` three-UAV eval | **SITL** | The sensor is ideal, but the vehicle and its control are the real ones; that is its isolation value |
| Hardware and HITL tuning (elevon, roll quality) | Real aircraft / HITL | Neither SITL nor JSBSim proves the airframe |

## Recommendation
1. Keep serial-step SITL for every eval that certifies ArduPilot integration (section 4). Swarm logic without the autopilot stays in baton tests. Drop the JSBSim-only lockstep idea.
2. Done: the 10h cost lines now give the fleet as 1.6-1.9x real time (wall/sim 0.52-0.62).
3. If speed matters, add host-only per-tick wait timing to the coordinator, then optimise the slowest participant.
4. Optional, later: put a JSBSim model of our airframe into the vision-law bench. Consider `--model jsbsim` in serial-step only after (a) the resend is patched and (b) terrain is handled, and only for aero fidelity.

Evidence level: source reading (ArduPilot tree `4c23c0a1` in WSL, repo at the merged 10h), the 10h host record, and a JSBSim physics-only micro-benchmark. No lockstep run was made with JSBSim, and the per-tick cost split is not measured.

Review: Codex (`codex exec -s read-only`, evidence inlined because its shell failed to start). Round 1: 3 P2, all accepted and fixed: the speed gain is now an estimate, the determinism claim is limited to what was run, and the recommendation is scoped by what each eval certifies. Round 2: 2 P2, both accepted and fixed: the two remaining cost statements are now hypotheses, and SITL concurrency is cited from `FleetFrames.reply`. Round 3: nothing material remains. Codex confirmed the unit correction (0.619 wall/sim, 1.615x), participant concurrency, the firmware excerpts and todo 46.
