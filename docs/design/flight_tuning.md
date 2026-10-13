# Flight tuning system (design draft v3, 2026-10-11)

Status: draft. Nothing here has flown. Aircraft: Peron15 flying wing, ArduPlane 4.6.3,
two elevons, no rudder, Lua scripting enabled.

v3 (2026-10-11): the owner asked for tuning in flight, like AUTOTUNE, with automatic
left-right rolls. The in-flight tuner below is that request. It replaces the stage 2
stick-override automation; stage 1 stays as the ground-analysis route.

v2 changes after the Codex design review (2026-10-11):

- Stage 1 now flies a pilot-flown test card. The on-board stick-moving script is moved to
  stage 2 with stricter safety requirements.
- Identification now uses the known controller in a closed-loop fit.
- Acceptance criteria are now numbers.
- The defect list for the old script is corrected.

## Goal

Roll and pitch gains for each Peron15 that follow attitude demands better than
ArduPlane AUTOTUNE, shown on the same manoeuvres flown alternately with both gain sets.

## In-flight tuner, Lua prototype (tools/tuning/lua/flight_tune.lua)

Status: reference prototype, replaced for flight by the firmware tuner (AP_FlightTune,
below). It works in the Lua test harness and in ArduPlane SITL. It is not for flight, and
it is kept out of scripts/lua so it is not deployed by accident. The method below is shared
by both versions; the firmware differences are listed in the firmware section.

### How it flies

- **Switch and modes.** FTUN_ENABLE 1 and a switch with RCx_OPTION 300 (Scripting1).
  It runs only in LOITER, CRUISE or FBWB, from a movement of the switch to high.
- **Control hook.** For each manoeuvre the script sends roll and pitch rate demands
  with vehicle:set_target_throttle_rate_rpy after vehicle:nav_scripting_enable(mode),
  the hook ArduPilot's aerobatics scripts use. The demands go into ArduPilot's own
  rate controllers (ArduPlane/Attitude.cpp L409-427). No RC overrides are used, so RC
  failsafe and the pilot's sticks keep working. Between manoeuvres the mode flies the
  aircraft for 3 s.
- **Not in FBWA.** chute_deploy.lua takes over by switching to FBWA. Mode::enter clears
  script control (ArduPlane/mode.cpp L27-31), but Plane::set_mode ignores a change to
  the active mode (ArduPlane/system.cpp L237-244). In FBWA the parachute's mode change
  would not end the manoeuvre.
- **Stops at once** (control released with nav_scripting_enable(255), gains of the axis
  under test restored): switch not high, FTUN_ENABLE 0, stick deflection over FTUN_STICK
  (this also catches the parachute's pitch and throttle overrides), mode change, RC loss,
  battery or EKF failsafe, unhealthy AHRS, disarm, height below FTUN_ALT_MIN or more than
  FTUN_ALT_LOSS below the start, airspeed outside AIRSPEED_MIN..MAX, roll beyond 60 deg
  or pitch beyond +-30 deg, the parachute switch (latched until reboot), or a Lua error.
  If the script dies without a cleanup, ArduPlane drops script control after 1 s
  (commands_logic.cpp L1250-1260) with the last rate demand held until then.
- **Refuses to start:** FBWA or any other mode, tune_pitch_roll.lua loaded (MTUNE_
  parameters present), tuning switch on the parachute channel, AUTOTUNE switch on, not
  flying, notch filters or PDMX set on the rate PIDs.
- **New gains** are active only while the script flies a check manoeuvre, until kept.

### What it measures and designs, per axis (roll, then pitch)

1. **Tests (4, up to 8 in rough air).** Level the wings, hold 0.6 s, then rate
   doublets of 0.5, 0.25, 0.12 and 0.06 s on top of an attitude hold that copies
   ArduPilot's angle loop. Roll test rate FTUN_RLL_RATE (60 deg/s), pitch FTUN_PIT_RATE
   (15 deg/s), both capped so the first doublet reaches at most 35 deg roll or 10 deg
   pitch. Tests with output saturation or a script delay over 100 ms are dropped.
2. **Frequency response** from controller output to body rate at 15 frequencies,
   1.5-38 rad/s: G = sum(P X*) / sum(U X*) with the test signal X as reference, so noise
   and turbulence that are not correlated with the test signal do not bias it. The
   Fourier sums integrate exactly between samples, so uneven script timing does not
   bias them either. The record-to-record scatter weights each frequency.
3. **Model fit** (Levenberg-Marquardt, from several starts). Roll: gain, pole, lag and
   delay. Pitch: gain, zero, short-period frequency and damping, lag and delay. The fit
   must match the measured response within 0.3 RMS in ln G.
4. **Design.**
   - FF: the model's steady-state gain, or 85, 70 or 55 % of it. A lower FF slows the
     angle loop when TCONST asks for more than the airframe's lag allows.
   - P and D are searched on a grid, with I set from the crossover.
   - Requirements:
     - rate-loop phase and gain margins of FTUN_PM and FTUN_GM at the test speed, with
       10 ms more delay than measured; at AIRSPEED_MIN and AIRSPEED_MAX (model scaled with
       speed) the phase margin may be 10 deg lower, because the speed scaling is an
       assumption there;
     - attitude-loop margins (the whole loop, broken at the servo) of FTUN_PM and FTUN_GM
       at the test speed;
     - crossover at most half the highest measured frequency;
     - the measured points stay away from -1 (sensitivity peak at most 2);
     - predicted servo jitter below a quarter of SMAX;
     - a simulated check step at the test speed, through a copy of ArduPilot's
       controller, with no more than 10 % overshoot and no more output than 35 deg or
       the current gains' peak.
   - Choice: the lowest predicted error on that step, the same measure the check flight
     uses. Within 3 %, the faster rate loop wins (better gust rejection).
   - TCONST and RMAX are not changed, so the comparison is at equal TCONST.
5. **Check, A-B-A-B.**
   - Attitude steps right, level, left, level (20 deg roll, 5 deg pitch), flown with
     the old and the new gains in turn.
   - The new gains are kept, and saved when FTUN_SAVE is 1, only if:
     - the mean error against the ideal angle-loop response drops by at least FTUN_IMPROVE
       and by more than the run-to-run scatter;
     - neither direction gets worse;
     - overshoot stays within 10 %;
     - servo activity at most doubles.

### Evidence so far

- **Harness tests** (tests/tools/tuning/test_flight_tune_lua.py, 51 cases, Lua 5.3 under
  ArduPilot's instruction limit of 10000 per call):
  - Inert when off.
  - Every refusal and every stop path, each checked for immediate release and gain
    restore.
  - A full roll and pitch tune: models within 1-10 % of the simulated aircraft, tracking
    error down about 70 % in the checks, as predicted.
  - The saved gains are checked on the simulator's exact discrete loop by independent
    Python code: margins are kept even with script calls up to 40 ms late on every call.
  - A test checks ArduPilot's Lua compiler limits (100 active locals, 127 upvalues).
- **ArduPlane SITL** (python3 -m tools.tuning.sitl_driver inflight-tune, plane-elevon model, SITL's default
  gains, TCONST 0.25):
  - The first run failed to load: "too many local variables (limit is 100)". It is now
    fixed and tested.
  - A run at 5x speed dropped every test as "script delayed" (median call gap 41 ms, 53
    gaps over 100 ms). SITL delays run on simulated time while Lua work takes real time
    (AP_HAL_SITL/Scheduler.cpp L129-159), so speed-up inflates the gaps.
  - At real-time speed the calls came every 23 ms (median), at most 31 ms apart.
  - Real-time result, check manoeuvres flown alternately with old (A) and new (B) gains:

    | Axis | Error A | Error B | Model prediction | Servo activity A -> B | Outcome |
    |---|---|---|---|---|---|
    | Roll | 3.63 deg | 2.53 deg (-30 %) | 3.61 -> 2.40 | 0.17 -> 0.22 | kept, saved |
    | Pitch | 0.97 deg | 0.54 deg (-45 %) | 0.96 -> 0.48 | 0.11 -> 0.37 | old kept (activity over 2x) |

    The prediction matched the flown result within about 5 % on both axes. The pitch
    design was too busy for the activity rule, which rejected it as intended.
  - **Against ArduPlane AUTOTUNE** (python3 -m tools.tuning.sitl_driver vs-autotune, AUTOTUNE_LEVEL 0 so
    TCONST 0.25 and RMAX stay equal):
    - AUTOTUNE reported "Roll: Finished" and "Pitch: Finished".
    - flight_tune.lua then started from the AUTOTUNE gains. Roll: new gains kept, check
      error -15 %. Pitch: the new design tracked 34 % better but more than doubled servo
      activity, so the AUTOTUNE pitch gains stayed.
    - The same FBWA stick card (ArduPilot's own angle loop, +-27 deg roll and pitch
      steps) was then flown with both sets, alternately, and scored by card_eval.py
      against the ideal angle-loop response:

      | Axis | AUTOTUNE gains | flight_tune gains | Steps |
      |---|---|---|---|
      | Roll | 1.22 deg mean error | 1.01 deg (-17 %) | 8 each, every step better |
      | Pitch | 0.73 deg | 0.72 deg (same gains in both: run-to-run repeatability) | 8 each |

    - This is the SITL plane-elevon model at one speed, not the Peron15.
  - Still to show: the timing on the real flight controller (timing_probe.lua).

### Independent review (Codex, 2026-10-11) and what was changed

Output: the session scratchpad, codex_ftune_out.md. The reviewer read the code and the
Plane-4.6.3 sources; it could not run the tests or SITL.

Fixed, each with a test:

- **Takeover in every state.** Stick input, any mode change (also between allowed modes)
  and the safety checks now stop the session while waiting and computing as well, not
  only during manoeuvres.
- **Frozen settings.** Gains, angle-loop and filter settings, MIXING_GAIN, SERVO1/2_TRIM,
  SCALING_SPEED and AIRSPEED_MIN/MAX are snapshotted; a change stops the session.
  SERVO_AUTO_TRIM 1 is refused.
- **Elevon saturation** after the mixer (servos.cpp clips each elevon at +-45) now drops
  a test record.
- **Refused settings:** D_FF is now refused, as notch filters and PDMX were.
- **Failsafe checks.** has_ekf_failsafed is not implemented by Plane (always false), so it
  is removed. AHRS health and fence breaches are checked.
- **Gain writes** are read back. A failed write stops the session and restores the old
  gains. An axis counts as done only after its write succeeded.
- **Step simulation:** the delay is exactly the intended number of steps; the lag and the
  roll pole are discretised exactly; I = 0 clears the integrator, as in AC_PID.
- **Check rules.** Overshoot of kept gains is capped at 20 % even when the old gains
  overshoot more. Servo activity is measured in deg/s, not per script call.

Not closable in Lua:

- If the script dies (instruction overrun, out of memory) during a check with new gains,
  ArduPlane drops script control after 1 s, but the new gains stay until reboot.
- Lua in 4.6 cannot read airspeed-sensor health.
- Script sampling is about 50 Hz with jitter (tolerated, see above).

The reviewer recommends firmware ownership of takeover, safety supervision, temporary
gains and synchronised sampling, with fitting and search allowed to stay as background
work.

### Decision: ArduPlane C++ (owner, 2026-10-12)

The owner proposed building the tuner into ArduPilot because the old script could not
keep roll timing. The independent review recommended the same for the flight-critical
part, because only firmware can always restore the old gains. The Lua script stays as
the reference prototype; it is not for flight.

| | Lua script (prototype) | C++ in ArduPlane (chosen) |
|---|---|---|
| Sampling | about 50 Hz, timing set by the scripting thread | every main loop, gyro and output from the same loop |
| Gains if the tuner fails | stay until reboot | restored in the same loop on every stop |
| Firmware | stock ArduPlane 4.6.3 | the fork's custom build, which the aircraft already fly |
| Limits | 10000 instructions per call, 100 locals | 1300-byte stack frames on the boards |

## Firmware tuner (AP_FlightTune, gartfeo/ardupilot)

Status: SITL only, not cleared for flight. Three independent reviews; the findings are
fixed or listed below as open. The last round of fixes (after the third review) has not
been reviewed again. Bench tests on the boards are still to do.

- **Where:**
  - WSL worktree ~/fw-worktrees/ardupilot-flight-tune, branch feat/flight-tune, based on
    the fork's ArduPilot-4.6 branch (4c23c0a1ff). Uncommitted.
  - The aircraft's current firmware is that branch at 96f7a82664: Plane-4.6.3 plus the
    swarm-scripts and navlink commits. The 20 commits between them are almost all SITL
    changes.
  - The branch records an older MAVLink submodule commit than its SITL code needs; the
    worktree uses a38fe8ab, as ~/ardupilot does.
- **Files:**
  - libraries/APM_Control/AP_FlightTune.{h,cpp}: engine.
  - libraries/APM_Control/AP_FlightTune_Math.{h,cpp}: calculations, no HAL.
  - libraries/APM_Control/tests/test_flight_tune_math.cpp: 14 gtest cases.
  - ArduPlane/flight_tune.cpp: glue.
  - Small edits:
    - aux function FLIGHT_TUNE = 240 (RC_Channel.h);
    - parameter group FTUN_ at G2 index 39;
    - a branch in Plane::stabilize() before nav scripting;
    - Mode::enter stops a session;
    - a 50 Hz scheduler task for the calculations;
    - rate-PID and angle-gain accessors on the roll and pitch controllers.
- **Use:** FTUN_ENABLE 1 and RCx_OPTION 240. FTUN_CHUTE_CH is the parachute switch
  channel (the aircraft's parachute switch). SERVO_AUTO_TRIM must be 0. Fly LOITER, CRUISE or FBWB and
  move the switch high.
- **Differences from the Lua prototype:**
  - Sampling and timing are exact. The gyro and the controller output come from the same
    loop, so the measured response contains the whole loop delay.
  - Speed and height stay with the mode's TECS. Its throttle is used throughout, except
    during pitch tests, when the throttle is held. Its pitch demand is the pitch hold in
    roll manoeuvres. A first SITL run with frozen throttle lost 25 m in one check.
  - Summary log records use WriteCritical, so they cannot be dropped.
  - Fit quality and the crossover cap use only test frequencies whose record-to-record
    scatter is acceptable.
  - Margins at the edge speeds are checked at 0.75x and 1.33x the test speed, within
    AIRSPEED_MIN..MAX. Further out, the model's speed scaling is not trustworthy, so tune
    at a second speed for full-envelope cover.
  - Servo activity above 2x the old gains is accepted only if AC_PID's slew-limiter
    modifier (Dmod) stayed 1.0 in every new-gain run, and activity stayed below a quarter
    of SMAX.
  - After a rejection for servo activity only, one gentler design is checked again.
- **Builds:**
  - SITL: no warnings in the new files.
  - Pixhawk 6X: 12.9 KB of flash is left after the review fixes (19.9 KB before).
  - Durandal: 109 KB of flash is left.
  - The 1300-byte stack-frame limit on the boards required moving the fitter's arrays
    into the object.
- **SITL results** (plane-elevon model, 200 Hz loop), with every review fix in:

  | Run | Roll | Pitch |
  |---|---|---|
  | From SITL default gains, check error | -56 % (predicted -57 %) | -44 % (predicted -44 %) |
  | From completed AUTOTUNE gains (level 0, equal TCONST), check error | -17 % (predicted -18 %) | -36 % (predicted -33 %) |
  | FBWA stick card through ArduPilot's own angle loop, AUTOTUNE gains vs tuner gains, mean error | 1.10 -> 0.92 deg, 8 of 8 steps better | 0.73 -> 0.48 deg, 8 of 8 steps better; overshoot 10 % -> 1.5 % |

  Before the review fixes the default-gain run gave -34 % roll and -45 % pitch, and the
  AUTOTUNE card 1.10 -> 0.92 deg roll and 0.66 -> 0.44 deg pitch. AUTOTUNE's own result
  differs from run to run.
- **Fault injection in SITL** (python3 -m tools.tuning.sitl_faults): each fault is
  injected while a check run has the new gains installed; gains read 2 s after the fault
  and again 3 s later. All 7 pass on the final build.

  | Fault | Stop reason | Old gains back (2 s) | 3 s later |
  |---|---|---|---|
  | Tuning switch off | switch off | yes | yes |
  | Mode change (LOITER to CRUISE) | mode changed | yes | yes |
  | RLL_RATE_FLTD changed from the ground station | roll settings changed | yes | yes |
  | Roll stick | stick input | yes | yes |
  | Pitch axis: tuning switch off | switch off | yes | yes |
  | Second new-gain check run: mode change | mode changed | yes | yes |
  | AUTOTUNE switched on over MAVLink | AUTOTUNE | AUTOTUNE was tuning (P 0.105) | yes: switched off, AUTOTUNE gave back exactly the old gains, so it started from them |

  SITL is not the Peron15. These numbers show that the method works and matches its own
  predictions, not that the real aircraft improves by the same amount.

### Independent review of the firmware (Codex, 2026-10-12) and what was changed

Output: the session scratchpad, codex_fw_out.md (13 findings). The reviewer read the code
and Plane-4.6.3; it could not build or fly. Fixed:

- **AUTOTUNE switch.** The FW_AUTOTUNE aux handler now stops the tuner before
  autotune_enable(true), so AUTOTUNE never takes a candidate's gains as its start.
- **Stop ordering.** The tuning switch going low stops at once in the aux handler. The
  background calculation does nothing while any stop condition holds. Accepted gains are
  set only by the main loop, after that loop's own checks passed.
- **Bounded calculation.** Step simulations run at most 100 loops per piece and resume on
  the next call. The longest piece is logged (FTPR.CUs, FTD2.CUs); SITL cannot measure it
  (its clock stands still inside a task), so it must be read on a board.
- **Settings watch, every loop, in every state:** angle-loop and filter settings, SMAX,
  PDMX, D_FF, notches, the gains (the old ones, the kept ones, or the candidate during its
  own check), MIXING_GAIN and MIXING_OFFSET, SCALING_SPEED, AIRSPEED_MIN/MAX, trim, min,
  max and reverse of the channels driving the surfaces (elevons, else aileron and
  elevator), and SERVO_AUTO_TRIM.
- **Roll RMAX.** The roll controller limits both directions with RLL2SRV_RMAX; the
  simulation used the pitch-style down limit.
- **Other control.** Nav scripting or quadplane code: refused, and a stop if it appears.
- **Check rules.** Old-gain and new-gain runs must fly within 1.5 m/s of each other. New
  gains may saturate a surface for at most 2 % more of the check. The slew limiter may not
  cut P and D more than with the old gains. Saturation counts both elevons.
- **Envelope.** Stops when no longer flying. The airspeed floor rises with the bank's load
  factor (AIRSPEED_MIN times the square root of 1/cos(bank)). Height is the lower of
  height above home and height above terrain, when terrain data exist.
- **Model checks.** At least 6 test frequencies with acceptable scatter. The crossover cap
  rests on the unbroken run of good frequencies, not on a lone good point above a gap.
  Margins fail when the loop still has gain at 150 rad/s (the top of the analysis), where
  they used to read "no crossover". The pitch model in the step simulation is now
  discretised exactly; the old explicit update diverged at 50 Hz for stiff, well-damped
  fits.
- **Saving.** FTUN_SAVE defaults to 0: accepted gains stay in RAM until reboot, for review
  on the ground. With FTUN_SAVE 1 the message says the save is queued, because ArduPilot
  writes parameters in the background.
- **Bad values.** Non-finite sensor values stop the session. FTUN_ parameters are checked
  against ranges at start.

A follow-up review of these fixes (codex_fw2_out.md) found gaps, now fixed:

- **AUTOTUNE over MAVLink or a mission** bypassed the switch-only stop. The stop is now in
  Plane::autotune_start(), which every way into AUTOTUNE passes.
- **Gain writes from the background calculation** are gone. Only a check run with the new
  gains has them installed, and only a stop during such a run writes the old ones back.
  The calculation also pauses while any setting differs from its snapshot.
- **Settings watch** now also covers the output channel of each surface function,
  PTCH2SRV_RLL and the FTUN_ settings themselves. Parameters found by name (notches,
  PTCH2SRV_RLL) are looked up once per session, not every loop. TCONST below 0.05 is
  refused, because the controllers raise it in flight.
- **Check rules are absolute where they must be:** each new-gain run is compared with the
  old-gain run beside it for airspeed (not the averages). No surface may reach its limit
  on a hold (the settle phase and the last 0.8 s of each step) with the new gains. The
  slew limiter must not act at all with the new gains. Activity is the largest of the
  controller output and both elevons.
- **Saturation policy, and why.** A first version capped saturation at 2 % of a check.
  That would have rejected AUTOTUNE's own roll gains (7 % in SITL) and the tuner's better
  design (5.6 %). An instant 20 degree attitude step asks for more roll acceleration than
  any surface gives, so full surface for a moment at each step start is the best answer;
  the risk is a surface at its limit once the aircraft should be holding, with no control
  margin left. The search screens designs the same way: it used to cap the simulated peak
  output at 35 degrees, which left 0 to 2 of about 350 roll designs from SITL's default
  gains and threw out the best ones (simulated error 3.62 -> 1.58 deg, 3 % overshoot).
  It now rejects only designs at the limit on the simulated hold, and allows the
  overshoot the flight check allows (10 %, or the current gains' if higher, at most 20 %).
- **Height:** loss is measured above home; clearance is the lower of height above home and
  above terrain, and losing terrain data during a manoeuvre stops it. A roll manoeuvre
  starts only above the stall speed for its planned 35 degree bank (load factor) plus
  FTUN_SPD_MARGIN.
- **Simulation:** the whole plant, attitude included, is discretised exactly for an output
  held over each loop. A unit test compares it with a fine independent integration of the
  continuous model (within 0.1 %); another shows identical results however the work is
  split into pieces, including a restart part way.
- **Model coverage:** the crossover at the test speed may not lie below the lowest
  reliable test frequency of the run the cap rests on.
- **Timing:** the log has both the longest calculation piece and the longest whole call
  (FTPR and FTD2: PcUs, CaUs), for the bench test on the boards.
- **Fault injection in SITL** (python3 -m tools.tuning.sitl_faults): see the results below.

A third review (codex_fw3_out.md) confirmed the AUTOTUNE handoff and the plant fixes and
found more gaps, now fixed:

- **Saturation at step starts was unbounded.** A check run with the new gains is now cut
  short at once, and the old gains kept, when a surface stays at its limit longer than the
  axis TCONST (the loop has no authority left), or when the other axis strays from its
  ideal angle-loop response by more than its own check step (5 deg pitch during roll
  checks, 20 deg roll during pitch checks). The ideal response has the same limits as the
  tuner's demand, including the 45 deg/s levelling cap: LOITER leaves the wing at up to
  45 deg bank before each run, and a first version without the cap read a false 20 deg. This covers every stage of the run, including
  levelling. The search and the simulated full check apply the same limits, and the
  simulated check must pass the flight check's overshoot rule before new gains are flown.
- **Terrain:** once terrain clearance is known during a manoeuvre, losing it stops the
  manoeuvre, also when it was not known at the start.
- **Mixer topology** (elevons or aileron and elevator) is part of the settings watch.
- **Airspeed** is compared step by step between each new-gain run and the old-gain run
  beside it, not as run averages. A mismatch makes the comparison invalid, not the new
  gains worse, so the checks are flown again once before the old gains are kept. In one
  SITL run a new-gain pitch run started 2.2 m/s slower than its old-gain partner, and a
  43 % better design was rejected before this repeat existed.
- **Timing:** the whole-call time now includes the entry checks.
- **Fault injection** now also covers the pitch axis and the second new-gain check run.
- **Coverage gaps get more data.** The minimum of 6 good test frequencies now counts only
  the unbroken run the crossover cap rests on. When one noisy frequency breaks the run
  below good ones, the tuner flies 2 more tests (up to 8) before designing. In one SITL run
  a single point (24 rad/s, scatter 0.55) had cut the cap from 12 to 9.5 rad/s and the
  roll improvement from 56 % to 41 %.

Not fixed, and why:

- Margins cover 0.75x to 1.33x the test speed. Tune again at a second speed for the rest
  of the envelope.
- Fitter steps and the design tables are single calculation pieces; their time on the
  boards is logged, not yet measured.
- Another writer saving parameters during a check run (a script, a ground station) is not
  locked out. Flight procedure: no other tuning tools or parameter writes during a tune.
- The simulation counts the half-loop output hold twice (the measured response already
  contains it): 2.5 ms extra delay at 200 Hz. In SITL the predictions still matched the
  flown checks within a few percent.
- The top of the P grid (2.83x the reference P) and of D/P (0.03) bound the chosen roll
  design in SITL; the margins would allow a little more.
- The fault runner reads the gains 2 s after a fault, not in the same loop; the same-loop
  restore rests on the code path (stop_session in the switch handler, update() and
  Mode::enter). The decision-to-apply boundary, terrain changes, disarm and failsafes are
  not fault-injected.
- Board timing, servo behaviour under load, and the aircraft's stall, load and recovery
  limits need bench tests and a staged first flight.
- The models cannot show structural modes or cross-coupling, and mechanical servo rate
  limits are not measured. These need bench tests and first flights with logs reviewed.

## What AUTOTUNE does (ArduPilot 4.6, libraries/APM_Control/AP_AutoTune.cpp)

Line numbers below are from the local ArduPilot-4.6 copy in WSL at ~/ardupilot
(describe Copter-4.6.3-22).

- An event starts when the demanded rate is above 0.4 x min(limit/TCONST, RMAX) and the
  angle error is at least 30 % of the attitude limit (L225-227).
- FF is computed per event from peak actuator / peak rate, using the median of 5 events.
- After event 4, D and then P are multiplied by 1.3 after each clean event, until the
  rate controller's slew limiter reduces P+D (Dmod < 1). Then they are cut. The first
  cut is D x0.3, or P x0.35 with D x0.75 when P dominates. Later cuts are x0.35 or x0.7,
  depending on state (L354-407). An axis is finished after 3 clean events with both
  limits found (L409-415).
- I is set to min(P, FF) for roll and max(0.75 P, FF) for pitch (L424-426).
- TCONST and RMAX come from the AUTOTUNE_LEVEL table (L75-88). FLTT is set to
  10/(2 pi TCONST) and FLTD to half the gyro filter.
- Gains are saved at "Finished", or at stop when both limits were found. Otherwise all
  gains go back to their values at session start (L142-152).

So the margin is set by where the slew limiter first acts, which depends on SMAX, noise
and the day's turbulence. AUTOTUNE uses no aircraft model and does not check left/right
response. The fleet's newest parameter files are consistent with level-8 settings, but that does
not prove AUTOTUNE was completed on each aircraft.

## Stage 1: pilot-flown card, ground analysis (no new flight code)

### Flight card (pilot-flown, FBWA, current gains frozen)

- **Before flight:**
  - LOG_BITMASK bit 20 set, so ATT, PIDR and PIDP are logged at the loop rate
    (ArduPlane/Log.cpp L61-67). The flight must also log MODE messages; the old logs have
    none, and the cause must be found first.
  - No auxiliary AUTOTUNE switch (RCx_OPTION 107) is active, so gains stay frozen.
- **Calm air.** The test height, speed band and abort heights come from the operator's
  test card.
- **Each axis, repeated at 2 airspeeds:**
  - **Roll:** steps from level to +-20 and +-30 deg, each held 4 s, back to level in
    between. Then sharp doublets of +-15 deg at about 0.5 s and 1 s per half cycle.
  - **Pitch:** steps of +-5 deg held 3 s, then doublets of +-4 deg. Throttle is held
    constant.
  - Also 20 s of hands-off level flight per speed, to measure noise and turbulence.
- **Baseline AUTOTUNE gains:** use a completed AUTOTUNE, shown by "Finished" messages in
  the log and saved values read back. If there is none, fly AUTOTUNE as ArduPilot
  documents, then fly the card with those gains frozen.

### Ground tool (tools/tuning)

1. **Controller port.** The ArduPilot 4.6 roll and pitch controllers are ported to
   Python (ap_controller.py): rate loop, AC_PID filters, slew limiter, integrator rules,
   speed scaler and the angle loop.
   - The port is checked against logged PIDR/PIDP terms (port_check.py).
   - SITL result in FBWA: roll FF/P/D errors are 0.1-0.4 % of the signal RMS and pitch
     errors 0.7-1.8 %. Dmod matches exactly.
   - The I term drifts slowly with the 25 Hz airspeed interpolation.
2. **Identification.** A closed-loop output-error fit drives the whole loop from the
   logged attitude demand (ATT DesRoll/DesPitch), through the controller port and the
   candidate plant model. The fitted output is the simulated attitude and body rate.
   - The simulated actuator input depends only on the demand and the model, not on
     measured noise. This avoids the bias of fitting measured controller output against
     measured rate under feedback.
   - Clipping to +-45 and the elevon mixing limits are modelled. Saturated segments are
     rejected.
   - **Candidate roll structures:**
     - first order with delay;
     - the same plus an actuator lag;
     - separate gains for positive and negative inputs, plus a trim offset.
   - **Candidate pitch structures:** second order with a zero, with and without an
     actuator lag, plus a speed-dependent trim.
   - **Scaling:** effectiveness scales with EAS squared; damping is fitted with and
     without airspeed scaling.
   - **Selecting a structure:** prediction error on manoeuvres not used for the fit,
     residual whiteness and correlation with the demand, and parameter spread over
     repeated manoeuvres (bootstrap).
3. **Gain design.**
   - **Search:** FF, P, I, D and TCONST, with RMAX, IMAX and the filters kept within
     bounds.
   - **Cost:** tracking error on the card demands, at the card airspeeds, over the set of
     models that fits the data (the bootstrap spread, not a fixed +-25 %).
   - **Constraints, on every model in that set:**
     - rate loop and complete attitude loop: gain margin at least 6 dB and phase margin
       at least 45 deg, including the logged loop delay;
     - no output saturation on 30 deg roll steps;
     - Dmod stays 1 on the card manoeuvres in simulation.
   - **Turbulence:** modelled as an input disturbance identified with the plant, not
     replayed from residuals of the old controller.
4. **Outputs:**
   - a parameter file;
   - predicted metrics with uncertainty;
   - the same predictions for the baseline gains.

### Acceptance (fixed before the verification flight)

Fly the card alternately with baseline and new gains (A-B-A-B), same aircraft, mass, speed
band and air.

- **Measured per step and per direction:**
  - time to 90 % of the step;
  - overshoot;
  - mean absolute error over 1-4 s after the step;
  - left/right difference;
  - in level flight, RMS attitude error and servo activity.
- **Keep the new gains only if all of these hold:**
  - roll and pitch overshoot at most 10 %;
  - no oscillation, and Dmod = 1 on the card;
  - mean absolute error is at least 15 % lower than baseline in both directions, with the
    improvement larger than the run-to-run spread of the repeats;
  - measured time to 90 % is within 30 % of the prediction.
- **Otherwise:** load the saved baseline parameter file.

TCONST/RMAX changes and rate-loop changes are reported separately, so it is clear which
one improved the result.

### Existing tools used as independent checks

- ArduPilot WebTools PID Review: frequency-domain review and step response estimate.
- ArduPilot WebTools Filter Review: gyro noise and filters.
- Native AUTOTUNE: the baseline.
- Plane 4.6.3 has no native fixed-wing excitation: its SystemID mode reports "SystemID:
  only for quadplane" (ArduPlane/systemid.cpp L106-107). Fixed-wing SystemID axes exist
  only in later firmware.

## Stage 2 (replaced): stick-override automation of the card

Replaced by the in-flight tuner above, which uses script rate control instead of RC
overrides. The requirements below were written for RC overrides; the ones about ownership,
failure, energy, frozen gains and pre-flight evidence still apply to the in-flight tuner.
They were:

1. **Pilot takeover.** Overrides are held for at most 2 s, then released for at least
   1.5 s.
   - Takeover is immediate through the mode switch or the tuning switch. Sticks are read
     only during releases.
   - RC-loss detection latency is at most 2 s + 1 s, and is measured in SITL with
     SIM_RC_FAIL.
2. **Ownership.**
   - The script clears only channels it wrote in this run, once.
   - It latches "parachute active" for the rest of the flight once the parachute switch
     has been seen high, or once chute_deploy.lua's mode change has happened.
   - The old tune_pitch_roll.lua must be removed from every aircraft first.
3. **Failure.**
   - A gap between script runs of more than 100 ms aborts the card; the script does not
     resume it.
   - RC_OVERRIDE_TIME (0.55 s in the fleet's files) is the fallback if the script is removed, and
     it is tested by forcing an instruction-count overrun in SITL.
   - pcall cleanup is extra protection only.
4. **Energy.**
   - Start requires terrain clearance where terrain data exist, otherwise height above
     home, with a start margin above the abort floor.
   - Start also requires an EAS band and an airspeed that agrees with GPS ground speed and
     the wind estimate.
   - An abort hands control to the pilot's sticks in the current stabilised mode. The
     test card states the recovery mode.
5. **Commands.**
   - Demands are converted with norm_input semantics (no deadzone), using RCn_MIN, TRIM,
     MAX and REVERSED.
   - The conversion is checked against ATT demands in SITL.
   - RCMAP collisions with the throttle, mode, tuning and parachute channels are refused.
6. **Gains stay frozen during cards.**
   - The card refuses to run if the auxiliary AUTOTUNE switch is on, or if a gain changes
     during the card.
   - An AUTOTUNE driver, if wanted, is a separate card with direct reversals and its own
     axis settings (AUTOTUNE_AXES). It stops only by mode change.
7. **Before first flight:**
   - Lua harness tests for every path.
   - SITL tests of RC loss, parachute interplay in both script orders, and an overrun.
   - Propeller-off tests on the aircraft.
   - A firmware/script manifest with parameter readback.

## Defects in the current tune_pitch_roll.lua (checked by Codex against the code)

1. When the interval knob is outside RC11_MIN..MAX, the script writes RC_OVERRIDE_TIME to
   permanent storage, as 0.55 or 5.05 s (L175-185). The old value is taken before the
   clamp.
2. There are no airspeed checks. With PTCH_LIM_MIN_DEG -45, full forward stick demands
   -45 deg. Pitch-down reverses on its timer, after 20 m of height loss, or below the
   minimum height (L242-260).
3. The roll height floor is -10 m (L17, L26 and all three parameter files).
4. When its switch is off, in FBWA it clears RC1/RC2 overrides every 10 ms (L109-113,
   L335-337). In other modes it also clears them repeatedly (L293-300). chute_deploy.lua
   holds pitch up through an RC2 override in FBWA, and both scripts are loaded on the fleet's aircraft.
5. Continuous overrides hide a silent receiver while the frozen throttle is still above
   the failsafe value.
6. A failed height check is skipped on the next run, because original_altitude is set
   before the check returns (L303-323). Pitch-up can continue indefinitely while waiting
   to regain the starting height (L244).
7. There is no armed or flying check, and RC1/RC2 are hard-coded. RC1_REVERSED is 1 in
   the files, so its "right" is actually left.
8. Roll reversals run on a fixed timer, with no response check. It writes only GCS text,
   no structured log.
9. Four parameter names are longer than 16 characters and are stored truncated.

## Status and open items (after the second Codex review, 2026-10-11)

Done and checked:

- **Source.** The local ArduPilot copy (commit 4c23c0a1ff) is identical to tag Plane-4.6.3
  for every file ported or cited:
  - AP_Roll/PitchController, AC_PID, SlewLimiter and SRV_Channel;
  - ArduPlane servos.cpp, Attitude.cpp and systemid.cpp;
  - AP_AutoTune.
- **Controller port.** It matches logged PIDR/PIDP in SITL (see above). Unit tests pin the
  integrator, underspeed, slew-limiter, coordination and scaler behaviour
  (tests/tools/tuning).
- **Known-plant identification test.**
  - The roll pole and the actuator lag can swap roles, so the parameters are not unique.
  - With 0.2 s 3-2-1-1 pulses, the fitted frequency response matches the true one within
    15 % in gain and 8 deg in phase at 2-10 rad/s.
  - With 0.5 s pulses, the phase error reached 12 deg at 20 rad/s. The card therefore
    needs short, sharp inputs.
- **Parachute conflict (SITL, both scripts loaded, old tuner switch off).** After the
  parachute switch went high:
  - pitch reached 4 deg instead of 33 deg;
  - throttle stayed at full;
  - the parachute fired on its pitch timeout, at speed.

Still open before stage 1 is flown:

1. Remove tune_pitch_roll.lua from every aircraft, and read back the script list.
2. Set the trims, then turn SERVO_AUTO_TRIM off for identification and A/B flights.
   Auto-trim moves SERVOn_TRIM, which changes the command-to-surface mapping (see
   model_id.ElevonMap). Snapshot the trims with the gains.
3. Find why MODE records are missing; check full-rate logging completeness on the real
   board.
4. Write the card as an executable protocol:
   - mass and CG, EAS bands, clearance and recovery;
   - repetitions and timing, including sharp 0.2-0.3 s pulses;
   - rules for which segments to accept (no pilot corrections inside a segment, a
     speed-drift limit).
5. Fix the acceptance windows to match the hold times, and define oscillation,
   run-to-run spread, servo-activity and cross-axis limits. Add an equal-TCONST
   comparison.

Still open before trusting the analysis:

6. Port check for the angle loop and coordination (ATT demand to PIDR/PIDP Tar), the
   active slew-limiter branch (SITL with a low SMAX), the summed and clipped output and
   mixer reconstruction (against RCOU).
7. Reject unsupported configurations: notch filters, missing parameters, inverted flight,
   no airspeed. Match the C++ edge cases (non-finite input, dt <= 0).
8. Identification:
   - fit at two or more airspeeds, which also resolves the pole/lag ambiguity in the
     speed scaling;
   - pitch model choice (the first SITL pitch fit predicted held-out data poorly);
   - noise models;
   - frequency-response checks near crossover;
   - the cross-axis responses.
9. SITL proof: fly AUTOTUNE gains against the designed gains on the same card, A-B-A-B,
   with turbulence on.
10. A code review of tools/tuning.

## HITL on the Pixhawk 6X (HIL link contract v2, 2026-10-13)

The board runs the fork's HIL firmware (branch feat/sim-hil, board Pixhawk6X-HIL). Its sensors
are ArduPilot's simulated backends, fed by the PC; it sends its servo outputs once per main loop.
The contract is the fork's `libraries/SITL/examples/HIL/readme.md`. The reference bridge is
`Tools/autotest/pysim/hil_bridge.py`, and the board guide is
`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X-HIL/README.md`. navpy's side:

| Piece | File | Role |
|---|---|---|
| Bridge | `tools/tuning/hitl_bridge/` (`bridge.py`, `frame_loop.py`, `board_clock.py`, `link.py`) | Answers every HIL_ACTUATOR_CONTROLS with one HIL_STATE_QUATERNION at the same `time_usec`, from the JSBSim Peron15 plant (model `Peron15_hitl3`, unchanged). |
| Relay | `tools/tuning/hitl_bridge/relay.py` | Copies bytes between the Windows COM port (COM11) and the bridge's TCP listener in WSL, next to the JSBSim program. |
| Runner | `tools/tuning/hitl.py`, `board.py`, `hitl_procs.py` | Writes the parameters, flies the card over COM15, starts and stops the bridge and relay, and fetches the logs. |
| Mission Planner mirror | `tools/tuning/mp_mirror.py` | UDP 9640 copy of the runner's COM15 link. |
| Gate | `tools/tuning/hitl_gate.py` | The board's `HIL` records plus the bridge's `link.csv`; a failing gate fails the run. |

### What v2 changed against v1

- **Frame format.** `flags` is 0, or 1 (lockstep) only from SITL's `hil:lockstep` model; a board
  never waits for the PC. v1 sent a frame counter in `flags`. The bridge now uses no counter: it
  detects reboots and lost frames from board time alone.
- **Ignored frames.** MAVLink 1 frames, frames carrying the bridge's own system id (251) and other
  message types are ignored and counted (`LinkStats.rx_mavlink1`, `rx_loopback`, `rx_other`).
- **Causal grid rule.** The plant is a whole number of fixed JSBSim steps (1/1200 s) since the
  first frame, counted in integers. The reply to frame k is the state at the last step at or
  before t_k. Outputs u_k act from the first step at or after t_k: a step that straddles t_k
  still uses u_(k-1), and of several frames before one step only the newest acts.
- **Reboot.** Board time going back, or jumping more than 2 s, re-initialises the aircraft at the
  origin. If the previous frame was armed, the bridge prints an `ALERT ... ARMED` line and counts
  it, and the run gate fails it (`resets_after_armed`).
- **GAP.** A jump of more than 1.5 loop periods (`--loop-rate-hz`, default 200) is stepped across
  with the held outputs and reported as a GAP.
- **Equal time.** The same frame again (a lockstep SITL resending after a reconnect) is answered
  with the same state, with nothing stepped (`retransmit`). Other content at the same time is
  reported as an `ERROR` and answered with the state at that time (`conflict`).
- **Serial port.** It is flushed on every open (bridge and relay), and reopened after a USB
  re-enumeration.
- **Board parameters.** `SERIAL8_PROTOCOL 100` (SIM HIL) is now required: v1 had a compiled-in
  port and set -1.
- **SITL stand-in.** SITL manages only SERIAL0-7, so `hitl sitl-backing --model hil` now serves
  the link on `SERIAL5` with `SERIAL5_PROTOCOL 100` (`tools/tuning/sitl/hitl_sitl.parm`).
- **Run gate.** It reads every `HIL` field: `TimeUS,LS,Age,AgeMx,Stale,Rx,Rej,Rpl,Tx,TxDrop,CrcE,ParE,Oth,Lp`.
  Inside the armed flight:
  - `LS` stays 2 (ok);
  - `AgeMx` stays at or below 50 ms, and at most 5% of the frames are stale;
  - `TxDrop`, `Rej`, `CrcE` and `ParE` do not grow, counted as changes between records after the
    link is up. `TxDrop` grows from boot until the PC reads the USB port, so its absolute value
    means nothing;
  - `Tx` plus `TxDrop` grows exactly as `Lp`, one frame per main loop.

  `Rpl` and `Oth` are reported, not gated. From the bridge's `link.csv`, the gate also fails a
  reset to the origin after the first armed frame and any conflicting frame; retransmissions are
  counted only.

### Clock determinism (owner rule)

Physics advances only on board timestamps: whole fixed JSBSim steps with a zero-order hold. The
PC clock is used only to stamp the link log and to pace reconnects and timeouts. Nothing is
random, and the board never waits for the PC. Tests in `tests/tools/tuning/test_hitl_bridge_v2.py`
check this:
- the same board frames give bit-identical replies, whatever the read chunking;
- replaying a session with every PC clock patched to raise, and again with every random source
  patched to raise, still gives the same replies;
- a static check that the physics-path modules import no clock and no random source.

Conformance is checked byte for byte against replies recorded from the reference bridge
(`tests/tools/tuning/hil_v2_reference_vectors.json`).

### Board parameters

- `tools/tuning/sitl/hitl_board.parm`: the HIL defaults of the fork's `HIL.parm` and
  `HAL_OTG2_PROTOCOL`. These are `GPS1_TYPE 100`, `ARSPD_TYPE 100`, `BATT_MONITOR 0`,
  `BRD_HEAT_TARG -1`, `SIM_OH_MASK 0`, `SERIAL8_PROTOCOL 100` and the accelerometer calibration
  markers. The file also sets `BRD_SAFETY_DEFLT 0`, which is safe only behind the runner's
  `SIM_OH_MASK == 0` gate. The runner loads it with `--bridge` on a serial board.
- The aircraft's own data is private and stays out of this repository (`tools/tuning/private_data.py`):
  the models, the aircraft's parameter files and, in the `hitl` subfolder, `hitl_aircraft.parm`
  (airspeeds and takeoff settings, loaded with `--bridge` after `hitl_bridge.parm`), `hitl_launch.parm`
  (launch detection and motor delay, loaded after the starting parameters) and `launch.json` (the
  measured catapult rail and the rest pose on the skids, read by the bridge). The folder is
  `NAVPY_HITL_TOOLS_DIR`, else `%LOCALAPPDATA%\navpy-hitl-tools`. The runner takes `--hitl-data DIR`
  and the bridge `--launch-file FILE` to read them elsewhere (a WSL bridge needs `--launch-file`). The
  tests use synthetic data; `tests/tools/tuning/test_hitl_private_data.py` checks the real files against
  their measurements where they exist.
- `hitl\hitl_board_servos.parm` in the private folder (opt-in with `--params-file`): the aircraft's own
  elevon setup (elevon functions, its trims, one servo reversed, its mixing gain).
  - **Gap:** the bridge's elevon unmix (as SIM_JSBSim's) reads each channel per microsecond
    around 1500 us, with both servos unreversed. With trims off 1500 the model sees a standing
    aileron and elevator command at neutral demand. A reversed servo turns part of a roll demand
    into pitch, and the reverse.
  - Use the file to check the board's outputs against the aircraft, not to fly or tune on the
    model. Closing the gap needs the elevon mapping to carry per-surface trim and direction; the
    aero model is unchanged for now.
- `SCHED_LOOP_RATE` must match the bridge's `--loop-rate-hz` (default 200). `peron_like.parm` sets
  200. A board left at Plane's default of 50 would make every frame a GAP.

### Owner commands (Windows PowerShell 5.1)

`<repo>` is the navpy checkout, `<aircraft>` the aircraft's parameter file in the private folder,
`<user>` the Windows user and `<distro>` the WSL distribution.

Bare board, flashed with Pixhawk6X-HIL as described in its README:
1. Turn safety on and power-cycle.
2. Reset the parameters.
3. Set `SIM_OPOS_LAT -35.363261`, `SIM_OPOS_LNG 149.165230`, `SIM_OPOS_ALT 584`,
   `SIM_OPOS_HDG 353` and `SCHED_LOOP_RATE 200`, then reboot.

COM15 is the MAVLink port (OTG1). COM11 is the HIL link (OTG2), and only the relay opens it. Run
each command on one line.

**A. Bench link check, Mission Planner on COM15.**

Window 1 starts the bridge in WSL. It listens on 127.0.0.1:9620 and logs to
C:\tune_hitl\bench\link.csv. `--client-grace-s 0` keeps it waiting for the relay:

```powershell
wsl.exe --cd /mnt/c/<repo in WSL> -- python3 -m tools.tuning.hitl_bridge --device tcpin:9620 --model-dir /mnt/c/Users/<user>/AppData/Local/navpy-hitl-tools/models/Peron15 --launch-file /mnt/c/Users/<user>/AppData/Local/navpy-hitl-tools/hitl/launch.json --origin-lat -35.363261 --origin-lon 149.165230 --origin-alt 584 --heading 353 --loop-rate-hz 200 --client-grace-s 0 --log-csv /mnt/c/tune_hitl/bench/link.csv
```

Window 2 starts the relay, from COM11 to the bridge:

```powershell
Set-Location <repo>; .venv\Scripts\python.exe -m tools.tuning.hitl_bridge.relay --device COM11 --bridge 127.0.0.1:9620
```

Then:
1. In Mission Planner, connect to COM15 (any baud rate). Expect `HIL: waiting for simulator on
   SERIAL8`, then `HIL: first state received`.
2. Set `LOG_DISARMED 1` if the link should also be logged before arming.
3. Arm in MANUAL. Press the safety button first (no actuators connected; flow A does not set
   `BRD_SAFETY_DEFLT`). Wait about 30 s, then disarm.
4. Download the log over MAVLink.
5. Stop the relay, then the bridge, with Ctrl+C.
6. Run the gate:

```powershell
Set-Location <repo>; .venv\Scripts\python.exe -m tools.tuning.hitl link-check --log C:\tune_hitl\bench\00000001.BIN --link-csv C:\tune_hitl\bench\link.csv
```

**B. Tuning run, the runner holding COM15, Mission Planner on UDP 9640.**

Recommended: the bridge on Windows opens COM11 itself (no `--relay`, no WSL), with the jsbsim
package in `.venv`. It reads the launch profile from the private folder:

```powershell
Set-Location <repo>; $env:ARDUPILOT_ROOT = "\\wsl.localhost\<distro>\home\<user>\ardupilot"; .venv\Scripts\python.exe -m tools.tuning.sitl_driver port-check --board COM15 --erase-params --mp-mirror --workdir C:\tune_hitl\<NEW_EMPTY_DIR> --params-file "$env:LOCALAPPDATA\navpy-hitl-tools\params\<aircraft>.parm" --bridge "{python} -m tools.tuning.hitl_bridge --device COM11 --plant package --reconnect-interval-s 0.5 --model-dir $env:LOCALAPPDATA\navpy-hitl-tools\models\Peron15_hitl3 --servo-speed-s 0.03 --origin-lat {lat} --origin-lon {lon} --origin-alt {alt} --heading {hdg} --exit-on-stdin-eof --log-csv {link_csv}"
```

The older WSL bridge with the relay:

```powershell
Set-Location <repo>; $env:ARDUPILOT_ROOT = "\\wsl.localhost\<distro>\home\<user>\ardupilot"
.venv\Scripts\python.exe -m tools.tuning.hitl backup --board COM15 --out C:\tune_hitl\before.parm
.venv\Scripts\python.exe -m tools.tuning.sitl_driver port-check --board COM15 --erase-params --mp-mirror --workdir C:\tune_hitl\sign_check --params-file "$env:LOCALAPPDATA\navpy-hitl-tools\params\<aircraft>.parm" --bridge "wsl.exe --cd {repo_wsl} -- python3 -m tools.tuning.hitl_bridge --device tcpin:{port} --model-dir /mnt/c/Users/<user>/AppData/Local/navpy-hitl-tools/models/Peron15 --launch-file /mnt/c/Users/<user>/AppData/Local/navpy-hitl-tools/hitl/launch.json --origin-lat {lat} --origin-lon {lon} --origin-alt {alt} --heading {hdg} --exit-on-stdin-eof --log-csv {link_csv_wsl}" --relay "{python} -m tools.tuning.hitl_bridge.relay --device COM11 --bridge 127.0.0.1:{port}"
```

While it runs:
- In Mission Planner, pick connection type UDP, Connect, local port **9640**. It must not open
  COM15 or COM11. Never use its legacy Simulation tab.
- The runner writes the run's JSON next to the workdir, with the gate under
  `vehicle.link_health`. A failed gate exits with 1.

Restore the flight parameters after the HIL session:

```powershell
Set-Location <repo>; .venv\Scripts\python.exe -m tools.tuning.hitl restore --board COM15 --file C:\tune_hitl\before.parm
```

Flash the flight firmware back first, then follow "Going back to flight firmware" in the board
README.

### Verified so far (2026-10-13, no hardware)

- **Unit and conformance tests:** `tests/tools/tuning`.
- **SITL hil model end to end.** The fork's SITL `hil` model (real time) was answered by navpy's
  bridge with the JSBSim program in WSL:
  - 31,479 frames, all answered, with no reset, gap, retransmission, conflict or bad frame;
  - arming passed the HIL pre-arm gate;
  - over the armed flight: `LS` 2 throughout, `AgeMx` at most 10.8 ms (median age 5.0 ms), one
    frame per loop, and no growth of `TxDrop`, `Rej`, `CrcE` or `ParE`;
  - the bridge's own turnaround was 1.0 ms median and 3.9 ms maximum.
- **Determinism.** Replaying the recorded board frames offline through the real JSBSim program
  reproduced all 31,479 logged states exactly. Two replays were bit-identical.
- **Open: stale frames in SITL.** The gate failed this SITL run on stale frames: 14.3% against a
  5% limit, a steady 1 loop in 7.
  - The fork's reference bridge on the same SITL binary and setup showed the same pattern
    (11.9%), with a 0.1 ms static model.
  - The stale loops are therefore SITL's real-time pacing in WSL, not the bridge. This is the
    1-in-7 symptom that the fork readme says `pace_realtime_frame` removes.
  - A board's own loop timing replaces this; the bench run will show the real figure.
- Simulation, HIL included, proves nothing about real flight, physical outputs or docking.

### First board runs (2026-10-13, Pixhawk 6X, Pixhawk6X-HIL 9a8c7d66)

Runs `sign_check2`, `fix1`, `fix2` under `C:\tune_hitl`. All failed the link gate.
- **EKF on the wrong pose (fixed).** The bridge started after the board's last boot, so EKF3 aligned
  on the backend's level placeholder and then took the jump to the rail attitude partly as gyro
  bias. It showed about 5 deg too much pitch on the rail and 3-5 deg too much in flight, so FBWA flew a
  steady descent. The runner now starts the bridge and relay, reboots, and checks that
  `HIL: first state received` comes before `EKF3 IMU0 initialised` (`board.py _boot_on_bridge`).
  In `fix1` the rail attitude matched the bridge's, and in-flight pitch agreed to within
  0.5 deg.
- **Start height (fixed).** The aircraft file's lower `TKOFF_ALT` (its real catapult climb) left little
  room above the 30 m floor. FBWA at the pilot's 65 % throttle does not hold height. `takeoff_to_fbwa` now climbs in LOITER to
  `TEST_HEIGHT_M` (120 m) after a lower takeoff.
- **Floor breach (fixed).** The runner switches to RTL and waits until the aircraft is back above the
  floor. Only then does it end the simulated flight. Before, it disarmed during the descent.
- **Bridge crash (fixed).** A state outside the envelope (non-finite, rates over 100 rad/s, or more
  than 1 m under the ground) is now a `crash` event. The board then reads the aircraft at rest where
  it last was, and the gate fails the run. Before, the bridge died with `OverflowError`.
- **Stale frames and stalls (fixed by the Windows bridge).** `relay --timing-csv` stamps every frame on
  Windows:
  - Board-to-relay delivery is steady: 0.5 ms median excess and 1.4 ms p99.
  - In `fix1` the PC round trip was 1.3 ms median and 2.9 ms p99. Bursts of frames still reached the
    WSL bridge 40-50 ms late.
  - In `fix2`, under host load, the WSL physics fell 0.3-0.6 s behind during the climb. The board
    flew on held states and crashed.
  - A 200 Hz TCP ping-pong measured 14.8 ms p99 and 44 ms worst through WSL. Windows loopback
    measured 0.36 ms p99.
  - **Fixed:** the bridge now runs on Windows and opens COM11 itself, with `--plant package` and
    no relay or WSL (jsbsim 1.3.2 in `.venv`, approved by the owner). The runner accepts `--bridge`
    without `--relay` and counts the bridge as up once it prints its first line.
  - Run `fix3` passed the gate with 1.9 % stale and a 12 ms worst state age.
    - Board-to-bridge delivery: 1.35 ms p99 extra delay.
    - Bridge turnaround: 0.55 ms median, 1.2 ms p99.
- **Package plant equals the program plant.** jsbsim 1.3.2 differs from the WSL program (FlightGear
  2020.3.18) in two ways that matter for Peron15_hitl3:
  - **Propeller ixx:** 1.3.2 keeps the file's value. The program floors it at 0.001 slug ft2, and
    the model's gyroscopic couples assume that floor.
  - **Spin-up torque:** 1.3.2 applies the whole engine torque to the airframe while the propeller
    spins up. The program applies only the propeller's aerodynamic torque, so at a full-throttle
    start the package rolled the airframe hard and the program did not.
  - The package plant loads a copy of the model with the ixx floor and a roll couple that removes the
    spin-up term (`jsbsim_plant.prepare_package_model`).
  - Closed-loop check, 20 s from the rail release, same commands on both plants. After the
    correction the roll differs by at most 0.25 deg, and after 12 s pitch by 0.06 deg and airspeed
    by 0.09 m/s. Height drifts 1.3 m in 20 s from a slightly slower thrust build-up in the first
    0.7 s. Before the correction the roll differed by up to 8.5 deg.
  - Tests: `tests/tools/tuning/test_hitl_bridge_package_plant.py`. Its integration tests load the
    owner's model from `%LOCALAPPDATA%` or `NAVPY_HITL_MODEL_DIR` and skip when it is absent.
- **Servo lag.** Peron15_hitl3's README asks for `--servo-speed-s 0.03`. The first runs used the
  bridge default of 0.14 s; the Windows template sets 0.03.
- **Tuner parameters (open, needs the owner).** `FTUN_*` are absent on the board. AP_FlightTune
  exists only as uncommitted work in the `ardupilot-flight-tune` worktree. `feat/sim-hil` was
  branched from that branch's committed tip, `4c23c0a1`.

### Board runs with the HIL + tuner firmware (2026-10-13, 82eb2f04)

Runs `diag1`, `trim1`, `trim2`, `tune1` under `C:\tune_hitl`, Windows bridge, Mission Planner on the mirror.
- **Rare 20-60 ms link stalls: the host, not the bridge.** `demo2` failed the gate on one 60 ms stall
  (and had one of 25 ms). Instrumented runs (scratch wrappers, not kept) showed:
  - in the stall the bridge's `read()` waited with no data and then got 11 frames at once, or its
    `write()` blocked 17 ms; Python's collector never paused for more than 1 ms in flight, and the
    bridge's own turnaround stayed 0.8 ms median, 2 ms p99.9;
  - core-pinned, time-critical canary processes (HIGH priority class) on every core stalled 11-26 ms
    at the same moments, on groups of four cores at a time, about once in 2-5 minutes, while the
    per-core DPC and interrupt counters stayed quiet. Only the hypervisor (VBS/Hyper-V is on) or
    firmware can stop such threads; a stall reaches the link when the paused group holds the bridge or
    the USB completion.
  - So it is not timer resolution (the serial read is an overlapped read completed by the driver's
    event), not garbage collection, not the mirror (another process) and not the JSBSim step.
  - No bridge change removes it; the 50 ms and 5 % gates stay. `diag1`, `trim1`, `trim2` and `tune1`
    passed with worst ages of 12, 12, 36 and 11 ms. If the gate fails again, check the host first
    (other virtual machines, power plan, VBS), e.g. with LatencyMon.
- **FBWA at the cruise (`port-check`).** The cruise is now read on a straight CRUISE leg at
  AIRSPEED_CRUISE (CRUISE's stick sets its airspeed; the low stick flew AIRSPEED_MIN in `trim1`), checked level,
  and FBWA's stick centre is trimmed to its pitch with `PTCH_TRIM_DEG` for the run only
  (`flight_steps.CruisePitchTrim`, set back at the end). PTCH_TRIM_DEG rather than AHRS_TRIM_Y keeps the
  board's axes the model's body axes, so PIDR/PIDP see the same rates; FBWB/CRUISE would turn the pitch
  doublets into TECS climb-rate demands. `trim2`: the cruise leg flew level at AIRSPEED_CRUISE, and FBWA,
  trimmed to its pitch, flew the cruise speed (median within 0.1 m/s over the doublets) where `demo2` flew
  several m/s too fast.
- **First tuner session (`tune1`, LOITER at 109 m, a little above the cruise with the 1650 us stick).** Both axes kept,
  not saved (`FTUN_SAVE` 0):
  - roll: fit 0.12, delay 42 ms; FF a little lower, P and I about 3 and 6 times the aircraft's, D about
    the same; check error 2.21 -> 1.63 deg (-26 %);
  - pitch: fit 0.04, delay 20 ms; FF a little lower, P and I about 3 and 4 times the aircraft's, D
    raised from almost 0; check error 0.35 -> 0.20 deg (-44 %);
  - no saturation, Dmod 1.0 throughout, airspeed within 0.3 m/s between the runs of a check.
  - **Hand-back:** after every manoeuvre the pitch swings for about a second (up to 3 deg off its
    demand), the aircraft sinks 1.5-2.5 m (down to -2.6 m/s), and TECS answers with 10-12 % more throttle
    (no dip) and a pitch demand of 5-8 deg; settled in about 4 s. Changed since: see the next section.
  - **CPU on the H7: over budget.** Longest piece (`PcUs`) 2005 us roll, 1111 us pitch; longest call
    (`CaUs`) 2500 us and 1663 us, against the 600 us budget and the 700 us slot. The scheduler still
    reported no long loop (PM worst 5137 us) on this HIL build, which has no real sensor load. Changed since (next
    section): the calculation runs in bounded pieces.
  - Link gate passed: 2.7 % stale, worst age 11.4 ms.

### Calculation in pieces and gradual hand-back (2026-10-13, feat/sim-hil-flight-tune 78bda16562)

Commits a7ab78e4be, 911e93e57d, 78bda16562 on top of 82eb2f040f. Not yet on a board.

- **CPU, cause.** `compute()` checked its 600 us budget only between pieces, and single pieces were big:
  the design tables in one piece (3 speeds x up to 126 frequencies of model response and controller
  factors), a fit iteration with up to 18 residual evaluations, a rate-loop check over three speeds, 100
  simulation loops. On the host the roll tables took 15 us; the H7 took about 130-140 times the host's time
  for this float code, hence 2005 us.
- **CPU, change.** Every job runs in parts, each a bounded piece: 4-10 test or design frequencies, 8
  frequencies of a residual evaluation, 20 simulation loops, one grid stage, a few log rows or messages.
  `compute()` starts another piece only while one as long as the longest of the session (at least
  `PIECE_RESERVE_US`, 200 us) still ends within `FLIGHT_TUNE_COMPUTE_US` (600 us). Estimated pieces on the
  H7 (host timing x the tune1 ratio): 25-130 us, so calls at most about 530 us. Not measured on a board:
  the new `FTCU` record gives the longest piece of each job and `PcUs`/`CaUs` when each calculation ends.
  The calculation takes about 1.2-2 times as long in wall time (the aircraft loiters meanwhile).
- **Identical results.** `test_flight_tune_slicing` compares the sliced estimate, fit, tables and loop
  checks bit for bit with copies of the old code at piece sizes 1-48. A whole session with every piece in
  its own call ends with the same gains as with the whole calculation in one call. Five simulated sessions
  run on the old commit and on the new code (hand-back made instant) gave bit-identical gains.
- **Hand-back, cause.** The handed-back states were continuous in tune1 (rate integrators, TECS's pitch
  demand, throttle and integrator). The step was LOITER's bank demand: each manoeuvre's straight leg
  leaves the 150 m circle, so LOITER asks for its 45 deg limit at once, and the roll controller took it at
  its rate limit (peak roll rate 120-200 deg/s). The pitch rose about 3 deg during the fast roll-in, then
  fell below its demand while the load factor grew to 1.4, and the aircraft sank.
- **Hand-back, change.** After a manoeuvre that ends without a stop the tuner keeps control, flies the
  mode's pitch (TECS) and throttle, and moves its roll reference to the mode's bank demand at the rate it
  levels the wings with (45 deg/s, by the clock), then hands back; at most 3 s. A manoeuvre dropped while
  levelling starts from the bank flown. Envelope checks and terrain apply; any stop hands back in the same
  loop; the calculation waits. FTUN logs the hand-back as Ph 6 (`handback_check.py` counts Ph 2-5 as holds).
- **Hand-back, numbers** (3 s after each manoeuvre; pitch off its demand, height loss, throttle rise;
  median and largest):

  | Run | Pitch off demand, deg | Height loss, m | Throttle rise, % |
  |---|---|---|---|
  | tune1, board, old firmware | 5.6, 6.1 | 1.4, 2.4 | 11, 14 |
  | SITL + HIL bridge, Peron15_hitl3, old firmware | 5.3, 6.2 | 1.3, 2.5 | 11, 14 |
  | SITL + HIL bridge, Peron15_hitl3, new firmware | 4.2, 4.8 | 1.0, 3.4 | 9.6, 13 |
  | SITL plane-elevon, old / new | 3.4 / 3.1 | 0.46 / 0.42 | 16 / 15 |

  The plane-elevon model already rolled at about 60 deg/s, so it shows little either way. On the Peron
  plant the peak roll rate fell from 120-170 to 40-50 deg/s and the pitch swing by about a fifth; roll
  hand-backs lose 1.0-1.2 m instead of 1.2-1.5 m. The rest is the energy a 45 deg turn (load factor 1.4)
  costs until TECS adds throttle: LOITER asks for that turn after every straight leg, and no transfer of
  controller states removes it. The largest loss in both runs (3.4 m new, 2.5 m old) follows the first pitch
  test; its cause was not analysed. Tuning in CRUISE (no bank demand at the
  hand-back) or with a larger loiter radius would remove the turn; not tried.
- **Review.** Codex (static, read-only) found four defects, all fixed in 78bda16562: the hand-back after
  a drop during levelling started from level, terrain first seen during the hand-back was not required,
  the hand-back's timing counted loops, and the timing record missed the last piece. It confirmed that
  slices cannot resume in a wrong part, that stops stay immediate and that accepted roll gains survive a
  stop during pitch.
- **vs-autotune** now flies AUTOTUNE, the tuner and both card sets at port-check's cruise trim
  (`CruisePitchTrim`, PTCH_TRIM_DEG for the run only).

### Straight legs instead of LOITER (2026-10-13, runner only, firmware unchanged at 78bda16562)

- **Why.** In LOITER every hand-back met LOITER's 45 deg bank demand (see above). The owner chose to try
  the tests on a straight line.
- **How (no firmware change).** AP_FlightTune already runs in CRUISE, and CRUISE holds the course line it
  locked, so after a manoeuvre it asks only for a small bank back onto that line. `inflight-tune` now
  defaults to `--pattern straight` (`tools/tuning/tuner_legs.py`); `--pattern loiter` is the earlier
  procedure.
  - Each axis is its own tuner session on its own CRUISE leg (`FTUN_AXES` one bit at a time). The roll
    session's kept gains stay in RAM for the pitch session, as within one session.
  - Between the legs, while the tuner is not testing, LOITER turns the aircraft until its course points back
    at the first leg's start, so the second leg flies back. A session that does not end with "FTUN: done"
    ends the legs.
  - The legs fly the airspeed LOITER flew at the pilot's 1650 us stick (THROTTLE_NUDGE, 26.3 m/s here). A
    first run at AIRSPEED_CRUISE never started a roll test ("no safe conditions for 60 s"): a roll test
    needs AIRSPEED_MIN x sqrt(1/cos 35 deg) + FTUN_SPD_MARGIN, above the cruise for this aircraft.
- **SITL + HIL bridge (Peron15_hitl3, package plant, 78bda16562), 3 s after each manoeuvre; median and
  largest:**

  | Run | Pitch off demand, deg | Height loss, m | Throttle rise, % | Peak roll rate, deg/s |
  |---|---|---|---|---|
  | tune2, board, LOITER (15 hand-backs) | 4.4, 4.8 | 1.2, 3.1 | 10, 12.6 | 71, 106 |
  | SITL + HIL, LOITER (15) | 4.2, 4.8 | 1.0, 3.4 | 9.6, 13.3 | 45, 51 |
  | SITL + HIL, straight legs (16) | 1.2, 2.5 | 0.11, 1.7 | 0.9, 7.2 | 14, 45 |

  - Roll manoeuvres: 0.0-0.26 m height loss, at most 1.5 % throttle, pitch at most 1.4 deg off its demand
    (LOITER: 0.9-1.5 m, about 9-10 %, 4-4.5 deg).
  - Pitch manoeuvres: 0.06-1.74 m, 0-7.2 %, 0.7-2.5 deg (LOITER: 0.1-3.4 m, 3-13 %, 3-4.8 deg). Every
    other pitch manoeuvre still loses about 1-1.7 m with almost no throttle rise: that is the pitch test
    itself (its last half-doublet), not a turn; not analysed further.
  - Both axes finished: roll new gains kept, check error -33 %; pitch kept, -58 %. Link gate passed.
  - Space: the two legs took the aircraft up to 3.05 km from where the first leg started; it ended 165 m
    from there.
- **Open (parked 2026-10-13, owner moved to the HITL camera stream):**
  - No board session with the straight legs yet: run `inflight-tune --firmware --axes 3 --board COM15 ...`
    as for `tune2` (no new argument needed), new workdir.
  - Range: about 3 km per leg pair. A racetrack (AUTO legs with the tuner waiting through the turns) would
    need the tuner to allow AUTO and to start a manoeuvre only with enough straight leg left: a firmware
    change, not started.
  - The 1-1.7 m loss after alternate pitch tests is not analysed.

## Limits

- The SITL plane-elevon model is not the Peron15. A better result in SITL shows that the
  method works, not that the real aircraft improves.
- ArduPilot's controller structure is fixed, and its gains are the same in both
  directions. Left/right differences from build defects need a mechanical trim; the tool
  reports them.
- Results apply to the mass, CG and speed band flown. A range of take-off masses needs
  identification at more than one mass, or margins checked across the range.
