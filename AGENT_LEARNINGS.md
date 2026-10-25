# Agent learnings

## ArduPilot SITL in WSL (tools/tuning, 2026-10-11)

- In this ArduPilot 4.6 tree the plane defaults are Tools/autotest/models/plane.parm,
  not default_params/plane.parm. A missing --defaults file only shows as
  "PANIC: Failed to load defaults" after a ground station connects, because SITL waits
  for the connection before it starts the sketch. Check the file paths first.
- `pkill -f "<pattern>"` inside `wsl bash -c "..."` also matches the bash command line
  that carries the pattern, so it kills its own shell (exit 15). Use `pgrep -af '[a]rduplane'`
  style patterns, or run the script through `bash -s` with a heredoc.
- Python heredocs that write "\n" into source files through Git Bash ended up as real
  newlines five times (three more in dev-sim step 1: a test, a scratch script and this
  note). Insert or change code with the Edit tool, never a heredoc; check with `ast.parse`.
- SITL's simulated receiver takes 8 or 16 little-endian uint16 PWM values on UDP
  5501 + 10 x instance (AP_RCProtocol_UDP.cpp). Driving the "pilot" this way keeps Lua
  RC overrides behaving as on the aircraft; MAVLink RC overrides would compete with them.
- `cat > file` with no heredoc or input in a Bash tool command waits for stdin and hangs
  the whole command. Write scratch files with the Write tool.
- Helpers started with `setsid nohup ... &` from `wsl -e bash` died within seconds of the wsl.exe
  call returning, with no output, in 3 of 14 starts. Started from Windows as a hidden
  `Start-Process wsl.exe -e ...`, they ran to the end every time.
- Before blaming CPU load for a link stall, log the load per second and run a 1 ms sleep ticker on
  each OS. The HITL chain's 42-60 ms link stalls came at 2-6 % CPU, with no late wake-up in WSL or
  Windows. A 200 Hz TCP ping-pong through WSL's localhost forwarding (worst 17 ms) and 200 Hz CSV
  writes to /mnt/c (worst 29 ms) did not reproduce them, and a bridge logging to the WSL disk still
  stalled. Time-stamp each hop inside the relay and bridge to find the stall.
- ArduPlane LOITER holds the height it is entered at (commands_logic.cpp:979), so
  "LOITER to climb back" never climbs. Enter LOITER, then send MAV_CMD_DO_CHANGE_ALTITUDE
  (COMMAND_INT, param2 = MAV_FRAME_GLOBAL_RELATIVE_ALT); entering LOITER again resets it.
- A throttle stick above mid-stick raises LOITER's, CRUISE's and FBWB's airspeed target
  (THROTTLE_NUDGE, radio.cpp:198-205). The driver's fixed 1650 us stick flew LOITER about
  2 m/s above the cruise. Set the stick deliberately in auto-throttle modes and say why.
- The repository has both LF and CRLF files. On Windows Python's `Path.write_text`
  writes CRLF, so a read_text/write_text edit script flips a whole LF file. Edit by script
  with read_bytes/write_bytes and keep each file's own line endings.
  Plain open(p, 'w') does the same (repeated in v12.1, 2026-10-20): pass newline=''.

## ArduPilot Lua scripts (tools/tuning/lua, 2026-10-11)

- ArduPilot compiles scripts with MAXVARS 100 (libraries/AP_Scripting/lua/src/lparser.c),
  stock Lua 5.3 with 200. A script that passed every lua5.3 harness test failed to load in
  SITL with "too many local variables (limit is 100) in main function". Check the active
  locals from `luac5.3 -l -l` listings (test_flight_tune_lua.py does this) and keep
  constants and helper functions in tables.
- Lua coroutines are not available to scripts (lua_scripts.cpp opens no coroutine
  library), and AP_Scripting's instruction hook counts per resume. Split long work into a
  state machine with a bounded piece per call and test it under the same 10000-instruction
  hook.
- SITL with --speedup N inflates Lua call gaps: thread delays run on simulated time
  (AP_HAL_SITL/Scheduler.cpp delay_microseconds) while the Lua work takes real CPU time.
  At speedup 5 a 20 ms script loop showed a 41 ms median and gaps up to 439 ms; at speedup
  1 the median was 23 ms and the maximum 31 ms. Measure script timing at speedup 1 only.
- A plain sum of samples as a Fourier integral breaks when samples are unevenly spaced.
  With up to 40 ms late calls the fit error rose from 0.03 to 0.6; integrating the straight
  line between samples exactly brought it back to 0.06.

## ArduPilot firmware work in the gartfeo fork (2026-10-12)

- Build in a separate git worktree (~/fw-worktrees/<name>), never in ~/ardupilot. Other
  sessions' GCS SITL instances run ~/ardupilot's build.
- ~/ardupilot's .git/config points submodule URLs at local paths. A worktree needs
  `git -c protocol.file.allow=always submodule update --init --recursive`.
  - CrashDebug's nested CrashCatcher and mri need their URLs set to
    ~/ardupilot/modules/CrashDebug/<name> first.
  - Board builds fail without them; SITL does not need them.
- ~/ardupilot's local ArduPilot-4.6 (4c23c0a1) is 7 commits behind the fork's published
  origin/ArduPilot-4.6 (bf0dd8f12f, 2026-09-21), which already pins modules/mavlink to
  a38fe8ab and has the pitot EAS backport. Base new fork branches on
  origin/ArduPilot-4.6, and check `git ls-remote origin` before calling a local branch the
  fork's tip. A branch made from the stale 4c23c0a1 needs the a38fe8ab mavlink checkout to
  build SITL (SIM_STATE time_us).
- Workflow agents do not see the owner's chat. An agent refused approved commits because
  the approval appeared only in the generated prompt. Quote the owner's exact words and
  the question they answered when a stage acts on an approval.
- New code must pass ArduPilot's strict flags:
  - -Werror=float-equal and -Werror=undef;
  - unused static data is an error;
  - board builds also enforce -Werror=frame-larger-than=1300, so keep large arrays in
    objects, not on the stack.
  - Feature flags follow `BOARD_FLASH_SIZE > 1024`, not HAL_PROGRAM_SIZE_LIMIT_KB.
- AP_Logger drops non-critical records when its buffer is full. SITL at --speedup 5 with
  full-rate logging dropped 405k messages, including whole record types. Write rare
  summary records with WriteCritical, and run SITL at speedup 2 or less when logs matter.
- Controller outputs: get_rate_out() returns centidegrees (+-4500), not degrees.
- SERVO_AUTO_TRIM never moves a trim outside 40-60 % of SERVOn_MIN..MAX
  (SRV_Channel_aux.cpp adjust_trim), 1 us per 0.5 s. A trim of exactly 1400 or 1600 on a
  1000-2000 servo is the limit, not a learned value. Identical trims on several
  airframes, or on one with little flight time, mean a copied parameter file.
- A port check that binds without SO_REUSEADDR reports a port in TIME_WAIT as busy right
  after the previous SITL run.
- In SITL, AP_HAL::micros() is simulated time and does not advance inside a scheduler
  task. A time-budget loop therefore runs to completion in one call, and measured CPU
  times read 0. Per-call work and timing can only be checked on a board.
- Board toolchains (arm-none-eabi) are on the PATH of a WSL login shell only: run board
  builds with `wsl bash -lc`, not `wsl bash script.sh`.
- wsl.exe hands `bash -lc '...'` to the login shell, so `$VAR` inside it expands there
  (often to empty), and Git Bash rewrites /mnt/c/... arguments. Run scripts with
  `MSYS_NO_PATHCONV=1 wsl -e bash script.sh` when they need neither the login PATH nor
  path rewriting.
- To check a hand-back between controllers that run at different rates, measure after
  the slower one's first update. The TECS hand-back check sampled the 50 Hz output
  before TECS's first 10 Hz update and reported a 0 % throttle step, while the real
  step was a dip to 0-16 %.
- A heredoc inside `wsl bash -lc "..."` can fail without an error, and the commands after
  it then run on unchanged files. Write the script to a file and run that, then grep the
  target to confirm the change landed.
- A design search that "worked" in SITL passed only 2 of about 350 candidates. A small,
  correct change to the simulation then left none. Replay logged data offline through the
  search and count candidates per rejection reason before trusting a result. A screen
  stricter than the in-flight acceptance rule (35 degree peak output) removed the best
  designs; screen with the rule the flight check applies.
- Before adding an absolute acceptance limit, check it against the reference: a 2 %
  saturation cap would have rejected AUTOTUNE's own gains.
- A new in-flight abort rule needs its monitored value logged for the old-gain runs too,
  and a look at those values before trusting it. The first "other axis disturbed" monitor
  read 20 deg (its limit) on good runs: its ideal response ignored the levelling rate cap
  the tuner's own demand has, and LOITER leaves the wing at 45 deg bank before each run.
- A gtest that runs AP_TECS (anything calling AP_Logger::should_log()) dereferences the
  bitmask pointer that only AP_Logger::init() sets. init() also starts the logger thread,
  and in a gtest SITL's delay_microseconds() advances the stopped clock (no model runs),
  so that thread drives the clock the test steps with hal.scheduler->stop_clock().
  AP_TECS/tests/test_tecs_hold.cpp sets the pointer without init().
- A controller's integrator is not a trim on its own. In steady flight TECS's throttle
  integrator moved by several percent between samples seconds apart, and even its 20 s
  average moved. Check such a value's spread in the logs before holding it.
- An rsync copy of a waf tree also copies `.lock-waf_*`, which points the copy's ./waf at
  the source tree's build ("invalid lock file"). Exclude it, and configure the copy with
  --no-submodule-update so waf never touches the source tree's submodules.
- waf refuses to build ("configure options have changed") after a commit that changes
  Tools/scripts/build_options.py. A per-commit build walk must reconfigure there.
- The Edit tool writing through \\wsl.localhost drops a file's exec bit. Check with stat
  and chmod it back (extract_features.py must stay 755).
- wsl.exe can drop the first lines of a script's output. Send output to a log file and
  read the file.
- Autotest needs MAVProxy from the login shell's venv: run it with `wsl bash -lc`.
- The Ubuntu distro restarted on its own during a long job (2026-10-12 22:35). That cleared
  /tmp and killed every running SITL, including other jobs'. Keep logs and builds outside
  /tmp, and expect parallel SITL runs to need a restart.
- The HIL autotest's spare ports 8000-8002 collide with the GCS default 8000; the
  instance-40 wrapper moves them to 6790-6792.
- The real-time HILBackend autotest's stale-loop limit (5 %) measures the in-process Python
  bridge's reply time against a 5 ms loop, so host load decides it: 0.1-13 % stale on the
  same binary. Judge firmware changes by HILBackendLockstep, and treat a real-time failure
  as a host-timing result until the bridge runs in its own process.
- In a gtest AP_HAL::millis() moves only when the test moves it (hal.scheduler->stop_clock).
  AC_PID's slew limiter times its windows with it; in a session sim that never moved the clock
  its Dmod stuck at 0.40 for every gain set, and the tuner's check rejected good gains for
  "slew limiter". Step the HAL clock with the sim's loops.
- A test aircraft built from the code under test (the tuner's own Plant) cannot show what the
  old code does: after a parameter-layout change the old code built a different aircraft and
  failed for another reason. Integrate the reference plant in the test itself.
- ArduPilot compiles with -fsingle-precision-constant, so `1.0` is a float: `1.0 + z` with a
  std::complex<double> z does not compile. Use a typed constant.
- ArduPilot's autotest.py has no instance option. To run it on SITL instance N, patch
  TestSuite.start_SITL (add "-IN" to customisations), adjust_ardupilot_port (+10 N), sitl_rcin_port
  (5501 + 10 N) and spare_network_port before running autotest.py with runpy, and set BUILDLOGS to a
  scratch folder (its default is ../buildlogs next to the tree).
- Autotest sets LOG_FILE_RATEMAX 10, which caps streamed log records (ATT, TECS, CTUN) at 10 Hz. A
  test that checks the log loop by loop must set it to 0 and add LOG_BITMASK bit 20 (full-rate ATT).
- wait_statustext(check_context=True) also matches texts collected earlier in the context. Call
  context_clear_collection("STATUSTEXT") before each wait for a text that repeats.
- Autotest's FBWA takeoff (takeoff() without mode) leaves the plane-elevon model on the runway;
  takeoff(mode="TAKEOFF") flies it.
- ArduPlane AUTOTUNE starts its search again on every entry: AP_AutoTune::start clears D_limit and
  P_limit, and stop() keeps the gains only when both were found, else restores the entry's gains. A
  procedure that leaves AUTOTUNE to climb and enters it again never finishes an axis that needs more
  height than one entry has. On a HITL model, 12 short entries from 120 m found only RollD; from a much
  higher start roll finished in one entry. Size the start height to one whole search.

## Offline numerical checks (2026-10-12)

- Git Bash cannot start a Windows program (the venv python.exe) from a working directory under
  the long session scratch path: "path longer than allowed for a Win32 working directory".
  Stay in a short directory and pass the script's Windows path as an argument.
- The augmented matrix of an exact zero-order-hold step (input as a constant state, attitude as
  an integrator) has a defective zero eigenvalue. An eigen-decomposition reference for its
  matrix exponential was wrong by order 1 while the port was right. Check expm against a long
  Taylor series on a heavily scaled matrix instead.
- Check a fit's starts on every plant it will meet, not only the one they were built for. The
  dipole roll starts, all with a Dutch roll, fitted the aircraft and HITL as planned, but plain
  SITL (no Dutch roll) only to RMS 0.26 under the tuner's budget, with a wrong model, where 0.05
  existed. Three starts without a Dutch roll found it.
- A margin check on a fixed frequency grid cannot see a resonance narrower than the grid
  spacing. The tuner's 40-point log grid (15.7 % spacing) passed a roll model with zeta 0.02
  (allowed by its bounds) at PM 180 deg, while a dense grid found a crossover at -193 deg. When
  a model's bounds allow light damping, check margins on a dense grid or refine around the
  fitted poles.
- ArduPlane's SCHED_LOOP_RATE default is 50 Hz (AP_Scheduler.cpp); the 300 Hz in
  ArduPlane/Parameters.cpp applies only when Q_ENABLE is set. Read the loop rate from the log's
  parameters before assuming one.

## JSBSim models and fleet-log propulsion (2026-10-12)

- The WSL JSBSim program (FlightGear 2020.3.18) never uses a propeller ixx below 0.001
  slug ft2, and says so only in its load report: the run log printed `IXX = 0.0010` for a
  file with less, and the spin-up confirmed 0.0010. Read the program's load report for
  the values it actually uses before crediting a result to a model change.
- JSBSim reads `<property value=...>` declarations only at the `<flight_control>` or
  `<system>` level. Inside a `<channel>` the 2020.3 program stops with "Unknown FCS
  component: property".
- Analog battery monitors (BATT_MONITOR 4) left at the default amps-per-volt scale can read
  well above an INA monitor (BATT_MONITOR 21) at the same thrust. Check the monitor type before
  using BAT.Curr as power.

## Coordinating parallel agents (2026-10-12)

- A SendMessage to an agent that runs inside a Workflow does not reach that running
  instance. It resumes a second copy from the transcript, and the two then edit the same
  files. To change a running workflow agent's task, let it finish and start a follow-up
  agent, or stop the workflow and resume it with an edited script.
- Two parallel agents writing into one folder collided on a test file name and one
  overwrote the other's tests. Give each parallel agent its own subfolder or file-name
  prefix in the prompt.

## Reading and claiming code behaviour

- Claims about scripts made from a quick read were wrong in detail twice (when
  tune_pitch_roll.lua saves RC_OVERRIDE_TIME, how its pitch-down ends). The independent
  review caught both. Quote the exact condition with line numbers before stating a defect.
- A mapping step can create left/right differences on its own. ArduPilot's
  SRV_Channel::pwm_from_angle scales positive and negative commands onto MAX-TRIM and
  TRIM-MIN separately, so an auto-trimmed servo gives different travel per command in each
  direction. Check the servo mapping before attributing asymmetry to the airframe.

## Flight logs and parameter snapshots (2026-10-12)

- A Mission Planner .tlog recorded while it connects holds the aircraft's full parameter
  list (PARAM_VALUE, about a thousand values in a few seconds of tlog). Check the tlogs for a parameter
  snapshot before asking the owner for a .param file. The tlog folder name is the SYSID,
  not an airframe: FIXED_WING\1 was the bench HITL Pixhawk 6X.
- PARM records in a .BIN log are not only the start-up dump: every later save is logged
  too (auto-trim every 10 s, ground-station edits). A log's last value can differ from the
  value in force during most of the flight; use the PARM history by time.

## Aero geometry files and CFD tables (2026-10-12)

- Before using a CFD pitching-moment table, compute the neutral point it implies:
  x_ref - (dCm/dCL) x c_ref. One table read about the CG with the MAC put it behind the wing
  trailing edge. Such a table has a different, unstated reference, and no control or
  stability fix will repair it.
- OneDrive "online-only" files (attribute 0x400000, RECALL_ON_DATA_ACCESS) are downloaded
  when they are opened. Check the attribute before a recursive grep or read, and ask before
  opening them.
- The session scratch path is longer than 260 characters. For Windows programs that write
  output next to their input (OpenVSP vspscript/VSPAERO), pass the 8.3 short path instead.
  PowerShell: (New-Object -ComObject Scripting.FileSystemObject).GetFolder(<path>).ShortPath
  gave 88 characters, and that worked.

## Motor speed without an RPM sensor (2026-10-13)

- A log with no ESC telemetry or RPM sensor can still give the propeller speed. Look in the
  logged IMU spectrum (3 s windows, steady throttle). The once-per-revolution vibration
  survives the 20 Hz INS filter as a sharp line folded about half the IMU log rate, so the
  rotation can be fs - f_obs. Being below the motor's no-load speed (Kv x duty x battery V) is
  not enough to pick the fold: several readings can pass it. Also require that the speed rises
  with throttle (a wrongly unfolded reading falls) and that the cruise advance ratio V/(n D) stays
  near or below the propeller's pitch ratio (a blade-pass reading can give far more, so no
  thrust). This showed a throttle-to-speed change after a hardware refit that the
  flight-controller parameters did not show.
- Before using fit windows from a log, drop windows with an impossible battery voltage
  (below about 3.0 V/cell). A pack run far below that had entered a propulsion fit.

## Bench measurement sheets for the owner (2026-10-13)

- A surface angle read as trailing-edge offset from a straight edge on the wing needs a zero
  per straight edge and the hinge pivot height. At centre the trailing edge sits a few mm off
  each surface line (on a short elevon 1 mm is most of a degree), and the elevon turns about
  its pivot, not about the edge. Ask for the centre reading against each edge used, the wing
  thickness and pivot height at the hinge, and use asin((d - p)/c) - asin((d0 - p)/c). Only
  subtracting d0 left errors of several degrees at full throw in a geometry sweep.
- Before calling one log "the main log" of a pooled fit, count its share of the fit windows.

## Vortex-lattice runs (2026-10-13)

- A sudden step in a VSPAERO lift or moment sweep is often one coarse outer strip, not the
  wake. Before naming it, rerun with about twice the spanwise strips on each surface and with
  more wake iterations; keep the result only if it survives both.

## Fleet log scope (2026-10-13)

- Before pooling fleet logs, ask the owner which airframe numbers are the target aircraft model
  and which airframes had hardware changes. A log of a different model and a refitted airframe's
  logs had both entered pooled statistics and gates. Keep, for each derived number, the list of logs
  that fed it, so a later scope change can be re-run instead of re-derived.
- A battery resistance from ArduPilot's (VoltR - Volt) / Curr carries the current sensor's scale
  error: a monitor that reads current high makes the resistance look low by the same factor. A
  check that rebuilds the terminal voltage as Voc - R I then overstates full-throttle voltage on
  those logs. Compare with the logged terminal voltage first.
- Bootstrap a median gate over a few items before trusting it. With 7 items, the 90 % interval
  of the median was wider than the gate band.

## Peron flight-log deep analysis (2026-10-17)

- Barometer and EKF height read falsely high in pull-outs (+13-38 m, about 2-3.5 m per g), and
  in four dive logs most pull-outs fail the EKF height test and switch the core (10 of 10 in one log). Scoring pull-out height against
  EKF height understated the loss by a median 4.4 m (up to 14.5 m). For height at speed or under
  load, use GPS, or IMU height fitted to GPS per window.
- Two different files are both called "log 30" (gdrive_20250927_00000030, SYSID 158, INA monitor,
  and peron_160_00000030, SYSID 157, analog monitor). A current maximum was put on the wrong one.
  Use full file names in claims, scripts and handoffs.
- ArduPilot 4.3-4.5 takes EAS2TAS from GndTemp, the airspeed sensor's temperature at arming, held for the flight (4.5-57 C), so a
  replay density built from it was 0.85-1.01 of ISA. Do not use it as outside-air density; ask
  the owner for the outside temperature.
- SYSID is a parameter, not an airframe, and values copied in a parameter file match exactly
  after a 7-digit round trip, so they do not identify a board. AP_Stats counters and boot gyro
  offsets follow the board (the counters are parameters and can be rewritten, so they are strong
  evidence, not proof). Check them before calling two logs "the same aircraft".
- An INA2xx monitor (BATT_MONITOR 21) logged exactly 0 A at full throttle near its BATT_MAX_AMPS
  range, and the logged charge skips those records. The mechanism is inferred, not read from the
  driver (an over-range sample seems to count as 0 in a 4-sample average). Treat such zeros as
  probable over-range, not as power off, and drop them from current statistics.
- Comparing per-log medians of mixed samples gave a power ratio of 1.97; with matched throttle,
  voltage and speed gates it was 1.71. Gate by state before comparing power or force across logs.
- The logs hold no MODE records, and the flight-mode slot read FBWA while TECS ran 77 % of that
  time. Derive the mode class from TECS, ATRP and demand records.
- A force check started the model at the replay TAS but took the real acceleration from EKF
  speed; the 2.4 m/s start gap hid most of an estimated 5 N deficit. Start the model from the same speed
  source as the comparison.
- A resolution round counted 2 dives ending in bad data where the flight-end inventory showed 4.
  Cross-check exclusion counts against an independent list before writing them down.
- The first draft said the throttle offset explained half of the row 13 gap, but no run had
  dropped that offset alone; the single-offset runs pointed elsewhere. Name a cause only when a
  run isolated it.
- Batch doc edits drafted by a workflow: check that each old text occurs once and that no two
  edits overlap before applying (4 pairs overlapped here), and keep the working copy's CRLF
  line endings (read and write with newline="").

## Owner data against the Peron model (2026-10-18)

- I told the user that mass, CG, battery, motor and elevon data were missing when most of it was
  already on disk (model header, measurements folder). Search the private tools folder, the model
  headers and the owner's ArduPilot checkout before saying owner data is missing.
- A model header's "from CAD" label was not proof, and git history alone was not either. I called
  e5830898fe's tensor (kg m2, with products) "the CAD tensor on record". But the owner's checkout
  had uncommitted edits holding the other tensor (0.239/0.534/0.766 slug ft2, zero products, the
  one every navpy model uses), and both versions' READMEs say "from CAD". Run git status and read
  the working-tree file as well as git log --all and merge-base; two commits with one parent are
  alternatives, not a replacement. Only the CAD tool's own output settles which numbers are real:
  the Fusion v96 output the owner gave later matched the working-tree file's moments of inertia
  (not its zero products), not e5830898fe's.
- IMU vibration lines that stay at one frequency from cruise to full throttle are unlikely to be
  motor rotation, and aliasing allows several rpm per line. Do not read rpm from the IMU without a line
  that moves with power; ask for ESC telemetry or an rpm sensor instead.
- Compare a static bench sheet with the model at the sheet's own current, not at the model's full
  throttle: thrust per amp falls with current, so the full-throttle comparison exaggerated the gap
  (20-37 % against 1-10 %).
- I told the user that a 0 A reading at high throttle meant the motor had stopped. The pack voltage
  had fallen 7-9 V at the same time; a motor switched off lets it rise (a jammed motor that is
  still powered does not). With a range-limited current monitor (here the INA at 90 A), 0 A can
  mean over-range. Before reading 0 A as no current, check
  the voltage direction and the samples next to the zeros (22.4 A = a quarter of 89.4 A here).
- I wrote that a 10 % light model has a (T-D)/m 1.11 times too large. But every force term the
  model had fitted to logged accelerations (lift, high-angle drag, the motor law) assumed the same
  7.5 kg, so force and mass cancel there (for the motor law only in the mass-times-acceleration part
  of its target thrust). Before scaling a model for a mass error, trace which
  terms were fitted from logged accelerations at the assumed mass and which came from elsewhere
  (electrical power, CFD). The first kind cancels only where the fit matches and only if every
  fitted flight had that mass; the second kind carries the error directly. (The 10 % premise
  itself was wrong; see the next item.)
- I built a mass correction (scale every log-fitted force by 1.113) on the owner's 8.351 kg,
  although both January 2026 commits and the checkout's model had 7.46 kg. The 8.351 kg came
  from later Fusion modifications that are not the flown aircraft; the flown aircraft is
  7.460 kg (Fusion v96). When an owner figure contradicts the files, ask which revision and date it comes
  from, and whether that is what flew, before building on it.
- I read a lower normal-force coefficient, computed at an assumed 7.5 kg, as a lighter airframe.
  CN = m g n / (q S) with the assumed m, so if the true coefficient is the same, a lower value
  means the true mass is higher (a different CG, control travel or aerodynamics could also cause
  it). Write the formula out before naming the direction of a mass effect.
- Fusion 360 lists products of inertia as tensor elements (Ixz = -integral(x z dm)); the
  parallel-axis test shows it (origin value minus centre-of-mass value = -m x z). JSBSim's <ixz>
  is also -integral(x z dm), but in its structural frame (x aft, y right, z up), so the sign of a
  copied CAD product depends on the CAD axes. Run the parallel-axis test and confirm the axes
  before copying a product of inertia.
- I recorded the owner's "180 %" bungee stretch as "1.8 times the rest length"; the owner's own
  words for it were not kept, and a percent stretch can also mean elongation (2.8 times). Keep the
  owner's exact words and mark any reading added to them.
- I described the log-30 event's 12-14 g normal load as part of the jolt. It was the flight's 10th
  commanded pull-out, and the 9 earlier ones peaked at 11.2-12.5 g; the abnormal signs were the
  x deceleration, the vibration and the current. Before calling a reading abnormal, compare it
  with the same manoeuvre elsewhere in the log and check the commanded state (NavPitch, servo
  outputs).

## Hardware-in-the-loop link work (2026-10-13)

- From Git Bash, `wsl.exe bash -lc '...'` lost every `$variable` in the command. Paths under
  `/mnt/...` were also rewritten to `C:/Program Files/Git/mnt/...`. Write the commands to a
  script file and run `MSYS_NO_PATHCONV=1 wsl.exe bash -l /mnt/c/.../script.sh`. Send long output
  to a file and read the file.
- Processes started with `&` from a non-interactive bash script ignore SIGINT, so a later
  `kill -INT <pid>` does nothing. Stop them with SIGTERM, by pid, and make sure the program turns
  SIGTERM into its clean shutdown. A Python runner without a handler dies without its `finally`,
  so stop its child process first, also by pid.
- pymavlink's `MAVLink` codec reads `time.time()` in its constructor, and a link builds a new codec
  on every (re)connect. A test that patches the PC clocks to raise must connect the link before
  the patch. Its scope then covers frame handling only, not port I/O, where the clock is allowed.
- A guard test against PC-clock or random use proves something only if it fails on a leak. Inject
  one call into the physics path once, and check that the guard raises.
- A simulator bridge's stale-frame rate can come from the simulator, not the bridge. Before tuning
  the bridge, run the reference bridge against the same simulator binary and setup. If the stale
  fraction stays a steady 1 in N whatever the bridge's turnaround, it is the simulator's pacing.
- To show that a live run's physics followed only the board clock, rebuild the board frames from
  the bridge's own log and replay them offline through the real plant, twice. The replies must be
  bit-identical, and every logged state must equal the live run's, column for column.
- In Git Bash, `grep -c $'\r$'` reported CRLF on every line of LF files. Count carriage returns
  with `tr -cd '\r' < file | wc -c`, or look with `od -c`. In this checkout `core.autocrlf` is
  true: tracked text files are LF in the index and CRLF in the working tree, so a working-tree
  CRLF is not a change once committed.
- Two counts of the same kind of event at the same time can be one mechanism. Here the board's
  "stale" count equalled its "replaced" count, meaning replies arriving in pairs. Compare counters
  pairwise before reading each one alone.

## HITL on a real board (tools/tuning, 2026-10-13)

- A HIL backend that shows a placeholder pose until the simulator's first state lets the
  autopilot's EKF align on the placeholder. When the first real state then jumps the attitude, the
  EKF books part of the jump as gyro bias and carries degrees of attitude error for minutes; FBWA
  then holds the wrong attitude. Start the simulator side before the board's last boot and check
  that the first-state text comes before the EKF's "initialised" text.
- ArduPilot's "Gyros inconsistent" pre-arm message also appears while the 10 s consistency timer
  runs, and that timer only starts after the accel consistency timer (10 s) has passed. Right after a
  boot the message means "wait", not "the gyros disagree"; check the IMU log before chasing it.
- Stamp each hop with one clock. In a Windows relay -> WSL bridge chain, the Windows timestamps showed
  board-to-relay delivery steady (about 1 ms jitter) while frames reached the WSL side in bursts after
  40-160 ms, and under host load the WSL physics process fell 0.3-0.6 s behind. A 5 ms real-time loop
  cannot cross the WSL boundary reliably; run the loop's physics natively next to the serial port.
- WSL2's monotonic clock ran about 1.4 % fast against both the board and Windows in these runs.
  Never compare WSL timestamps with another host's clock without fitting the rate first.
- A physics engine can blow up after a ground strike (rates beyond float32). Check the state against
  a physical envelope before packing it into a message, and freeze a crashed aircraft instead of
  letting the encoder raise.
- JSBSim versions differ in propeller physics. Older builds floor a propeller's ixx (0.001 slug ft2) and react
  only the propeller's aerodynamic torque on the airframe. Newer builds (1.3.x) take the file's ixx and also react
  the torque that spins the propeller up. When a model was calibrated on one build and runs on another, compare a
  closed-loop flight from the same start on both (roll right after a throttle step shows it) before trusting it.
- Read a model folder's own README before choosing bridge options: a model variant can expect a non-default
  servo time constant.
- To tell an application stall from a platform stall on Windows, run core-pinned, time-critical sleep canaries
  (one process per core, 1 ms sleeps with timeBeginPeriod(1)) beside the real-time process and stamp both with one
  clock. When the canaries on several cores stall for tens of milliseconds at the same moment as the process, while
  the per-core DPC and interrupt counters stay quiet, the pause is below the scheduler (hypervisor or firmware); no
  priority, garbage-collector or I/O change inside the process removes it. Check that before tuning the process.
- ArduPlane CRUISE and FBWB take their airspeed target from the throttle stick (AIRSPEED_MIN at the bottom to
  AIRSPEED_MAX at the top, through the channel's dead zone) unless FLIGHT_OPTIONS says otherwise; a low stick there
  flies AIRSPEED_MIN, not the cruise. LOITER and AUTO use AIRSPEED_CRUISE.
- A reference state measured in flight (cruise pitch, cruise throttle) is only a reference once it is verified
  steady: check the climb rate and the speed against the target before using it. A leg still climbing back to its
  height gives a pitch that is wrong by the flight-path angle.

## Bounded background work on a flight controller (2026-10-13)

- A time-budget loop that reads the clock only between pieces overruns by its longest piece: a 600 us budget
  gave 2500 us calls. Cut the work into pieces of a known size and start another piece only while one as long as
  the longest measured so far (with a floor) still fits.
- Size the pieces before the board run: time each piece kind on the development host and scale by the ratio of
  a board measurement to the host time of the same unsliced code. For float code full of logf/atan2f/expf the
  STM32H7 took about 130-140 times the desktop's time; plain multiply-add code scales less, so the ratio is
  conservative there.
- On a busy Windows/WSL host the largest of many per-call medians is biased upward by pre-emption. Take the
  fastest of repeated runs of each call as its cost, normalise every run by a reference calculation timed in the
  same run, and repeat the benchmark.
- When moving a calculation into resumable pieces, keep every expression as it was, carry loop state in the
  object, and test bit for bit against a verbatim copy of the old code at several piece sizes, including 1 and
  sizes that do not divide the loops. Then compare whole sessions: build the old commit in a second worktree
  (submodules from the local checkouts) with one scratch test file, run the same scenarios on both, and compare
  the results as hex floats. Remove the scratch file and the worktree afterwards.

## Diagnosing a hand-back between controllers (2026-10-13)

- Before calling a bump at a hand-back a state discontinuity, log every handed-back state across the boundary
  (rate-loop integrators, the mode's pitch demand, the throttle and its integrator) and the mode's own demands.
  In one case all states were continuous; the step was the mode's navigation demand (a loiter's bank limit after a
  straight leg), which the mode then flew at the roll controller's rate limit.
- A simulator airframe can hide a rate-dependent effect: SITL's plane-elevon model rolled far slower than the
  aircraft model, so a fix that limits the roll rate showed little in SITL. Verify such a fix
  on the plant that showed the problem.

## Moving a tuner's tests to another mode (2026-10-13)

- Before switching the mode a supervised test runs in, list every start condition the test checks against
  the new mode's own setpoints. A tuner moved from LOITER (stick nudge, above the cruise) to CRUISE at
  AIRSPEED_CRUISE never started: its roll test needs the stall speed at its planned bank plus a margin,
  which lay above the cruise. Fly the speed the earlier runs were tuned at, and say why.
- Keep aircraft data out of a public repository from the first commit: read it from a private folder by a
  named argument or environment variable, test on synthetic numbers, and check the real files in tests that
  skip where they are absent. Sweep comments and test values too, not only data files.

## Fencing a second command channel (B3, 2026-10-15)

- When a signature gains a required argument, grep every caller in src and scripts, not only src. Two sim scripts
  missed the new `use_sim_gimbal` and would have failed at setup; Codex found them. A small AST test over src and
  scripts now pins the callers.
- A fence that only drops commands can leave an actuator moving: SIYI rate and manual-zoom commands keep acting
  until the next one. Drop everything, but let the stop commands through and send them once.
- Bound every wait that sits before a record or handoff another process reads, and keep separate devices on separate
  locks and threads, so one stuck send cannot block another device's stop.
- Timing tests on Windows: time.monotonic ticks about every 16 ms, so a 0.3 s join measured 0.297 s once in four
  runs. Allow one tick on a lower bound; repeat a new timing test several times before trusting it.

## ZR10 SIYI emulator (tools/hitl, 2026-10-15)

- From Git Bash, PYTHONPATH for the Windows venv Python must use Windows paths joined by ";"
  ("C:/.../navpy-hitl/src;C:/.../navpy-hitl"). POSIX paths are ignored silently, and the imports came from the
  shared navpy-temp2 checkout instead of the worktree. Check `module.__file__` once.
- navpy's SIYI readback used to decompose roll-pitch-yaw (matrix_to_xyz, g_seq "XYZ"; fixed 2026-10-16, see
  below); the real gimbal and Unreal are yaw-outer (Rz Ry Rx). They differ once yaw and pitch are both non-zero (yaw 90 / pitch -45: horizontal vs 45 deg
  down). Name the Euler order of every angle set that crosses a module boundary.
- navpy's sim plant clamps world-angle differences, which equal body angles only when the aircraft is level. Test
  limits and conventions banked and turning, not only level; Codex found a banked camera past the pitch stop.
- Run the module-size guard (tests/guards/test_solid_architecture.py) before review, not after: new modules over
  300 lines failed it in the full suite and forced a split after the first review.
- "Fail closed" covers every output: a stale input stopped the Unreal stream but navpy still got fresh-looking
  SIYI replies until the review pointed it out.

## ZR10 real behaviour (2026-10-16)

- Several tests pinned the raw wire yaw passing through unchanged (the sign bug itself). When a convention changes,
  grep the tests for the old values and update them with the reason; a green suite there proved the bug, not the fix.
- GimbalData.att and g_seq are one contract: publish them together in the same replace(). The sim test rig built
  its state with the default g_seq "XYZ", so a ZYX readback was read as XYZ until the stepper published g_seq too.
- Check a file's line count against the 300-line guard before adding to it: bridge.py was at the limit, so the new
  option went into unreal_pose.py instead.
- A SendMessage answer from another session may not arrive mid-turn. Ask it to write the facts to a notes file in
  your notes folder, and read that.

## 3-UAV lockstep acceptance (step 10h, 2026-10-16)

- **Add every new file before the first acceptance run.** A test file
  written while run 1 flew changed input_manifest.json's
  source.untracked_sha256, so run 1 was no longer byte-equal to runs 2 and
  3. Write and stage every file first; otherwise compare only runs that
  started after the last edit. Editing a tracked file mid-run breaks it too
  (source.uncommitted_diff_sha256; radio budget round 4, 2026-10-20): touch
  nothing in the worktree until the matrix ends.
- **Targeted tests miss the root `tests/*.py` files.** A new abstract method
  passed tests/modules and tests/gcs but broke the pinned IVehicle surface in
  tests/test_vehicle_logger_runtime_solid_regressions.py; grep tests/ for the
  class you change before trusting a targeted run.
- **A passing gate on a real run may be vacuous.** Break a copy of the run's
  evidence for each gate (delete a pose CSV, drop SNAP lines, drop one
  emission, drop the polling record), re-run analyze_run, and check that
  each copy fails the gate meant to catch it. A text grep for a message name
  misses: packet records carry message ids, not names.
- **Codex's read-only shell can fail to start.** Three of four codex exec
  runs ended with "helper_unknown_error: setup refresh had errors" before
  reading anything (cause and fix: see "Approach-window cadence
  diagnostics" below). Inline the diff and evidence in the prompt, and pass a
  long prompt on stdin (codex exec ... - < prompt.md): a 52 KB argument
  exits 126 on Windows without running Codex.

## Lockstep baton tests (step 10f-2, 2026-10-15)

- **Prove each test with a mutation.** Two first drafts passed with the rule
  broken: a stale-bid test (the peer never reported FREE), and an image test
  (distinct task ids hid a shared reassembler). Codex later found two more:
  an ACK test where the acceptance cancelled the timer, and a coalescing test
  without a repeated id. Next time, break the product rule, confirm the test
  fails, then restore the file. Check that the mutation really breaks the rule:
  in dev-sim step 1 a "shared key" mutation still gave each parameter its own
  key and survived; the real one (one key) showed a missing test.
- **Import-time locks stay on the wall runtime.** Class attributes made with
  rt.Lock() at import (MsgMetaProvider._lock, MsgRegistry._registry_lock)
  and module singletons (full_param_cache) are not rebound by a test's
  RuntimeBinding. Patch them with monkeypatch after binding; it also restores
  them.
- **Product code may swallow harness errors.** remove_vehicle ignores any
  exception from forget_vehicle. When a call passes through an except-pass,
  spy on it once to confirm it really ran on the baton.
- **Run codex exec with stdin closed.** In a background shell it waits on
  "Reading additional input from stdin..." until the timeout. Add < /dev/null.
- **Keep scratch scripts out of shared temp folders.** A stray dis.py in the
  temp folder shadowed the stdlib; use a task-specific directory.

## Sim A/B probes (20 ms spike residual, 2026-10-15)

- **Env vars do not reach the NavPy child of the demo eval.** An
  env-var-gated A/B arm silently ran the baseline code and gave a false
  negative. Hard-code the arm in a temp edit, and prove the arm is live (a
  dump file appears, or a metric moves) before reading its result.
- **Run-dir names cannot be reused.** A second eval with an old --run-dir
  fails with "WSL input step failed" (its WSL root exists). Use a new name.
- **Split the rate before guessing.** Logging ATTITUDE yaw, body rates and
  the dock NED vector per rendered frame splits the inertial LOS rate into
  azimuth, yaw-vs-gyro and pixel terms; that named the cause in one run.
- **Check the frame cadence before an A/B.** The sim poll loop waits with
  Event.wait, which on Windows follows the system timer tick (~15.6 ms)
  unless the process has 1 ms resolution; at 10x speedup that tripled the
  frame gaps between two sessions on the same code. Compare per-UAV poll
  counts (source-time detector_pose CSV) with the baseline run first.

## Gate split (step 10g-1, 2026-10-15)

- **Check a design's premises against later commits.** The fleet design
  asked for a strict xfail on todo 46. The EKF launch gate (61bdb67) had
  already made the slice dock, so the xfail was dropped. Before building a
  workaround the design names, grep the log for its todo.
- **Relaxing a check: do not drop evidence rows.** First fix for "a miss
  hides other gates" filtered the miss row out; Codex found it also hid a
  cadence gap before that row. Relax the refusal (a flag), keep the data.

## Polling-schedule replay (step 10g-2, 2026-10-15)

- **Grep every reader before widening a record.** Turning on operator
  evidence appends fields to `received`; the slice's confirmation timing
  unpacked it as a pair and would have crashed on the first evidence run.
- **A replay needs both ends anchored.** Rounds 1-2 of Codex found that
  dropping the first or the last passes still passed: anchor the start
  (a journaled loop start) and the end (the stream's unsubscribe), and
  account for every record in between.
- **Pick mutations that change behaviour on the tested inputs.** `<=` to
  `<` on "queued at start" was equivalent (the next branch gave the same
  answer) and survived; a stronger mutation showed the test was fine.
- **Wiring needs its own test.** Mutating the slice gate's call to the
  replay survived all replay tests; each integration point got a test.
- **Worktree files are CRLF here (core.autocrlf).** Byte-level mutation
  scripts must normalise line endings before matching.
- **Build gate fixtures from the real producer.** The replay tests and the
  slice fixtures run the real confirmation workflow on the baton and then
  change one record, instead of hand-writing a schedule the gate would only
  echo.

## Lockstep runs beside wall stacks (owner-plan scenario, 2026-10-15)

- **A "relay closed" with no firmware line can be another chat's kill.**
  The SITL test failed at frame 3722 with the journal byte-equal to a
  passing run up to the fault. Cause: a slot-0 wall eval stopped and
  relaunched, and gcs_stop's cleanup pkills `--sysid 1..3` across WSL,
  which matches lockstep SITLs on any slot (todo 51). SIGTERM ends the
  child silently; KILL, SEGV and HUP make the launcher print a line.
  Before a lockstep run, check `gcs_launch.py --list` and ask any slot-0..N
  wall user for a quiet window; after a fault, compare the WSL eval-root
  creation times with the run window.

## Fleet journal gates (step 10g-3, 2026-10-15)

- **Read these notes before the first edit.** The CRLF note above was
  already here and was missed. Check each file: tracked files are CRLF in
  the worktree, files new from the Write tool are LF.
- **Delete a runner's result file before each run and check its exit
  code.** A broken edit gave the mutation runner a syntax error, and the
  last run's "39/39 killed" file stayed in place and looked current.
- **Take fixture values from the producer, not from memory.** The fixture's
  ACK ref types were 6/7; NavPy writes `MsgType` values 130/140 and routes
  ACKs by them. The gate passed a chain NavPy would ignore. Import the
  producer's constants and match every field the consumer routes on.
- **Read the measurement the design names.** The fleet reused the slice's
  response-send reader (SITL admissions), but the design names the backend's
  emissions; under a backlog the two differ.

## Wall poll cadence and the Windows timer (2026-10-15)

- **A held timeBeginPeriod is not an honoured one.** Windows 11 dropped
  the 1 ms request of windowless eval processes once the screen locked;
  the API still returned 0. Measure the actual wait (perf_counter around
  Event.wait) instead of trusting the call; the fix is the
  ProcessPowerThrottling opt-out in windows_timer_resolution.py.
- **time.monotonic ticks at 15.6 ms on Windows.** Work times measured
  with it read 0 or 16 ms; use perf_counter for sub-tick diagnostics.
- **Run tests/guards before the full suite.** The SOLID 300-line module
  cap failed only after a 10-minute full run.

## Approach-window cadence diagnostics (2026-10-16)

- **Measure cadence inside the approach window.** Whole-run poll-gap
  medians hid starved final-approach windows; report gaps per episode in
  source time, entry->first and last->exit included
  (scripts/eval_gcs_demo_cadence.py).
- **codex exec -s read-only can fail every shell call on this host**
  ("helper_unknown_error: setup refresh had errors"). Cause found on
  2026-10-17: the standalone CLI on PATH (0.162.1) updates the ACL of
  node_repl.exe during sandbox setup and gets os error 32 while any
  node_repl.exe runs; the Codex desktop app keeps about ten running. Fix:
  run the CLI bundled with the desktop app
  (%LOCALAPPDATA%\OpenAI\Codex\bin\<hash>\codex.exe, 0.163), whose setup
  succeeds in the same state. Do not close the desktop app, and do not
  change ACLs. Inside that sandbox the user's Python 3.11 gives "Access is
  denied"; give Codex its own runtime Python
  (~\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe:
  numpy and pandas, no scipy or pymavlink). Pipe the prompt on stdin with
  `-`; PowerShell quoting broke a long argument.
- **time.thread_time steps 15.625 ms on Windows** (GetThreadTimes). Per-row
  CPU reads 0 or 16 ms; compare CPU with wall per wall second instead.
- **Bash heredocs fail on an apostrophe in the body.** Even a quoted
  heredoc ('PYEOF') ended in "unexpected EOF while looking for matching"
  when the body held a word like CSV's, and applied nothing; cat > file
  without a heredoc waited on stdin. Write edit scripts with the Write tool
  into the scratchpad and assert each replacement's count.
- **Log order is not event order across threads.** DOCK_DISARMED (NAV
  exit) was logged before DOCKED (the contact thread) in 2 of 3 wall UAVs,
  so "first terminal row" picked the wrong exit. Resolve an episode from all
  its rows; unit fixtures written in the intuitive order hid it, the wall
  eval showed it.
- **A mutation must remove the behaviour, not just the counter.** Dropping
  only the dedupe continue still counted duplicates as out_of_order, so
  tests passed; mutate the whole branch to see whether a test pins it.
- **Join two clocks through the producer, not the evaluator's timezone.**
  Mapping epoch wall_s to the debug CSV's time of day with
  datetime.fromtimestamp broke on midnight, dates and other timezones
  (Codex round 3). The fix: one epoch wall_s on the anchor row
  (DOCK_ARMED), and later rows placed by their offset from it, mod a day.

## Merging main into the HITL branch (2026-10-16)

- Two thread tests passed alone and failed once in the full suite. Do not re-run until green: force the suspected
  ordering (a delay in the bridge's open, the reopen without its wake-up) and check that the old test fails every time
  and the fixed one passes. Both were test races against fail-closed behaviour, not product bugs.
- A test that sends right after a fail-closed transition must allow the one counted stale drop, not retry until a
  packet lands: an open-ended retry hides an unwanted drop (`send_across_transition`, test_mav_router.py).
- pytest imports test modules at collection, so a run started before an edit tests the old file. Restart it.

## Bug-fix batch (hub load, gimbal stop, swarm clocks, todo 42; 2026-10-16)
- I repeated the heredoc mistake above: escaped newlines in a Bash heredoc became real
  newlines in Python source. Read this file before editing, and write edit scripts with
  the Write tool.
- A regression test I wrote passed on the old code too (Codex caught it). To prove a test
  fails before the fix, export the old source and run the test against it:
  `git archive HEAD src | tar -x -C <tmp>`, then run pytest with PYTHONPATH set to `<tmp>/src`.
- The shared .venv now has OpenCV 5.0.0, so the 4 `test_board_detector.py` failures are an
  environment issue (`board_detector.py:34`, IndexError), not a code regression.
- A new worktree has no `src/gcs/frontend/node_modules`: run `npm ci` before vitest, and
  delete `dist/` after a build. It is not gitignored: on 2026-10-20 a `git add -A src` after
  `npm run build` staged 404 dist files. Delete it before staging, and stage files by name.
- One-way message timestamps cannot separate clock offset from queue delay. An offset
  averaged from heartbeats absorbs reader backlog, so stale messages pass TTL.

## Delivery e2b slice (2026-10-16)

- **A Python heredoc turned `\b` in a regex into a backspace.** I wrote
  `rf"\bCONFIRMING"` through a Bash heredoc into Python source and got
  a 0x08 byte; the gate then never matched. Make regex edits with the Edit
  tool, and grep the file for control bytes after any scripted edit.
  Repeated 2026-10-19 (GCS vehicle window): same cause, same fix; the
  control-byte grep caught it.
- **Read the done criterion's own words before choosing a gate window.** I
  opened the approval window at `CONFIRMING`; the criterion said "during
  that approval orbit", and NavPy offers a second dock about 120 ms before
  `CONFIRMING`, in the same orbit. Changing a gate after a run is a goalpost
  risk: write down why, keep the run-time report, and get it reviewed.
- **A swarm offer carries no obj_id on the wire.** To prove which world dock
  was offered, log the link at the first registration
  (`Task offered: ... obj_id=`), not a position match.

## Lockstep matrix and digest attribution (step 11, 2026-10-16)

- **Prove a digest move with a run at the commit before, not by reading diffs.** I named
  2d44662 (a sim gimbal change) as the cause of a new digest; a run at the commit before it
  already had the new digest. Single seed-1 runs in detached worktrees at 10h's commit, the
  merge and its parent located it (the HITL merge 79ace48) in three runs.
- **Recertify after any rebase.** Main moved twice during step 11, and each move changed the
  digest, so each final certification ran on the exact tree that would fast-forward onto main.
  (A quiet PC is no longer needed since todo 51; see below.)
- **`gcs_stop.py --eval` cannot find a lockstep slot** (todo 55: the claim's owner is not
  normcased). Stop it with `gcs_stop.py --chat <its slot>` after checking that the registry
  entry's clone is this directory.
- **`sed -i` in Git Bash rewrites a CRLF file as LF.** Edit CRLF files with the Edit tool or
  with read_bytes/write_bytes, and check with `file` afterwards.

## Coordinating nested sessions (2026-10-17)

- **Do not ask a nested session to report back.** The owner opens nested sessions
  directly, so a "report to me" or "report it" step in a brief adds nothing. My
  briefs had one, and I relayed every interim note ("still waiting", "suite running")
  to the owner. The owner said this is not needed. In a brief, state the task and
  what done means. Pass on a note only when it brings a decision for the owner or
  finished work.

## Delivery e5 dock roles (2026-10-17)

- **A bare `Mock()` task actor looks assigned.** Its `selected_poi()` returns a truthy
  Mock, so `PeerTaskPriority.pending()` is true. Nine selector tests relied on that
  by accident once "assigned UAVs offer nothing" landed. Give an idle mock actor
  `selected_poi.return_value = None`.
- **Run the SOLID guard after every constructor change, not only `tests/modules`.**
  A Codex fix added a 13th dependency to `NavigationTaskAction`; only the full suite
  caught it. Prefer a port on the class that owns the behaviour.
- **Sim detections carry truth, not a projected position.** `SelfDetectedApproach`
  centres its circle on `truth_poi_location`; a test dock with only
  `set_p_t_g_loc` circles somewhere else. Build sim test docks with
  `replace_geo(replace(poi.geo, truth_poi_location=...))`.

## Wall cleanup by ownership (todo 51, 2026-10-17)

- **Do not edit the tree while a matrix runs.** `input_manifest.json` hashes the uncommitted
  diff and untracked files. A doc edit and a `git add -N` mid-run failed the artifact gate in
  two cells, although every digest was equal. Commit first, keep doc drafts outside the
  worktree, and apply them after the runs.
- **A `pgrep -f` probe matches its own `bash -c` shell.** My waits returned at once, so the
  first "wall stack mid-run" cycles stopped the stack before its SITLs existed. Anchor the
  pattern (`^/[^ ]*/bin/arduplane .*[-]-sim-address`), and log the matched pids with their
  environment, so the evidence shows that the target processes really existed.
- **Backslash continuations in a heredoc'd Python edit collapse into one line.** `\` plus a
  newline inside a normal string literal is a line continuation. Edit test code that has `\`
  continuations with the Edit tool.

## Delivery e6 gates and e10 runs (2026-10-17)

- **Check a gate's source and order against a real run before a review locks it in.** Three
  e6 checks were wrong on first contact with real logs: the frame-sampled SNAP line read
  2.1 m where the contact sensor read 0.05 m; the offerer locks d1 before its self-detect
  orbit, not after (my build note had it backwards, and a review then tightened the wrong
  chain); `approved_at_unix_s` is taken after the approve POST returns, so NavPy arms about
  0.5 s before it. Each fix took a run. Before writing an ordering or timing check, grep one
  existing run log (e2b, s11) for the exact markers and timestamps.
- **Read what a "check" field actually computes.** The approval record's
  `nav_before_approval` is the literal `False` (todo 59). A field name is not evidence.
- **`--analyze-only` prints the new report and leaves the run's `report.json` alone**, so a
  failed run can be re-gated after a gate fix without losing its original report.

## DOCKED final state (2026-10-17)

- The local `main` ref in a worktree can be far behind `origin/main`. Rebasing onto it replayed
  other chats' commits. Fetch, then rebase onto `origin/main`.
- The SNAP row is sampled only at command rows, and the dock contact latch stops the commands, so
  SNAP misses the crossing segment (SITL 2026-10-17: contact 0.07 m, SNAP 4.4 m). Decide or score
  docking from the contact sensor's `DOCKED` row, never from SNAP.

## Todo 64 disarm offset (2026-10-18)
- A position reported "at disarm" is the first GCS telemetry frame after the event, and frames are 0.2 s wall = 2 s sim
  at SIM_SPEEDUP 10. Before suspecting a frame offset, take the frames on both sides from the WS trace and check whether
  the line between them passes the dock horizontally; here within 0.7 m and the aircraft kept moving after a net-less contact.

## Todo 65 load (2026-10-18)
- A process priority set at spawn on a venv `python.exe` does not reach NavPy: the venv exe is a launcher and its child
  interpreter starts Normal. Check the real interpreter with Get-Process; set the priority inside the process.
- On a hybrid CPU, Windows EcoQoS runs a windowless process on E-cores under load (work 2x slower). Opt out with
  SetProcessInformation(ProcessPowerThrottling) and measure with a small benchmark first.
- On Windows Python 3.11 `time.monotonic` ticks at 15.6 ms. Never time 1-10 ms of work with it; use perf_counter.
- A full pytest run is not light load: about 70% CPU average with 100% peaks. Never overlap it with a wall eval.
- The eval env is an allow-list: ad-hoc env vars do not reach NavPy; derive probe paths from NAVPY_LOG_DIR.
- To tell a NavPy stall from a SITL stall, compare NavPy's ATTITUDE arrival lag with the GCS receiver's for the same UAV:
  equal lag is upstream (SITL/WSL), NavPy-only lag is NavPy.
- In Git Bash pass Windows paths in PYTHONPATH (`pwd -W`); a /c/... path silently imports the main checkout's package.
  Repeated 2026-10-23 (todo 31 fix): a new regression test "failed on the fix" because it ran main's code. Put
  `python -c "import navpy; print(navpy.__file__)"` in the same command as the first test run.

## GCS video streams and FPV (2026-10-18)
- `plugins/vehicle` must not import `core/perception`; pass the camera's stream specs in as plain
  data from the app composition layer, or the layer guard fails.
- `rt` (runtime_time) has no bounded-queue full exception; design request queues so they never need one.
- Standalone HTML mocks: never declare a global `top` (it clashes with `window.top` and blanks the page),
  and the app's file preview runs no JS, so pre-render mocks to static HTML or PNG.
- A saved "free map" camera must be taken on every departure from the free map, chase included;
  saving only when FPV/RC starts missed Free → chase → FPV → Free (Codex round 1).
- A profile field that mirrors a protocol numbering must enforce the protocol's rule (MAVLink stream
  ids are 1..count), or the consumer silently discards it.
- Untracked `.codex/agents/janitor.toml` / `live-tester.toml` in a worktree fail
  `test_gen_codex_agents.py` as STALE; check `git status` before blaming a change.
- v9 (FPV in the panel): I started coding a styling fix plus a wording change before the owner
  had agreed the layout, and the owner stopped it ("before agree with me design and show"). For
  UI feedback, show a side-by-side mock of the options first, then code the chosen one.
- Cesium 1.138 ignores the Viewer option `imageryProvider` (removed in 1.107); use `baseLayer`.
  A throw in a `scene.preRender` listener stops that widget's render loop with no `renderError`
  (`Scene.js:4440` raises it outside the error wrapper), so catch inside the listener.
- Two Cesium widgets cannot share attribution: a `Credit` owns one DOM element that each
  `CreditDisplay` re-parents, and Bing metadata (with its credits) is cached across provider
  instances, so a second view needs its own non-Bing providers (v10 chose terrain only).
- Mocks for the owner: the browser pane cannot screenshot local files; render with headless
  Edge (`msedge --headless=new --screenshot=... file:///...`) and send the PNG.

## Missed dock ends like DOCKED (todo 74, 2026-10-18)
- Making every FAILED end commands broke runtime tests that send a frame after a NAV_FAIL; those tests
  cover the real build, which arms no episode. Gate a new terminal rule on the scope that needs it (here
  an armed sim episode) instead of rewriting the old tests.
- Evidence parsers: check the same-identity conflict before generic order checks, or tests see a
  less specific message; require strictly rising episode numbers at every arm, whatever the outcome.
- SOLID size guards fail late in the full suite; split a new concern into its own port (dock_outcome)
  rather than growing a class past 15 methods.
- A terminal outcome must be read before `navigation_reset`: reset_phase clears the episode latch. Take an
  immutable snapshot in NAV cleanup, then decide and report after the exit succeeded.
- The venv has no pytest-xdist; `-n auto` fails at once. Run the full suite serially in the background.
- Tests that build a factory with `object.__new__` break when the code uses a new attribute (here the
  logger); set it in the test helper.

## Whole-chain HITL plan (2026-10-18)
- Codex turned a short firmware hash (9a8c7d66b2) into a full hash that does not exist and called
  it the reviewers' pin. Check every commit hash in a plan with `git cat-file -t` in the repo it
  names (WSL for the firmware) before it goes into a doc, and record the binary that runs (its
  embedded version string), not the branch tip.
- Codex cannot run the Claude CLI from its sandbox (connection refused), so its "review incomplete"
  note was about its own attempt. Start cross-reviews from the orchestrating session.
- Another chat had commits and uncommitted edits in this worktree's files. Run `git status` before
  editing. To commit only my change, build it from `git show HEAD:<file>`, stage it with
  `git hash-object -w --path` and `git update-index --cacheinfo`, then apply the same edit to the
  working copy. With autocrlf=true the blob is LF and the working copy CRLF.

## Run record and model input (HITL stage 1 step 1, 2026-10-18)
- **Never glob-delete in a shared temp folder.** I ran `rm -rf "$(cygpath "$TEMP")"/tmp*` to clean my
  test leftovers. In Git Bash `$TEMP` is `/tmp`, the shared `%LOCALAPPDATA%\Temp`, so it deleted every
  top-level `tmp*` entry there, including other sessions' files, and it cannot be undone. Delete only
  exact paths I created, and keep scratch files in the session scratchpad. pytest's `tmp_path` cleans
  itself.
- The private folder (`%LOCALAPPDATA%\navpy-hitl-tools`) is itself a git repository, so "refuse any git
  checkout" would refuse it. And "not inside REPO_ROOT" misses the main navpy-temp2 checkout, which is
  outside a worktree's REPO_ROOT. Require the run folder inside `private_dir()`, and refuse it when any
  folder above it holds `src/navpy`. A git-history test failed open: in Codex's sandbox git refused the
  main checkout ("dubious ownership"), and shallow clones report another root (Codex rounds 1-3).
- In this Bash tool, `\\` inside a quoted heredoc reached Python as `\`: `"\\n"` became a real newline
  and `C:\\Users` a bad `\U` escape. Write multi-line Python to a scratchpad file first.
- On this host `os.symlink` for a directory fails without the privilege; tests fall back to
  `_winapi.CreateJunction`. `Path.is_symlink()` and `rglob` treat a junction as a plain folder, while
  `shutil.copytree` copies through it. Find links with `os.lstat` and `FILE_ATTRIBUTE_REPARSE_POINT`.
- `math.isfinite(10**400)` raises OverflowError, and so does the reciprocal of 5e-324. JSON allows both,
  so a validator must catch it (Codex round 1).
- Mutating a "refuse inside the repository" guard lets the test write into REPO_ROOT/.logs. The
  mutation script must remove exactly that folder afterwards, and remove `.logs` only when it is empty.

## Reading-keyed optics (HITL stage 1 step 3, 2026-10-19)
- I first copied gimbal-controller's console-zoom knots (1.1-10.0) into the profile as measured rows. They are a
  focal-only estimate: centre and distortion are borrowed from the wide model, and POINTING-MODEL.md:180 says that
  distortion has the wrong sign at high zoom (Codex round 1). Before reusing a calibration, read its caveats and
  check that every field was measured at that zoom.
- Measured ZR10 calibrations on this PC (1920x1080): reading 1.0 = wall model at the wide stop
  (output/wall-matrix/run1/solution_v3.json); reading 1.8 = ChArUco after absolute zoom 2.0
  (output/tuning-2x/scale/base-model.json; held-out groups in out/recovered-unreachable-20261009). The per-zoom
  checkerboard fits were on the rig only (copied to the private folder 2026-10-19). navpy's older siyi_zr10 "1" row records no zoom state.
- The rig's per-zoom fits (~/calib-helpers/fits.json points at gimbal-controller out/zoom_calibrations, 2026-09-13
  sessions; the 2026-10-08 sweep is single-view and cannot fit a lens) pass held-out board projection but read
  1.8-2.6 % narrower in FOV than the reference, like the laser finding that board focal lengths are ~1.8 % high.
  Low held-out error does not validate focal scale. Check a fit against an independent scale (laser, pointing)
  and read gimbal-controller's reports on it (output/flat-zoom-trial-20260914/REPORT.md) before enabling a row.
- Resolved 2026-10-20 (owner: interpolation is a temporary solution): a single-view still-board sweep cannot fit a
  lens, but the similarity scale between its views is the focal ratio. tools/hitl/zr10_lens_table.py takes focal
  length from the sweep x one constant from the tilted fits x the laser factor, and centre/distortion from the
  tilted sets, interpolated and marked `basis`. Place each measured shape on the interpolation axis at the table's
  focal length for its setting, not its own biased fit focal length, or measured rows get blended with neighbours.
- Section 9 keeps private measurements out of the repository: measured constants, session ids and dated input
  folder names go in the private manifest (measurements/zr10_lens_inputs.json), not in the builder (Codex rounds 3-4).
- Do not mutate source files while a broad pytest run is in progress: a mutation loop on the builder made that run
  report false failures. Run mutations before or after, never during.
- One reading can be two lens states: reading 1.1 is scale 1.030 after absolute zoom 1.0 but 1.069 after a held
  zoom-in. Key rows by reading and by how the zoom was set.
- `Path.read_text` turns CRLF into LF, and `write_text(newline="")` then writes LF; `read_text(newline=...)` needs
  Python 3.13. Read with `open(path, newline="")` and write back the line ending found.
- The heredoc backslash collapse (step 1 above) hit again in a generated mutation list. Write any script that holds
  `\n` escapes with the Write tool.
- `tests/gcs/test_vision_profiles_route.py::test_known_gimbal_contract_metadata` fails on eee4cc3 without this change
  (expects gimbal_seq XYZ; the siyi_zr10 profile has ZYX).

## Deterministic dev sim step 2: seeded SITL noise and wind (2026-10-18)
- Every firmware commit moves the lockstep digest, even with every model off: the boot line
  `Init ArduPlane V4.6.3 (<git hash>)` is frame 1's serial0 egress. Prove a firmware patch inert
  by comparing journals record by record with a run on main (old firmware): only frame 1 may differ.
- A hook blocks the Edit tool on `\\wsl.localhost\...` paths. Copy the file out with `wsl cp`,
  edit the copy, copy it back, `chmod 644`, and check `git diff` in WSL (no CR, no mode change).
- In Git Bash, `wsl.exe -e bash ~/x.sh` expands `~` to the Windows home. Use
  `wsl.exe -e bash -c 'bash ~/x.sh'`.
- Two mutations survived because the tests could not tell them apart: an off draw carried no
  values, and the tap test fed the same bytes on both ports. Give each input distinct values.
- A new realism model can fail the mission on some seeds; that is a product finding, not a model
  bug. Separate the causes with seeds that turn one model off (here wind off, light wind, strong
  wind) before blaming either side. In the SITL, turbulence and noise share glibc `rand()`, so a
  wind A/B at one seed also changes the noise (todo 81).
- Independent Python streams do not make models independent inside the firmware. Before claiming
  that, check which RNG each firmware consumer draws from.
- When a step moves a baseline or makes a "today" claim false, grep the whole design doc for the
  old value and the old claim. Step 2 left section 1 and section 2's baseline stale.
- Quote a SITL wind as it applies: `SIM_WIND_SPD` is the speed from `SIM_WIND_T_ALT` (60 m) up, and
  the default square-root law cuts it near the ground. Turbulence is not scaled by height.
- A hook blocks Edit and Write on another worktree's files. To prepare a fix while a long check
  runs from your own worktree, draft in the scratchpad and copy the files in after the check.

## Moving a product out of core in reviewed steps (delivery move c2-c9, 2026-10-19)
- Never edit a worktree while its pytest or lockstep gate runs: the suite reads the files mid-run. Gate in
  one worktree, build the next commit in a side worktree, and fast-forward the gate worktree between commits.
- Long edit scripts sent inline through the Bash tool failed to parse ("unexpected EOF while looking for
  matching `''"), even in a quoted heredoc. Write them to files with the Write tool and run them; a small helper
  that keeps CRLF and fails on a missing or repeated match made every edit checkable.
- Before adding a test helper, grep the file for the name: a new `_offered` replaced an existing one and broke
  two unrelated tests.
- A word guard must check each whole token in any case before splitting camelCase, again after each `_` or `-`
  joiner (even a leading or trailing one), and count adjacent parts that join into a refused word (pick_up,
  PickUpLatch). Three review rounds each found one of these gaps. Pin each allow row to an exact count.
- When a plugin hands data to a product it may not import, pass a neutral core record and pin the shared wire
  constant with a test on both sides.
- Check every doc claim against the code before committing, even a closing note: a rewritten todo said `NavArgs`
  reads params only at start-up, which the same design doc contradicts (it re-reads at each task reset). An
  adversarial docs check caught it, plus stale line counts and cites in docs the move made stale.
- Delivery demo prep (2026-10-19): a stack launched with the frontend from a fresh worktree "came up" with a dead
  `npm run dev` (no `node_modules`, nothing logged); `delivery_demo.py launch` now waits for the frontend to answer.
- For a `source_command_stale` failure, read the source-time CSVs, not the navigation log (its times are cached
  and late): the lineage CSV gives the last frame NAV consumed, and the detector-poll CSV's `publish_ms` and
  `work_cpu_ms` show whether the poll was waiting (CPU near 0) or computing. Two runs on one day showed both.

## GCS vehicle window v10 (2026-10-19)
- `git add a b missing-path` aborts the whole add on the missing path, and a
  following `git commit` then records only what was staged before (here the
  renames, no content). After a rename, stage the new path only, and check
  `git show --stat HEAD` before reviewing a commit.
- Vitest 4: under React StrictMode the second dynamic `import('cesium')` of a
  `vi.mock`ed module got the real Cesium, even after warming the mock. Trace
  the import (which module came back) before blaming the component; test the
  effect's cancel guard directly (unmount before the import resolves) and
  check that the test fails with the guard removed.

## GCS video design v11 (2026-10-19)
- I wrote "nothing moves the small map" after removing the chase; Codex found the sidebar
  coordinate fly-to (`flyToRef` -> `camera.flyTo`). Before claiming nothing moves the Cesium
  camera, grep every caller of `camera.flyTo`, `setView`, `flyToRef` and `resetViewRef`.
- The design doc is CRLF in the worktree; the Edit tool and a Python splice left mixed line
  endings. After scripted edits, compare the CRLF and LF counts and normalise
  before committing.
- A conflict-resolving script failed and the chained git add and commit still
  ran, committing the markers. Chain them with && (or check the script's exit)
  and grep for markers before git add.

## GCS video v11 implementation (2026-10-20)
- Moving the window to the top right dropped the reset-view reserve, so the
  window got wider and 3 UAV buttons fit at 375 px: the overflow-menu fixture
  silently stopped testing the menu. After changing a layout anchor, redo the
  fixture sizes from the new formula and assert the precondition
  (overflow.length > 0).
- A test helper that found UAV buttons by accessible name broke for link-lost
  UAVs, whose name differs. Pick by a stable attribute (data-sysid).
- Line endings differ per file (locale JSON and some tests CRLF, others LF);
  a remembered "LF" was wrong. Run `file` on each target before a scripted edit.
- A full `pytest tests/gcs` started while edits went on: its vitest leg timed
  out at 5 s under load and the same test passed alone. Run the full suite
  after the edits, on a quiet PC, and rerun a timeout alone before blaming
  the change.
- A dirty file that main also changed blocks a fast-forward. Without a WIP
  commit or the shared stash: save `git diff -- <file>` as a patch, check the
  file out, fast-forward, `git apply --3way` (it stages), then `git reset -- <file>`.
- A chained `git rev-parse A B && cat > PROMPT.md <<EOF` stopped at the bad
  rev-parse, so the Codex prompt was never written and the run failed on a
  missing file. Write review prompts with the Write tool, and guard the run
  with `test -s PROMPT.md`.
- Each Synthetic mount made a WebGL context that `widget.destroy()` does not
  release; after about 16 the browser dropped the oldest, the main map's.
  Unit tests could not show it; a live 20x switch loop did. Release a
  Cesium widget's context (WEBGL_lose_context) on unmount, and add a
  mount/unmount loop to live checks of any per-view Cesium widget.
- Size floors chosen by feel (96 px, then 1 px) each broke a different design
  rule (credits, never size 0). Derive a floor from the thing it must hold
  (here the 138 px ion logo), and when the room is missing, offer no swap
  rather than a box that breaks a rule.

## Link model (dev sim step 3, 2026-10-19)
- A mutation can survive because a second mechanism enforces the same rule: removing the jitter clamp
  left serial bytes in order, since the delay line releases only its head. Mutate both mechanisms
  together to prove the test, and say which one each test covers.
- An order assertion that compares a list with itself sorted proves nothing when the input is already
  sorted. Pick parameters where the broken code would reorder, and confirm the test fails on it.
- Measure the traffic before applying a designed rate: the SITL sends 12.2 kB/s to NavPy, more than
  the 11.5 kB/s line the design assumed, so a rate on that direction would only build backlog.
- The bundled Codex binary route (see "Approach-window cadence diagnostics") was still needed: the
  PATH CLI failed its sandbox setup again.
- A "shared stream" mutation must share one RNG object. Giving every link the same key only gives each
  link its own copy of one sequence, so the isolation test survived it. Check that the mutant really
  shares state before trusting a kill or a survival.
- `sed -i` in Git Bash wrote LF over CRLF docs twice; re-add CRLF (`sed -i 's/$/\r/'`) or use the Edit
  tool on CRLF files.
- Never pass `-x` to the full pytest here: the known board_detector failures stop it early.
- An eval error late in a report can be a consequence: "demo_mission_plan.json missing" came from the
  operator failing at setup, not from the relative `--run-dir` it looked like. Read the first error.
- Check what the simulator already offers before building a model. SITL had a sim-time line rate
  (`SIM_BAUDLIMIT_EN`, with real `txspace` backpressure) and byte loss (`SIM_UART_LOSS`); I built a
  coordinator rate and loss anyway, and the owner had it replaced by SITL's own (2026-10-20). Grep the
  firmware's `SIM_` parameters for the effect first; build only what SITL lacks.

## Radio budget build (2026-10-20)
- `TaskStop` on a background bash runner killed only the current child: the runner went on to the
  next step and its `trap ... EXIT` (restore launcher, remove pc_busy) never ran. Do not stop a
  runner that way; if it must stop, kill its process tree and run the restore steps by hand.
- `pytest ... | tail -5` hid two real failures behind four known ones (the untracked codex toml), and
  one of them, the 300-line budget, was my own earlier commit's. Print every FAILED line (`-rf |
  grep FAILED`) and check each one before calling the rest "known".
- A filter change can break a test double that uses the filtered message as generic load (the
  router overload test's ENCAPSULATED_DATA blob). Grep the tests for the filtered ids when adding a
  filter.
- Codex's measurement corrected mine: the image is 35.9 kB of frames, not the 23.5 kB of one window
  of chunk payloads. Quote sizes as framed bytes from the journal's byte ranges.
- When Codex cannot read files (sandbox "setup refresh had errors" twice), paste the revised text
  with line numbers into the prompt for a narrow re-review.

## Todo 31 trace: detector frames late in NAV (2026-10-19)
- A wall eval's WSL SITL root is named from the run dir (`~/navpy-eval-roots/<name>-<hash>`), so a retry with the
  same run name fails at once with `WSL input step failed (1):` and empty stderr (`test ! -e` under `set -e`).
  Use a new run name for every attempt; remove your own roots afterwards.
- Under 16 busy processes the stack launch can time out at 60 s. Start load only after NavPy is up.
- To tell OS starvation from in-process contention, sample per-thread `QueryThreadCycleTime` from outside (TSC-based,
  not 15.6 ms quantized) and name threads with `py-spy dump --nonblocking`: if the process still gets its quiet CPU
  share while one thread's work stretches, the wait is inside the process (GIL or a lock), not the OS.
- Python `gc` gen-2 passes (30-70 ms here) look like a pure-CPU spike in whichever thread triggers them and like a
  wait in every other thread; log `gc.callbacks` before blaming a stage.

## GCS video design v12 (2026-10-20)
- SIYI's site refuses WebFetch and plain curl (HTTP 403, or the home page).
  The product's /download page, fetched with a browser user agent, links
  the manual PDF on res.siyi.biz; save it in its own scratch folder and read
  it with `pdftotext -layout`. Resellers disagree with SIYI (1080P vs
  1920x1200); cite the manual's page numbers, and state what it does not
  say (display density, browser) as unknown, not inferred.
- `backdrop-filter` makes a stacking context. On a box whose menu must draw
  above a sibling (the swapped small map) put the blur on a background
  layer; in a mock, a `z-index: -1` layer paints behind the map pane, so
  give the content `position: relative` instead (Codex rounds 1 and 3).
- A "not drawn" state must say what stops: hiding a picture left the
  reader and the WebGL widget running, and a status row that joins the
  measured chrome changes the room test it reports (Codex rounds 1-2).
- A mock over app screenshots: put each screenshot's data URI in one CSS class and give the
  slices that class. Inlining the URI per slice made the HTML 6.7 MB; one class made it 0.55 MB.
  Draw a mock on real screenshots from the start; the owner judges "same design as we have" on them.
- Implementing v12, a failure state inside the component that a room test unmounts erased itself: its
  row grew the chrome, the picture lost room, the failure unmounted, room returned and a new reader
  started. Latch a failure in the parent keyed by session; mount only the reader by room (Codex impl r1).
- A measured height reported from a passive effect before the ResizeObserver's first value is 0, and
  that 0 mounted a reader the real height had no room for. Report only once measured, start the
  consumer's state as null (unknown), and test with the measurement fed back as the app does.
- A new worktree has no .venv, so AAS_GCS.bat fails there; the live-tester launched gcs_launch.py
  with the main checkout's venv on PATH. The SITL fleet advertises no camera stream, so camera
  states need simulation.video_test_source to be seen live.
- v12.1 live: turning simulation.video_test_source on over the settings API after launch, then
  stop-all/start-all, made each camera seen but discovery ended "retries exhausted"; no UAV ever
  advertised a stream, so the mixed fleet was not set up. Unconfirmed cause: the launcher starts
  the test MediaMTX only at launch. Next time launch with the setting already on.

## GCS fence switch: show the fleet's fence (2026-10-20)
- Copying the vehicles' fence into plan state (adopt on read) raced with
  operator edits, stale reads and save/load; Codex found 7 bugs. Deriving
  it on each render from the newest fleet answer, and writing plan state only
  on an operator edit, removed the whole class. Prefer derived display state
  over copying observed state into request state.
- Edit wrappers that take a React functional updater must resolve it first:
  a helper may refuse (return null), and side effects run before that check
  act on an edit that never happened.
- Python-generated JS test titles with an apostrophe inside single quotes
  broke the parse twice. Use double-quoted titles.
- Editing files while a live-tester run was going made Vite hot-reload under
  it; the run's results were void. Freeze the tree during live runs.
- A live failure that looked like the fix not working (switch off after a
  reload) was a different bug: the backend timeline showed the downloaded
  plan replaced 50 ms later. Read plan_rendered and request order in
  timeline.jsonl before changing code for a live failure.

## Unreal capture cadence, HITL step 4 (2026-10-20)
- USceneCaptureComponent::IsCulledByDetailMode is protected; a pre-capture check
  must reimplement it (r.SceneCapture.CullByDetailMode and the cached scalability
  DetailMode). Check engine access levels before planning to call a helper.
- The editor build ran 2 compile actions at a time (3 GB free memory) and took
  17 minutes. Run it in the background with a monitor on its log and do other
  work meanwhile; filter for "Result:" and " error ", not "Error" ("-WarningsAsErrors"
  matches).
- `Automation RunTests NavPy` logs two "Condition failed" errors at startup from a
  CesiumRuntime struct check, before any test runs. Count "Result={Success}"
  against "Found N automation tests" instead of grepping for Error.
- Before writing a new offline helper, grep tools/unreal for one: the pose
  slerp and the well-conditioned quaternion angle already lived in
  pose_stamp_log.py.
- The first 10-minute run (2026-10-21) failed on my setup, not the code: a
  killed emulator lost its buffered log tail (the log is now line-buffered),
  the zoom flag was missing, and the deliberate recorder close cut one frame
  in flight. Check a run's health 30 s in (outcome counts, ticks per second)
  before waiting the full 10 minutes.
- This laptop's full scene ticks ~18-26 times per second, so a cadence above
  the tick rate fails the rate gate. F = 20 passed (19.66). Measure the tick
  rate before choosing F.
- A dock-pass replay with an unaimed gimbal keeps the board in detection
  range under 1 s per pass: 11 board frames in 10 minutes. Aim the gimbal at
  the board (harness only) to get enough frames for the timing probe.
- The Codex binary under AppData\Local\OpenAI\Codex\bin\<hash>\ moves on
  update, and bin\codex.exe there was an old alpha that rejects the current
  config. Use AppData\Local\Programs\OpenAI\Codex\bin\codex.exe (check
  `--version`).
- The SOLID guard (tests/guards) caps a new tools/ module at 300 lines and
  wants every new definition annotated. The 736-line checker passed five Codex
  rounds and only the full suite caught it. Run tests/guards with the targeted
  tests before the first review, not at the end.
- Build Unreal with -DisableAdaptiveUnity before a PR. Adaptive unity compiles
  changed files alone, so the incremental build hid a name clash (two test files'
  anonymous-namespace constants) that broke the clean unity build on main
  (aas-sim-unreal PR 2).

## Delivery demo as a normal run (2026-10-20)
- I designed dock waypoints around the ZR10 preset altitudes (403/383/363 m),
  but the owner's settings name no camera profile, so the UI planner flew its
  defaults (240/220/200 m). Codex caught it in code review; the live log
  confirmed it (v_dist at approach start plus dock height above home). Before
  reasoning from planner altitudes, read the settings the run actually uses
  (`camera.vision_profile`) and confirm the flown altitude in a log.
- A subprocess test of a script's imports passed without the fix, because the
  shared venv's editable install points at another checkout that also
  imports. Assert where the module came from (`navpy.__file__`), and
  mutation-check by restoring the old file.
- A background waiter that grepped Codex output for "verdict" fired at once:
  the echoed prompt contains the word. Wait for `tokens used`.
- `codex exec` sometimes fails before any command runs ("setup refresh had
  errors") and still prints REJECT. That is no review; retry once.

## Plan/camera check (2026-10-20)
- Search optics are the device's first zoom (CameraIntrinsics starts there; a track stop resets to it), not
  settings `vision_zoom`. Codex caught the selected-zoom version in design review; check which zoom a sim gate uses.
- Exact visibility on a straight segment is cheap: camera coordinates are linear in along-track distance, so edges
  are linear bounds. Keep the boundary cases the simulator rejects (zero distance) out of a closed interval.
- Do not run the full pytest suite while a mutation script edits files: its vitest step read a mutated file and
  failed. Run mutation proofs alone, then the suites.
- From Python on Windows, capture `npx` output with `encoding="utf-8", errors="replace"`; the cp1252 default
  floods stderr with reader-thread UnicodeDecodeErrors (the return code is still right).
- A worktree has no node_modules. `npm ci` there took 13 s; a link to the main checkout's copy would let a build
  wipe a running dev server's `.vite` cache.
- App.jsx hands hooks new vehicleList arrays and param getters on telemetry and param updates, so an effect keyed on
  a memo built from them refired the backend check about once a second. Unit tests missed it; the live test caught it.
  Key such effects on the request content (a JSON key) and test a rerender with equal but new inputs.
- When `codex exec` cannot start its shell ("helper_unknown_error: setup refresh had errors"), inline the full patch
  (tracked diff plus `git diff --no-index /dev/null <new file>`) in the prompt; it then reviews statically.

## GCS Trails control (2026-10-20)

- A live-tester brief that said "never ports 3000/8000, never slot 0" stopped
  the run: the launcher correctly gave this checkout the free slot 0. Brief
  by owner, not by port: "your auto-resolved slot; never another session's".
- The default 3-UAV sim has camera.vision_profile null, so no camera marks
  record. Set a profile through the settings API before opening the page; a
  later reload was denied in auto mode. Restore it to null afterwards.
- A real link outage is hard to make live (suspending SITL or the router left
  link_ok true); the stale-then-fresh run break is proven by unit tests only.

## Todo 31 root cause: reader blocked at approval (2026-10-23)

- A "stale source" failure was a telemetry backlog, not a slow frame path. Before tracing detector stages, list
  every NavPy ATTITUDE arrival gap above 150 ms and compare with the GCS's own reader: one NavPy-only gap per UAV,
  with boot time contiguous afterwards (buffered, nothing lost), pointed at a blocked reader within minutes.
- MAVLink callbacks run on the vehicle's single reader thread. Anything a callback waits on (a lock held across
  paced sends, a sleep) stalls every stream of that vehicle; NetworkMavlink now dispatches on its own thread.
- On a CRLF working-copy file, appending after the line's `\r` makes git treat the whole file as changed. Insert
  before the `\r`. Git Bash `grep -c $'\r'` printed 0 on that CRLF file; check endings with `git ls-files --eol`.

## Unreal lens table, HITL step 5 (2026-10-23)
- Codex code review took five rounds, each finding a real hole in the checker's
  coverage: a pooled p95 hid one wrong setting, travels counted from the actor's
  own held captures skipped a travel it never captured, and travels without an
  end let later held frames stand in. Derive every coverage requirement from the
  independent source (emulator steps), gate each group on its own, and count only
  frames whose mapped query falls inside the interval.
- I set a 0.5 px reprojection gate in the design from a guess; the right rows
  score 0.39-1.06 px live. Before fixing a pixel gate, measure the right row and
  the nearest wrong rows on live data (scratch discriminate.py), then set it.
- ReadingIntrinsicsTable.settings() and optics() take numbers: a string reading
  ("1.1") returns () or None without an error. Pass float(reading).
- A bare `cat > file` in a Bash command (no heredoc) waits on stdin until the
  tool times out. Write scripts with the Write tool.
- Other chats' pytest and GCS stacks started during the live run; check the
  process list before and after a timing measurement and report the load.

## NPYC v2 and reference-copy sync, HITL step 6 (2026-10-23)
- I repeated the heredoc mistake again (a regex lost its backslashes). For edits with
  backslashes or quotes, use the Edit tool or a script written with the Write tool and
  run with `python -I`.
- Syncing aas-hitl's Unreal reference copy moved every cited line number, and two
  docstrings stated behaviour the new receiver no longer has (it now reads `sequence`
  and `session_id`). After a sync, map each old citation by its line text
  (scratch line_remap.py), then reread the producer for every "does not read" claim.
- My first plan note claimed fragment sizes and a fixture count that no test used.
  Codex caught it. Write each number in a completion note from the test's own
  parametrization, and name what a test excludes.
- A relay refusal test used a receiver sized 4x2 for 1280x720 fixtures, so a size
  mismatch could stand in for the real refusal. Size the receiver to the fixture and
  assert the fixture's own reason.

## Downloaded plan hold and loaded-plan fence read (todos 90, 91, 2026-10-23)

- The Edit tool rewrote a CRLF hook file as LF. For CRLF files, edit with a small
  script that normalizes to LF, replaces exact strings (assert one match) and saves CRLF.
- Check phase gating before trusting a "reload" repro: every plan regen effect runs only
  in PLANNING, and after a reload the app starts in MONITOR, so the race needed the
  planner open. Reproduce the mechanism, not the reported path.
- A download always saves its delivery hubs to settings, so a settings round trip
  follows every download; anything keyed on settingsVersion runs after it.
- A one-shot "skip the next regen" flag is consumed by whichever effect runs first.
  Key a hold on the inputs it protects (a JSON of the generator inputs), released by
  any input change or an explicit replacement.
- Mirror App's effect declaration order in hook harnesses: a harness that ran
  useVehicleConnection before App's demo-default effect hid a real window.
- Five Codex rounds each found real lifecycle gaps (late joins, reconnects, corridor
  files, multi-set arrays). Ask the reviewer to try roster changes, every file-load
  variant and multi-set counts up front.

## UniRC 10 Pro emulator (2026-10-23)
- Application Control blocks cmdline-tools 23's `sdkmanager` (it calls the new
  `android` CLI, os error 4551). emulator, adb and avdmanager still run.
  Install system images with Android Studio's SDK Manager.
- A 1920x1200 AVD is naturally landscape: user_rotation 0 is landscape, 1 is
  portrait. `wm density` changes dpr live, with no reboot.
- Measure the page through Chrome's DevTools socket (`adb forward tcp:<p>
  localabstract:chrome_devtools_remote`), not screenshots. Chrome reads
  `/data/local/tmp/chrome-command-line` only as the debug app, which skips
  first run. The `application_id` extra did not make it reuse a tab.
- Android Studio's "Intel x86_64 Atom System Image" is the plain image
  (`default`): no Chrome, only WebView and its test browser
  (`org.chromium.webview_shell`); its devtools socket is
  `webview_devtools_remote_<pid>`. An old WebView exposes CSS gaps that
  Chrome hides (todo 113: `dvh`).
- A background tool call that starts the emulator through a `2>&1 |` pipe
  did not return (likely the emulator holds the pipe). Write output to a
  file instead.
- On a busy host the first boot hangs System UI (ANR) and the Vite dev page
  takes about a minute to leave its splash. Wait for state, not for a fixed
  time.

## Media runtime check, HITL step 7 (2026-10-24)

- Read this file before the first test run and Codex call. I repeated two recorded mistakes: Git Bash POSIX
  PYTHONPATH (imports came from the shared checkout; see "ZR10 SIYI emulator") and Codex's read-only shell failing
  to start (see "Codex's read-only shell"); both cost a round.
- Official GStreamer 1.26.x Windows installers ship PyGObject for one Python only (`_gi.cp313-win_amd64.pyd`, per a
  GStreamer maintainer); navpy's venv is 3.11. Match `gi\_gi*.pyd` against `importlib.machinery.EXTENSION_SUFFIXES`
  before assuming a venv can import the bindings.
- A probe that must run from a separate Python without navpy's dependencies still meets the 300-line module guard
  by splitting into sibling stdlib-only modules and running `python -E -s -m tools.hitl.<name>` from the repository
  root. `-I` implies `-P`, which drops the root from sys.path, so package imports fail under it.
- Check the host's code-integrity policy before installing native Windows binaries: Smart App Control (registry
  `HKLM\SYSTEM\CurrentControlSet\Control\CI\Policy` VerifiedAndReputablePolicyState=1, CodeIntegrity events
  3033/3077) refused the unsigned `_gi.cp313-win_amd64.pyd` after both installs succeeded, and it has no per-file
  exception. Linux binaries in WSL are outside it; the media stack moved to WSL Ubuntu-24.04 (2026-10-24).
- An element that registers can still fail on frames: `nvh265enc` (1.24) refused its preset with driver 597.13 while
  `nvcudah265enc` worked. A media check must push real buffers, not only find and create elements.
- packages.ubuntu.com can return HTTP 500; `dpkg-deb -c` on the .deb from archive.ubuntu.com's pool and
  Launchpad's `git.launchpad.net/ubuntu/+source/<src>/plain/debian/...?h=ubuntu/<suite>` settle package contents.
  Codex's claim that jammy lacks nvcodec came from that site and was wrong.
- A session hook refuses writes to another session's worktree; branch inside this session's own worktree.

## Todo 105: bounded mission transfers (2026-10-23)

- An "exact" check against autopilot state needs its differences logged from the first version. The readback
  after a lost final ACK never matched under `stress` (ArduPilot returns frame 0 for a command without a
  location), and the first run could not say why; one logged line and a 2.5 min single stress cell
  (`eval_gcs_navigation_demo.py --lockstep --sim-profile stress`) found it.
- Passing deadline plumbing through a class or route can trip the SOLID guards (12 constructor dependencies,
  300-line modules, 150-line functions). Run tests/guards with the first targeted tests, not at the end.
- A request deadline must be checked where the work starts (in the executor thread), not before an awaited
  broadcast, and a nested wait (reconnect's link wait) must take only the time left.
- Mutation patterns in a Python list written through a bash heredoc lose their `\n` escapes; write them
  with the Edit tool or build them with chr(92).

## Todo 114: swarm handover under byte loss (2026-10-24)

- The owner rejected raising copy counts: find why each message is lost first (per-message journal
  traces) and fix the cause. Here the sim lost bytes on the serial0 cable too, and the protocol stopped
  after a fixed count instead of repeating until answered.
- When a sim model deletes data itself, keep the clean stream beside it and match by position (offset
  map), not by content: byte-identical repeats and frames embedded in payloads defeat content matching.
- At the end of a lossy stream, ask which clean packets can still be waiting and why. My first settle
  recorded only packets a lost byte fell in, and missed an intact packet held behind a frame the loss
  started; Codex found it. Every packet complete in the clean stream but unmatched is loss-caused.
- model_stream keys allow only [A-Za-z0-9._-]; key per-device draws by router endpoint name.
- Both Manifest and RoutedPorts sat at the 12-field SOLID limit; adding a field means removing one or
  passing it as an argument. Run tests/guards early.

## Handheld app build and todo 113 (2026-10-25)
- The Windows APK build (`scripts/build_apk.ps1`) needs `zipalign -p 4` before
  `apksigner`: Android 11+ refuses an APK whose resources.arsc is not aligned.
  Adding classes.dex works with .NET `ZipFile` (no `zip` on Windows).
- PowerShell 5.1 with `$ErrorActionPreference = "Stop"` and `*>` redirection
  turns adb's stderr ("daemon not running") into a terminating error. Run
  `unirc_emulator.ps1` from Bash with `> file 2>&1` instead; the script then
  also sets `accelerometer_rotation 0`, which `user_rotation` needs.
- To test the APK against this PC: start the GCS with `GCS_FRONTEND_DIST` set
  to the built dist, then `adb reverse tcp:8000 tcp:<backend port>`. This
  tests the production bundle, which is what the handheld runs.
- The Claude browser pane is hidden during tool work, so rAF stops and Cesium
  never sizes its canvas (300x150). For desktop checks use headless Chrome
  with its own `--user-data-dir` in the scratchpad and CDP.
- A desktop map that is black with Cesium's CSS missing came from the service
  worker (todo 118), not the layout: bypass it with CDP
  `Network.setBypassServiceWorker` to split the two.
- Python one-liners in a Bash heredoc turned `\b` and `\a` in Windows paths
  into control bytes. Write such text with the Edit or Write tool.

## Todos 92, 110: demo open points (2026-10-25)
- A retry that opens a new protocol session drops the late reply to the old one. Before changing an
  FTP/MAVLink retry, read the firmware's server (ArduPilot GCS_FTP.cpp replays its last reply for a
  same-session, same-seq resend). Build test fakes from that source, not from the client.
- A UI default and the companion default can disagree silently: check which profile each side picks
  when settings name none, and guard the preset with a test against the real reach check.
- Hold a round's fence per wire unit, not around a whole multi-chunk send; do slow work outside it.
- The shared venv had OpenCV 5.0.0 on 2026-10-25, so 4 board-detector tests failed; not a code fault.

## Todos 116, 117: image share and confirmed launch mode (2026-10-25)

- I repeated the time.monotonic mistake above: a pacer test timed 30 ms gaps with it and failed
  at random under mutation runs. Time wall-clock tests with time.perf_counter.
- Never run a test baseline while a mutation script is editing the same source files: the
  baseline then tests a mutant. Wait for the mutation run, then run the baseline.
- A surviving mutation can mean the guarded state is unreachable through the caller (a new
  round already abandons the old image). Test the guard at its own unit, or remove it.
- Guards to run early: launch_execution.py is near its 300-line cap and task_confirm_rounds.py
  near 180. VehicleMav's method count is a ratchet: raise it only with a dated comment.
