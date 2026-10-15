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
  newlines twice. Edit such lines with the Edit tool and check with `ast.parse`.
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
- navpy's SIYI readback decomposes roll-pitch-yaw (matrix_to_xyz, g_seq "XYZ"); the real gimbal and Unreal are
  yaw-outer (Rz Ry Rx). They differ once yaw and pitch are both non-zero (yaw 90 / pitch -45: horizontal vs 45 deg
  down). Name the Euler order of every angle set that crosses a module boundary.
- navpy's sim plant clamps world-angle differences, which equal body angles only when the aircraft is level. Test
  limits and conventions banked and turning, not only level; Codex found a banked camera past the pitch stop.
- Run the module-size guard (tests/guards/test_solid_architecture.py) before review, not after: new modules over
  300 lines failed it in the full suite and forced a split after the first review.
- "Fail closed" covers every output: a stale input stopped the Unreal stream but navpy still got fresh-looking
  SIYI replies until the review pointed it out.
