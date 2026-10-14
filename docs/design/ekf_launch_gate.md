# EKF launch gate (design v2, todo 46)

## Problem
The deterministic 1-UAV slice passes 50 m above its dock (todo 46). Cause
(`.logs/d46/`, `scratchpad/diag46/progress.md`): the operator launches at
~11 s sim time while ArduPlane's EKF3 is uninitialised (flags `0x400`, then
`0x0`). The EKF origin is set in flight at ~47 m, and `GLOBAL_POSITION_INT`
jumps +46 m at 21.0 s, while `GPS_RAW_INT` and `AHRS2` stay right. The
aircraft flies the approach on that altitude and hits the ground ("SIM Hit
ground", 102.2 s). With a probe gate (and lockstep terrain, below) the slice
docks at 0.11 m with matching digests. This shows the timing failure; it does
not prove the law correct elsewhere.

Wall mode hides it only because launch comes ~280 s of sim time after
connect. The GCS launch gate (`src/gcs/backend/routes/control_readiness.py`)
never checks the estimator, and the demo sets `ARMING_CHECK=0`
(`scripts/eval_gcs_demo_sitl_root.py:54`), so ArduPlane's own checks are off.

## Fix
1. **Stream.** On each vehicle's GCS connection the backend requests
   `EKF_STATUS_REPORT` (193) with `MAV_CMD_SET_MESSAGE_INTERVAL` at a named
   rate, re-requested on reconnect (`src/gcs/backend/vehicle_manager.py`).
2. **Fresh, owned report.** Readiness reads the latest report with its receipt
   time; one older than a named limit (a few intervals) blocks as "EKF status
   stale". Only the vehicle's own autopilot component counts (sysid and
   component admission at the router).
3. **Bits** (`EKF_STATUS_FLAGS`, one named mask pair with a reason per bit):
   - required `0x37`: attitude, horizontal and vertical velocity, absolute
     horizontal position, absolute vertical position;
   - forbidden `0x8480`: uninitialised, constant-position mode (no GPS aiding
     yet), GPS glitch. A glitch blocks only while reported; a fresh clear
     report clears it.
   - no report: "waiting for EKF status" (fail closed).
   Baro-only height can set absolute vertical position; that is accepted, as
   ArduPlane does, and noted.
4. **Every launch path.** Checked at preparation, and again right before
   every arm and release after any wait: container trigger (both arm
   branches and the already-armed release, `launch_execution.py:70,105,124`),
   bungee (`control_launch_actions.py:66`) and restart
   (`control_launch_workflow.py:133`). So a delayed multi-vehicle launch
   cannot arm or release a vehicle whose EKF went bad. The prepared session
   carries its gates and `force` policy to these re-checks; `force` keeps
   today's meaning. `/restart` keeps its fixed default gates
   (`control_readiness.py:48`), which now include `check_ekf`.
5. **Config/UI.** `LaunchSettings.check_ekf_enabled` (default on) maps into
   `ReadinessGates.check_ekf`; the toggle lives in
   `frontend/src/components/settings/LaunchTuningSection.jsx`. The vehicle
   snapshot (`vehicle_snapshot.py:71`) adds the report age, and the
   existing EKF row (`frontend/src/utils/preflightReadiness.js:105`) uses the
   same masks, freshness and gate setting as the backend, so the operator
   never sees "EKF ok" while launch is blocked (and the reverse).

Firmware checks: the demo's `ARMING_CHECK=0` stays for the eval (it disables
RC/battery checks the sim cannot pass), now covered by this gate; direct ARM
(`control_vehicle_commands.py:66`) bypasses GCS readiness, as today. Restoring
ArduPlane's INS/AHRS and GPS checks is logged as a todo.

Lockstep terrain (`TERRAIN_ENABLE=0`, no tiles: SITL ground 3.46 m above the
dock terrain) is taken by the deterministic-eval chat.

## Tests
- Readiness: `0x400`, `0x0`, missing absolute vertical position, constant
  position mode, GPS glitch, stale report, other component's report and no
  report each block with their message; ready flags pass; glitch then clear
  passes; the gate off skips it.
- Every re-check: container (both arm branches, release), bungee and restart
  do not arm or release a vehicle whose EKF turns bad during the wait.
- Frontend EKF row: same verdict as the backend for each flag/age case.
- Stream: the interval request is sent per vehicle on connect and reconnect.
- Frontend: `npm test`, `npm run build`.
- SITL: a wall 3-UAV eval still docks; the lockstep slice (with the terrain
  fix) docks twice with one digest. Live app check via `live-tester`.
