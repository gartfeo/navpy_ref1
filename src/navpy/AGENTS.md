# NavPy Core — layer guide

Read this when working on the navigation framework (`src/navpy/`), simulations,
or navigation-log analysis.

## Architecture

NavPy provides mission-agnostic navigation and swarm coordination; mission
modules plug in on top (see root `AGENTS.md`). Source lives in `src/navpy/`.

- `modules/common`: shared types such as `Location`, `Attitude`, `Wind`
- `modules/vehicle`: `IVehicle` and MAVLink implementation via `VehicleMav`
- `modules/vision`: detector abstractions and implementations
- `modules/navigation`: navigation algorithms, `GeoRefCalc`, `ZcUtil`, L1/PID/PN navigation
- `modules/nav`: `NavController` state machine (`nav_state.NavState`): `ONHOLD`, `DETECT`,
  `CONFIRM`, `NAV`, `RESET`, `RECOVERY`
- `modules/comm`: `NetworkWifi`, `NetworkSerial`, `NetworkMavlink`
- `modules/swarm`: `TaskActor`, `TaskDispatch`, multi-vehicle coordination
  (assignment acks: `docs/design/swarm-task-assignment-ack.md`)

Common patterns: ABC interfaces with `@abstractmethod`, args dataclasses in
`src/navpy/args/`, factory functions such as `create_vehicle()` and
`create_network()`.

## Navigation parameters

Priority: `CLI arg > MAVLink param > default`.

Important params: `AAS_DEL_PITCH`, `AAS_DEL_THR`, `AAS_DEL_DIR`,
`AAS_USE_TRN`, `AAS_DEL_PLD`, `AAS_DEL_PLRD` (authoritative list:
`src/navpy/args/navigation_args.py`).

## Running / simulation

```powershell
python -m pip install -e .          # editable install
python -m navpy.main [options]      # run NavPy
```

Simulation from Git Bash (standalone, without the GCS stack):

```bash
PYTHONPATH="$PWD/src" .venv/Scripts/python.exe -m navpy.main -c udp:0.0.0.0:14560 -ss 1 -nt mav -lsd Vehicle Network
```

For SITL + the GCS app, use the isolated launcher (root `AGENTS.md` "Running
the GCS stack"); for headless SITL evals, `python scripts/swarm_run.py --eval`.

## Navigation log analysis

For navigation accuracy analysis, the compact CSV log is authoritative — do not
rely on console grep as the source of truth.

Compact logs live under `.logs/` and usually match:

```text
.logs/{date}/{time}/uav_{sys_id}_navigation_compact.csv
```

Find the latest compact log from PowerShell:

```powershell
Get-ChildItem -Recurse -Filter "*_compact.csv" .logs |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1
```

SNAP rows contain closest-point accuracy fields such as `3d=`, `h=`, and `v=`.
Useful columns include `ts`, `dist`, `cmd_r`, `cmd_p`, `yaw_err`, `pitch_err`,
`act_r`, `act_p`, `x_err`, and `y_err`.
