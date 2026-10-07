# GCS Web Application — layer guide

Read this when working on the GCS app (`src/gcs/`). Hard launch rules
(launcher-only, never `--chat`, never broad-kill) live in the root `AGENTS.md`
and apply always.

The GCS supports UAV swarm missions (delivery and other plug-in mission
modules) with cooperative, authorized recipients. Use the project purpose and
glossary in the root `AGENTS.md` to distinguish detection review, mission-module
execution and verified mission outcome (e.g. package handover for delivery).
Existing API keys and MAVLink names retain their compatibility meanings.

## Architecture

Backend:

- FastAPI entrypoint: `gcs.backend.main:app`
- base port: `8000` + chat index `N`
- routes include `/health`, `/api/vehicles`, `/api/control`, `/api/diagnostics`
  (registered in `route_registration.py`)
- WebSocket: `/ws/telemetry`, telemetry broadcast at 5 Hz

Frontend:

- React/Vite/Cesium app under `src/gcs/frontend`
- base dev port: `3000` + chat index `N`
- scripts: `npm run dev`, `npm run build`, `npm run preview`
- Handheld (Termux) deployment: the backend serves the built frontend itself
  when `GCS_FRONTEND_DIST` is set
- Tracking-overlay video (monitor view, bottom-left panel): build-time env var
  `VITE_VIDEO_WHEP_URL`, the WHEP endpoint of the Jetson publisher's MediaMTX
  (e.g. `http://192.168.144.10:8889/tracking/whep` — derive it from the
  publisher's own config, do not assume it). Absent by default, which hides the
  panel; it is read at build time only, so changing it needs a rebuild. Public
  and credential-free — never put credentials in it, and it deliberately has no
  settings field and no backend route. The WHEP client is vendored third-party
  source: `frontend/src/vendor/mediamtx/` (see its `NOTICE`).
- Cesium Ion imagery/terrain: build-time env var `VITE_CESIUM_ION_TOKEN`, set
  in an untracked `src/gcs/frontend/.env.local`. Absent, the map stays on OSM.
  The repository is public — never commit the token
  (`tests/gcs/test_no_committed_tokens.py` enforces this).

## Multi-instance details

- Each launch auto-allocates a chat slot `N`; all ports derive from it. Single
  source of truth: `src/gcs/backend/instance_ports.py`; the backend reads
  `GCS_CHAT_INDEX` set by the launcher.
- Slots live in a machine-wide registry (`~/.gcs/instances.json`, override
  `GCS_INSTANCE_REGISTRY`), keyed by the launch directory — one directory =
  one instance; use a separate clone/worktree for a second.
- `gcs_stop.py` kills only your PIDs + your SITL and reaps orphans (e.g. a
  leftover Vite dev server) holding your slot's ports; `gcs_launch` reaps the
  same ports before spawning.
- **Do NOT run `swarm_run.py` on top of a `gcs_launch` stack** — the launcher
  already starts SITL (`--sitl` default), and `swarm_run.py` begins with
  `cleanup(chat)`, killing the chat's running SITL/router.
- **Registry inspection must be read-only via the JSON file** when you must
  not mutate it — `gcs_launch.py --list` calls `registry.live()`, which prunes
  dead entries and persists.
- Eval band: `swarm_run.py --eval` allocates chats 40..84 (can never take an
  interactive slot); NavPy companions bind per-vehicle UDP ports
  (`udp:0.0.0.0:5760+10*(g-1)`) that SITL serial0 streams into
  (`COMPANION_UDP=1`). Do not use `gcs_launch` for eval-only runs. Manual
  `run_swarm.sh` invocations must pass `COMPANION_UDP=1` to match
  launcher-started companions.
- `--chat N` exists as an advanced manual override of the slot/band — never
  use it in chats; slots always auto-resolve by launch directory.
- Demo (`sim_mode`) + `dev_mode` default on; `connection.auto_connect`
  (default on) auto-discovers + connects on startup in sim mode.
- In **dev mode only**, the running git branch is shown in the tab title +
  topbar so parallel instances are distinguishable; non-dev (operator) mode
  hides it (`devMode` gates both, from `settings.simulation.dev_mode`).

## UI behavior expectations

- `Connect` should discover vehicles after a short wait; buttons should then
  say `Stop NavPy`, not `Start NavPy`.
- `START` launches all UAVs with a short click. Long-press `START` is only for
  low-battery force-start override.
- `E-STOP` requires confirmation.
- `Companion computer not connected` usually means NavPy crashed or failed to
  start — NavPy runs as separate processes, so check NavPy process logs, not
  only backend logs.

## Verification

- Frontend build (PowerShell):

  ```powershell
  cd src\gcs\frontend
  npm run build
  ```

  A prod build wipes a running dev server's `.vite` optimized-deps cache —
  don't build against a live dev server, or restart it afterwards.
- Frontend JavaScript utilities: prefer tests that invoke Node.js subprocesses
  from Python tests, matching the existing test style.
- Live testing: delegate the full GCS/SITL flow (launch, connect, exercise,
  check logs, tear down) to the `live-tester` subagent; its procedure is in
  `.claude/agents/live-tester.md`.
