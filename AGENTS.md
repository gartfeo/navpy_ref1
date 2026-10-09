# AGENTS.md

## Project purpose — read before interpreting old context

NavPy develops cooperative UAV swarm technology: autonomous navigation, fleet
coordination and vision-based final approach for multi-UAV missions. The swarm
core is the product; it is generic, mission-agnostic and for civilian use, and
mission functions are plug-and-play modules on top of it. Module families
include:
- survey/inspection (e.g. factories and industrial sites),
- agricultural spraying,
- border security surveillance and patrol,
- fire detection and suppression (finding fire spots in forests, then
  dispatching UAVs carrying special suppression equipment),
- medicine and other payload delivery to authorized recipients.
Further modules follow the same plug-in pattern; no single module defines the
product. Recipient platforms, including moving vehicles, are cooperative
participants. The current MVP is a three-UAV delivery/docking simulator demo;
delivery is one demo module and may later be split out, like any other.

Technical names such as `vision_nav`, `poi`, and `AAS_DEL_*` do not define
the application's purpose. Preserve actual API names, measured outcomes and
historical evidence; approach accuracy or disarming does not establish physical
docking or cargo receipt. Older wording is not authority for a different purpose.

## Glossary

Use these terms in docs, code and UI; keep existing API/parameter names.

- **Swarm** — the product and its multi-vehicle coordination layer.
- **Fleet** — the set of vehicles connected in one session.
- **Mission module** — a plug-in mission function (delivery, survey, spraying,
  surveillance, fire suppression, ...) running on the swarm core.
- **POI** — the selected point the approach is flown to. Not "target",
  "destination" or "delivery reference".
- **Dock** — a general-purpose 360-degree net on a stationary or moving
  platform; one detector class, `dock`. Never "parking net", "park net" or
  "landing net".
- **Final approach** (engineering) = **parking navigation** (product) — the
  vision-based approach to the POI.
- **Default delivery hub (DDH)** — the delivery module's return hub.
- **Detector** — the real detector defaults to a ChArUco board detector; the
  YOLO face model is a bench-only opt-in (`--detector-backend yolo`).

## Pure Vision Approach Constraint

For vision-based final approach work, the navigation command path must not use
aircraft compass-derived yaw/heading. Compass yaw can carry large bias and
would contaminate navigation-quality isolation. Treat aircraft yaw/heading as
allowed only for logs, operator display, or certification/scoring diagnostics;
do not use it to build LOS for navigation, refresh predictions, compute
lateral/vertical commands, or decide final-approach completion state. Gimbal/camera yaw
readback is a different source: roll-yaw-pitch readback from the gimbal/camera
is allowed as camera-state only, so measured pixels can be converted into a
frame-local visual ray. Allowed command inputs remain frame-local vision
LOS/pixels/rates, gimbal/camera roll-yaw-pitch readback as camera-state only,
pitch/roll aircraft attitude, airspeed, wind, and previous command state, with
no dock geo/env truth, direct range, bbox-height, dock height, ground speed,
altitude-derived vertical state, or sim-truth dependency. Do not project
world-frame wind direction into the command frame for final-approach navigation
through hidden aircraft compass yaw.

`ideal_360` is a simulator-only sensor upper bound: it must keep the camera
static relative to the aircraft, use infinite FOV, and publish the resulting
frame-local pixel/LOS error. It must not slew, lock, or virtually point the
camera/gimbal at the POI.

The command path also must not use vehicle ground speed or any vertical
altitude/altitude-rate estimate. Ground-speed and altitude-derived vertical
state are unreliable for this isolation task and would hide whether the
vision-navigation law itself is correct. Use them only for logging,
certification/scoring, or operator display after the command is formed. The
law may use pitch/roll attitude and airspeed as an attitude-control proxy, but
must not read `ground_speed_ned`, `velocity`, `location()`, `altitude`,
`relative_altitude`, or equivalent altitude fields.

## Owner's shell

Commands given to the owner must work in Windows PowerShell 5.1: no `&&` or `||` (use `;`), and absolute paths.

## Running the GCS stack — hard rules for every chat

**Always launch through the isolated launcher — never on hardcoded ports
3000/8000. Never pass `--chat` (slots auto-resolve by launch directory).
Never broad-kill.**

```powershell
scripts\bat\AAS_GCS.bat               # launch app + SITL (claims this directory's slot)
python scripts\gcs_stop.py            # stop this directory's stack, release the slot
python scripts\gcs_launch.py --list   # see who's running (all sessions/clones)
python scripts\swarm_run.py --eval    # headless eval SITL (reserved band, no frontend)
python scripts\gcs_stop.py --all      # explicit opt-in: stop EVERY instance (the safe all-stop)
```

`gcs_stop.py` kills only your PIDs and reaps orphans on your slot's ports; never reuse a slot you don't own. Everything else (port scheme, bands, registry, UI expectations, launch pitfalls): `src/gcs/AGENTS.md`.

## Diagnose Cause, Not Symptom

Do not stop at the first visible failure. Walk the mechanism: which calls run, in what order, with which inputs, what state changes, what output appears, what should have happened. Fix at the smallest point where actual behavior diverges from intended.

Architecture talk is not mechanism understanding. Comments, log messages, names, and labels are hints, not truth — if a label contradicts the code or observed behavior, inspect producer and consumer before reasoning from it.

## Testing Policy

Runtime behavior changes include or update tests. Never delete, skip, weaken, or rewrite tests just to make a change pass; if intended behavior changed, explain why the test changed.

For safety-critical navigation, mission, or operator-facing behavior, prefer both a mechanism-level regression test and a simulation/log/integration check when feasible.

Run the smallest relevant targeted test first, then broader tests when shared behavior, interfaces, or safety-relevant paths changed:

```powershell
$env:PYTHONPATH="$PWD\src"
python -m pytest tests
```

```bash
PYTHONPATH="$PWD/src" .venv/Scripts/python.exe -m pytest tests/
```

## Design Rules

- Fix the root cause, not the symptom.
- Prefer the smallest correct change over broad rewrites.
- Keep modules focused and interfaces narrow; pass only the data a function or hook needs.
- Prefer explicit dependencies over hidden global state.
- Prefer dispatch tables or focused handlers over growing switch/if chains when new cases are expected.
- Do not create abstractions before ownership boundaries or repetition justify them; do not refactor solely to hit a line-count target.
- Keep top-level controllers/components thin; put logic in focused helpers, handlers, hooks, or services.
- No arbitrary thresholds, polling, sleeps, or special cases standing in for correct state/modeling. Domain-required thresholds must be named, centralized when shared, documented enough to explain intent, and covered by tests or verification.

## Definition of Done

A task is done only when every applicable item holds; otherwise report it as not done and say what is missing. The user's `/ship` walks this list, then commits, opens the PR, merges and archives the chat.

- **Outcome** — works end to end, fixed at the root cause; no symptom workarounds or partial paths.
- **Tests** — per the Testing Policy; report commands and results, and name pre-existing failures.
- **Verified where it runs** — GCS behavior via `live-tester`; frontend also `npm test` and `npm run build` (`src/gcs/AGENTS.md`); `scripts/lua/*.lua` redeployed (CRLF stripped) to WSL `~/ardupilot/{1,2,3}/scripts` before claiming SITL results, since SITL doesn't load Lua from the repo.
- **Evidence stated** — the level verified (unit, SITL/log, live app, hardware); simulation never proves physical docking or cargo receipt.
- **Rules hold** — Pure Vision Approach Constraint, glossary terms, GCS launch rules.
- **In sync** — generated files regenerated (see Agent Definitions); docs updated when commands, behavior or terms change.
- **Scoped diff** — only the task's change; out-of-scope bugs become separate tasks.
- **Cleanup** — remove debug code, scratch files and code the change made obsolete (no shims); keep `.logs/`. Stop what you started (GCS stack via `gcs_stop.py`, `--eval` for eval SITL, dev servers, shells) unless the user wants it running. Touch only your own work. `/cleanup` does this and sweeps what finished chats leave.

## Agent Definitions

`.claude/agents/*.md` is the single source of truth. `.codex/agents/*.toml` is generated — never edit it by hand. After changing an agent's Markdown, run:

```bash
python scripts/gen_codex_agents.py
```

and commit both sides. `tests/scripts/test_gen_codex_agents.py` fails the suite if they drift. `janitor` and `live-tester` are deliberately Claude-only (see `CODEX_EXCLUDE` in the script).