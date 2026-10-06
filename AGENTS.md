# AGENTS.md

## Project purpose — read before interpreting old context

NavPy develops cooperative UAV swarm technology: autonomous navigation, fleet
coordination and vision-based final approach for multi-UAV missions. The swarm
core is the product; mission functions are plug-and-play modules on top of it.
Example modules: payload delivery to authorized recipients and fire
detection/suppression (e.g. finding and stopping fire spots in forests);
further module scope is yet to be verified. Recipient platforms,
including moving vehicles, are cooperative participants.
For delivery, parking nets are general purpose 360 degree nets; parking nets may
be on stationary or moving platforms.
The current MVP is a three-UAV delivery/docking simulator demo.

Parking navigation names the vision-based final approach.
Technical names such as `vision_nav`, `poi`, and `AAS_DEL_*` do not define
the application's purpose. Preserve actual API names, measured outcomes and
historical evidence; approach accuracy or disarming does not establish physical
docking or cargo receipt. Older wording is not authority for a different purpose.

## Pure Vision Approach Constraint

For vision-based final approach work, the navigation command path must not use
aircraft compass-derived yaw/heading. Compass yaw can carry large bias and
would contaminate navigation-quality isolation. Treat aircraft yaw/heading as
allowed only for logs, operator display, or certification/scoring diagnostics;
do not use it to build LOS for navigation, refresh predictions, compute
lateral/vertical commands, or decide delivering/delivered state. Gimbal/camera yaw
readback is a different source: roll-yaw-pitch readback from the gimbal/camera
is allowed as camera-state only, so measured pixels can be converted into a
frame-local visual ray. Allowed command inputs remain frame-local vision
LOS/pixels/rates, gimbal/camera roll-yaw-pitch readback as camera-state only,
pitch/roll aircraft attitude, airspeed, wind, and previous command state, with
no park net geo/env truth, direct range, bbox-height, park net height, ground speed,
altitude-derived vertical state, or sim-truth dependency. Do not project
world-frame wind direction into the command frame for final-approach navigation
through hidden aircraft compass yaw; 

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

For safety-critical navigation, navigation, mission, or operator-facing behavior, prefer both a mechanism-level regression test and a simulation/log/integration check when feasible.

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

## Agent Definitions

`.claude/agents/*.md` is the single source of truth. `.codex/agents/*.toml` is generated — never edit it by hand. After changing an agent's Markdown, run:

```bash
python scripts/gen_codex_agents.py
```

and commit both sides. `tests/scripts/test_gen_codex_agents.py` fails the suite if they drift. `janitor` and `live-tester` are deliberately Claude-only (see `CODEX_EXCLUDE` in the script).