# Final approach: docking is physical, not a vision decision (v4, 2026-10-13, owner-approved)

Replaces the v1 terminal-divergence rule ("rework"). Codex resolved all 5 v2 findings in v3; v4 adds its 2 v3 findings
(the thread that requests RESET, and an episode-level latch). Fixes
`architecture_todo.md` items 35 and 36. Owner approved v4 and the 0.5 m net radius. Nothing is built.

## Owner ruling (2026-10-13)
- "In our simulation we can use GPS to check if dock happened."
- "In real, if we don't lose the dock from sight and continuously approach, it will finally physically dock."

## Rules
1. **The picture never declares success.** While the dock is in view, the approach keeps closing.
   - The centred-loss pass (a7) and its zoom guard are removed.
   - `VisualPassDetector` keeps its arming and aft-frame gating, but an aft dock now means a miss: commands stop and
     the approach ends as `NAV_FAIL: missed_dock`. It is never a pass. This keeps today's aft-command suppression,
     which matters for `ideal_360`, where an aft dock stays visible.
2. **Real flight:** the net catches the aircraft, so no software decision is made in the approach. A capture signal for
   the autopilot is later work.
3. **Simulation: a contact sensor stands in for the net.** It is the only success source.

## Sim contact sensor
- **Where it lives:** `src/navpy/sim/world`, next to `FleetDockWorld` / `PoiProvider`. Only the sim composition wires it,
  and only for a sim autopilot. It exposes one correlated event to the approach lifecycle and nothing else. No position
  or range reaches frames, the law or the command path; a guard test pins that.
- **Input:**
  - the aircraft position from vehicle telemetry, sampled on every position update, independent of detector
    visibility or the source mode;
  - the true dock position (stationary docks only for now; moving docks need relative motion later);
  - both at absolute altitude (AMSL), with the dock's height from `PoiAltitude`.
- **Identity and episodes:**
  - It arms at approach start with (vehicle, task, dock uid, approach episode).
  - It resets on a new assignment, an approach restart, a world reload or the aircraft clock going back.
  - Samples must have increasing aircraft time. It never joins two samples across a gap or a reset into a crossing that
    did not happen.
- **Contact:** the closest point of the segment between consecutive samples (as `ClosestApproachTracker` does) is
  strictly under `SIM_DOCK_NET_RADIUS_M` (0.5 m, owner-set; matches the eval's strict `< 0.5 m` SNAP gate).
- **Event:** `DOCKED` with (vehicle, task, dock uid, episode, aircraft time, closest 3D/h/v distance), published once.

## Terminal transaction
The existing a7 latch machinery is kept and re-pointed from the vision trigger to the contact trigger:
1. Contact.
2. Under the runtime lock: latch completion (first latch wins), retire held and queued commands, stage the `DOCKED`
   evidence row behind earlier command rows (the ordered pass-evidence path).
3. Set `final_approach_nav_completed` and request RESET.

- **Threads.** The position callback does steps 1-2 only: it latches and fences at once. Step 3 runs on the navigation
  loop thread, which picks up the latched completion in its next decision. It never runs from the callback, so a
  contact cannot change the phase in the middle of a transition (`navigation_loop.py:88`, `nav_state.py:106`).
- **The latch belongs to the episode.** It lives at the episode level, outside `VisualPassState`, so
  `invalidate_commands()` (`command_reset.py:80`) and a continuity-key change (`visual_pass.py:94`) cannot erase it.
  Only RESET or the next episode clears it.

A detection loss or `NAV_FAIL` that arrives after the latch is ignored. A contact that arrives after a `NAV_FAIL` latch
is ignored. Zero commands are actuated after the latch.

**After docking: the existing completion lifecycle, unchanged.**
- One-shot mode force-disarms SITL (`nav_oneshot_completion.py:58`), which matches being caught by the net.
- Normal mode follows today's RESET flow.
- No new LOITER hold, so no mode-acceptance race with `_await_guided()`.

## Evidence
- The `DOCKED` row carries the sensor's closest distance. It includes the crossing segment, so it agrees with the
  contact decision. The SNAP row stays as a diagnostic.
- The eval counts docking only from `DOCKED`. It rejects:
  - a duplicate, stale or wrong-episode event;
  - an event that does not match the UAV's own dock;
  - any command row after the event.
- `FINAL_APPROACH_PASS` and its consumers (the parser, metrics, workflow markers, standalone evaluators) are switched
  to `DOCKED` or removed.

## Steps
| Step | Change | Gate |
|---|---|---|
| d1 | The sim contact sensor (world plus composition) | A segment within the radius docks once; just outside does not; another UAV's dock never docks; stale samples, gaps and resets never create a crossing; the real build has no sensor; the Pure Vision guard |
| d2 | Re-point the latch to `DOCKED`; remove the centred-loss pass and zoom guard; an aft dock becomes `NAV_FAIL: missed_dock` | Races: contact vs loss, contact vs `NAV_FAIL`, contact during a navigation transition, and contact → source reset → late frame (still zero commands); zero commands after the latch; aft and visible-miss regressions including `ideal_360`; one-shot and normal lifecycle; the full suite |
| d3 | Eval and log consumers move to `DOCKED` | Script tests for the rejection cases |
| d4 | Two 3-UAV SITL runs | All three UAVs dock in both runs, with no command after docking. Any failure gets a stated cause |

