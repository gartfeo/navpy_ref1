# Peer camera tracking (design v1)

## Problem
A peer UAV told about a dock flies to it and circles at 500 m to confirm it.
Its gimbal stays forward (−14°), the dock is ~90° off the nose, so it is
never seen (3-UAV SITL dock-e, UAV 2).

## Root cause
Peer geo pointing (gimbal aimed at the dock's reported position until it is
detected) exists, but is gated on `final_approach.is_active`. That flag means
"the vision law is configured" (`navigation_final_approach.py:19-21`), true
from startup, not "the final approach is running". So under the vision law
the peer never aims its camera at the dock. Same mistake in the post-loss
coordinate fallback.

Gates:
- `core/skills/acquire/peer_approach_start.py:128` (arm at peer setup)
- `core/skills/acquire/peer_geo_tick.py:44` (DETECT tick)
- `core/skills/confirm/track_recovery.py:241` (re-point after a loss in CONFIRM)

## Fix
Replace the config flag with the phase: geo pointing is allowed until NAV
(final approach) starts, never during it.
- Each gate checks `phase.is_current(NavState.NAV)` instead of
  `final_approach.is_active`.
  The composition owners inject `lambda: state.phase.is_current(NavState.NAV)`.
- Hand-off: detection calls `start_tracking`, which clears the geo session of
  the tracking camera only (`gimbal_detection_lifecycle.py:41-59`); with
  several cameras the others stay geo-armed (`geo_pointing_fleet.py:43`,
  `tracking_command_router.py:64`). So after a successful `start_tracking`
  the hand-off also calls the fleet's `stop_geo_tracking()`. A member whose
  geo session is already clear returns early (`gimbal_geo_lifecycle.py:86-92`),
  so the tracking camera is not moved; the others go back to neutral.
- Fence: `NavEntry.run` stops all geo sessions before GUIDED and fails the
  entry if `is_geo_armed` is still true. NAV exit to DETECT/CONFIRM needs
  nothing extra: no session survives NAV entry and none can be armed in NAV.
- Owner ruling 2026-10-07: GPS is allowed outside the approach. Geo pointing
  only moves the gimbal; it never feeds the command path or the final-approach
  inputs. `ideal_360` has no rate tracker, so geo start stays a no-op there.

## Tests
- Rewrite `test_final_approach_peer_navigation_never_arms_geo_pointing`
  (`tests/modules/nav/test_nav_controller.py:8993`): under the vision law the
  peer arms geo pointing at setup; detection clears it; in NAV it is never
  armed or updated. It pinned the config flag where the rule means the phase;
  the real guarantee (no geo pointing in the final approach) stays pinned.
- New mechanism tests: `PeerGeoTick` and the track-recovery fallback under the
  vision law, before NAV (acts) and in NAV (does not).
- Multi-camera: detection on one source clears geo on all, without moving
  the tracking camera; NAV entry with a leftover geo session stops it, and
  fails if it cannot; RESET → DETECT → NAV re-entry has no geo session.
- Keep `test_manual_review_poison_proves_final_approach_path_never_uses_poi_geo`
  unchanged.
- SITL: d4, two 3-UAV runs where all three dock, including a peer that starts
  inside the circle radius.

## Watch (not in scope unless SITL shows it)
- NAV entry resets zoom to minimum (`nav_entry.py:41-45`); at 500 m the dock
  is ~5-7 px. Own UAVs docked in dock-e, so it may not bite.
- Sim places the dock exactly at the reported position, so position error is
  not exercised.

## Result (2026-10-13)
SITL `.logs/peer-cam-1`, `-2`: peers detect their dock; hand-off and the NAV fence held. d4 not met: remaining failures predate this change (todo 31, 41, 42).
