"""How NAV left an armed dock episode (gcs_sim_closest_approach.md, todo 108)."""

from __future__ import annotations

from navpy.core.approach.dock_episode_end import CANCELLED_REASON, dock_episode_end
from navpy.core.approach.vision_nav.episode_latch import (
    UNNAMED_FAILURE_REASON,
    FinalApproachEpisodeTerminal,
    FinalApproachTerminal,
)
from navpy.core.ports.dock_contact import DockContactIdentity, DockEpisodeEnd

ARMED = DockContactIdentity(vehicle=1, task_id=3, dock_uid=0, episode=2)


def _terminal(kind, reason=None, identity=ARMED):
    return FinalApproachEpisodeTerminal(identity, kind, reason)


def test_docked_wins_over_everything():
    failed = _terminal(FinalApproachTerminal.FAILED, "source_command_stale")
    assert dock_episode_end(ARMED, True, failed, True) == DockEpisodeEnd(docked=True)


def test_a_latched_miss_is_missed():
    missed = _terminal(FinalApproachTerminal.MISSED, "missed_dock")
    assert dock_episode_end(ARMED, False, missed, False) == DockEpisodeEnd(missed=True)


def test_a_nav_fail_is_navs_own_failure_with_its_reason():
    failed = _terminal(FinalApproachTerminal.FAILED, "source_command_stale")
    assert dock_episode_end(ARMED, False, failed, False) == DockEpisodeEnd(
        failed=True, abort_reason="source_command_stale"
    )
    unnamed = dock_episode_end(ARMED, False, _terminal(FinalApproachTerminal.FAILED), False)
    assert (unnamed.failed, unnamed.abort_reason) == (True, UNNAMED_FAILURE_REASON)


def test_a_cancel_aborts_as_cancelled():
    assert dock_episode_end(ARMED, False, None, True) == DockEpisodeEnd(
        abort_reason=CANCELLED_REASON
    )


def test_another_episodes_terminal_is_ignored():
    other = DockContactIdentity(vehicle=1, task_id=3, dock_uid=0, episode=1)
    failed = _terminal(FinalApproachTerminal.FAILED, "source_command_stale", other)
    assert dock_episode_end(ARMED, False, failed, False) == DockEpisodeEnd()


def test_no_terminal_and_no_cancel_leaves_the_reason_unknown():
    assert dock_episode_end(ARMED, False, None, False) == DockEpisodeEnd()
