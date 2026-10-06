"""Runtime service for NavPy simulation subprocess lifecycle."""

from __future__ import annotations

import logging
from collections.abc import Iterable
from typing import Any

from gcs.backend.navpy_sim_configured_restart import (
    NAVPY_RESTART_READY_TIMEOUT_S,
    _configuration_restart_lock,
    _validate_runtime_expectation,
    restart_configured_instances,
)
from gcs.backend.navpy_sim_heartbeat import (
    HB_EXIT_WINDOW_S as _HB_EXIT_WINDOW_S,
    STALL_DIAGNOSIS_AFTER as _STALL_DIAGNOSIS_AFTER,
    track_heartbeat_exit_streak,
)
from gcs.backend.navpy_sim_launch_policy import (
    _ENV_LOG_LEVEL,
    detector_debug_enabled,
    effective_vision_profile,
    selected_vision_profile,
    sim_connection_from_settings,
    sim_log_level,
)
from gcs.backend.settings_model import GcsSettings
from gcs.backend.settings_store import settings_store


log = logging.getLogger(__name__)
_navpy_mgr: Any | None = None
_auto_managed_sysids: set[int] = set()
_hb_exit_streaks: dict[int, int] = {}


def get_or_create_mgr() -> Any:
    global _navpy_mgr
    with _configuration_restart_lock:
        if _navpy_mgr is None:
            from gcs.backend.navpy_process_manager import NavpyProcessManager

            _navpy_mgr = NavpyProcessManager()
        return _navpy_mgr


def has_manager() -> bool:
    return _navpy_mgr is not None


def get_all_status() -> list[dict[str, Any]]:
    manager = _navpy_mgr
    return [] if manager is None else manager.get_all_status()


def start_instance(
    sys_id: int,
    connection: str,
    settings: GcsSettings,
) -> Any:
    with _configuration_restart_lock:
        kwargs: dict[str, Any] = {
            "detector_debug_show": detector_debug_enabled(settings),
            "vision_profile": selected_vision_profile(settings),
        }
        level = sim_log_level()
        if level is not None:
            kwargs["log_level"] = level
        return get_or_create_mgr().start(sys_id, connection, **kwargs)


def managed_start_instance(
    sys_id: int,
    connection: str,
    settings: GcsSettings,
) -> Any:
    """Start a companion and mark it auto-managed on success/adoption."""
    with _configuration_restart_lock:
        try:
            inst = start_instance(sys_id, connection, settings)
        except ValueError:
            _auto_managed_sysids.add(sys_id)
            raise
        _auto_managed_sysids.add(sys_id)
        return inst


def clear_managed_intent(sys_id: int) -> None:
    with _configuration_restart_lock:
        _auto_managed_sysids.discard(sys_id)
        _hb_exit_streaks.pop(sys_id, None)


def stop_instance_process(sys_id: int) -> bool:
    with _configuration_restart_lock:
        if _navpy_mgr is None:
            return False
        return _navpy_mgr.stop(sys_id)


def stop_instance(sys_id: int) -> bool:
    with _configuration_restart_lock:
        clear_managed_intent(sys_id)
        return stop_instance_process(sys_id)


def stop_all_instances() -> None:
    with _configuration_restart_lock:
        _auto_managed_sysids.clear()
        _hb_exit_streaks.clear()
        if _navpy_mgr is not None:
            _navpy_mgr.stop_all()


def sim_connection(sys_id: int) -> str:
    return sim_connection_from_settings(sys_id, settings_store.get())


def auto_start_navpy_sim(sys_id: int) -> bool:
    with _configuration_restart_lock:
        settings = settings_store.get()
        if not settings.simulation.sim_mode:
            return False
        try:
            connection = sim_connection_from_settings(sys_id, settings)
            managed_start_instance(sys_id, connection, settings)
            log.info(
                "Auto-started NavPy sim for sys_id=%d on %s",
                sys_id,
                connection,
            )
            return True
        except ValueError:
            _auto_managed_sysids.add(sys_id)
            return False
        except Exception as error:
            log.warning(
                "Failed to auto-start NavPy sim for sys_id=%d: %s",
                sys_id,
                error,
            )
            return False


def ensure_auto_navpy_sim_running(
    active_sys_ids: Iterable[int] | None,
    settings: GcsSettings | None = None,
) -> list[int]:
    """Restart auto-managed companions whose subprocess generation exited."""
    if not _configuration_restart_lock.acquire(blocking=False):
        return []
    try:
        return _ensure_auto_navpy_sim_running(active_sys_ids, settings)
    finally:
        _configuration_restart_lock.release()


def _ensure_auto_navpy_sim_running(
    active_sys_ids: Iterable[int] | None,
    settings: GcsSettings | None,
) -> list[int]:
    if settings is None:
        settings = settings_store.get()
    if not settings.simulation.sim_mode:
        return []
    wanted = sorted(_auto_managed_sysids & set(active_sys_ids or []))
    if not wanted:
        return []
    restarted: list[int] = []
    manager = get_or_create_mgr()
    for sys_id in wanted:
        status = manager.get_status(sys_id)
        if status is not None and status.get("running") is True:
            if status.get("uptime_s", 0) > _HB_EXIT_WINDOW_S:
                _hb_exit_streaks.pop(sys_id, None)
            continue
        _track_heartbeat_exit_streak(sys_id, status)
        connection = (
            (status or {}).get("connection")
            or sim_connection_from_settings(sys_id, settings)
        )
        try:
            start_instance(sys_id, connection, settings)
            restarted.append(sys_id)
            log.info(
                "Restarted auto-managed NavPy sim for sys_id=%d on %s",
                sys_id,
                connection,
            )
        except ValueError:
            pass
        except Exception as error:
            log.warning(
                "Failed to restart NavPy sim for sys_id=%d: %s",
                sys_id,
                error,
            )
    return restarted


def _track_heartbeat_exit_streak(
    sys_id: int,
    status: dict[str, object] | None,
) -> None:
    with _configuration_restart_lock:
        track_heartbeat_exit_streak(sys_id, status, _hb_exit_streaks, log)


def restart_running_navpy_sim(
    settings: GcsSettings | None = None,
) -> list[int]:
    with _configuration_restart_lock:
        if _navpy_mgr is None:
            return []
        if settings is None:
            settings = settings_store.get()
        if not settings.simulation.sim_mode:
            return []
        restarted: list[int] = []
        for status in list(_navpy_mgr.get_all_status()):
            if status.get("running") is False:
                continue
            sys_id = status["sys_id"]
            connection = (
                status.get("connection")
                or sim_connection_from_settings(sys_id, settings)
            )
            try:
                if not _navpy_mgr.stop(sys_id):
                    log.warning(
                        "Skipped NavPy restart for sys_id=%d: instance not running",
                        sys_id,
                    )
                    continue
                start_instance(sys_id, connection, settings)
                restarted.append(sys_id)
            except Exception as error:
                log.warning(
                    "Failed to restart NavPy sim for sys_id=%d: %s",
                    sys_id,
                    error,
                )
        return restarted


def restart_configured_navpy_sim(
    expected_by_sys_id: dict[int, dict[str, int]],
    settings: GcsSettings | None = None,
    timeout_s: float = NAVPY_RESTART_READY_TIMEOUT_S,
) -> list[dict[str, object]]:
    """Restart only requested companions and validate fresh hydrated runtimes."""
    with _configuration_restart_lock:
        if not expected_by_sys_id:
            raise ValueError("At least one NavPy sys_id is required")
        if _navpy_mgr is None:
            raise ValueError("NavPy process manager is not running")
        if settings is None:
            settings = settings_store.get()
        if not settings.simulation.sim_mode:
            raise ValueError("NavPy sim only available in sim mode")
        return restart_configured_instances(
            expected_by_sys_id,
            manager=_navpy_mgr,
            settings=settings,
            managed_sysids=_auto_managed_sysids,
            start_instance=start_instance,
            connection_for=sim_connection_from_settings,
            timeout_s=timeout_s,
        )


def stop_all_navpy_if_running() -> None:
    global _navpy_mgr
    with _configuration_restart_lock:
        _auto_managed_sysids.clear()
        _hb_exit_streaks.clear()
        if _navpy_mgr is not None:
            _navpy_mgr.stop_all()
            _navpy_mgr = None
