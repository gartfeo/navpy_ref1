"""Own launch session state, task lifetime, serialization and broadcasts."""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Callable

from gcs.backend.broadcast import manager as ws_manager
from gcs.backend import launch_execution, launch_observation, launch_session
from gcs.backend.launch_state import (
    VehicleState, UAVS_PER_CONTAINER, _container_map, _ALREADY_AIRBORNE_MIN_ALT_M,
)

log = logging.getLogger(__name__)


class LaunchController:
    """Own the shared state for a prepared launch and all of its tasks."""

    def __init__(self):
        self._states: dict[int, VehicleState] = {}
        self._errors: dict[int, str] = {}
        self._altitudes: dict[int, float] = {}
        self._armed: dict[int, bool] = {}
        self._climb_rates: dict[int, float] = {}
        self._throttles: dict[int, float] = {}  # commanded throttle %, VFR_HUD
        # monotonic time since climb first met the threshold (None = not climbing)
        self._climb_ok_since: dict[int, float] = {}
        self._abort_event = asyncio.Event()
        self._abort_notified = False
        self._launch_lock = asyncio.Lock()
        self._tasks: dict[int, asyncio.Task] = {}
        # Session parameters (set by prepare)
        self._prepared = False
        self._channel_map: dict[int, int] = {}
        self._esp32_client = None
        self._altitude_threshold: float = 10.0
        self._arm_timeout: float = 5.0
        self._altitude_timeout: float = 30.0
        self._settle_s: float = 0.3       # AUTO->ARM settle pause
        self._stagger_s: float = 0.0      # delay between consecutive launches
        self._container_gap_s: float = 0.0  # extra delay at container boundaries
        self._container_of: dict[int, int] = {}  # sys_id -> container idx (by sys_id)
        self._require_armed: bool = False  # require armed to mark airborne
        self._require_throttle: bool = False  # require commanded throttle to mark airborne
        self._min_throttle_pct: float = 20.0  # min commanded throttle % when required
        self._min_climb_rate_ms: float = 0.0  # 0 = climb confirmation disabled
        self._climb_confirm_s: float = 0.0    # sustained-climb duration required
        self._arm_func: Callable[[int], None] | None = None
        self._auto_func: Callable[[int], None] | None = None

    @property
    def is_running(self) -> bool:
        return any(not t.done() for t in self._tasks.values())

    @property
    def is_prepared(self) -> bool:
        return self._prepared

    def get_states(self) -> dict[int, dict]:
        """Return per-vehicle launch states for WS broadcast."""
        return {
            sid: {"state": self._states[sid].value, "error": self._errors.get(sid)}
            for sid in self._states
        }

    def update_telemetry(self, sys_id: int, alt_rel: float, armed: bool,
                         climb: float = 0.0, throttle: float = 0.0):
        return launch_observation.update_telemetry(self, sys_id, alt_rel, armed, climb, throttle)

    async def prepare(
        self,
        sys_ids: list[int],
        channel_map: dict[int, int],
        esp32_host: str,
        esp32_port: int,
        altitude_threshold: float,
        arm_timeout: float,
        altitude_timeout: float,
        arm_func: Callable[[int], None],
        auto_func: Callable[[int], None],
        settle_s: float = 0.3,
        stagger_s: float = 0.0,
        container_gap_s: float = 0.0,
        uavs_per_container: int = 0,
        require_armed: bool = False,
        require_throttle: bool = False,
        min_throttle_pct: float = 20.0,
        min_climb_rate_ms: float = 0.0,
        climb_confirm_s: float = 0.0,
    ):
        await launch_session.prepare(
            self,
            sys_ids=sys_ids,
            channel_map=channel_map,
            esp32_host=esp32_host,
            esp32_port=esp32_port,
            altitude_threshold=altitude_threshold,
            arm_timeout=arm_timeout,
            altitude_timeout=altitude_timeout,
            arm_func=arm_func,
            auto_func=auto_func,
            settle_s=settle_s,
            stagger_s=stagger_s,
            container_gap_s=container_gap_s,
            uavs_per_container=uavs_per_container,
            require_armed=require_armed,
            require_throttle=require_throttle,
            min_throttle_pct=min_throttle_pct,
            min_climb_rate_ms=min_climb_rate_ms,
            climb_confirm_s=climb_confirm_s,
            broadcast=ws_manager.broadcast,
        )

    async def trigger_vehicle(self, sys_id: int):
        """Queue a single vehicle for launch.

        Returns immediately. The background task gates itself, waiting for
        any prior vehicle to reach a terminal state before proceeding.
        """
        if not self._prepared:
            raise RuntimeError("Launch controller not prepared")
        if self._abort_event.is_set():
            raise RuntimeError("Launch session aborted")
        if sys_id not in self._states:
            raise ValueError(f"Vehicle {sys_id} not in launch session")

        existing = self._tasks.get(sys_id)
        if existing and not existing.done():
            raise RuntimeError(f"Vehicle {sys_id} already launching")

        log.info("[launch] V%d queued for launch (state=%s)", sys_id, self._states[sys_id].value)
        self._tasks[sys_id] = asyncio.create_task(self._run_vehicle(sys_id))

    async def abort(self):
        """Signal abort and cancel the launch session."""
        log.warning("[launch] ABORT requested, signalling all tasks")
        self._abort_event.set()
        for task in self._tasks.values():
            if not task.done():
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._prepared = False
        if not self._abort_notified:
            await ws_manager.broadcast({"type": "launch_aborted"})
            self._abort_notified = True
        log.info("[launch] Launch sequence aborted")

    def cleanup(self):
        """Reset state after all vehicles are done or aborted."""
        self._states.clear()
        self._errors.clear()
        self._tasks.clear()
        self._climb_ok_since.clear()
        self._prepared = False
        self._esp32_client = None
        self._arm_func = None
        self._auto_func = None
        self._abort_event.clear()
        self._abort_notified = False
        # Replace the lock so a stale held lock doesn't block the next session
        self._launch_lock = asyncio.Lock()

    async def _set_state(self, sys_id: int, state: VehicleState, error: str | None = None):
        prev = self._states.get(sys_id)
        self._states[sys_id] = state
        if error:
            self._errors[sys_id] = error
        log.info("[launch] V%d state: %s -> %s%s",
                 sys_id,
                 prev.value if prev else "?",
                 state.value,
                 f" error={error}" if error else "")
        await ws_manager.broadcast({
            "type": "launch_progress",
            "sys_id": sys_id,
            "state": state.value,
            "error": error,
        })

    async def _abort_vehicle(self, sys_id: int):
        """Mark a vehicle as aborted. Does not emit launch_complete."""
        if self._states.get(sys_id) in (VehicleState.airborne, VehicleState.failed):
            return
        log.info("[launch] V%d aborted", sys_id)
        await self._set_state(sys_id, VehicleState.failed, "Aborted")

    async def _fail_session(self, sys_id: int, error: str):
        """Fail one vehicle and atomically stop every later launch.

        The frontend queues the whole fleet up front, so marking only the
        current vehicle failed is not enough: queued tasks would otherwise
        acquire the launch lock and trigger their ESP32 channels.  Set the shared
        abort signal while the current task still owns the lock, then make all
        non-terminal peers terminal before broadcasting the session abort.
        """
        log.error("[launch] V%d FAILED — aborting launch session: %s", sys_id, error)
        self._abort_event.set()
        await self._set_state(sys_id, VehicleState.failed, error)
        for sid, state in list(self._states.items()):
            if sid == sys_id or state in (VehicleState.airborne, VehicleState.failed):
                continue
            await self._set_state(
                sid,
                VehicleState.failed,
                f"Aborted after V{sys_id} failed",
            )
        if not self._abort_notified:
            await ws_manager.broadcast({
                "type": "launch_aborted",
                "failed_sys_id": sys_id,
                "error": error,
            })
            self._abort_notified = True

    def _finish_aborted_session_if_quiescent(self):
        """Allow a new prepare only after every task from this session stops."""
        if not self._abort_event.is_set():
            return
        current = asyncio.current_task()
        if all(task is current or task.done() for task in self._tasks.values()):
            self._prepared = False

    async def _run_vehicle(self, sys_id: int):
        return await launch_execution._run_vehicle(self, sys_id)

    async def _run_vehicle_locked(self, sys_id: int):
        return await launch_execution._run_vehicle_locked(self, sys_id)

    async def _check_all_done(self):
        """Broadcast launch_complete when ALL vehicles in the session are terminal."""
        terminal = {VehicleState.airborne, VehicleState.failed}
        status_str = ", ".join(f"V{sid}={st.value}" for sid, st in self._states.items())
        log.info("[launch] _check_all_done: %s", status_str)
        if self._states and all(s in terminal for s in self._states.values()):
            await ws_manager.broadcast({"type": "launch_complete"})
            log.info("[launch] ALL DONE — launch sequence complete")

    def _is_airborne(self, sys_id: int) -> bool:
        return launch_observation._is_airborne(self, sys_id)

    def _has_pending_vehicle(self) -> bool:
        return launch_observation._has_pending_vehicle(self)

    def _post_vehicle_delay(self, sys_id: int) -> float:
        return launch_observation._post_vehicle_delay(self, sys_id)

    async def _sleep_or_abort(self, seconds: float) -> None:
        return await launch_execution._sleep_or_abort(self, seconds)

    async def _poll(self, condition, timeout: float) -> bool | None:
        return await launch_execution._poll(self, condition, timeout)


launch_controller = LaunchController()
