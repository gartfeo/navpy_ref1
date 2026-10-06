"""Navigation-law selection and algorithm-owned runtime composition."""

from __future__ import annotations

import threading
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, Iterator, Mapping, Optional

from navpy.args.navigation_args import NavigationAlgorithm, NavigationArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.navigation.navigation_runtime import NavigationRuntime
from navpy.modules.navigation.nav.nav_law_factory import (
    NavigationLawLifecycle,
    NavAlgorithmSpec,
    get_nav_algorithm_spec,
)
from navpy.modules.navigation.nav.vision_nav.runtime_api import (
    VisionNavConfirmation,
    VisionNavStatus,
)


@dataclass(frozen=True)
class NavigationTerminalCapabilities:
    """Immutable references to the narrow terminal operation groups."""

    status: VisionNavStatus
    confirmation: VisionNavConfirmation


@dataclass(frozen=True)
class RuntimeBuildResult:
    runtime: NavigationRuntime
    terminal: Optional[NavigationTerminalCapabilities] = None


@dataclass(frozen=True)
class ActiveNavigationMode:
    spec: NavAlgorithmSpec
    runtime: NavigationRuntime
    terminal: Optional[NavigationTerminalCapabilities]


class NavigationModeState:
    """Synchronize access to the currently selected algorithm bundle."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._active: Optional[ActiveNavigationMode] = None

    def snapshot(self) -> ActiveNavigationMode:
        with self._lock:
            return self._active_locked()

    @contextmanager
    def session(self) -> Iterator[ActiveNavigationMode]:
        """Keep one selected mode stable for a complete runtime operation."""
        with self._lock:
            yield self._active_locked()

    @contextmanager
    def runtime_session(self) -> Iterator[NavigationRuntime]:
        """Expose only the selected runtime while retaining the mode fence."""
        with self.session() as active:
            yield active.runtime

    def install_initialized(
        self,
        active: ActiveNavigationMode,
        *,
        expected_previous: Optional[ActiveNavigationMode],
    ) -> ActiveNavigationMode:
        """Fence old work, initialize a candidate, then publish it atomically."""
        with self._lock:
            if self._active is not expected_previous:
                raise RuntimeError("navigation mode changed during mode construction")
            if expected_previous is not None:
                expected_previous.runtime.invalidate_commands()
            self._active = active
            return active

    def _active_locked(self) -> ActiveNavigationMode:
        if self._active is None:
            raise RuntimeError("navigation mode has not been initialized")
        return self._active


RuntimeFactory = Callable[[NavigationLawLifecycle], RuntimeBuildResult]


@dataclass(frozen=True)
class NavigationModeBuilder:
    spec: NavAlgorithmSpec
    build_law: Callable[[], NavigationLawLifecycle]
    runtime_factory: RuntimeFactory

    def build(self) -> ActiveNavigationMode:
        nav = self.build_law()
        built = self.runtime_factory(nav)
        return ActiveNavigationMode(
            spec=self.spec,
            runtime=built.runtime,
            terminal=built.terminal,
        )


class NavigationModeSelector:
    """Select a configured mode without giving factories the Navigation root."""

    def __init__(
        self,
        args: NavigationArgs,
        builders: Mapping[NavigationAlgorithm, NavigationModeBuilder],
        state: NavigationModeState,
        logger: ILogger,
    ) -> None:
        self._args = args
        self._builders = dict(builders)
        self._state = state
        self._logger = logger

    def _selected_spec(self) -> NavAlgorithmSpec:
        configured = getattr(self._args, "navigation_algorithm", None)
        if configured is None:
            configured = getattr(
                self._args,
                "pitch_controller",
                NavigationAlgorithm.PN.value,
            )
        return get_nav_algorithm_spec(configured)

    def refresh(
        self,
        *,
        force: bool = False,
    ) -> ActiveNavigationMode:
        spec = self._selected_spec()
        previous: Optional[ActiveNavigationMode]
        try:
            previous = self._state.snapshot()
        except RuntimeError:
            previous = None
        if (
            not force
            and previous is not None
            and previous.spec.algorithm is spec.algorithm
        ):
            return previous

        active = self._builders[spec.algorithm].build()
        active = self._state.install_initialized(
            active,
            expected_previous=previous,
        )
        self._logger.info(spec.log_message)
        return active

    def initialize(self) -> ActiveNavigationMode:
        self._args.refresh()
        return self.refresh()

    @property
    def algorithm_info(self) -> tuple[str, Optional[float]]:
        spec = self._state.snapshot().spec
        return spec.log_name, spec.kp_provider(self._args)


__all__ = [
    "ActiveNavigationMode",
    "NavigationModeBuilder",
    "NavigationModeSelector",
    "NavigationModeState",
    "NavigationTerminalCapabilities",
    "RuntimeBuildResult",
]
