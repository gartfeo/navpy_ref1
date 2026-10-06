"""Post-fence NAV SNAP diagnostics."""

from __future__ import annotations

from collections.abc import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger


class NavSnapReporter:
    """Render one navigation summary after command ownership is fenced."""

    def __init__(
        self,
        algorithm_info: Callable[[], tuple[str, object]],
        logger: ILogger,
    ) -> None:
        self._algorithm_info = algorithm_info
        self._logger = logger

    def report(self, snap: object) -> None:
        algorithm, kp = self._algorithm_info()
        kp_suffix = f"; kp={kp:.2f}" if kp is not None else ""
        self._logger.info(
            f"\nSNAP({algorithm.upper()}): {snap}{kp_suffix}",
            key="nav",
            dest=LogStatusDest.DRONE,
            status=f"SNAP({algorithm.upper()}): {snap.status()}{kp_suffix}",
        )


__all__ = ["NavSnapReporter"]
