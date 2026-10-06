"""Per-vehicle full parameter snapshot returned by `VehicleMav.fetch_full_param_snapshot`."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

from navpy.modules.vehicle._mavftp.param_pck import ParamPck, ParamRecord


@dataclass(frozen=True)
class FullParamSnapshot:
    """One parsed `@PARAM/param.pck` blob plus identifying metadata.

    `by_name` is a name-keyed view computed once at construction so callers
    that only need lookup (most of them) don't re-walk `pck.records`.
    """
    target_system: int
    fetched_at_unix_s: float
    with_defaults: bool
    pck: ParamPck
    by_name: Mapping[str, ParamRecord] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        # Frozen dataclass — must use object.__setattr__.
        object.__setattr__(
            self,
            "by_name",
            {r.name: r for r in self.pck.records},
        )

    @property
    def num_params(self) -> int:
        return self.pck.num_params

    @property
    def total_params(self) -> int:
        return self.pck.total_params
