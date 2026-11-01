"""A participant's environment record: everything its spawn gives it.

The coordinator writes it as JSON into the child's environment
(:data:`ENVIRONMENT_VARIABLE`) and nowhere else, so the capability reaches
only that child. The coordinator address is the one host value in it; it is
never journaled.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any

from navpy.sim.lockstep.part_wire import Register, RequestId
from navpy.sim.lockstep.participant import ParticipantSpec

ENVIRONMENT_VARIABLE = "NAVPY_LOCKSTEP_PARTICIPANT"
#: The coordinator closed the link before this participant ended: its fault, never a membership change.
EXIT_COORDINATOR_GONE = 3
#: The child never got ``go``.
EXIT_NO_GO = 4
#: The child never reached the coordinator's listener.
EXIT_NO_CONNECT = 5
#: What a process says it left for a host reason, not the product's (``deterministic_dev_sim.md``, section 4).
HOST_CAUSED_EXITS = frozenset({EXIT_COORDINATOR_GONE, EXIT_NO_GO, EXIT_NO_CONNECT})


@dataclass(frozen=True)
class EnvironmentRecord:
    coordinator: tuple[str, int]
    capability: str
    role: str
    ordinal: int
    spawn_id: RequestId
    generation: int
    seed: int
    epoch_us: int
    #: The hooks factory, ``module:attribute``.
    hooks: str
    config: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, text: str) -> "EnvironmentRecord":
        raw = json.loads(text)
        raw["coordinator"] = (str(raw["coordinator"][0]), int(raw["coordinator"][1]))
        raw["spawn_id"] = (int(raw["spawn_id"][0]), int(raw["spawn_id"][1]))
        return cls(**raw)

    def spec(self) -> ParticipantSpec:
        return ParticipantSpec(self.seed, self.ordinal, self.generation, self.epoch_us)

    def register(self) -> Register:
        return Register(self.capability, self.role, self.ordinal, self.spawn_id, self.generation)


__all__ = ["ENVIRONMENT_VARIABLE", "EnvironmentRecord"]
