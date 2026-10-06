"""Legacy detector marker.

Production consumers depend on the structural capability protocols in
``detector_ports``.  This class remains only so external code that used
``isinstance(detector, DetectorAbc)`` does not break during the migration.
It deliberately owns no behavior or defaults.
"""

from abc import ABC


class DetectorAbc(ABC):
    """Behaviorless compatibility aggregate; do not type new consumers with it."""


__all__ = ["DetectorAbc"]
