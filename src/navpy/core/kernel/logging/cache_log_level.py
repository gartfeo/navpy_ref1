from enum import Enum


class CacheLogLevel(Enum):
    VERBOSE = -1
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3

    @classmethod
    def from_name(cls, name: "str | CacheLogLevel") -> "CacheLogLevel":
        """Resolve a case-insensitive member NAME (e.g. ``'debug'``) to a level.

        Used as the argparse converter for the log-level flags. argparse hands
        the converter the raw CLI *string*, so the default ``CacheLogLevel(x)``
        lookup — which matches on the integer member *values* (-1..3) — can
        never succeed for a name like ``'DEBUG'`` or the string ``'1'``. Resolve
        by member name instead.

        An existing ``CacheLogLevel`` is returned unchanged so the converter is
        idempotent. Anything else raises ``ValueError`` listing the valid names.
        """
        if isinstance(name, cls):
            return name
        try:
            return cls[str(name).strip().upper()]
        except KeyError:
            valid = ", ".join(level.name for level in cls)
            raise ValueError(
                f"invalid log level {name!r}; choose from {valid}"
            ) from None

    def __lt__(self, other):
        if isinstance(other, CacheLogLevel):
            return self.value < other.value
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, CacheLogLevel):
            return self.value <= other.value
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, CacheLogLevel):
            return  self.value > other.value
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, CacheLogLevel):
            return self.value >= other.value
        return NotImplemented
