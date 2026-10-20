"""Field checks for the HITL tools' private JSON records: a check appends "<path> <reason>" to a problem list,
so one pass reports every missing, unknown or malformed field, and RecordError carries them all."""
from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Callable, Mapping
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

UTC_FORMAT = "%Y-%m-%dT%H:%M:%SZ"
NAME_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
# Names Windows keeps for devices, with any extension (learn.microsoft.com/windows/win32/fileio/naming-a-file).
WINDOWS_DEVICE_NAMES = frozenset({"CON", "PRN", "AUX", "NUL",
                                  *(f"{port}{n}" for port in ("COM", "LPT") for n in range(10))})
HASH_CHUNK = 1 << 20

Check = Callable[[Any, str, list], None]


class RecordError(ValueError):
    """A refused record; problems lists every reason."""

    def __init__(self, problems: list[str]) -> None:
        super().__init__("refused: " + "; ".join(problems))
        self.problems = tuple(problems)


def is_number(value: Any) -> bool:
    """A finite int or float, not a bool."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:       # an int beyond the float range
        return False


def rule(test: Callable[[Any], bool], reason: str) -> Check:
    def check(value: Any, path: str, problems: list) -> None:
        if not test(value):
            problems.append(f"{path} {reason}")
    return check


def _utc_time(value: Any) -> bool:
    try:
        datetime.strptime(value, UTC_FORMAT)
    except (TypeError, ValueError):
        return False
    return True


def _folder_name(value: Any) -> bool:
    """Usable as a folder name on every host: Windows drops a final '.' and opens a device for its names."""
    return (isinstance(value, str) and NAME_PATTERN.fullmatch(value) is not None and not value.endswith(".")
            and value.split(".")[0].upper() not in WINDOWS_DEVICE_NAMES)


text = rule(lambda v: isinstance(v, str) and bool(v.strip()), "must be non-empty text")
folder_name = rule(_folder_name, "must be a name of letters, digits, '.', '_' or '-', not ending in '.' and not "
                                 "a Windows device name")
finite = rule(is_number, "must be a finite number")
positive = rule(lambda v: is_number(v) and v > 0, "must be a number above 0")
non_negative = rule(lambda v: is_number(v) and v >= 0, "must be a number of 0 or more")
index = rule(lambda v: isinstance(v, int) and not isinstance(v, bool) and v >= 0, "must be a whole number >= 0")
utc = rule(_utc_time, f"must be a UTC time {UTC_FORMAT}")


def hex_digits(digits: int) -> Check:
    return rule(lambda v: isinstance(v, str) and re.fullmatch(f"[0-9a-f]{{{digits}}}", v) is not None,
                f"must be {digits} lowercase hex digits")


def equals(expected: Any) -> Check:
    return rule(lambda v: type(v) is type(expected) and v == expected, f"must be {expected!r}")


def choice(options: tuple) -> Check:
    return rule(lambda v: v in options, f"must be one of {', '.join(options)}")


def obj(spec: Mapping[str, Check]) -> Check:
    """An object with exactly spec's fields."""
    def check(value: Any, path: str, problems: list) -> None:
        if not isinstance(value, Mapping):
            problems.append(f"{path} must be an object")
            return
        problems.extend(f"{path}.{key} is missing" for key in sorted(spec.keys() - value.keys()))
        problems.extend(f"{path}.{key} is not a known field" for key in sorted(value.keys() - spec.keys()))
        for key, field_check in spec.items():
            if key in value:
                field_check(value[key], f"{path}.{key}", problems)
    return check


def map_of(value_check: Check, required: tuple = (), keys: Optional[tuple] = None) -> Check:
    """A non-empty object of named entries, each checked by value_check; keys limits the names."""
    def check(value: Any, path: str, problems: list) -> None:
        if not isinstance(value, Mapping) or not value:
            problems.append(f"{path} must be a non-empty object")
            return
        problems.extend(f"{path}.{key} is missing" for key in required if key not in value)
        for key, item in value.items():
            if not isinstance(key, str) or not key.strip() or (keys is not None and key not in keys):
                problems.append(f"{path} has an unknown entry {key!r}")
            else:
                value_check(item, f"{path}.{key}", problems)
    return check


def require(data: Any, spec: Mapping[str, Check], what: str,
            agreement: Callable[[Mapping[str, Any]], list[str]]) -> None:
    """Refuse data that does not match spec; once it does, refuse what agreement finds between its fields."""
    problems: list[str] = []
    obj(spec)(data, what, problems)
    if not problems:
        problems = agreement(data)
    if problems:
        raise RecordError(problems)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(HASH_CHUNK), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _refuse_constant(constant: str) -> Any:
    raise ValueError(f"{constant} is not a number")


def read_json(path: Path) -> Any:
    """The JSON value in path; RecordError when it is missing, malformed or holds NaN/Infinity."""
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=_refuse_constant)
    except FileNotFoundError:
        raise RecordError([f"no file {path}"]) from None
    except ValueError as exc:
        raise RecordError([f"{path}: not valid JSON ({exc})"]) from None
