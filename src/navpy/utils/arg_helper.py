import argparse
from enum import Enum
from pathlib import Path


def parse_boolean(value):
    value = value.lower()

    if value in ["true", "yes", "y", "1", "t"]:
        return True
    elif value in ["false", "no", "n", "0", "f"]:
        return False

    return False


def parse_none_boolean(value):
    value = value.lower()

    if value in ["true", "yes", "y", "1", "t"]:
        return True
    elif value in ["false", "no", "n", "0", "f"]:
        return False

    return None


class StoreWithFlag(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, values)
        overrides = getattr(namespace, "_cli_overrides", set())
        overrides.add(self.dest)
        setattr(namespace, "_cli_overrides", overrides)


class StoreTrueWithFlag(argparse.Action):
    """store_true that also records the dest in ``_cli_overrides`` so
    value-vs-default comparison is never needed to detect an explicit flag."""

    def __init__(self, option_strings, dest, **kwargs):
        kwargs["nargs"] = 0
        super().__init__(option_strings, dest, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, True)
        overrides = getattr(namespace, "_cli_overrides", set())
        overrides.add(self.dest)
        setattr(namespace, "_cli_overrides", overrides)


def _normalize_value(value):
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (list, tuple, set)):
        items = [_normalize_value(item) for item in value]
        if isinstance(value, set):
            return sorted(items)
        return items
    return value


def collect_non_default_args(args, default_args, exclude_keys=None):
    """Return a dict of args that differ from parser defaults."""
    result = {}
    defaults = vars(default_args)
    exclude = set(exclude_keys or ())
    exclude.add("_cli_overrides")
    for key, value in vars(args).items():
        if key in exclude or key not in defaults:
            continue
        if _normalize_value(value) != _normalize_value(defaults.get(key)):
            result[key] = value
    return result


def collect_param_overrides(param_getter, defaults):
    """Return param values that differ from defaults."""
    result = {}
    for key, default in defaults.items():
        value = param_getter(key, default)
        if _normalize_value(value) != _normalize_value(default):
            result[key] = value
    return result


def format_kv(args_map):
    if not args_map:
        return "none"
    normalized = {k: _normalize_value(v) for k, v in args_map.items()}
    parts = [f"{key}={normalized[key]!r}" for key in sorted(normalized)]
    return ", ".join(parts)
