"""SITL's AAS Lua defaults must equal NavPy's own PARAMS defaults.

NavPy reads the AAS_* MAVLink params whenever the autopilot exposes them, so
a Lua default that differs from NavPy's silently changes SITL behaviour.
"""

import re
from pathlib import Path

import pytest

from navpy.args.nav_args import NavArgs
from navpy.args.navigation_args import NavigationArgs
from navpy.args.navigation_poi_args import NavigationPoiArgs, encode_wp_bitmask

LUA = Path(__file__).resolve().parents[2] / "scripts" / "lua" / "aas_params.lua"
_TABLE = re.compile(r'param:add_table\(PARAM_TABLE_KEY,\s*"([A-Z_]+)"')
_PARAM = re.compile(
    r'param:add_param\(PARAM_TABLE_KEY,\s*\d+,\s*"([A-Z_0-9]+)",\s*(-?[0-9.]+)\)'
)

NAVPY_PARAMS = {**NavArgs.PARAMS, **NavigationArgs.PARAMS, **NavigationPoiArgs.PARAMS}


def _lua_defaults() -> dict[str, float]:
    src = LUA.read_text(encoding="utf-8")
    prefix = _TABLE.search(src).group(1)
    return {prefix + name: float(value) for name, value in _PARAM.findall(src)}


def _as_mavlink_value(name: str, default) -> float:
    """Encode a NavPy default the way it is stored in the MAVLink param."""
    if name == "AAS_TARG_WPS" and isinstance(default, str):
        # CLI form is 1-based WP numbers; the param is a WP bitmask.
        return float(encode_wp_bitmask([int(s) for s in default.split(",") if s.strip()]))
    return float(default)


def test_lua_parser_sees_every_param_line():
    src = LUA.read_text(encoding="utf-8")
    assert len(_lua_defaults()) == src.count("param:add_param(")


def test_every_navpy_param_is_defined_in_lua():
    missing = set(NAVPY_PARAMS) - set(_lua_defaults())
    assert not missing, f"aas_params.lua lacks NavPy params: {sorted(missing)}"


@pytest.mark.parametrize("name", sorted(NAVPY_PARAMS))
def test_lua_default_matches_navpy_default(name):
    lua = _lua_defaults()[name]
    navpy = _as_mavlink_value(name, NAVPY_PARAMS[name])
    assert lua == navpy, f"{name}: Lua default {lua} != NavPy default {navpy}"


def test_lua_never_auto_confirms_unapproved_poi():
    # MISS-04: confirm-window expiry must reject, not commit, the POI.
    assert _lua_defaults()["AAS_NAV_CM_FL"] == 0
