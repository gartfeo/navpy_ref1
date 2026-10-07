"""The AAS Lua parameter table must expose exactly the names the GCS/companion read."""

import re
from pathlib import Path

from gcs.backend.aas_params import AAS_PARAM_MAP

LUA = Path(__file__).resolve().parents[2] / "scripts" / "lua" / "aas_params.lua"
_TABLE = re.compile(r'param:add_table\(PARAM_TABLE_KEY,\s*"([A-Z_]+)"')
_PARAM = re.compile(r'param:add_param\(PARAM_TABLE_KEY,\s*(\d+),\s*"([A-Z_0-9]+)"')


def _lua_params() -> tuple[str, dict[int, str]]:
    src = LUA.read_text(encoding="utf-8")
    prefix = _TABLE.search(src).group(1)
    params: dict[int, str] = {}
    for index, name in _PARAM.findall(src):
        assert int(index) not in params, f"duplicate Lua param index {index}"
        params[int(index)] = name
    return prefix, params


def test_lua_table_defines_every_param_the_gcs_reads():
    prefix, params = _lua_params()
    lua_names = {prefix + name for name in params.values()}
    missing = set(AAS_PARAM_MAP.values()) - lua_names
    assert not missing, f"aas_params.lua lacks params read by the GCS: {sorted(missing)}"


def test_lua_table_has_no_params_the_gcs_does_not_know():
    prefix, params = _lua_params()
    lua_names = {prefix + name for name in params.values()}
    unknown = lua_names - set(AAS_PARAM_MAP.values())
    assert not unknown, f"aas_params.lua defines params unknown to the GCS: {sorted(unknown)}"


def test_lua_param_names_fit_mavlink_16_char_limit():
    prefix, params = _lua_params()
    too_long = [prefix + n for n in params.values() if len(prefix + n) > 16]
    assert not too_long, too_long
