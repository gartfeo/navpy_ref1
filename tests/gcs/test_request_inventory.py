"""The request inventory (``docs/design/gcs_request_inventory.md``) matches the app.

Every backend route, WebSocket message and built URL the frontend sends has a
row, and every row is still sent (dev sim step 10). A new request without an
entry fails here.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FRONTEND = ROOT / "src/gcs/frontend/src"
INVENTORY = ROOT / "docs/design/gcs_request_inventory.md"
#: Vendored code talks to the media server, not the backend.
SKIPPED_DIRS = ("vendor",)

_INTERPOLATION = re.compile(r"\$\{[^{}]*\}")
_ROUTE = re.compile(r"(?:['\"`]|\{\})(/(?:api|ws)/[^'\"`\s]*)")
_FETCH_BY_NAME = re.compile(r"\bfetch(?:WithRetry)?\(\s*([A-Za-z_]\w*)\s*[,)]")
_WS_SEND = re.compile(r"\bsendWsMessage\(\s*[{\w]")
_WS_TYPE = re.compile(r"\btype\s*[:=]\s*'([a-z_]+)'")
_NAMED = re.compile(r"\{[a-z_]+\}")


def _sources() -> list[Path]:
    files = []
    for path in sorted(FRONTEND.rglob("*")):
        rel = path.relative_to(FRONTEND)
        if path.suffix not in {".js", ".jsx", ".ts", ".tsx"} or rel.parts[0] in SKIPPED_DIRS:
            continue
        if "__tests__" in rel.parts or ".test." in path.name:
            continue
        files.append(path)
    return files


def normalize(route: str) -> str:
    """``/api/vehicles/${sid}/parameters${force ? '?refresh=true' : ''}`` and
    ``/api/vehicles/{sys_id}/parameters`` both give ``/api/vehicles/{}/parameters``."""
    route = _NAMED.sub("{}", _INTERPOLATION.sub("{}", route))
    route = route.split("?", 1)[0]
    return re.sub(r"(?<!/)\{\}", "", route)


def source_routes(text: str) -> set[str]:
    return {normalize(m) for m in _ROUTE.findall(_INTERPOLATION.sub("{}", text))}


def source_ws_messages(text: str) -> set[str]:
    return set(_WS_TYPE.findall(text)) if _WS_SEND.search(text) else set()


def source_built_urls(path: Path, text: str) -> set[tuple[str, str]]:
    rel = path.relative_to(FRONTEND).as_posix()
    return {(rel, name) for name in _FETCH_BY_NAME.findall(text)}


def _table(heading: str, text: str) -> list[list[str]]:
    """The rows of the first table after ``heading``, cells stripped of backticks."""
    rows = []
    started = False
    for line in text[text.index(heading):].splitlines()[1:]:
        if line.startswith("|"):
            started = True
            cells = [c.strip().strip("`") for c in line.strip().strip("|").split("|")]
            if not set(cells[0]) <= {"-"}:
                rows.append(cells)
        elif started:
            break
    return rows[1:]  # drop the header row


def inventory() -> tuple[set[str], set[str], set[tuple[str, str]]]:
    text = INVENTORY.read_text(encoding="utf-8")
    routes, messages = set(), set()
    for method, route, *_ in _table("## Requests", text):
        if method == "WS-SEND":
            messages.add(route)
        else:
            routes.add(normalize(route))
    built = set()
    for file, name, *_ in _table("**Built URLs**", text):
        stem = file.split("/", 1)[-1] if file.startswith("settings/") else file
        built.add((_resolve(stem), name))
    return routes, messages, built


def _resolve(short: str) -> str:
    """The inventory names files as in its prose (``settings/LaunchTab.jsx``
    for ``components/settings/...``); match them to the one source file."""
    matches = [p.relative_to(FRONTEND).as_posix() for p in _sources()
               if p.relative_to(FRONTEND).as_posix().endswith(short)]
    assert len(matches) == 1, f"{short!r} names {matches}"
    return matches[0]


def scanned() -> tuple[set[str], set[str], set[tuple[str, str]]]:
    routes, messages, built = set(), set(), set()
    for path in _sources():
        text = path.read_text(encoding="utf-8")
        routes |= source_routes(text)
        messages |= source_ws_messages(text)
        built |= source_built_urls(path, text)
    return routes, messages, built


def test_every_request_the_frontend_sends_is_in_the_inventory_and_no_more() -> None:
    want_routes, want_messages, want_built = inventory()
    got_routes, got_messages, got_built = scanned()
    assert got_routes - want_routes == set(), "routes sent but not in the inventory"
    assert want_routes - got_routes == set(), "inventory routes no longer sent"
    assert got_messages == want_messages
    assert got_built == want_built


def test_the_scanner_finds_each_way_a_request_is_written() -> None:
    text = """
      fetch('/api/settings');
      fetch(`/api/vehicles/${sid}/parameters${force ? '?refresh=true' : ''}`, { headers });
      fetch(`/api/vehicles/connect?${params}`, { method: 'POST' });
      const endpoint = on ? '/api/control/launch/esp32-sim/stop'
        : `/api/control/launch/esp32-sim/start?port=${l.esp32_port}`;
      new WebSocket(`${protocol}//${window.location.host}/ws/telemetry`);
      await fetchWithRetry(url, options);
      payload.type = 'manual_control';
      sendWsMessage({ type: 'manual_control_stop', sys_id: sysId });
    """
    assert source_routes(text) == {
        "/api/settings", "/api/vehicles/{}/parameters", "/api/vehicles/connect",
        "/api/control/launch/esp32-sim/stop", "/api/control/launch/esp32-sim/start",
        "/ws/telemetry",
    }
    assert source_ws_messages(text) == {"manual_control", "manual_control_stop"}
    assert source_built_urls(FRONTEND / "x.js", text) == {("x.js", "url")}
    assert normalize("/api/vision-profiles/{profile_name}/devices/{device_name}") == \
        "/api/vision-profiles/{}/devices/{}"


def test_a_new_request_without_an_entry_fails() -> None:
    want_routes, _, _ = inventory()
    assert source_routes("fetch(`/api/vehicles/${id}/rally`)") - want_routes == {"/api/vehicles/{}/rally"}
