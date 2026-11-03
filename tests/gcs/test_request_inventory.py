"""The request inventory (``docs/design/gcs_request_inventory.md``) matches the app.

Every call that sends a backend request (method and route), every WebSocket
message and every built URL in ``src/gcs/frontend/src`` has a row, and every
row is still sent (dev sim step 10). A new request without a row fails, and so
does a request the scanner cannot resolve: a URL built by concatenation, a
route string no call uses, another request API, or an external host the
inventory does not name.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlsplit

import pytest

ROOT = Path(__file__).resolve().parents[2]
FRONTEND = ROOT / "src/gcs/frontend/src"
INVENTORY = ROOT / "docs/design/gcs_request_inventory.md"
#: Vendored code talks to the media server, not the backend; its requests are
#: rows of the inventory's "Outside the run" table.
SKIPPED_DIRS = ("vendor",)

_CALL = re.compile(r"(?<![\w.#$])(fetch|fetchWithRetry|new\s+WebSocket|sendWsMessage)\s*\(")
_OTHER_APIS = re.compile(r"\b(XMLHttpRequest|EventSource|sendBeacon|axios)\b")
_METHOD = re.compile(r"\bmethod\s*:\s*['\"](\w+)['\"]")
_ROUTE = re.compile(r"/(?:api|ws)/")
_IDENT = re.compile(r"[A-Za-z_$][\w$]*\Z")
_NAMED = re.compile(r"\{[a-z_]+\}")


class Unresolved(AssertionError):
    pass


@dataclass(frozen=True)
class Literal:
    start: int
    end: int
    text: str  # template interpolations as ``{}``


@dataclass
class Scan:
    requests: set[tuple[str, str]]
    messages: set[str]
    built: dict[str, set[str]]  # identifier -> methods; "?" when not a literal
    hosts: set[str]


def _skip_string(text: str, i: int) -> int:
    """Past a quoted string; a quote with no close on its line (JSX text such
    as ``Don't``) is a plain character, since a JS string cannot cross a line."""
    quote, j = text[i], i + 1
    while j < len(text) and text[j] not in (quote, "\n"):
        j += 2 if text[j] == "\\" else 1
    return j + 1 if j < len(text) and text[j] == quote else i + 1


def _skip_template(text: str, i: int, out: list[str] | None) -> int:
    """From the opening backtick to past the closing one; ``out`` gets the
    literal text with each ``${...}`` as ``{}``."""
    i += 1
    while text[i] != "`":
        if text[i] == "\\":
            if out is not None:
                out.append(text[i:i + 2])
            i += 2
        elif text.startswith("${", i):
            i = _skip_code(text, i + 2, "}")
            if out is not None:
                out.append("{}")
        else:
            if out is not None:
                out.append(text[i])
            i += 1
    return i + 1


def _skip_code(text: str, i: int, close: str) -> int:
    """Past the ``close`` that ends this code at depth 0."""
    depth = 0
    while True:
        c = text[i]
        if c in "'\"":
            i = _skip_string(text, i)
        elif c == "`":
            i = _skip_template(text, i, None)
        elif c in "([{":
            depth += 1
            i += 1
        elif c in ")]}":
            if depth == 0 and c == close:
                return i + 1
            depth -= 1
            i += 1
        else:
            i += 1


def _blank(text: str) -> str:
    return re.sub(r"[^\n]", " ", text)


def _template_code(text: str, i: int, j: int) -> str:
    """The template from ``i`` to ``j`` with its text blanked but each
    ``${...}`` kept, since that is code and may send a request."""
    out, k = ["`"], i + 1
    while k < j - 1:
        if text[k] == "\\":
            out.append(_blank(text[k:k + 2]))
            k += 2
        elif text.startswith("${", k):
            end = _skip_code(text, k + 2, "}")
            out.append(text[k:end])
            k = end
        else:
            out.append(_blank(text[k]))
            k += 1
    out.append("`")
    return "".join(out)


def strip_comments(text: str) -> tuple[str, str, list[Literal]]:
    """The code with comments blanked; the same with string contents blanked
    too, to search for code only; and the top-level string literals."""
    out, code_only, literals, i = [], [], [], 0
    while i < len(text):
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = len(text) if j < 0 else j
            out.append(" " * (j - i))
            code_only.append(out[-1])
            i = j
        elif text.startswith("/*", i):
            j = text.index("*/", i) + 2
            out.append(_blank(text[i:j]))
            code_only.append(out[-1])
            i = j
        elif c in "'\"`":
            parts: list[str] = []
            j = _skip_template(text, i, parts) if c == "`" else _skip_string(text, i)
            body = "".join(parts) if c == "`" else text[i + 1:j - 1]
            literals.append(Literal(i, j, body))
            out.append(text[i:j])
            code_only.append(_template_code(text, i, j) if c == "`" else
                             c + _blank(text[i + 1:j - 1]) + text[j - 1] if j - i > 1 else c)
            i = j
        else:
            out.append(c)
            code_only.append(c)
            i += 1
    return "".join(out), "".join(code_only), literals


def _split_args(args: str) -> list[str]:
    parts, depth, start, i = [], 0, 0, 0
    while i < len(args):
        c = args[i]
        if c in "'\"":
            i = _skip_string(args, i)
            continue
        if c == "`":
            i = _skip_template(args, i, None)
            continue
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == "," and depth == 0:
            parts.append(args[start:i])
            start = i + 1
        i += 1
    parts.append(args[start:])
    return [p.strip() for p in parts]


def normalize(route: str) -> str:
    """``/api/vehicles/{}/parameters{}?x`` and ``/api/vehicles/{sys_id}/parameters``
    both give ``/api/vehicles/{}/parameters``."""
    route = _NAMED.sub("{}", route).split("?", 1)[0]
    return re.sub(r"(?<!/)\{\}", "", route)


def _route_of(literal: Literal) -> str:
    match = _ROUTE.search(literal.text)
    if match is None:
        raise Unresolved(f"a request to {literal.text!r}, not a backend route")
    return normalize(literal.text[match.start():])


def scan_text(text: str) -> tuple[Scan, list[Literal]]:
    """The requests in one file; also its route literals no call used."""
    code, code_only, literals = strip_comments(text)
    if (other := _OTHER_APIS.search(code_only)):
        raise Unresolved(f"another request API: {other.group(1)}")
    by_start = {lit.start: lit for lit in literals}
    scan = Scan(set(), set(), {}, set())
    used: set[int] = set()
    for call in _CALL.finditer(code_only):
        if code_only[:call.start()].rstrip().endswith("function"):
            continue  # a definition, not a call
        open_paren = call.end() - 1
        args = code[open_paren + 1:_skip_code(code, open_paren + 1, ")") - 1]
        first, *rest = _split_args(args)
        name = re.sub(r"\s+", " ", call.group(1))
        if name == "sendWsMessage":
            scan.messages |= _ws_types(first, code)
            continue
        start = open_paren + 1 + (len(args) - len(args.lstrip()))
        method = "WS" if name == "new WebSocket" else _method(rest)
        literal = by_start.get(start)
        if literal is not None and literal.end - literal.start == len(first):
            if method == "?":
                raise Unresolved(f"a request whose method is not a literal: {args.strip()!r}")
            scan.requests.add((method, _route_of(literal)))
            used.add(literal.start)
        elif _IDENT.match(first):
            scan.built.setdefault(first, set()).add(method)
        else:
            raise Unresolved(f"a request URL that is neither a literal nor a name: {first!r}")
    for lit in literals:
        if (host := urlsplit(lit.text).hostname) and lit.text.startswith(("http://", "https://")):
            scan.hosts.add(host)
    left = [lit for lit in literals if lit.start not in used and _ROUTE.search(lit.text)]
    return scan, left


def _method(rest: list[str]) -> str:
    """The ``method:`` literal of the options object; GET when it names no
    method; ``?`` when the text cannot tell (options not an object literal,
    a spread, or a method that is not a literal)."""
    if not rest:
        return "GET"
    options = rest[0]
    if not (options.startswith("{") and options.endswith("}")):
        return "?"
    method = "GET"
    for prop in _split_args(options[1:-1]):
        if prop.startswith("..."):
            return "?"
        if re.match(r"['\"]?method\b", prop):
            m = _METHOD.fullmatch(prop)
            if m is None:
                return "?"
            method = m.group(1).upper()
    return method


def _ws_types(first: str, code: str) -> set[str]:
    if first.startswith("{") and "..." in strip_comments(first)[1]:
        found = set()  # a spread could carry its own type
    elif first.startswith("{"):
        found = set(re.findall(r"\btype\s*:\s*['\"]([a-z_]+)['\"]", first))
    elif _IDENT.match(first):
        found = set(re.findall(rf"\b{re.escape(first)}\.type\s*=\s*['\"]([a-z_]+)['\"]", code))
    else:
        found = set()
    if not found:
        raise Unresolved(f"a WebSocket message whose type is not a literal: {first!r}")
    return found


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


def _table(heading: str, text: str) -> list[list[str]]:
    """The body rows of the first table after ``heading``, cells without backticks."""
    rows, started = [], False
    for line in text[text.index(heading):].splitlines()[1:]:
        if line.startswith("|"):
            started = True
            cells = [c.strip().replace("`", "") for c in line.strip().strip("|").split("|")]
            if not set(cells[0]) <= {"-"}:
                rows.append(cells)
        elif started:
            break
    return rows[1:]


@dataclass
class Inventory:
    requests: set[tuple[str, str]]
    messages: set[str]
    built: dict[tuple[str, str], list[str]]  # (file, name) -> routes; [] for a wrapper
    outside: str


def inventory(text: str) -> Inventory:
    requests, messages = set(), set()
    for method, route, *_ in _table("## Requests", text):
        if method == "WS-SEND":
            messages.add(route)
        else:
            requests.add((method, normalize(route)))
    built = {}
    for file, name, routes, *_ in _table("**Built URLs**", text):
        built[(file, name)] = [] if routes == "wrapper" else [normalize(r.strip()) for r in routes.split(",")]
    return Inventory(requests, messages, built, text[text.index("## Outside the run"):])


def _short(path: Path) -> str:
    return path.relative_to(FRONTEND).as_posix().removeprefix("components/")


def scan_frontend(inv: Inventory) -> Scan:
    total = Scan(set(), set(), {}, set())
    for path in _sources():
        scan, left = scan_text(path.read_text(encoding="utf-8"))
        file = _short(path)
        total.requests |= scan.requests
        total.messages |= scan.messages
        total.hosts |= scan.hosts
        built_routes: set[str] = set()
        for name, methods in scan.built.items():
            if (file, name) not in inv.built:
                raise Unresolved(f"{file}: a request URL built in {name!r} has no Built URLs row")
            routes = inv.built[(file, name)]
            if routes and "?" in methods:
                raise Unresolved(f"{file}: a request through {name!r} whose method is not a literal")
            total.built[f"{file}:{name}"] = methods
            total.requests |= {(method, route) for method in methods for route in routes}
            built_routes |= set(routes)
        # Both ways: a route string no call uses, or a Built URLs route the file no longer holds.
        left_routes = {_route_of(lit) for lit in left}
        if left_routes != built_routes:
            raise Unresolved(f"{file}: route strings {sorted(left_routes)}, Built URLs routes {sorted(built_routes)}")
    return total


def test_every_request_the_frontend_sends_is_in_the_inventory_and_no_more() -> None:
    inv = inventory(INVENTORY.read_text(encoding="utf-8"))
    got = scan_frontend(inv)
    assert got.requests - inv.requests == set(), "sent but not in the inventory"
    assert inv.requests - got.requests == set(), "in the inventory but no longer sent"
    assert got.messages == inv.messages
    assert set(got.built) == {f"{file}:{name}" for file, name in inv.built}
    unnamed = {host for host in got.hosts if host not in inv.outside}
    assert unnamed == set(), "external hosts the inventory does not name"


SAMPLE = """
  // fetch('/api/commented-out');
  /* fetch('/api/also-commented') */
  fetch('/api/settings');
  fetch('/api/settings', { method: 'PUT', body: JSON.stringify(d) });
  fetch(`/api/vehicles/${sid}/parameters${force ? '?refresh=true' : ''}`, { headers });
  fetch(`/api/vehicles/connect?${params}`, { method: 'POST', signal: t.signal });
  new WebSocket(`${protocol}//${window.location.host}/ws/telemetry`);
  export default async function fetchWithRetry(url, options = {}) { return fetch(url, options); }
  await fetchWithRetry('/api/vehicles/upload', { method: 'POST' });
  const endpoint = on ? '/api/control/launch/esp32-sim/stop' : `/api/control/launch/esp32-sim/start?port=${p}`;
  fetch(endpoint, { method: 'POST' });
  payload.type = 'manual_control';
  sendWsMessage(payload);
  sendWsMessage({ type: "manual_control_stop", sys_id: sysId });
  const link = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
"""


def test_the_scanner_pairs_each_method_with_its_route_and_skips_comments() -> None:
    scan, left = scan_text(SAMPLE)
    assert scan.requests == {
        ("GET", "/api/settings"), ("PUT", "/api/settings"), ("GET", "/api/vehicles/{}/parameters"),
        ("POST", "/api/vehicles/connect"), ("WS", "/ws/telemetry"), ("POST", "/api/vehicles/upload"),
    }
    assert scan.built == {"url": {"?"}, "endpoint": {"POST"}}
    assert scan.messages == {"manual_control", "manual_control_stop"}
    assert {_route_of(lit) for lit in left} == {
        "/api/control/launch/esp32-sim/stop", "/api/control/launch/esp32-sim/start"}
    assert scan.hosts == {"tile.openstreetmap.org"}


@pytest.mark.parametrize("source", [
    "fetch('/api/' + kind);",
    "fetch(base + '/params');",
    "fetch(`${base}/x`);",
    "fetch('https://example.com/data');",
    "const xhr = new XMLHttpRequest();",
    "navigator.sendBeacon('/api/diagnostics/events', b);",
    "new EventSource('/api/stream');",
    "sendWsMessage(buildMsg());",
    "const msg = {}; sendWsMessage(msg);",
    "const method = 'DELETE'; fetch('/api/settings', { method });",
    "fetch('/api/settings', { ...opts });",
    "fetch('/api/settings', opts);",
    "sendWsMessage({ type: 'manual_control', ...message });",
    "fetch('/api/settings', { method: 'PUT', ...opts });",
    "const t = `${await fetch('/api/new-route')}`;",
])
def test_a_request_the_scanner_cannot_resolve_fails(source: str) -> None:
    with pytest.raises(Unresolved):
        scan_text(source)


def test_words_inside_strings_are_not_code() -> None:
    scan, left = scan_text("const help = 'axios, fetch(x) and new EventSource';")
    assert scan.requests == set() and not scan.built and not left


def test_a_built_url_with_two_methods_keeps_both() -> None:
    scan, _ = scan_text("fetch(u, { method: 'PUT' }); fetch(u, { method: 'DELETE' });")
    assert scan.built == {"u": {"PUT", "DELETE"}}


def test_only_the_options_own_method_counts() -> None:
    scan, _ = scan_text("fetch('/api/x', { body: JSON.stringify({ method: 'PUT' }) });")
    assert scan.requests == {("GET", "/api/x")}


def _frontend_with(extra: str, inv: Inventory) -> Scan:
    scan, left = scan_text(extra)
    assert not left and not scan.built
    total = scan_frontend(inv)
    total.requests |= scan.requests
    return total


def test_a_new_method_or_route_without_a_row_fails_and_a_stale_row_fails() -> None:
    text = INVENTORY.read_text(encoding="utf-8")
    inv = inventory(text)
    added = _frontend_with("fetch('/api/settings', { method: 'DELETE' });", inv)
    assert added.requests - inv.requests == {("DELETE", "/api/settings")}
    added = _frontend_with("fetch(`/api/vehicles/${id}/rally`);", inv)
    assert added.requests - inv.requests == {("GET", "/api/vehicles/{}/rally")}
    stale = inventory(text.replace("| PUT | /api/settings |", "| PUT | /api/settings |\n| DELETE | /api/settings | x | x | x | x | x |"))
    assert stale.requests - scan_frontend(stale).requests == {("DELETE", "/api/settings")}
    dropped = text.replace("/api/control/launch/esp32-sim/start, /api/control/launch/esp32-sim/stop |",
                           "/api/control/launch/esp32-sim/start |")
    assert dropped != text
    with pytest.raises(Unresolved):  # the stop branch is in the source but not in the row
        scan_frontend(inventory(dropped))
    extra = text.replace("| `url` | /api/vehicles/{sys_id}/parameters |", "| `url` | /api/vehicles/{sys_id}/parameters, /api/gone |")
    assert extra != text
    with pytest.raises(Unresolved):  # a row route the file no longer holds
        scan_frontend(inventory(extra))
    renamed = text.replace("| `settings/LaunchTab.jsx` | `endpoint` |", "| `settings/LaunchTab.jsx` | `other` |")
    assert renamed != text
    with pytest.raises(Unresolved):
        scan_frontend(inventory(renamed))
