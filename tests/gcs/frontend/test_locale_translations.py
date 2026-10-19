import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
LOCALE_DIR = ROOT / "src" / "gcs" / "frontend" / "src" / "locales"
SETTINGS_COMPONENT_DIR = ROOT / "src" / "gcs" / "frontend" / "src" / "components" / "settings"
PLACEHOLDER_RE = re.compile(r"{{\s*([^}]+?)\s*}}")

IDENTICAL_VALUE_ALLOWLIST = {
    "settings.tabs.defaultDeliveryHubs",  # Owner-selected DDH tab acronym.
    "mapMarkers.dock",  # D is the owner-selected international dock marker.
    # Short technical label rather than untranslated prose.
    "connectionTab.id",
    # Universal acronyms — identical in Armenian by design, not untranslated.
    "preflight.ekf",
    "preflight.gps",
    # Firmware mode / acronym / product labels that are the same token
    # in Armenian (flight-mode names, protocol acronyms, ArduPilot product names).
    "planningSidebar.fenceActionRtl",                   # RTL
    "settings.failsafe.actions.BATT_ACTION_COPTER.2",   # RTL
    "settings.failsafe.actions.BATT_ACTION_PLANE.1",    # RTL
    "settings.failsafe.actions.BATT_ACTION_PLANE.4",    # QLand
    "settings.failsafe.actions.FS_GCS_ENABL.1",         # Heartbeat
    "settings.failsafe.actions.FS_GCS_ENABL.2",         # Heartbeat + REM RSSI
    "settings.failsafe.actions.FS_GCS_ENABL.3",         # Heartbeat + AUTO
    "settings.failsafe.actions.FS_GCS_ENABLE.1",        # RTL
    "settings.failsafe.actions.FS_LONG_ACTN.5",         # AUTOLAND
    "settings.failsafe.actions.FS_SHORT_ACTN.4",        # FBWB
    "settings.failsafe.firmware.copter",                # ArduCopter
    "settings.failsafe.firmware.plane",                 # ArduPlane
}

VISIBLE_LITERAL_GUARDS = {
    "ProfileSelector.jsx": [
        'title="Vision Profile"',
        ">Profile</label>",
        ">Approach pitch range</label>",
        ">Min</label>",
        ">Max</label>",
        ">Min Alt</label>",
        ">Optimize",
        ">Device</span>",
        ">Zoom</span>",
        ">Pitch</span>",
    ],
    "DetectionRangeDiagram.jsx": [
        ">Side View</text>",
        ">Top View</text>",
        ">GND</text>",
        ">fwd</text>",
    ],
}


def _load_locale(name):
    return json.loads((LOCALE_DIR / f"{name}.json").read_text(encoding="utf-8"))


def _flatten_strings(value, prefix=""):
    if isinstance(value, str):
        return {prefix: value}
    if isinstance(value, dict):
        flattened = {}
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else key
            flattened.update(_flatten_strings(child, child_prefix))
        return flattened
    raise AssertionError(f"Unexpected locale value at {prefix}: {value!r}")


def _placeholders(value):
    return {match.strip() for match in PLACEHOLDER_RE.findall(value)}


def test_armenian_locale_matches_english_keys_and_placeholders():
    en = _flatten_strings(_load_locale("en"))
    hy = _flatten_strings(_load_locale("hy"))

    assert hy.keys() == en.keys()

    placeholder_mismatches = {
        key: (en[key], hy[key])
        for key in en
        if _placeholders(en[key]) != _placeholders(hy[key])
    }
    assert placeholder_mismatches == {}


def test_armenian_locale_has_no_copied_english_values():
    en = _flatten_strings(_load_locale("en"))
    hy = _flatten_strings(_load_locale("hy"))

    copied_values = {
        key: value
        for key, value in hy.items()
        if value == en[key] and key not in IDENTICAL_VALUE_ALLOWLIST
    }

    assert copied_values == {}


def test_status_badges_read_distinctly_in_each_locale():
    """A UAV card badge must never show one word for two states (e.g. the
    Armenian idle label used to read "waiting", now a status of its own)."""
    for locale in ("en", "hy"):
        strings = _load_locale(locale)
        labels = [
            *strings["missionStatus"].values(),
            strings["vehicle"]["swarmBusy"],
            strings["vehicle"]["swarmUnknown"],
        ]
        assert len(set(labels)) == len(labels), (locale, labels)


def test_vision_profile_surface_does_not_render_known_english_literals_directly():
    offenders = {}
    for filename, literals in VISIBLE_LITERAL_GUARDS.items():
        source = (SETTINGS_COMPONENT_DIR / filename).read_text(encoding="utf-8")
        found = [literal for literal in literals if literal in source]
        if found:
            offenders[filename] = found

    assert offenders == {}


def test_dock_planning_and_settings_literal_keys_resolve():
    components = ROOT / "src/gcs/frontend/src/components"
    consumers = [
        "sidebar/PlanningSidebar.jsx",
        "settings/WaypointEditorOverlay.jsx",
        "settings/VisionTab.jsx",
    ]
    for locale in ("en", "hy"):
        strings = _flatten_strings(_load_locale(locale))
        missing = []
        for name in consumers:
            source = (components / name).read_text(encoding="utf-8")
            keys = re.findall(r'''\bt\(\s*['"]([^'"]+)['"]\s*(?:\)|,)''', source)
            for key in keys:
                if key not in strings and key + "_other" not in strings:
                    missing.append((name, key))
        assert not missing, (locale, missing)
