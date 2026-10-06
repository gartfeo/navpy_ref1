"""Tests for AAS parameter section localization metadata."""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
_UTIL_PATH = os.path.join(
    _ROOT,
    "src", "gcs", "frontend", "src", "components", "settings",
    "paramSections.js",
)
_EN_LOCALE = os.path.join(_ROOT, "src", "gcs", "frontend", "src", "locales", "en.json")
_HY_LOCALE = os.path.join(_ROOT, "src", "gcs", "frontend", "src", "locales", "hy.json")

_raw = open(_UTIL_PATH, encoding="utf-8").read()
_JS_SRC = _raw.replace("export const ", "const ")


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _sections():
    return json.loads(_run_js(
        "console.log(JSON.stringify([...PARAM_SECTIONS, CONFIRMATION_PARAM_SECTION, ...SIM_PARAM_SECTIONS]));"
    ))


def _load_locale(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _has_key(locale, dotted_key):
    node = locale
    for part in dotted_key.split("."):
        if not isinstance(node, dict) or part not in node:
            return False
        node = node[part]
    return isinstance(node, str)


class TestParamSectionLocalizationMetadata(unittest.TestCase):
    """AAS section metadata must point user-facing strings at locale keys."""

    def test_all_user_facing_metadata_has_translation_keys(self):
        technical_option_labels = {"PID", "PN", "Vision PN"}
        missing = []

        for section in _sections():
            if not section.get("titleKey"):
                missing.append(f"section {section.get('title')}: titleKey")
            for field in section.get("fields", []):
                if not field.get("labelKey"):
                    missing.append(f"field {field.get('key')}: labelKey")
                if field.get("placeholder") and not field.get("placeholderKey"):
                    missing.append(f"field {field.get('key')}: placeholderKey")
                for option in field.get("options", []):
                    if option.get("label") in technical_option_labels:
                        continue
                    if not option.get("labelKey"):
                        missing.append(
                            f"field {field.get('key')} option {option.get('label')}: labelKey"
                        )

        self.assertEqual(missing, [])

    def test_metadata_translation_keys_exist_in_locales(self):
        locales = {
            "en": _load_locale(_EN_LOCALE),
            "hy": _load_locale(_HY_LOCALE),
        }
        keys = set()

        for section in _sections():
            keys.add(section["titleKey"])
            for field in section.get("fields", []):
                keys.add(field["labelKey"])
                if field.get("placeholderKey"):
                    keys.add(field["placeholderKey"])
                for option in field.get("options", []):
                    if option.get("labelKey"):
                        keys.add(option["labelKey"])

        missing = [
            f"{locale_name}:{key}"
            for locale_name, locale in locales.items()
            for key in sorted(keys)
            if not _has_key(locale, key)
        ]
        self.assertEqual(missing, [])


if __name__ == "__main__":
    unittest.main()
