"""Guard the app height on old Android WebViews (architecture_todo 113).

Chromium added the dynamic viewport units (dvh, svh, lvh and their width and
inline/block forms) in version 108. Android 13's factory WebView is 101; the
SIYI UniRC 10 Pro may keep it, since it has no Play Store. A declaration with
an unknown unit is dropped, and a React inline style has no second declaration
to fall back on: `height: '100dvh'` left the app at its content height (292 of
800 CSS px in the UniRC emulator) with the screen below it empty.

So the viewport height lives in one place, index.html, where `100vh` comes
first and `100dvh` only behind `@supports`. The app sizes itself from #root.
"""
import os
import re
import unittest


_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_FRONTEND = os.path.join(_ROOT, "src", "gcs", "frontend")
_SRC = os.path.join(_FRONTEND, "src")
_INDEX_HTML = os.path.join(_FRONTEND, "index.html")
_APP = os.path.join(_SRC, "App.jsx")

# A number followed by a dynamic, small or large viewport unit.
_NEW_VIEWPORT_UNIT = re.compile(r"\d\s*[dsl]v(?:h|w|i|b|min|max)\b")


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


class TestViewportHeight(unittest.TestCase):
    def test_no_new_viewport_units_in_frontend_scripts(self):
        # Scripts only: a stylesheet can give a fallback declaration first.
        offenders = []
        for folder, _dirs, files in os.walk(_SRC):
            for name in files:
                if not name.endswith((".js", ".jsx")):
                    continue
                path = os.path.join(folder, name)
                for number, line in enumerate(_read(path).splitlines(), 1):
                    if _NEW_VIEWPORT_UNIT.search(line):
                        offenders.append(f"{os.path.relpath(path, _ROOT)}:{number}: {line.strip()}")
        self.assertEqual(
            offenders, [],
            "dvh/svh/lvh are dropped before Chromium 108 and an inline style has no "
            "fallback; size from #root (index.html) instead",
        )

    def test_index_html_falls_back_to_vh_before_dvh(self):
        html = _read(_INDEX_HTML)
        root_rules = r"html\s*,\s*body\s*,\s*#root\s*\{([^}]*)\}"
        rule = re.search(root_rules, html).group(1)
        heights = re.findall(r"(?<![-\w])height\s*:\s*([^;]+);", rule)
        self.assertEqual(heights[-1].strip(), "100vh", "the unconditional rule must end on 100vh")
        self.assertRegex(
            html,
            r"@supports\s*\(\s*height\s*:\s*100dvh\s*\)\s*\{\s*html\s*,\s*body\s*,\s*#root\s*\{"
            r"[^}]*(?<![-\w])height\s*:\s*100dvh\s*;",
            "100dvh must apply only where it is supported",
        )

    def test_app_root_fills_root_element(self):
        # The first element App renders is the full-screen column.
        app = _read(_APP)
        root_style = re.search(r"return \(\s*<div\s*style=\{\{(.*?)\}\}", app, re.S).group(1)
        self.assertRegex(root_style, r"height:\s*'100%'")


if __name__ == "__main__":
    unittest.main()
