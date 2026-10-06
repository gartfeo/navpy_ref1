"""Tests for delivery icon SVG generators in icons.js and deliveryHubIcons.js."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re
import urllib.parse


_CONSTANTS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "constants",
))


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_ICONS_JS = _strip_es_modules(
    open(os.path.join(_CONSTANTS_DIR, "icons.js"), encoding="utf-8").read()
)

_DOCK_ICONS_JS = _strip_es_modules(
    open(os.path.join(_CONSTANTS_DIR, "deliveryHubIcons.js"), encoding="utf-8").read()
)


def _run_js(script):
    code = _ICONS_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _run_dock_js(script):
    code = _DOCK_ICONS_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _decode_svg(data_url):
    """Extract and decode SVG content from a data:image/svg+xml URL."""
    prefix = "data:image/svg+xml,"
    assert data_url.startswith(prefix), f"Not a data URL: {data_url[:60]}"
    return urllib.parse.unquote(data_url[len(prefix):])


class TestMakeDockIcon(unittest.TestCase):
    """makeDockIcon — outlined dock pad."""

    def test_returns_data_url(self):
        out = _run_js('console.log(makeDockIcon(32));')
        self.assertTrue(out.startswith("data:image/svg+xml,"))

    def test_contains_pad_and_letter_path(self):
        out = _run_js('console.log(makeDockIcon(32));')
        svg = _decode_svg(out)
        self.assertIn("<rect", svg)
        self.assertIn("<path", svg)
        self.assertIn("#00d2ff", svg)

    def test_size_attribute(self):
        out = _run_js('console.log(makeDockIcon(48));')
        svg = _decode_svg(out)
        self.assertIn('width="48"', svg)
        self.assertIn('height="48"', svg)


class TestMakeAvailableTaskIcon(unittest.TestCase):
    """makeAvailableTaskIcon — dashed orange circle with center dot."""

    def test_returns_data_url(self):
        out = _run_js('console.log(makeAvailableTaskIcon(32));')
        self.assertTrue(out.startswith("data:image/svg+xml,"))

    def test_contains_dashed_circle(self):
        out = _run_js('console.log(makeAvailableTaskIcon(32));')
        svg = _decode_svg(out)
        self.assertIn("<circle", svg)
        self.assertIn("stroke-dasharray", svg)

    def test_orange_color(self):
        out = _run_js('console.log(makeAvailableTaskIcon(32));')
        svg = _decode_svg(out)
        self.assertIn("#ff9800", svg)

    def test_size_attribute(self):
        out = _run_js('console.log(makeAvailableTaskIcon(48));')
        svg = _decode_svg(out)
        self.assertIn('width="48"', svg)
        self.assertIn('height="48"', svg)

    def test_has_center_dot(self):
        out = _run_js('console.log(makeAvailableTaskIcon(32));')
        svg = _decode_svg(out)
        # Two circles: outer dashed + inner center dot
        count = svg.count("<circle")
        self.assertEqual(count, 2, f"Expected 2 circles, got {count}")


class TestMakeAssignmentIcon(unittest.TestCase):
    """makeAssignmentIcon — crosshair with configurable color."""

    def test_returns_data_url(self):
        out = _run_js('console.log(makeAssignmentIcon(32, "#D90012"));')
        self.assertTrue(out.startswith("data:image/svg+xml,"))

    def test_contains_pad_and_letter_path(self):
        out = _run_js('console.log(makeAssignmentIcon(32, "#D90012"));')
        svg = _decode_svg(out)
        self.assertIn("<rect", svg)
        self.assertIn("<path", svg)

    def test_uses_given_color(self):
        out = _run_js('console.log(makeAssignmentIcon(32, "#44DDDD"));')
        svg = _decode_svg(out)
        self.assertIn("#44DDDD", svg)

    def test_size_attribute(self):
        out = _run_js('console.log(makeAssignmentIcon(48, "red"));')
        svg = _decode_svg(out)
        self.assertIn('width="48"', svg)
        self.assertIn('height="48"', svg)

    def test_no_dashes(self):
        """Assignment icon should be solid, not dashed like confirm icon."""
        out = _run_js('console.log(makeAssignmentIcon(32, "#D90012"));')
        svg = _decode_svg(out)
        self.assertNotIn("stroke-dasharray", svg)


class TestMakeLaunchIcon(unittest.TestCase):
    """makeLaunchIcon — upward arrow inside a circle."""

    def test_returns_data_url(self):
        out = _run_js('console.log(makeLaunchIcon("#ff9800", 28));')
        self.assertTrue(out.startswith("data:image/svg+xml,"))

    def test_contains_launch_arrow(self):
        out = _run_js('console.log(makeLaunchIcon("#ff9800", 28));')
        svg = _decode_svg(out)
        self.assertIn("<path", svg)
        self.assertIn("<circle", svg)

    def test_circle_is_stroke_only(self):
        out = _run_js('console.log(makeLaunchIcon("#ff9800", 28));')
        svg = _decode_svg(out)
        self.assertIn('fill="none"', svg)


class TestMakeCorridorIcon(unittest.TestCase):
    """makeCorridorIcon — filled circle with inner dot (control point)."""

    def test_returns_data_url(self):
        out = _run_js('console.log(makeCorridorIcon("#00aaff", 18));')
        self.assertTrue(out.startswith("data:image/svg+xml,"))

    def test_contains_two_circles(self):
        out = _run_js('console.log(makeCorridorIcon("#00aaff", 18));')
        svg = _decode_svg(out)
        count = svg.count("<circle")
        self.assertEqual(count, 2, f"Expected 2 circles, got {count}")


class TestLaunchPointIcon(unittest.TestCase):
    """LAUNCH_POINT_ICON constant — assembly point default."""

    def test_is_data_url(self):
        out = _run_js('console.log(LAUNCH_POINT_ICON);')
        self.assertTrue(out.startswith("data:image/svg+xml,"))

    def test_arrow_in_svg(self):
        out = _run_js('console.log(LAUNCH_POINT_ICON);')
        svg = _decode_svg(out)
        self.assertIn("<path", svg)


class TestDockIcons(unittest.TestCase):
    """Neutral delivery host icons."""

    DELIVERY_HUB_TYPES = ['building', 'vehicle', 'antenna', 'operations_site', 'bridge', 'fuel', 'other']

    def test_all_types_return_data_url(self):
        for t in self.DELIVERY_HUB_TYPES:
            out = _run_dock_js(f'console.log(getDeliveryHubIcon("{t}"));')
            self.assertTrue(
                out.startswith("data:image/svg+xml,"),
                f"DOCK type '{t}' did not return data URL",
            )

    def test_all_types_contain_svg_tag(self):
        for t in self.DELIVERY_HUB_TYPES:
            out = _run_dock_js(f'console.log(getDeliveryHubIcon("{t}"));')
            svg = _decode_svg(out)
            self.assertIn("<svg", svg, f"DOCK type '{t}' missing <svg> tag")

    def test_building_has_outline_and_windows(self):
        out = _run_dock_js('console.log(getDeliveryHubIcon("building"));')
        svg = _decode_svg(out)
        self.assertIn("<path", svg)

    def test_fuel_has_pump(self):
        out = _run_dock_js('console.log(getDeliveryHubIcon("fuel"));')
        svg = _decode_svg(out)
        self.assertIn('<rect x="3" y="3" width="11" height="18"', svg)

    def test_operations_has_briefcase(self):
        out = _run_dock_js('console.log(getDeliveryHubIcon("operations_site"));')
        svg = _decode_svg(out)
        self.assertIn('<rect x="3" y="7" width="18" height="14"', svg)

    def test_other_has_dock_pad(self):
        out = _run_dock_js('console.log(getDeliveryHubIcon("other"));')
        svg = _decode_svg(out)
        self.assertIn('M9 7h3a5 5 0 0 1 0 10H9z', svg)

    def test_host_symbols_are_distinct_and_same_size(self):
        icons = [_decode_svg(_run_dock_js(f'console.log(getDeliveryHubIcon("{t}"));')) for t in self.DELIVERY_HUB_TYPES]
        self.assertEqual(len(set(icons)), len(self.DELIVERY_HUB_TYPES))
        for svg in icons:
            self.assertIn('width="24"', svg)
            self.assertIn('height="24"', svg)

    def test_unknown_type_falls_back_to_other(self):
        out = _run_dock_js('console.log(getDeliveryHubIcon("nonexistent"));')
        other = _run_dock_js('console.log(getDeliveryHubIcon("other"));')
        self.assertEqual(out, other)

    # DOCK_TYPE_LABELS removed — labels now live in i18n locale files (en.json / hy.json)


if __name__ == "__main__":
    unittest.main()
