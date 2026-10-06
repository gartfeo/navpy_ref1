"""Regression test for loading-overlay dismissal timing in the Cesium map.

Guards Codex finding P2 #2 (PR #92): the "AAS LOADING" overlay must NOT be
dismissed on Ion imagery-*provider* resolution. `IonImageryProvider.fromAssetId`
resolving proves only that provider metadata loaded — not that any tile IMAGE was
fetched or is renderable. Flipping `tilesReady` there dismissed the overlay to a
blank/dark map on slow or failing Ion networks, and bypassed the bounded
LOADING_FALLBACK_MS safety net (which cannot help once tilesReady has latched).

`tilesReady` may be flipped only by the `tileLoadProgressEvent` drain watcher
(fires once the tile load queue drains after doing work — imagery renderable),
with CesiumMap's bounded LOADING_FALLBACK_MS as the backstop.

Source-level checks — the hook imports React/Cesium so it can't be loaded into
plain Node; we assert on source text (same approach as
test_terrain_ready_marker_hooks_js.py).
"""
import os
import re
import unittest


def _read_map_file(*parts):
    path = os.path.normpath(os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "gcs", "frontend", "src", "components", "map",
        *parts,
    ))
    with open(path, encoding="utf-8") as f:
        return f.read()


def _compact(src):
    return re.sub(r"\s+", " ", src)


_VIEWER_SRC = _read_map_file("hooks", "useCesiumViewer.js")
_MAP_SRC = _read_map_file("CesiumMap.jsx")


def _ion_upgrade_block(src):
    """Return the async Ion upgrade IIFE body (imagery + terrain swap)."""
    start = src.index(
        "const ionImagery = await Cesium.IonImageryProvider.fromAssetId(3)"
    )
    end = src.index("Cesium Ion upgrade failed", start)  # inside the catch's warn
    return src[start:end]


class TestTilesReadyDismiss(unittest.TestCase):
    def test_drain_watcher_is_the_only_flip_site(self):
        """tilesReady is flipped only by the tileLoadProgressEvent drain watcher."""
        compact = _compact(_VIEWER_SRC)
        # Watcher registered on the globe tile-load progress event.
        self.assertIn(
            "viewer.scene.globe.tileLoadProgressEvent.addEventListener(onTileProgress)",
            compact,
        )
        # The watcher flips tilesReady only after tiles drain to 0 (imagery loaded).
        self.assertIn(
            "if (!sawTiles || cancelled) return; "
            "viewer.scene.globe.tileLoadProgressEvent.removeEventListener(onTileProgress); "
            "setTilesReady(true);",
            compact,
        )
        # Exactly one flip site in the whole hook — no premature latch elsewhere.
        self.assertEqual(
            _VIEWER_SRC.count("setTilesReady(true)"), 1,
            "tilesReady must be flipped from exactly one place (the drain watcher)",
        )

    def test_ion_provider_add_does_not_flip_tiles_ready(self):
        """The Ion imagery swap must NOT latch tilesReady on provider-add."""
        block = _ion_upgrade_block(_VIEWER_SRC)
        # The imagery swap itself is intact...
        self.assertIn("viewer.imageryLayers.removeAll()", block)
        self.assertIn("viewer.imageryLayers.addImageryProvider(ionImagery)", block)
        # ...but it must not touch tilesReady (that would be the P2 #2 regression).
        self.assertNotIn(
            "setTilesReady", block,
            "Ion upgrade must not flip tilesReady — let the drain watcher do it",
        )

    def test_terrain_upgrade_still_flips_terrain_ready(self):
        """Terrain path is independent and unchanged (does not gate dismissal)."""
        self.assertEqual(_VIEWER_SRC.count("setTerrainReady(true)"), 1)
        self.assertIn("setTerrainReady(true)", _ion_upgrade_block(_VIEWER_SRC))

    def test_bounded_fallback_backstop_remains(self):
        """CesiumMap keeps the bounded fallback that dismisses if paint never fires."""
        compact = _compact(_MAP_SRC)
        self.assertIn("const LOADING_FALLBACK_MS = 8000;", _MAP_SRC)
        # Overlay dismisses immediately once tiles paint...
        self.assertIn(
            "if (tilesReady) { onViewerReady(true); return undefined; }", compact,
        )
        # ...otherwise a bounded timer is the backstop.
        self.assertIn(
            "const fallback = setTimeout(() => onViewerReady(true), LOADING_FALLBACK_MS);",
            compact,
        )
        self.assertIn("[viewerReady, tilesReady, onViewerReady]", compact)


if __name__ == "__main__":
    unittest.main()
