"""Tests for offline Cesium support: service worker and local asset config."""

import pathlib

FRONTEND = pathlib.Path(__file__).resolve().parents[2] / "src" / "gcs" / "frontend"


class TestServiceWorker:
    """Validate sw.js exists and contains expected cache patterns."""

    def setup_method(self):
        self.sw_path = FRONTEND / "public" / "sw.js"
        assert self.sw_path.exists(), f"sw.js not found at {self.sw_path}"
        self.sw_text = self.sw_path.read_text()

    def test_tile_patterns(self):
        """SW caches OSM, Bing, and Cesium Ion tile URLs."""
        assert "openstreetmap" in self.sw_text
        assert "virtualearth" in self.sw_text
        assert "ion" in self.sw_text and "cesium" in self.sw_text
        assert "api" in self.sw_text

    def test_static_cache_pattern(self):
        """SW caches local /Cesium/ assets."""
        assert "Cesium" in self.sw_text

    def test_cache_version(self):
        """SW defines a versioned cache name."""
        assert "CACHE_VERSION" in self.sw_text

    def test_eviction_limit(self):
        """SW enforces a max entry limit for tile cache."""
        assert "MAX_TILE_ENTRIES" in self.sw_text

    def test_network_first_strategy(self):
        """Tile requests use network-first with cache fallback."""
        assert "fetch(event.request)" in self.sw_text
        assert "caches.match(event.request)" in self.sw_text


class TestViteConfig:
    """Validate vite.config.js uses local Cesium paths."""

    def setup_method(self):
        self.config_path = FRONTEND / "vite.config.js"
        assert self.config_path.exists()
        self.config_text = self.config_path.read_text()

    def test_local_cesium_base_url(self):
        """CESIUM_BASE_URL points to local /Cesium/ not CDN."""
        assert "'/Cesium/'" in self.config_text
        assert "unpkg.com" not in self.config_text

    def test_static_copy_plugin(self):
        """vite-plugin-static-copy is configured."""
        assert "viteStaticCopy" in self.config_text
        assert "node_modules/cesium/Build/Cesium" in self.config_text


class TestCesiumMapLocalPaths:
    """Validate Cesium viewer hook uses local Cesium paths."""

    def setup_method(self):
        self.map_path = (
            FRONTEND / "src" / "components" / "map" / "hooks" / "useCesiumViewer.js"
        )
        assert self.map_path.exists()
        self.map_text = self.map_path.read_text()

    def test_css_local_path(self):
        """CSS loaded from /Cesium/ not CDN."""
        assert "/Cesium/Widgets/widgets.css" in self.map_text
        # No CDN reference for CSS
        assert "unpkg.com" not in self.map_text.split("widgets.css")[0].split("\n")[-1]

    def test_base_url_local(self):
        """CESIUM_BASE_URL set to /Cesium/ not CDN."""
        assert "self.CESIUM_BASE_URL = '/Cesium/'" in self.map_text


class TestMainJsxSwRegistration:
    """Validate main.jsx registers the service worker."""

    def setup_method(self):
        self.main_path = FRONTEND / "src" / "main.jsx"
        assert self.main_path.exists()
        self.main_text = self.main_path.read_text()

    def test_sw_registration(self):
        """main.jsx registers /sw.js."""
        assert "serviceWorker" in self.main_text
        assert "register" in self.main_text
        assert "/sw.js" in self.main_text
