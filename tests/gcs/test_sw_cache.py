"""Tests for offline Cesium support: service worker and local asset config."""

import json
import pathlib

from tests.gcs.js_runner import run_node

FRONTEND = pathlib.Path(__file__).resolve().parents[2] / "src" / "gcs" / "frontend"

# Runs sw.js in Node with stub caches/fetch and dispatches one fetch event.
# Prints {kind, tag, fetched, put, unhandled}: kind is "response", "undefined"
# or "rejected" for what respondWith settled with.
_SW_HARNESS = r"""
const listeners = {};
globalThis.self = {
  addEventListener: (type, fn) => { listeners[type] = fn; },
  skipWaiting() {},
  clients: { claim() {} },
};
const S = SCENARIO;
const unhandled = [];
process.on('unhandledRejection', (e) => unhandled.push(String(e)));
const calls = { fetched: 0, put: 0 };
const fail = (what) => Promise.reject(new Error(`UnknownError: ${what}`));
const net = { tag: 'net', ok: true, clone() { return { tag: 'net-clone' }; } };
const cache = {
  match: () => (S.matchRejects ? fail('match') : Promise.resolve(S.cached ? { tag: 'cached' } : undefined)),
  put: () => { calls.put += 1; return S.putRejects ? fail('put') : Promise.resolve(); },
  keys: () => Promise.resolve([]),
  delete: () => Promise.resolve(true),
};
globalThis.caches = {
  open: () => (S.openRejects ? fail('open') : Promise.resolve(cache)),
  match: () => Promise.resolve(undefined),
  keys: () => Promise.resolve([]),
  delete: () => Promise.resolve(true),
};
globalThis.fetch = () => { calls.fetched += 1; return S.offline ? fail('offline') : Promise.resolve(net); };
""" + "SW_SOURCE" + r"""
(async () => {
  let responded = null;
  const waits = [];
  listeners.fetch({
    request: { url: S.url },
    respondWith: (p) => { responded = Promise.resolve(p); },
    waitUntil: (p) => { waits.push(p); },
  });
  let out;
  try {
    const r = await responded;
    if (r === undefined) out = { kind: 'undefined' };
    else out = { kind: 'response', tag: r.tag ?? r.type };
  } catch (e) {
    out = { kind: 'rejected' };
  }
  await Promise.allSettled(waits);
  await new Promise((r) => setTimeout(r, 20));
  console.log(JSON.stringify({ ...out, ...calls, unhandled }));
})();
"""

_CESIUM_URL = "http://127.0.0.1:8000/Cesium/Widgets/widgets.css"
_TILE_URL = "https://tile.openstreetmap.org/1/0/0.png"


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


def _dispatch(**scenario):
    """Run one fetch event through sw.js; return the harness's JSON result."""
    sw_source = (FRONTEND / "public" / "sw.js").read_text(encoding="utf-8")
    code = (
        _SW_HARNESS
        .replace("SCENARIO", json.dumps(scenario))
        .replace("SW_SOURCE", sw_source)
    )
    result = run_node(code, timeout=10)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout.strip().splitlines()[-1])


class TestServiceWorkerFetch:
    """Behaviour of the fetch handler (todo 118).

    A Chrome profile whose CacheStorage cannot be written makes caches.open
    reject ("Unexpected internal error"). The worker must then serve the file
    from the network, never answer undefined (a network error for every
    /Cesium/ file: no widgets.css, a 300x150 black map).
    """

    def test_static_cache_open_failure_falls_back_to_network(self):
        out = _dispatch(url=_CESIUM_URL, openRejects=True)
        assert out["kind"] == "response" and out["tag"] == "net", out
        assert out["unhandled"] == [], out

    def test_static_cache_match_failure_falls_back_to_network(self):
        out = _dispatch(url=_CESIUM_URL, matchRejects=True)
        assert out["kind"] == "response" and out["tag"] == "net", out
        assert out["unhandled"] == [], out

    def test_static_cache_hit_is_served_without_network(self):
        """Cache-first stays: the handheld serves Cesium offline from the cache."""
        out = _dispatch(url=_CESIUM_URL, cached=True, offline=True)
        assert out["kind"] == "response" and out["tag"] == "cached", out
        assert out["fetched"] == 0, out

    def test_static_cache_miss_fetches_and_stores(self):
        out = _dispatch(url=_CESIUM_URL)
        assert out["kind"] == "response" and out["tag"] == "net", out
        assert out["put"] == 1, out

    def test_static_put_failure_still_serves_and_is_handled(self):
        out = _dispatch(url=_CESIUM_URL, putRejects=True)
        assert out["kind"] == "response" and out["tag"] == "net", out
        assert out["unhandled"] == [], out

    def test_static_offline_miss_is_a_network_error_not_undefined(self):
        out = _dispatch(url=_CESIUM_URL, offline=True)
        assert out["kind"] != "undefined", out

    def test_tile_offline_miss_is_a_network_error_not_undefined(self):
        out = _dispatch(url=_TILE_URL, offline=True)
        assert out["kind"] != "undefined", out

    def test_tile_cache_failure_is_handled(self):
        out = _dispatch(url=_TILE_URL, openRejects=True)
        assert out["kind"] == "response" and out["tag"] == "net", out
        assert out["unhandled"] == [], out
