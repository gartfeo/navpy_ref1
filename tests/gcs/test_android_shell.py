"""Source-level checks for the full-screen Android shell (src/gcs/android)."""
import json
import re
import unittest
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
_ANDROID = _REPO / "src" / "gcs" / "android"
_BUILD = _REPO / "scripts" / "termux" / "build_apk.sh"


def _read(path):
    return path.read_text(encoding="utf-8")


class TestAndroidShellWiring(unittest.TestCase):
    def setUp(self):
        self.manifest = _read(_ANDROID / "AndroidManifest.xml")
        self.build = _read(_BUILD)
        self.activity = _read(_ANDROID / "java" / "com" / "aas" / "gcs" / "MainActivity.java")
        self.bridge = _read(_ANDROID / "java" / "com" / "aas" / "gcs" / "DownloadBridge.java")

    def test_launcher_activity_is_the_java_class(self):
        self.assertIn('package="com.aas.gcs"', self.manifest)
        self.assertIn('android:name=".MainActivity"', self.manifest)
        self.assertIn("package com.aas.gcs;", self.activity)
        self.assertIn("public class MainActivity extends Activity", self.activity)

    def test_shell_opens_the_backend_start_gcs_serves(self):
        start = _read(_REPO / "scripts" / "termux" / "start_gcs.sh")
        port = re.search(r'PORT="\$\{GCS_PORT:-(\d+)\}"', start).group(1)
        self.assertIn(f'GCS_ORIGIN = "http://127.0.0.1:{port}"', self.activity)

    def test_shell_loads_the_gcs_only_after_health_answers(self):
        """While the backend is down the WebView can serve the page from its
        cache, so a load error alone never shows the waiting page and the GCS
        starts with its API calls failed."""
        self.assertIn('HEALTH_URL = GCS_ORIGIN + "/health"', self.activity)
        self.assertIn("return health.getResponseCode() == HttpURLConnection.HTTP_OK;", self.activity)
        self.assertIn("if (up) {", self.activity)
        on_create = self.activity[self.activity.index("protected void onCreate"):]
        on_create = on_create[:on_create.index("\n    }\n")]
        self.assertIn("loadWhenBackendUp();", on_create)
        self.assertNotIn("web.loadUrl(GCS_URL)", on_create)

    def test_build_min_api_matches_the_manifest(self):
        min_sdk = re.search(r'android:minSdkVersion="(\d+)"', self.manifest).group(1)
        self.assertIn(f"--min-api {min_sdk}", self.build)

    def test_launcher_icon_comes_from_the_web_app(self):
        icon = re.search(r'ICON="\$REPO/([^"]+)"', self.build).group(1)
        self.assertTrue((_REPO / icon).is_file(), icon)
        self.assertIn("ic_launcher_foreground.png", self.build)
        adaptive = _read(_ANDROID / "res" / "mipmap-anydpi-v26" / "ic_launcher.xml")
        self.assertIn("@drawable/ic_launcher_foreground", adaptive)

    def test_launcher_background_matches_the_web_manifest(self):
        web = json.loads(_read(_REPO / "src" / "gcs" / "frontend" / "public" / "manifest.json"))
        colors = _read(_ANDROID / "res" / "values" / "colors.xml")
        self.assertIn(f'<color name="launcher_background">{web["background_color"]}</color>', colors)

    def test_downloads_reach_only_the_gcs_origin(self):
        """addJavascriptInterface exposes an object to every frame and to any
        page the WebView reaches, and a URL prefix test also accepted
        "http://127.0.0.1:8000@localhost:8000/", another origin (both seen on
        the handheld). The channel is posted to the exact GCS origin."""
        self.assertNotIn("addJavascriptInterface", self.activity)
        self.assertNotIn("startsWith(GCS_ORIGIN)", self.activity)
        self.assertIn(
            "web.postWebMessage(new WebMessage(token, new WebMessagePort[] {channel[1]}), gcsOrigin);",
            self.bridge,
        )
        self.assertIn("if (isGcsPage(url)) return false;", self.activity)
        on_finished = self.activity[self.activity.index("public void onPageFinished"):]
        self.assertIn("if (!isGcsPage(Uri.parse(url))) return;", on_finished)
        self.assertIn("downloads.attach(view, GCS);", on_finished)

    def test_downloads_go_one_acknowledged_chunk_at_a_time(self):
        """A whole file as one string needs several times its size in memory."""
        self.assertIn("reader.readAsDataURL(job.blob.slice(offset, offset + CHUNK));", self.bridge)
        self.assertIn("if (reply !== 'next') { state.onReply = null; state.sendNext(); return; }", self.bridge)
        self.assertIn('reply = "next";', self.bridge)

    def test_back_never_returns_to_the_waiting_page(self):
        """On the handheld, Back after a waiting-page start showed the stale
        waiting page instead of leaving the app."""
        load = self.activity[self.activity.index("if (up) {"):]
        self.assertIn("clearHistoryOnLoad = true;", load[:load.index("web.loadUrl(GCS_URL);")])
        self.assertIn("view.clearHistory();", self.activity)

    def test_build_script_keeps_lf_endings(self):
        self.assertNotIn(b"\r", _BUILD.read_bytes())
        self.assertIn("scripts/termux/*.sh text eol=lf", _read(_REPO / ".gitattributes"))


if __name__ == "__main__":
    unittest.main()
