# Handheld GCS app: WebView shell, Capacitor or React Native

Status: reviewed by Codex (two rounds, 2026-10-25). Owner chose A on 2026-10-25.

## Question

How to ship the GCS on the SIYI UniRC 10 Pro (Android 13, 1920x1200, no
Play Store listed). The GCS backend runs on the handheld (Termux) and serves
the built React/Vite/Cesium frontend at `http://127.0.0.1:8000`. A full
native UI rewrite is not wanted.

Options:

- **A. Existing shell** (`src/gcs/android`, package `com.aas.gcs`): about
  450 lines of Java. One activity with a platform `WebView` that loads
  `127.0.0.1:8000` once `/health` answers, hides the status and navigation
  bars, keeps the screen on, and has a download bridge and a file chooser.
  Built today only by `scripts/termux/build_apk.sh` (aapt2, javac, d8,
  apksigner; no Gradle).
- **B. Capacitor** (v8): a Gradle Android project that hosts the web UI in
  the platform WebView, with plugins for native features.
- **C. React Native** (0.87): React components rendered as native Android
  views.

## Facts that decide it

1. **Cesium is web-only.** It needs a DOM and WebGL. React Native renders
   native views: "At runtime, React Native creates the corresponding Android
   and iOS views" ([RN core components][rn-native]). So under C the map, and
   every panel in the same page, still runs in a WebView: the community
   `react-native-webview`, extracted from core ([react-native-webview][rnwv]),
   or Expo DOM components (SDK 56+ default to `@expo/dom-webview`, earlier
   SDKs use `react-native-webview`). Expo recommends native views where
   possible and DOM components for migration or web-suited content
   ([Expo DOM components][expo-dom]).
2. **All three use the Android System WebView.** None ships its own engine.
   Capacitor supports API 24+ and needs WebView/Chrome 60 or later
   ([Capacitor Android][cap-android]); `android.minWebViewVersion` can go to
   55 ([Capacitor config][cap-config]). So the Android 13 image's WebView 101
   limits A, B and C alike. Two known gaps: todo 113 (`dvh` used without a
   fallback in `App.jsx:601`, `:607`; Chromium added it in 108
   ([web.dev][webdev-vp])), and the build target: Vite 7's default target is
   Baseline Widely Available, Chrome 107 or later, with syntax transforms
   only, no polyfills ([Vite 7 build][vite-build]); `vite.config.js` sets no
   `build.target`. Whether the production bundle, Cesium workers and WebGL
   run on WebView 101 is not yet checked, nor the black landscape map in
   todo 113. The handheld's real WebView version is unknown until read on the
   device.
3. **Loading the on-device backend.** Capacitor's normal mode serves the
   bundled `webDir` build from inside the app at `https://localhost`
   (`server.androidScheme` default `https`, `server.hostname` default
   `localhost`; [config][cap-config], [workflow][cap-workflow]). Loading
   `127.0.0.1:8000` instead needs `server.url` and `server.cleartext`, which
   the docs mark "not intended for use in production" ([config][cap-config]).
   The bundled mode needs a configurable backend endpoint: the frontend
   builds its WebSocket from `window.location` (`telemetryStore.js:97-98`)
   and calls relative `/api/...` paths. The backend already allows any
   origin (`main.py:282-288`); an https page calling http loopback is not
   verified. Termux can then stop serving the frontend (`frontend_static.py`,
   `dist` None), or keep it; two copies of the UI are a deployment choice
   that can drift. A loads the backend's own page, so there is no separately
   bundled UI version; deployment and cache refresh (`public/sw.js`) still
   need care.
4. **Fullscreen and touch.** A hides both bars from launch with
   `WindowInsetsController`. Capacitor 8's bundled SystemBars plugin can hide
   the navigation bar and has a `hidden` start option; it notes that WebView
   below 140 can report wrong safe-area insets and offers `insetsHandling`
   for that ([SystemBars][cap-systembars]). Its Status Bar `overlaysWebView`
   no longer works for apps targeting Android 16
   ([Status Bar][cap-statusbar]). Web gestures stay the page's in all three;
   wrapper layout, gesture interception (react-native-webview changes parent
   interception for nested scrolling, [RNCWebView.java][rnwv-touch]),
   keyboard resizing and edge swipes need checking per wrapper.
5. **Offline.** With the local backend running, the UI and API work without
   internet in every option. Map coverage does not: OSM
   (`useCesiumViewer.js:167`) and Cesium Ion are remote; `public/sw.js` only
   caches tiles seen before (network first, up to 2000 entries). Mission-area
   imagery must be provisioned or shown cached by a cold-start offline check.
   Same for all options.
6. **Backend lifetime is shared work.** None of the options starts or
   supervises the Termux backend; A waits for it (`MainActivity.java:83-117`),
   `start_gcs.sh` takes a wake lock and runs uvicorn without a restart
   supervisor, and Android 12+ may kill such processes
   ([Termux README][termux]). This needs its own check whichever option wins.

## Comparison

| Criterion | A. Existing shell | B. Capacitor 8 | C. React Native 0.87 |
|---|---|---|---|
| Reuse of React + Cesium UI | Full, unchanged | Full; configurable HTTP/WS endpoint, or a non-production `server.url` | Only inside a WebView; native screens would be new work |
| Work to ship | Windows build script; todo 113 fix; WebView 101 check of the bundle and any fixes it needs | New Gradle project, config, endpoint change, plus the same WebView 101 work | New RN app, WebView wrapper, blob-download adaptation, RN toolchain, plus the same fixes |
| Loads `127.0.0.1:8000` | Yes, with `/health` wait | Only via `server.url` (dev-only per docs), or bundled + cross-origin | Yes, through the WebView |
| WebView 101 on Android 13 | Same engine | Same engine (min 60) | Same engine |
| Fullscreen, keep-on, back | Done (bars hidden; Back goes back in page history, else leaves the app) | SystemBars plugin; inset handling to check on old WebView | Libraries or native code |
| Downloads, file pick | Done (`DownloadBridge` incl. blob downloads, file chooser) | File chooser built in ([BridgeWebChromeClient][cap-chooser]); downloads, blob downloads via plugins or native code | File input and plain downloads built in ([guide][rnwv-guide]); blob downloads need adapting |
| Maintenance | ~450 lines of Java, no Gradle or npm in the app | Capacitor majors, Gradle, plugin versions | RN majors (e.g. 0.87: AGP 9, minCompileSdk 34 [RN 0.87][rn-087]), Gradle, native modules |

## Recommendation

**A, the existing shell,** as the implementation choice. It keeps the web UI
unchanged and served by its own backend, has the least to maintain, and B
and C run the same WebView, so they remove none of the handheld risks. It is
provisional on checks every option needs anyway: the production bundle,
Cesium and WebGL on WebView 101 (todo 113, build target), offline map
coverage, and the backend's lifetime on the device.

Revisit B or C if cross-platform builds, maintained plugins, or substantial
native screens become requirements.

Next steps under A: a PowerShell build script for this PC (SDK build-tools,
JDK 17), install on the UniRC emulator against the PC backend through an adb
port mapping (Android 13 and 16), and fix todo 113.

[rn-native]: https://reactnative.dev/docs/intro-react-native-components
[rnwv]: https://github.com/react-native-webview/react-native-webview
[rnwv-guide]: https://github.com/react-native-webview/react-native-webview/blob/master/docs/Guide.md#add-support-for-file-upload
[rnwv-touch]: https://github.com/react-native-webview/react-native-webview/blob/master/android/src/main/java/com/reactnativecommunity/webview/RNCWebView.java
[expo-dom]: https://docs.expo.dev/guides/dom-components/
[cap-android]: https://capacitorjs.com/docs/android
[cap-config]: https://capacitorjs.com/docs/config
[cap-workflow]: https://capacitorjs.com/docs/basics/workflow
[cap-systembars]: https://capacitorjs.com/docs/apis/system-bars
[cap-chooser]: https://github.com/ionic-team/capacitor/blob/8.0.0/android/capacitor/src/main/java/com/getcapacitor/BridgeWebChromeClient.java#L257-L284
[cap-statusbar]: https://capacitorjs.com/docs/apis/status-bar
[rn-087]: https://reactnative.dev/blog/2026/08/11/react-native-0.87
[webdev-vp]: https://web.dev/blog/viewport-units
[vite-build]: https://v7.vite.dev/guide/build
[termux]: https://github.com/termux/termux-app#readme
