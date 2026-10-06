package com.aas.gcs;

import android.app.Activity;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import android.webkit.ValueCallback;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Full-screen shell for the handheld GCS.
 *
 * Shows the page the Termux backend serves at 127.0.0.1:8000 with the Android
 * status and navigation bars hidden from launch (Chrome allows that only after
 * a tap). A swipe from the edge shows the bars briefly.
 */
public class MainActivity extends Activity {
    private static final String GCS_ORIGIN = "http://127.0.0.1:8000";
    private static final Uri GCS = Uri.parse(GCS_ORIGIN);
    private static final String GCS_URL = GCS_ORIGIN + "/";
    private static final String HEALTH_URL = GCS_ORIGIN + "/health";
    private static final long RETRY_MS = 3000;
    private static final int HEALTH_TIMEOUT_MS = 2000;
    private static final int PICK_FILE = 1;
    private static final String WAITING_PAGE =
        "<html><body style='margin:0;height:100vh;display:flex;align-items:center;"
        + "justify-content:center;background:#0b1120;color:#9fb3c8;"
        + "font:18px sans-serif'>Waiting for the GCS backend at 127.0.0.1:8000"
        + "</body></html>";

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final ExecutorService healthProbe = Executors.newSingleThreadExecutor();
    private WebView web;
    private final Runnable loadWhenBackendUp = this::loadWhenBackendUp;
    private DownloadBridge downloads;
    private boolean showingWaitingPage;
    private boolean clearHistoryOnLoad;
    private ValueCallback<Uri[]> pendingFileChooser;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // A dozing screen would drop the operator out of the GCS mid-flight.
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getWindow().setDecorFitsSystemWindows(false);

        // Reachable only over adb, which already has full control of the device.
        WebView.setWebContentsDebuggingEnabled(true);
        web = new WebView(this);
        WebSettings settings = web.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setAllowFileAccess(false);
        settings.setMediaPlaybackRequiresUserGesture(false);
        downloads = new DownloadBridge(this);
        web.setWebViewClient(new ShellClient());
        web.setWebChromeClient(new ShellChromeClient());
        setContentView(web);

        loadWhenBackendUp();
    }

    // Loads the GCS only once the backend answers /health. A load error is not
    // enough: while the backend is down the WebView can still serve the page
    // from its cache, and the GCS then starts with its API calls failed.
    private void loadWhenBackendUp() {
        handler.removeCallbacks(loadWhenBackendUp);
        healthProbe.execute(() -> {
            boolean up = backendAnswers();
            handler.post(() -> {
                if (isDestroyed()) return;
                if (up) {
                    showingWaitingPage = false;
                    clearHistoryOnLoad = true;
                    web.loadUrl(GCS_URL);
                    return;
                }
                if (!showingWaitingPage) {
                    showingWaitingPage = true;
                    web.loadDataWithBaseURL(null, WAITING_PAGE, "text/html", "utf-8", null);
                }
                handler.postDelayed(loadWhenBackendUp, RETRY_MS);
            });
        });
    }

    private static boolean backendAnswers() {
        try {
            HttpURLConnection health = (HttpURLConnection) new URL(HEALTH_URL).openConnection();
            health.setConnectTimeout(HEALTH_TIMEOUT_MS);
            health.setReadTimeout(HEALTH_TIMEOUT_MS);
            health.setUseCaches(false);
            try {
                return health.getResponseCode() == HttpURLConnection.HTTP_OK;
            } finally {
                health.disconnect();
            }
        } catch (IOException e) {
            return false;
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        hideSystemBars();
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) hideSystemBars();
    }

    private void hideSystemBars() {
        WindowInsetsController bars = getWindow().getInsetsController();
        if (bars == null) return;
        bars.setSystemBarsBehavior(WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
        bars.hide(WindowInsets.Type.statusBars() | WindowInsets.Type.navigationBars());
    }

    @Override
    public void onBackPressed() {
        // Back never closes the GCS page; it only leaves the app, which keeps
        // the page and its connections alive.
        if (web.canGoBack()) {
            web.goBack();
        } else {
            moveTaskToBack(true);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == PICK_FILE && pendingFileChooser != null) {
            pendingFileChooser.onReceiveValue(
                WebChromeClient.FileChooserParams.parseResult(resultCode, data));
            pendingFileChooser = null;
            return;
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    @Override
    protected void onDestroy() {
        handler.removeCallbacksAndMessages(null);
        healthProbe.shutdownNow();
        downloads.close();
        web.destroy();
        super.onDestroy();
    }

    // Compares the parsed origin: a prefix test would also accept
    // "http://127.0.0.1:8000@localhost:8000/", which loads another origin.
    private static boolean isGcsPage(Uri url) {
        return GCS.getScheme().equals(url.getScheme())
            && GCS.getHost().equals(url.getHost())
            && GCS.getPort() == url.getPort();
    }

    private final class ShellClient extends WebViewClient {
        @Override
        public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
            Uri url = request.getUrl();
            if (isGcsPage(url)) return false;
            // Links elsewhere (map attribution, docs) open in the browser.
            try {
                startActivity(new Intent(Intent.ACTION_VIEW, url));
            } catch (ActivityNotFoundException ignored) {
                // Nothing can open it; stay on the GCS page.
            }
            return true;
        }

        @Override
        public void onPageFinished(WebView view, String url) {
            if (!isGcsPage(Uri.parse(url))) return;
            // Back must leave the app, not return to the waiting page.
            if (clearHistoryOnLoad) {
                clearHistoryOnLoad = false;
                view.clearHistory();
            }
            downloads.attach(view, GCS);
        }

        @Override
        public void onReceivedError(WebView view, WebResourceRequest request, WebResourceError error) {
            // The backend went away between the health check and the page load.
            if (request.isForMainFrame()) loadWhenBackendUp();
        }
    }

    private final class ShellChromeClient extends WebChromeClient {
        @Override
        public boolean onShowFileChooser(WebView view, ValueCallback<Uri[]> callback,
                                         FileChooserParams params) {
            if (pendingFileChooser != null) pendingFileChooser.onReceiveValue(null);
            pendingFileChooser = callback;
            try {
                startActivityForResult(params.createIntent(), PICK_FILE);
            } catch (ActivityNotFoundException e) {
                pendingFileChooser = null;
                return false;
            }
            return true;
        }
    }
}
