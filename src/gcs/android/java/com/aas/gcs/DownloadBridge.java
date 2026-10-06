package com.aas.gcs;

import android.app.Activity;
import android.content.ContentResolver;
import android.content.ContentValues;
import android.net.Uri;
import android.os.Handler;
import android.os.HandlerThread;
import android.provider.MediaStore;
import android.util.Base64;
import android.webkit.WebMessage;
import android.webkit.WebMessagePort;
import android.webkit.WebView;
import android.widget.Toast;

import java.io.IOException;
import java.io.OutputStream;
import java.util.UUID;

/**
 * Saves files the page offers as downloads.
 *
 * The GCS saves plans, parameter files and logs through
 * {@code <a download href="blob:...">}. A WebView cannot download blob URLs,
 * so the page script sends the bytes over a message channel instead, and they
 * are written to Download/AAS GCS.
 *
 * The channel is posted only to a document whose origin is the GCS, so no
 * other site or frame gets a way to write files.
 */
final class DownloadBridge {
    private static final String FOLDER = "Download/AAS GCS";
    // One chunk is all the page and the shell hold of a file at a time.
    static final int CHUNK_BYTES = 512 * 1024;

    private final Activity activity;
    private final HandlerThread ioThread = new HandlerThread("aas-gcs-downloads");
    private final Handler io;
    // Touched only on the io thread.
    private WebMessagePort port;
    private Uri file;
    private OutputStream out;
    private String name;

    DownloadBridge(Activity activity) {
        this.activity = activity;
        ioThread.start();
        io = new Handler(ioThread.getLooper());
    }

    /** Offers the save channel to the document just loaded in {@code web}; call on the UI thread. */
    void attach(WebView web, Uri gcsOrigin) {
        String token = UUID.randomUUID().toString();
        web.evaluateJavascript(pageScript(token), result -> {
            // onPageFinished can repeat for one document (a fragment change,
            // for example); a second channel would drop its download.
            if ("\"attached\"".equals(result)) return;
            WebMessagePort[] channel = web.createWebMessageChannel();
            WebMessagePort ours = channel[0];
            ours.setWebMessageCallback(new WebMessagePort.WebMessageCallback() {
                @Override
                public void onMessage(WebMessagePort from, WebMessage message) {
                    handle(from, message.getData());
                }
            }, io);
            io.post(() -> {
                abort();
                if (port != null) port.close();
                port = ours;
            });
            // Delivered only if the document's origin is exactly gcsOrigin.
            web.postWebMessage(new WebMessage(token, new WebMessagePort[] {channel[1]}), gcsOrigin);
        });
    }

    void close() {
        io.post(() -> {
            abort();
            if (port != null) port.close();
            port = null;
        });
        ioThread.quitSafely();
    }

    // The page sends "start\n<type>\n<name>", then "chunk\n<base64>" pieces,
    // then "end". The shell answers each with "next", "done" or "error", and
    // the page sends the next piece only after "next".
    private void handle(WebMessagePort from, String data) {
        if (from != port || data == null) return;
        String reply;
        try {
            if (data.startsWith("chunk\n")) {
                if (out == null) throw new IOException("no file is open");
                out.write(Base64.decode(data.substring(6), Base64.DEFAULT));
                reply = "next";
            } else if (data.startsWith("start\n")) {
                String[] parts = data.split("\n", 3);
                if (parts.length != 3) throw new IOException("the page sent no file name");
                abort();
                open(parts[2], parts[1]);
                reply = "next";
            } else if (data.equals("end")) {
                finish();
                reply = "done";
            } else {
                return;
            }
        } catch (IOException | IllegalArgumentException e) {
            toast("Could not save " + name + ": " + e.getMessage());
            abort();
            reply = "error";
        }
        from.postMessage(new WebMessage(reply));
    }

    private void open(String fileName, String mimeType) throws IOException {
        name = fileName;
        ContentValues values = new ContentValues();
        values.put(MediaStore.MediaColumns.DISPLAY_NAME, fileName);
        values.put(MediaStore.MediaColumns.MIME_TYPE, mimeType);
        values.put(MediaStore.MediaColumns.RELATIVE_PATH, FOLDER);
        values.put(MediaStore.MediaColumns.IS_PENDING, 1);
        ContentResolver resolver = activity.getContentResolver();
        file = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values);
        if (file == null) throw new IOException("the Downloads folder refused the file");
        out = resolver.openOutputStream(file);
        if (out == null) throw new IOException("cannot open " + file);
    }

    private void finish() throws IOException {
        if (out == null) throw new IOException("no file is open");
        out.close();
        out = null;
        ContentValues done = new ContentValues();
        done.put(MediaStore.MediaColumns.IS_PENDING, 0);
        activity.getContentResolver().update(file, done, null, null);
        file = null;
        toast("Saved to " + FOLDER + "/" + name);
    }

    // Drops a half-written file: after an error, or when the page went away
    // in the middle of a download.
    private void abort() {
        if (out != null) {
            try {
                out.close();
            } catch (IOException ignored) {
                // The file is deleted next anyway.
            }
            out = null;
        }
        if (file != null) {
            activity.getContentResolver().delete(file, null, null);
            file = null;
        }
    }

    private void toast(String message) {
        activity.runOnUiThread(() -> Toast.makeText(activity, message, Toast.LENGTH_LONG).show());
    }

    // Remembers each Blob by its object URL (the page may revoke the URL right
    // after click()), then sends a download link's Blob over the channel
    // instead of following the link. Downloads queue until the channel
    // arrives and go one at a time.
    static String pageScript(String token) {
        return "(function (token) {\n"
            + "  var CHUNK = " + CHUNK_BYTES + ";\n"
            + "  var state = window.__aasGcsDownloads;\n"
            + "  if (!state) {\n"
            + "    state = window.__aasGcsDownloads = { port: null, queue: [], onReply: null };\n"
            + "    var blobs = new Map();\n"
            + "    var create = URL.createObjectURL, revoke = URL.revokeObjectURL;\n"
            + "    URL.createObjectURL = function (obj) {\n"
            + "      var url = create.call(URL, obj);\n"
            + "      if (obj instanceof Blob) blobs.set(url, obj);\n"
            + "      return url;\n"
            + "    };\n"
            + "    URL.revokeObjectURL = function (url) { blobs.delete(url); revoke.call(URL, url); };\n"
            + "    var click = HTMLAnchorElement.prototype.click;\n"
            + "    HTMLAnchorElement.prototype.click = function () {\n"
            + "      var blob = this.download && blobs.get(this.href);\n"
            + "      if (!blob) return click.call(this);\n"
            + "      state.queue.push({ name: this.download, blob: blob });\n"
            + "      if (!state.onReply) state.sendNext();\n"
            + "    };\n"
            + "    state.sendNext = function () {\n"
            + "      if (!state.port || !state.queue.length) { state.onReply = null; return; }\n"
            + "      var job = state.queue.shift(), port = state.port, offset = 0;\n"
            + "      state.onReply = function (reply) {\n"
            + "        if (reply !== 'next') { state.onReply = null; state.sendNext(); return; }\n"
            + "        if (offset >= job.blob.size) { port.postMessage('end'); return; }\n"
            + "        var reader = new FileReader();\n"
            + "        reader.onload = function () {\n"
            + "          var data = reader.result;\n"
            + "          port.postMessage('chunk\\n' + data.slice(data.indexOf(',') + 1));\n"
            + "        };\n"
            + "        reader.readAsDataURL(job.blob.slice(offset, offset + CHUNK));\n"
            + "        offset += CHUNK;\n"
            + "      };\n"
            + "      port.postMessage('start\\n' + (job.blob.type || 'application/octet-stream')\n"
            + "        + '\\n' + job.name);\n"
            + "    };\n"
            + "  }\n"
            + "  if (state.port || state.waiting) return 'attached';\n"
            + "  state.waiting = true;\n"
            + "  window.addEventListener('message', function listen(event) {\n"
            + "    if (event.data !== token || !event.ports || event.ports.length !== 1) return;\n"
            + "    window.removeEventListener('message', listen);\n"
            + "    var port = event.ports[0];\n"
            + "    port.onmessage = function (m) {\n"
            + "      if (state.port === port && state.onReply) state.onReply(m.data);\n"
            + "    };\n"
            + "    state.port = port;\n"
            + "    state.waiting = false;\n"
            + "    if (!state.onReply) state.sendNext();\n"
            + "  });\n"
            + "  return 'waiting';\n"
            + "})('" + token + "');";
    }
}
