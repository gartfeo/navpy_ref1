"""Runs the Android shell's injected download script in Node.

DownloadBridge.pageScript builds the script the shell injects into the GCS
page. These tests rebuild it from the Java source and run it against a fake
page and a fake shell that answers like DownloadBridge.handle.
"""
import codecs
import json
import re
import shutil
import subprocess
import unittest
from pathlib import Path

_BRIDGE = (Path(__file__).resolve().parents[2] / "src" / "gcs" / "android" / "java"
           / "com" / "aas" / "gcs" / "DownloadBridge.java")

# Plays the shell: posts the channel after each attach the page does not
# refuse (as DownloadBridge.attach does), answers start/chunk/end, and
# re-injects the script in the middle of the first download, as a repeated
# onPageFinished does.
_HARNESS = r"""
const input = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const listeners = [];
global.window = {
  addEventListener: (type, fn) => listeners.push(fn),
  removeEventListener: (type, fn) => { const i = listeners.indexOf(fn); if (i >= 0) listeners.splice(i, 1); },
};
global.HTMLAnchorElement = class { click() { this.followed = true; } };
global.FileReader = class {
  readAsDataURL(blob) {
    blob.arrayBuffer().then((b) => {
      this.result = 'data:application/octet-stream;base64,' + Buffer.from(b).toString('base64');
      this.onload();
    });
  }
};

const messages = [];
const files = [];
let current = null;
let shellPort = null;
let reinjected = null;

function makePort() {
  const port = {
    postMessage(data) {
      messages.push(data.split('\n')[0]);
      setImmediate(() => answer(port, data));
    },
  };
  return port;
}

// Mirrors DownloadBridge.handle: only the newest port is served, and a piece
// with no file open is answered "error".
function answer(port, data) {
  if (port !== shellPort) return;
  let reply = 'error';
  if (data.startsWith('start\n')) {
    const [, type, name] = data.split('\n');
    current = { name, type, parts: [] };
    reply = 'next';
  } else if (data.startsWith('chunk\n') && current) {
    current.parts.push(Buffer.from(data.slice(6), 'base64'));
    reply = 'next';
    if (reinjected === null) reinjected = attach(input.second, 'token-2');
  } else if (data === 'end' && current) {
    files.push({ name: current.name, type: current.type, bytes: Buffer.concat(current.parts) });
    current = null;
    reply = 'done';
  }
  port.onmessage({ data: reply });
}

// Mirrors DownloadBridge.attach: a new channel drops the half-written file.
function attach(script, token) {
  const result = (0, eval)(script);
  if (result !== 'attached') {
    current = null;
    shellPort = makePort();
    for (const fn of listeners.slice()) fn({ data: 'not-the-token', ports: [makePort()] });
    for (const fn of listeners.slice()) fn({ data: token, ports: [shellPort] });
  }
  return result;
}

const size = 1200 * 1024 + 5;
const big = Buffer.alloc(size);
for (let i = 0; i < size; i++) big[i] = (i * 31 + 7) & 255;
const first = attach(input.first, 'token-1');

function link(name, blob) {
  const a = new HTMLAnchorElement();
  a.download = name;
  a.href = URL.createObjectURL(blob);
  a.click();
  return a;
}
link('big.bin', new Blob([big]));
link('small.txt', new Blob(['hi'], { type: 'text/plain' }));
const plain = new HTMLAnchorElement();
plain.href = 'http://127.0.0.1:8000/other';
plain.click();

const started = Date.now();
(function report() {
  if (files.length < 2 && Date.now() - started < 5000) return setTimeout(report, 10);
  console.log(JSON.stringify({
    first,
    second: reinjected,
    messages,
    plainFollowed: !!plain.followed,
    files: files.map((f) => ({
      name: f.name,
      type: f.type,
      size: f.bytes.length,
      content: f.name === 'big.bin' ? f.bytes.equals(big) : f.bytes.toString(),
    })),
  }));
})();
"""


def page_script(source, token):
    """Rebuilds the string DownloadBridge.pageScript(token) returns."""
    factors = re.search(r"static final int CHUNK_BYTES = ([^;]+);", source).group(1)
    chunk = 1
    for factor in factors.split("*"):
        chunk *= int(factor)
    body = source[source.index("static String pageScript(String token) {"):]
    body = body[body.index("return") + len("return"):body.index(";\n    }")]
    values = {"CHUNK_BYTES": str(chunk), "token": token}
    parts = []
    for literal, name in re.findall(r'"((?:[^"\\]|\\.)*)"|([A-Za-z_]\w*)', body):
        parts.append(codecs.decode(literal, "unicode_escape") if not name else values[name])
    return "".join(parts)


def run_harness(source):
    payload = json.dumps({
        "first": page_script(source, "token-1"),
        "second": page_script(source, "token-2"),
    })
    done = subprocess.run(["node", "-e", _HARNESS], input=payload, capture_output=True,
                          text=True, timeout=30, check=True)
    return json.loads(done.stdout)


@unittest.skipUnless(shutil.which("node"), "node is not installed")
class TestDownloadScript(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = run_harness(_BRIDGE.read_text(encoding="utf-8"))

    def test_files_arrive_whole_in_order(self):
        self.assertEqual(self.result["files"], [
            {"name": "big.bin", "type": "application/octet-stream", "size": 1200 * 1024 + 5,
             "content": True},
            {"name": "small.txt", "type": "text/plain", "size": 2, "content": "hi"},
        ])

    def test_each_piece_waits_for_the_shell(self):
        # 1200 KiB + 5 bytes in 512 KiB pieces is three chunks.
        self.assertEqual(self.result["messages"],
                         ["start", "chunk", "chunk", "chunk", "end", "start", "chunk", "end"])

    def test_a_repeated_injection_keeps_the_download_in_progress(self):
        """onPageFinished can fire again for the same document; a second
        channel used to drop the active download without a word."""
        self.assertEqual(self.result["first"], "waiting")
        self.assertEqual(self.result["second"], "attached")

    def test_the_shell_posts_no_second_channel_when_the_page_has_one(self):
        """The page's refusal is half the fix. A shell that ignored it would
        drop the file in progress and post a channel the page never takes."""
        attach = _BRIDGE.read_text(encoding="utf-8")
        attach = attach[attach.index("void attach(WebView web, Uri gcsOrigin) {"):]
        attach = attach[:attach.index("\n    }\n")]
        guard = re.search(r'if \("((?:[^"\\]|\\.)*)"\.equals\(result\)\) return;', attach)
        self.assertIsNotNone(guard, "attach must stop when the page already has a channel")
        # evaluateJavascript hands the script's result over JSON-encoded.
        self.assertEqual(codecs.decode(guard.group(1), "unicode_escape"),
                         json.dumps(self.result["second"]))
        self.assertLess(guard.start(), attach.index("createWebMessageChannel"))

    def test_other_links_are_followed(self):
        self.assertTrue(self.result["plainFollowed"])


if __name__ == "__main__":
    unittest.main()
