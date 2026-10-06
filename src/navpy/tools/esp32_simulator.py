#!/usr/bin/env python3
"""
ESP32 Launch Controller Simulator

Mimics the ESP32 HTTP API for testing the swarm launch system
without physical hardware.

Usage:
    python -m navpy.tools.esp32_simulator --port 8032

Then configure GCS to use http://localhost:8032 as ESP32 address.
"""

import argparse
import json
import threading
import time
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from typing import Optional, Callable

# Simulated channel configuration
NUM_CHANNELS = 6
TRIGGER_DURATION_MS = 500


class Esp32SimulatorState:
    """Shared state for the simulator."""

    def __init__(self):
        self.active_channel: Optional[int] = None
        self.trigger_log: list = []
        self.on_trigger_callback: Optional[Callable[[int], None]] = None
        self._lock = threading.Lock()

    def trigger_channel(self, channel: int) -> bool:
        """Simulate triggering a channel."""
        with self._lock:
            if channel < 1 or channel > NUM_CHANNELS:
                return False

            self.active_channel = channel
            timestamp = datetime.now().isoformat()
            self.trigger_log.append({
                "channel": channel,
                "timestamp": timestamp,
                "duration_ms": TRIGGER_DURATION_MS
            })
            print(f"[ESP32-SIM] Channel {channel} TRIGGERED at {timestamp}")

            # Notify callback (e.g., to simulate altitude change in GCS)
            if self.on_trigger_callback:
                self.on_trigger_callback(channel)

        # Simulate trigger duration
        time.sleep(TRIGGER_DURATION_MS / 1000.0)

        with self._lock:
            self.active_channel = None
            print(f"[ESP32-SIM] Channel {channel} trigger complete")

        return True

    def reset(self):
        """Reset all channels."""
        with self._lock:
            self.active_channel = None
            print("[ESP32-SIM] All channels reset")

    def get_status(self) -> dict:
        """Get current status."""
        with self._lock:
            return {
                "channels": list(range(1, NUM_CHANNELS + 1)),
                "active": self.active_channel,
                "ip": "127.0.0.1 (simulator)",
                "rssi": -50,
                "trigger_count": len(self.trigger_log)
            }


# Global state instance
_state = Esp32SimulatorState()


class Esp32RequestHandler(BaseHTTPRequestHandler):
    """HTTP request handler mimicking ESP32 API."""

    def log_message(self, format, *args):
        """Override to use custom logging format."""
        print(f"[ESP32-SIM] {args[0]}")

    def _send_json(self, status: int, data: dict):
        """Send JSON response."""
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _send_text(self, status: int, text: str):
        """Send plain text response."""
        self.send_response(status)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(text.encode())

    def _send_html(self, status: int, html: str):
        """Send HTML response."""
        self.send_response(status)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(html.encode())

    def do_GET(self):
        """Handle GET requests."""
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        if path == "/" or path == "":
            self._handle_root()
        elif path == "/health":
            self._handle_health()
        elif path == "/launch":
            self._handle_launch(query)
        elif path == "/status":
            self._handle_status()
        elif path == "/reset":
            self._handle_reset()
        elif path == "/log":
            self._handle_log()
        else:
            self._send_json(404, {"error": "Not found"})

    def _handle_root(self):
        """Serve web UI."""
        html = """
        <html>
        <head><title>ESP32 Simulator</title></head>
        <body style="font-family: monospace; background: #1a1a2e; color: #eee; padding: 20px;">
            <h1 style="color: #00ff88;">ESP32 Launch Controller (SIMULATOR)</h1>
            <p style="color: #ff8800;">This is a simulation - no hardware connected</p>
            <h2>Manual Trigger</h2>
            <div>
        """
        for i in range(1, NUM_CHANNELS + 1):
            html += f'''<a href="/launch?channel={i}">
                <button style="margin:5px; padding:10px 20px; font-size:16px;
                    background:#0066cc; color:white; border:none; cursor:pointer;">
                    Launch Ch {i}
                </button></a>'''
        html += """
            </div>
            <h2>Endpoints</h2>
            <ul>
                <li><a href="/health" style="color:#4a9eff;">/health</a> - Health check</li>
                <li><a href="/status" style="color:#4a9eff;">/status</a> - System status</li>
                <li><a href="/log" style="color:#4a9eff;">/log</a> - Trigger history</li>
                <li><a href="/reset" style="color:#4a9eff;">/reset</a> - Reset all channels</li>
            </ul>
        </body>
        </html>
        """
        self._send_html(200, html)

    def _handle_health(self):
        """Health check endpoint."""
        self._send_text(200, "OK")

    def _handle_launch(self, query: dict):
        """Handle launch trigger."""
        if "channel" not in query:
            self._send_json(400, {"error": "Missing channel parameter"})
            return

        try:
            channel = int(query["channel"][0])
        except (ValueError, IndexError):
            self._send_json(400, {"error": "Invalid channel parameter"})
            return

        if channel < 1 or channel > NUM_CHANNELS:
            self._send_json(400, {
                "error": f"Invalid channel. Must be 1-{NUM_CHANNELS}"
            })
            return

        # Trigger in background thread to not block response
        def do_trigger():
            _state.trigger_channel(channel)

        thread = threading.Thread(target=do_trigger, daemon=True)
        thread.start()

        # Respond immediately
        self._send_json(200, {
            "success": True,
            "channel": channel,
            "duration_ms": TRIGGER_DURATION_MS,
            "simulated": True
        })

    def _handle_status(self):
        """Get system status."""
        self._send_json(200, _state.get_status())

    def _handle_reset(self):
        """Reset all channels."""
        _state.reset()
        self._send_json(200, {"success": True, "message": "All channels reset"})

    def _handle_log(self):
        """Get trigger history."""
        self._send_json(200, {
            "triggers": _state.trigger_log[-50:],  # Last 50 triggers
            "total": len(_state.trigger_log)
        })


class Esp32Simulator:
    """ESP32 simulator server."""

    def __init__(self, port: int = 8032):
        self.port = port
        self.server: Optional[HTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def set_trigger_callback(self, callback: Callable[[int], None]):
        """Set callback for when a channel is triggered."""
        _state.on_trigger_callback = callback

    def start(self):
        """Start the simulator server."""
        self.server = HTTPServer(("0.0.0.0", self.port), Esp32RequestHandler)
        self._thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self._thread.start()
        print(f"[ESP32-SIM] Simulator running at http://localhost:{self.port}")

    def stop(self):
        """Stop the simulator server."""
        if self.server:
            self.server.shutdown()
            print("[ESP32-SIM] Simulator stopped")

    def get_state(self) -> Esp32SimulatorState:
        """Get the simulator state for integration."""
        return _state


def main():
    parser = argparse.ArgumentParser(description="ESP32 Launch Controller Simulator")
    parser.add_argument("--port", type=int, default=8032, help="Port to run on (default: 8032)")
    args = parser.parse_args()

    print("=" * 50)
    print("  ESP32 Launch Controller Simulator")
    print("=" * 50)
    print(f"  Channels: {NUM_CHANNELS}")
    print(f"  Trigger duration: {TRIGGER_DURATION_MS}ms")
    print("=" * 50)

    sim = Esp32Simulator(port=args.port)
    sim.start()

    print(f"\nOpen http://localhost:{args.port} in browser for Web UI")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        sim.stop()


if __name__ == "__main__":
    main()
