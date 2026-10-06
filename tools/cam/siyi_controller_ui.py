#!/usr/bin/env python
"""
SIYI Gimbal Controller UI — lightweight tkinter control panel.

Controls:
  - Live RTSP video feed from the SIYI camera
  - Joystick pad: click-drag to command yaw/pitch rates
  - Zoom slider or +/- buttons
  - Mode buttons: FPV / Lock / Follow / Center
  - Photo & Record triggers
  - Live attitude readout

Usage:
    python tools/cam/siyi_controller_ui.py
    python tools/cam/siyi_controller_ui.py --ip 192.168.144.25 --port 37260
    python tools/cam/siyi_controller_ui.py --camera rtsp://192.168.144.25:8554/main.264
"""

from __future__ import annotations

import argparse
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_PAD_SIZE = 160
_PAD_HALF = _PAD_SIZE // 2
_DEADZONE = 4  # pixels from center before commanding rate
_POLL_MS = 50  # attitude refresh interval
_VIDEO_W = 640
_VIDEO_H = 360
_VIDEO_FPS = 30


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

class SiyiControllerApp:
    """Main application window."""

    def __init__(self, root: tk.Tk, ip: str, port: int, camera: str | None = None):
        self.root = root
        self.ip = ip
        self.port = port
        self.camera_url = camera  # None = auto from IP
        self.sdk = None
        self.connected = False
        self._stop = threading.Event()
        self._poll_thread = None
        self._commanding = False  # True while mouse is held on pad
        self._video_cap = None
        self._video_thread = None
        self._video_running = False
        self._tk_image = None  # prevent GC of PhotoImage

        root.title("SIYI Gimbal Controller")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()

    # ---- UI construction ---------------------------------------------------

    def _build_ui(self):
        # ---- Row 0: Connection bar ----
        conn = ttk.Frame(self.root, padding=(6, 4))
        conn.grid(row=0, column=0, columnspan=2, sticky="ew")

        ttk.Label(conn, text="IP:").pack(side="left")
        self.ip_var = tk.StringVar(value=self.ip)
        ttk.Entry(conn, textvariable=self.ip_var, width=14).pack(side="left", padx=2)

        ttk.Label(conn, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value=str(self.port))
        ttk.Entry(conn, textvariable=self.port_var, width=6).pack(side="left", padx=2)

        self.conn_btn = ttk.Button(conn, text="Connect", width=9, command=self._toggle_connection)
        self.conn_btn.pack(side="left", padx=4)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(conn, textvariable=self.status_var, foreground="gray").pack(side="left", padx=4)

        default_cam = self.camera_url or f"rtsp://{self.ip}:8554/main.264"
        self.camera_var = tk.StringVar(value=default_cam)
        self.video_btn = ttk.Button(conn, text="Video", width=6, command=self._toggle_video)
        self.video_btn.pack(side="right", padx=2)
        ttk.Entry(conn, textvariable=self.camera_var, width=30).pack(side="right", padx=2)

        # ---- Row 1 col 0: Video panel (pixel-sized via canvas) ----
        self.video_canvas = tk.Canvas(
            self.root, bg="black", width=_VIDEO_W, height=_VIDEO_H,
            highlightthickness=0,
        )
        self.video_canvas.grid(row=1, column=0, padx=(6, 2), pady=4, sticky="nsew")
        self._canvas_image_id = self.video_canvas.create_image(0, 0, anchor="nw")
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # ---- Row 1 col 1: Controls panel ----
        ctrl = ttk.Frame(self.root, padding=2)
        ctrl.grid(row=1, column=1, padx=(2, 6), pady=4, sticky="nsew")

        # Attitude
        att = ttk.LabelFrame(ctrl, text="Attitude", padding=3)
        att.pack(fill="x", pady=(0, 2))
        self.yaw_var = tk.StringVar(value="---")
        self.pitch_var = tk.StringVar(value="---")
        self.roll_var = tk.StringVar(value="---")
        self.zoom_display_var = tk.StringVar(value="---")
        for col, (label, var) in enumerate([
            ("Y", self.yaw_var), ("P", self.pitch_var),
            ("R", self.roll_var), ("Z", self.zoom_display_var),
        ]):
            ttk.Label(att, text=f"{label}:").grid(row=0, column=col * 2, sticky="e")
            ttk.Label(att, textvariable=var, width=6, anchor="w").grid(
                row=0, column=col * 2 + 1, padx=(1, 4),
            )

        # Joystick pad
        pad_frame = ttk.LabelFrame(ctrl, text="Rate", padding=2)
        pad_frame.pack(pady=2)
        self.pad = tk.Canvas(
            pad_frame, width=_PAD_SIZE, height=_PAD_SIZE,
            bg="#1a1a2e", highlightthickness=1, highlightbackground="#555",
        )
        self.pad.pack()
        self._draw_pad_bg()
        self.pad_dot = self.pad.create_oval(
            _PAD_HALF - 5, _PAD_HALF - 5, _PAD_HALF + 5, _PAD_HALF + 5,
            fill="#00cc66", outline="#00ff88",
        )
        self.pad.bind("<ButtonPress-1>", self._pad_press)
        self.pad.bind("<B1-Motion>", self._pad_drag)
        self.pad.bind("<ButtonRelease-1>", self._pad_release)
        self.rate_label_var = tk.StringVar(value="0, 0")
        ttk.Label(pad_frame, textvariable=self.rate_label_var, anchor="center").pack()

        # Zoom
        zoom_row = ttk.Frame(ctrl)
        zoom_row.pack(fill="x", pady=2)
        ttk.Button(zoom_row, text="-", width=2, command=lambda: self._step_zoom(-1)).pack(side="left")
        self.zoom_var = tk.DoubleVar(value=1.0)
        ttk.Scale(
            zoom_row, from_=1, to=30, variable=self.zoom_var,
            orient="horizontal", command=self._on_zoom_scale,
        ).pack(side="left", fill="x", expand=True, padx=2)
        ttk.Button(zoom_row, text="+", width=2, command=lambda: self._step_zoom(1)).pack(side="left")
        self.zoom_label_var = tk.StringVar(value="1x")
        ttk.Label(zoom_row, textvariable=self.zoom_label_var, width=4).pack(side="left", padx=2)

        # Modes
        mode_row = ttk.Frame(ctrl)
        mode_row.pack(fill="x", pady=2)
        for text, cmd in [("FPV", self._mode_fpv), ("Lock", self._mode_lock),
                          ("Follow", self._mode_follow), ("Center", self._center)]:
            ttk.Button(mode_row, text=text, width=6, command=cmd).pack(side="left", expand=True, fill="x", padx=1)

        # Set angles
        ang_row = ttk.Frame(ctrl)
        ang_row.pack(fill="x", pady=2)
        ttk.Label(ang_row, text="Yaw:").pack(side="left")
        self.set_yaw_var = tk.StringVar(value="0")
        ttk.Entry(ang_row, textvariable=self.set_yaw_var, width=5).pack(side="left", padx=1)
        ttk.Label(ang_row, text="Pitch:").pack(side="left")
        self.set_pitch_var = tk.StringVar(value="0")
        ttk.Entry(ang_row, textvariable=self.set_pitch_var, width=5).pack(side="left", padx=1)
        ttk.Button(ang_row, text="Go", width=3, command=self._set_angles).pack(side="left", padx=2)

        # Camera actions
        cam_row = ttk.Frame(ctrl)
        cam_row.pack(fill="x", pady=2)
        ttk.Button(cam_row, text="Photo", command=self._take_photo).pack(side="left", expand=True, fill="x", padx=1)
        self.rec_btn = ttk.Button(cam_row, text="Rec", command=self._toggle_record)
        self.rec_btn.pack(side="left", expand=True, fill="x", padx=1)
        ttk.Button(cam_row, text="AF", command=self._auto_focus).pack(side="left", expand=True, fill="x", padx=1)

    def _draw_pad_bg(self):
        """Draw crosshair and circle guides on the joystick pad."""
        c = self.pad
        mid = _PAD_HALF
        # Outer circle
        c.create_oval(10, 10, _PAD_SIZE - 10, _PAD_SIZE - 10, outline="#333", width=1)
        # Crosshair
        c.create_line(mid, 10, mid, _PAD_SIZE - 10, fill="#333", width=1)
        c.create_line(10, mid, _PAD_SIZE - 10, mid, fill="#333", width=1)
        # Labels
        c.create_text(mid, _PAD_SIZE - 4, text="Down", fill="#555", font=("Arial", 7))
        c.create_text(mid, 8, text="Up", fill="#555", font=("Arial", 7))
        c.create_text(6, mid, text="Left", fill="#555", font=("Arial", 7), angle=90)
        c.create_text(_PAD_SIZE - 6, mid, text="Right", fill="#555", font=("Arial", 7), angle=90)

    # ---- Pad events --------------------------------------------------------

    def _pad_press(self, event):
        self._commanding = True
        self._pad_move(event.x, event.y)

    def _pad_drag(self, event):
        if self._commanding:
            self._pad_move(event.x, event.y)

    def _pad_release(self, _event):
        self._commanding = False
        self._pad_move(_PAD_HALF, _PAD_HALF)
        self._send_rate(0, 0)

    def _pad_move(self, x, y):
        x = _clamp(x, 0, _PAD_SIZE)
        y = _clamp(y, 0, _PAD_SIZE)
        self.pad.coords(self.pad_dot, x - 6, y - 6, x + 6, y + 6)

        dx = x - _PAD_HALF
        dy = y - _PAD_HALF

        if abs(dx) < _DEADZONE:
            dx = 0
        if abs(dy) < _DEADZONE:
            dy = 0

        yaw_rate = int(_clamp(dx / _PAD_HALF * 100, -100, 100))
        pitch_rate = int(_clamp(-dy / _PAD_HALF * 100, -100, 100))

        self.rate_label_var.set(f"Rate: {yaw_rate:+d}, {pitch_rate:+d}")
        if self._commanding:
            self._send_rate(yaw_rate, pitch_rate)

    # ---- SDK commands ------------------------------------------------------

    def _send_rate(self, yaw_rate: int, pitch_rate: int):
        if self.sdk and self.connected:
            self.sdk.requestGimbalSpeed(yaw_rate, pitch_rate)

    def _on_zoom_scale(self, _value):
        zoom = round(self.zoom_var.get(), 1)
        self.zoom_label_var.set(f"{zoom:g}x")
        if self.sdk and self.connected:
            self.sdk.requestAbsoluteZoom(zoom)

    def _step_zoom(self, direction: int):
        current = self.zoom_var.get()
        new = _clamp(round(current + direction, 1), 1.0, 30.0)
        self.zoom_var.set(new)
        self.zoom_label_var.set(f"{new:g}x")
        if self.sdk and self.connected:
            self.sdk.requestAbsoluteZoom(new)

    def _mode_fpv(self):
        if self.sdk and self.connected:
            self.sdk.requestFPVMode()
            self._set_status("Mode: FPV")

    def _mode_lock(self):
        if self.sdk and self.connected:
            self.sdk.requestLockMode()
            self._set_status("Mode: Lock")

    def _mode_follow(self):
        if self.sdk and self.connected:
            self.sdk.requestFollowMode()
            self._set_status("Mode: Follow")

    def _center(self):
        if self.sdk and self.connected:
            self.sdk.requestCenterGimbal()
            self._set_status("Centering...")

    def _set_angles(self):
        if not (self.sdk and self.connected):
            return
        try:
            yaw = float(self.set_yaw_var.get())
            pitch = float(self.set_pitch_var.get())
        except ValueError:
            self._set_status("Invalid angle value")
            return
        self.sdk.requestSetAngles(yaw, pitch)
        self._set_status(f"Angles: {yaw:.1f}, {pitch:.1f}")

    def _take_photo(self):
        if self.sdk and self.connected:
            self.sdk.requestPhoto()
            self._set_status("Photo taken")

    def _toggle_record(self):
        if self.sdk and self.connected:
            self.sdk.requestRecording()
            self._set_status("Record toggled")

    def _auto_focus(self):
        if self.sdk and self.connected:
            self.sdk.requestAutoFocus()
            self._set_status("Auto focus")

    # ---- Video -------------------------------------------------------------

    def _toggle_video(self):
        if self._video_running:
            self._stop_video()
        else:
            self._start_video()

    def _start_video(self):
        source = self.camera_var.get().strip()
        if not source:
            self._set_status("No camera source")
            return
        # Allow integer sources (webcam index)
        try:
            source = int(source)
        except ValueError:
            pass
        self._video_running = True
        self.video_btn.configure(text="Stop Video")
        self._video_thread = threading.Thread(
            target=self._video_loop, args=(source,), daemon=True,
        )
        self._video_thread.start()

    def _stop_video(self):
        self._video_running = False
        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=3.0)
        self._video_thread = None
        if self._video_cap is not None:
            self._video_cap.release()
            self._video_cap = None
        self.video_btn.configure(text="Start Video")
        self._tk_image = None
        self.video_canvas.itemconfigure(self._canvas_image_id, image="")

    def _video_loop(self, source):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            self.root.after(0, self._set_status, "Failed to open video")
            self.root.after(0, self._stop_video)
            return

        self._video_cap = cap
        period = 1.0 / _VIDEO_FPS
        while self._video_running:
            start = time.time()
            ok, frame = cap.read()
            if not ok:
                continue
            # Resize to display size
            frame = cv2.resize(frame, (_VIDEO_W, _VIDEO_H))
            # BGR -> RGB -> PIL -> ImageTk
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            self.root.after(0, self._display_frame, img)

            elapsed = time.time() - start
            sleep_time = max(0.0, period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _display_frame(self, pil_image):
        if not self._video_running:
            return
        # Resize to current canvas size
        cw = self.video_canvas.winfo_width()
        ch = self.video_canvas.winfo_height()
        if cw > 1 and ch > 1:
            pil_image = pil_image.resize((cw, ch), Image.NEAREST)
        self._tk_image = ImageTk.PhotoImage(pil_image)
        self.video_canvas.itemconfigure(self._canvas_image_id, image=self._tk_image)

    # ---- Connection --------------------------------------------------------

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        ip = self.ip_var.get().strip()
        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            self._set_status("Invalid port")
            return

        self._set_status("Connecting...")
        self.conn_btn.configure(state="disabled")

        def do_connect():
            try:
                from navpy.modules.vision.peripheral.siyi import SIYISDK
                sdk = SIYISDK(server_ip=ip, port=port)
                ok = sdk.connect()
                self.root.after(0, lambda: self._on_connected(sdk, ok))
            except Exception as exc:
                self.root.after(0, lambda: self._on_connect_error(str(exc)))

        threading.Thread(target=do_connect, daemon=True).start()

    def _on_connected(self, sdk, ok):
        if ok:
            self.sdk = sdk
            self.connected = True
            self.conn_btn.configure(text="Disconnect", state="normal")
            cam = sdk.getCameraTypeString()
            self._set_status(f"Connected ({cam})" if cam else "Connected")
            self._start_polling()
        else:
            self.conn_btn.configure(state="normal")
            self._set_status("Connection failed")

    def _on_connect_error(self, msg):
        self.conn_btn.configure(state="normal")
        self._set_status(f"Error: {msg}")

    def _disconnect(self):
        self._stop_video()
        self._stop.set()
        if self._poll_thread and self._poll_thread.is_alive():
            self._poll_thread.join(timeout=2.0)
        if self.sdk:
            self.sdk.requestGimbalSpeed(0, 0)
            self.sdk.disconnect()
        self.sdk = None
        self.connected = False
        self.conn_btn.configure(text="Connect")
        self._set_status("Disconnected")
        self.yaw_var.set("---")
        self.pitch_var.set("---")
        self.roll_var.set("---")
        self.zoom_display_var.set("---")

    # ---- Attitude polling --------------------------------------------------

    def _start_polling(self):
        self._stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _poll_loop(self):
        while not self._stop.is_set():
            if self.sdk and self.connected:
                try:
                    yaw, pitch, roll = self.sdk.getAttitude()
                    zoom = self.sdk.getCurrentZoomLevel()
                    self.root.after(0, self._update_telemetry, yaw, pitch, roll, zoom)
                except Exception:
                    pass
            time.sleep(_POLL_MS / 1000.0)

    def _update_telemetry(self, yaw, pitch, roll, zoom):
        self.yaw_var.set(f"{yaw:.1f}\u00b0")
        self.pitch_var.set(f"{pitch:.1f}\u00b0")
        self.roll_var.set(f"{roll:.1f}\u00b0")
        if zoom is not None and zoom > 0:
            self.zoom_display_var.set(f"{zoom:g}x")

    # ---- Misc --------------------------------------------------------------

    def _set_status(self, msg: str):
        self.status_var.set(msg)

    def _on_close(self):
        self._stop_video()
        if self.connected:
            self._disconnect()
        self.root.destroy()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="SIYI Gimbal Controller UI")
    p.add_argument("--ip", default="192.168.144.25", help="SIYI gimbal IP (default: 192.168.144.25)")
    p.add_argument("--port", type=int, default=37260, help="SIYI gimbal UDP port (default: 37260)")
    p.add_argument("--camera", default=None, help="Video source (RTSP URL or webcam index, default: rtsp://{ip}:8554/main.264)")
    return p.parse_args()


def main():
    args = parse_args()
    root = tk.Tk()
    SiyiControllerApp(root, args.ip, args.port, camera=args.camera)
    root.mainloop()


if __name__ == "__main__":
    main()
