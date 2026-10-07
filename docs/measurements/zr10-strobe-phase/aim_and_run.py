#!/usr/bin/env python3
"""Locate LED live, then exec measure with ROI centered on it."""
import os
import sys
import time
import numpy as np
import cv2
import Jetson.GPIO as GPIO

out = sys.argv[1]
events = sys.argv[2] if len(sys.argv) > 2 else "400"
ontime = sys.argv[3] if len(sys.argv) > 3 else "0.003"

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT, initial=GPIO.HIGH)
cap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264", cv2.CAP_FFMPEG)
for _ in range(120):
    cap.read()
ok, f = cap.read()
GPIO.output(7, GPIO.LOW)
GPIO.cleanup()
cap.release()
red = f[:, :, 2]
ys, xs = np.nonzero(red >= 200)
if len(xs) < 20:
    raise SystemExit("LED not visible")
cx, cy = int(np.median(xs)), int(np.median(ys))
W, H = 400, 40
x = max(0, min(1280 - W, cx - W // 2))
y = max(0, min(720 - H, cy - H // 2))
print(f"LED at ({cx},{cy}) ROI ({x},{y},{W},{H})", flush=True)
os.execv("/home/pf/navpy/.venv/bin/python", [
    "python", "/home/pf/bench/zr10-latency-20260830/measure_zr10_latency.py",
    "--output", out, "--no-recenter",
    "--roi-x", str(x), "--roi-y", str(y), "--roi-w", str(W), "--roi-h", str(H),
    "--events", events, "--on-time", ontime,
    "--interval-min", "0.15", "--interval-max", "0.45",
])
