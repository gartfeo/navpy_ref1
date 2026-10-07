#!/usr/bin/env python3
"""Wait until LED centroid is stable, then exec the fused aim+measure."""
import os
import sys
import time
import numpy as np
import cv2
import Jetson.GPIO as GPIO

out = sys.argv[1]
events = sys.argv[2] if len(sys.argv) > 2 else "250"
ontime = sys.argv[3] if len(sys.argv) > 3 else "0.003"

def centroid(cap):
    GPIO.output(7, GPIO.HIGH)
    time.sleep(0.3)
    for _ in range(8):
        cap.read()
    ok, f = cap.read()
    GPIO.output(7, GPIO.LOW)
    if not ok:
        return None
    red = f[:, :, 2]
    ys, xs = np.nonzero(red >= 200)
    if len(xs) < 2000:
        return None
    return int(np.median(xs)), int(np.median(ys))

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT, initial=GPIO.LOW)
cap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264", cv2.CAP_FFMPEG)
for _ in range(120):
    cap.read()

deadline = time.monotonic() + 600
prev = None
while time.monotonic() < deadline:
    cur = centroid(cap)
    print(f"centroid {cur}", flush=True)
    if cur and prev and abs(cur[0] - prev[0]) < 15 and abs(cur[1] - prev[1]) < 10:
        break
    prev = cur
    t0 = time.monotonic()
    while time.monotonic() - t0 < 20:
        cap.read()   # keep decoder warm
else:
    raise SystemExit("never stabilized")

GPIO.cleanup()
cap.release()
os.execv("/home/pf/navpy/.venv/bin/python", [
    "python", "/home/pf/bench/aim_and_run.py", out, events, ontime,
])
