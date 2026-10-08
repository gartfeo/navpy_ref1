#!/usr/bin/env python3
"""Measure ZR10 optical-event-to-frame latency using GPIO and GStreamer.

All times ending in ``_mono_ns`` come from CLOCK_MONOTONIC.  RTP arrival is
observed at rtpjitterbuffer's sink: after kernel/TCP/RTSP demultiplexing but
before GStreamer's RTP reordering/buffering.
"""

import argparse
import ctypes
import fcntl
import json
import os
import platform
import random
import queue
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import gi
import numpy as np

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
gi.require_version("GstRtp", "1.0")
from gi.repository import GLib, Gst, GstRtp  # noqa: E402


GPIOHANDLE_REQUEST_OUTPUT = 1 << 1
GPIO_GET_LINEHANDLE_IOCTL = 0xC16CB403
GPIOHANDLE_SET_LINE_VALUES_IOCTL = 0xC040B409


class GpioHandleRequest(ctypes.Structure):
    _fields_ = [("lineoffsets", ctypes.c_uint32 * 64),
                ("flags", ctypes.c_uint32),
                ("default_values", ctypes.c_uint8 * 64),
                ("consumer_label", ctypes.c_char * 32),
                ("lines", ctypes.c_uint32), ("fd", ctypes.c_int)]


class GpioHandleData(ctypes.Structure):
    _fields_ = [("values", ctypes.c_uint8 * 64)]


class GPIO:
    def __init__(self, chip, line):
        chip_fd = os.open(chip, os.O_RDONLY | os.O_CLOEXEC)
        req = GpioHandleRequest()
        req.lineoffsets[0] = line
        req.flags = GPIOHANDLE_REQUEST_OUTPUT
        req.default_values[0] = 0
        req.consumer_label = b"zr10-latency"
        req.lines = 1
        try:
            fcntl.ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, req)
        finally:
            os.close(chip_fd)
        self.fd = req.fd

    def set(self, value):
        data = GpioHandleData()
        data.values[0] = int(bool(value))
        before = time.monotonic_ns()
        fcntl.ioctl(self.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, data)
        after = time.monotonic_ns()
        return before, after

    def close(self):
        if self.fd is not None:
            try:
                self.set(0)
            finally:
                os.close(self.fd)
                self.fd = None


class Jsonl:
    def __init__(self, path):
        self.path = path
        self.fp = open(path, "a", encoding="utf-8", buffering=1024 * 1024)
        self.queue = queue.SimpleQueue()
        self.thread = threading.Thread(target=self._writer, name="jsonl-writer", daemon=True)
        self.thread.start()

    def _writer(self):
        while True:
            row = self.queue.get()
            if row is None:
                return
            self.fp.write(json.dumps(row, separators=(",", ":")) + "\n")

    def write(self, kind, **fields):
        self.queue.put({"event": kind, **fields})

    def close(self):
        self.queue.put(None)
        self.thread.join()
        self.fp.flush()
        os.fsync(self.fp.fileno())
        self.fp.close()


def valid_ts(value):
    return None if value == Gst.CLOCK_TIME_NONE else int(value)


class Measurement:
    def __init__(self, args):
        self.a = args
        self.log = Jsonl(args.output)
        self.gpio = GPIO(args.gpio_chip, args.gpio_line)
        self.stop = threading.Event()
        self.loop = GLib.MainLoop()
        self.lock = threading.Lock()
        self.completed_rtp = deque()
        self.au_by_pts = {}
        self.decode_by_pts = {}
        self.rtp_frames = {}
        self.frame_index = 0
        self.last_led = None
        self.probed_jitterbuffers = set()
        self.baseline_scores = deque(maxlen=120)
        self.pipeline = None

    def build(self):
        Gst.init(None)
        desc = (
            f'rtspsrc name=src location="{self.a.rtsp}" protocols=tcp '
            'latency=0 buffer-mode=none drop-on-latency=true do-retransmission=false '
            '! rtph265depay name=depay '
            '! nvv4l2decoder name=decoder disable-dpb=true enable-max-performance=true '
            'num-extra-surfaces=0 '
            '! nvvidconv name=convert '
            '! video/x-raw,format=BGRx '
            '! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )
        self.pipeline = Gst.parse_launch(desc)
        src = self.pipeline.get_by_name("src")
        src.connect("deep-element-added", self.on_deep_element)
        self.pipeline.get_by_name("depay").get_static_pad("src").add_probe(
            Gst.PadProbeType.BUFFER, self.on_au)
        self.pipeline.get_by_name("decoder").get_static_pad("src").add_probe(
            Gst.PadProbeType.BUFFER, self.on_decode)
        self.pipeline.get_by_name("sink").connect("new-sample", self.on_sample)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_bus)
        versions = {
            "python": platform.python_version(), "gstreamer": Gst.version_string(),
            "glib": ".".join(map(str, GLib.glib_version)), "numpy": np.__version__,
            "kernel": platform.release(),
        }
        self.log.write("run_start", mono_ns=time.monotonic_ns(), argv=sys.argv,
                       pipeline=desc, roi=[self.a.roi_x, self.a.roi_y,
                                           self.a.roi_w, self.a.roi_h],
                       threshold=self.a.threshold, versions=versions,
                       rtp_arrival_definition=("rtpjitterbuffer sink probe; after "
                           "kernel TCP receive and RTSP interleaved demux, before jitter buffer"),
                       random_seed=self.a.seed)

    def recenter(self, reason="startup"):
        if not self.a.recenter:
            return
        command = [str(self.a.siyi_python), str(self.a.recenter_helper),
                   "--ip", self.a.siyi_ip, "--port", str(self.a.siyi_port),
                   "--settle", str(self.a.recenter_settle)]
        started = time.monotonic_ns()
        completed = subprocess.run(command, cwd=self.a.navpy_root,
                                   env={**os.environ, "PYTHONPATH": str(self.a.navpy_root / "src")},
                                   text=True, capture_output=True, timeout=20)
        ended = time.monotonic_ns()
        self.log.write("gimbal_recenter", reason=reason,
                       start_mono_ns=started, end_mono_ns=ended,
                       returncode=completed.returncode,
                       stdout=completed.stdout.strip(), stderr=completed.stderr.strip())
        if completed.returncode:
            raise RuntimeError(f"gimbal recenter failed: {completed.stderr.strip()}")

    def on_deep_element(self, _bin, _sub_bin, element):
        factory = element.get_factory()
        if factory and factory.get_name() == "rtpjitterbuffer":
            pad = element.get_static_pad("sink")
            identity = element.get_name()
            if identity not in self.probed_jitterbuffers:
                self.probed_jitterbuffers.add(identity)
                pad.add_probe(Gst.PadProbeType.BUFFER, self.on_rtp)
                self.log.write("rtp_probe_attached", mono_ns=time.monotonic_ns(),
                               element=element.get_name())

    def on_rtp(self, _pad, info):
        now = time.monotonic_ns()
        buf = info.get_buffer()
        if not buf:
            return Gst.PadProbeReturn.OK
        ok, rtp = GstRtp.RTPBuffer.map(buf, Gst.MapFlags.READ)
        if not ok:
            self.log.write("rtp_map_error", mono_ns=now)
            return Gst.PadProbeReturn.OK
        try:
            ts, seq, marker = int(rtp.get_timestamp()), int(rtp.get_seq()), bool(rtp.get_marker())
            payload_len = int(rtp.get_payload_len())
        finally:
            rtp.unmap()
        with self.lock:
            f = self.rtp_frames.get(ts)
            if f is None:
                f = {"rtp_timestamp": ts, "first_seq": seq, "last_seq": seq,
                     "first_packet_mono_ns": now, "last_packet_mono_ns": now,
                     "packet_count": 0, "payload_bytes": 0}
                self.rtp_frames[ts] = f
            f["last_seq"] = seq
            f["last_packet_mono_ns"] = now
            f["packet_count"] += 1
            f["payload_bytes"] += payload_len
            if marker:
                done = dict(f)
                self.rtp_frames.pop(ts, None)
                self.completed_rtp.append(done)
        self.log.write("rtp_packet", arrival_mono_ns=now, rtp_timestamp=ts,
                       sequence=seq, marker=marker, payload_bytes=payload_len)
        if marker:
            self.log.write("rtp_frame", **done)
        return Gst.PadProbeReturn.OK

    def on_au(self, _pad, info):
        now = time.monotonic_ns()
        buf = info.get_buffer()
        if not buf:
            return Gst.PadProbeReturn.OK
        pts, dts = valid_ts(buf.pts), valid_ts(buf.dts)
        with self.lock:
            rtp = self.completed_rtp.popleft() if self.completed_rtp else None
            self.au_by_pts[pts] = {"au_mono_ns": now, "rtp": rtp}
        self.log.write("access_unit", mono_ns=now, pts_ns=pts, dts_ns=dts,
                       size_bytes=buf.get_size(), rtp_frame=rtp)
        return Gst.PadProbeReturn.OK

    def on_decode(self, _pad, info):
        now = time.monotonic_ns()
        buf = info.get_buffer()
        if not buf:
            return Gst.PadProbeReturn.OK
        pts, dts = valid_ts(buf.pts), valid_ts(buf.dts)
        with self.lock:
            au = self.au_by_pts.pop(pts, None)
            self.decode_by_pts[pts] = {"decoder_output_mono_ns": now, "au": au}
        self.log.write("decoder_output", mono_ns=now, pts_ns=pts, dts_ns=dts,
                       source=au)
        return Gst.PadProbeReturn.OK

    def on_sample(self, sink):
        publish = time.monotonic_ns()
        sample = sink.emit("pull-sample")
        buf, caps = sample.get_buffer(), sample.get_caps()
        pts, dts = valid_ts(buf.pts), valid_ts(buf.dts)
        s = caps.get_structure(0)
        width, height = s.get_value("width"), s.get_value("height")
        ok, mapped = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            stride = mapped.size // height
            image = np.ndarray((height, stride // 4, 4), dtype=np.uint8,
                               buffer=mapped.data)[:, :width, :]
            x, y, w, h = self.a.roi_x, self.a.roi_y, self.a.roi_w, self.a.roi_h
            roi = image[y:y+h, x:x+w, :3]
            # BGRx: red dominance rejects the white monitor and ambient changes.
            red = roi[:, :, 2].astype(np.int16)
            other = np.maximum(roi[:, :, 1], roi[:, :, 0]).astype(np.int16)
            score = float(np.percentile(red - other, 98))
            red_p95 = float(np.percentile(red, 95))
        finally:
            buf.unmap(mapped)
        led = bool(score >= self.a.threshold and red_p95 >= self.a.min_red)
        with self.lock:
            source = self.decode_by_pts.pop(pts, None)
            idx = self.frame_index
            self.frame_index += 1
        event = "frame"
        self.log.write(event, frame_index=idx, publication_mono_ns=publish,
                       pts_ns=pts, dts_ns=dts, width=width, height=height,
                       roi_score=score, roi_red_p95=red_p95, led_on=led,
                       source=source)
        if self.last_led is None or led != self.last_led:
            self.log.write("led_observation_transition", frame_index=idx,
                           publication_mono_ns=publish, led_on=led, pts_ns=pts,
                           source=source, roi_score=score, roi_red_p95=red_p95)
            self.last_led = led
        return Gst.FlowReturn.OK

    def on_bus(self, _bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            self.log.write("pipeline_error", mono_ns=time.monotonic_ns(),
                           error=str(err), debug=debug)
            self.stop.set(); self.loop.quit()
        elif msg.type == Gst.MessageType.EOS:
            self.log.write("pipeline_eos", mono_ns=time.monotonic_ns())
            self.stop.set(); self.loop.quit()

    def stimulus(self):
        rng = random.Random(self.a.seed)
        if self.stop.wait(self.a.warmup):
            return
        for event_id in range(self.a.events):
            if event_id and self.a.recenter_every and event_id % self.a.recenter_every == 0:
                self.recenter(reason=f"before_event_{event_id}")
            if self.stop.wait(rng.uniform(self.a.interval_min, self.a.interval_max)):
                break
            pre, post = self.gpio.set(1)
            self.log.write("gpio", event_id=event_id, value=1,
                           pre_write_mono_ns=pre, post_write_mono_ns=post)
            if self.stop.wait(self.a.on_time):
                break
            pre, post = self.gpio.set(0)
            self.log.write("gpio", event_id=event_id, value=0,
                           pre_write_mono_ns=pre, post_write_mono_ns=post)
        self.stop.set()
        GLib.idle_add(self.loop.quit)

    def run(self):
        self.recenter()
        self.build()
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("pipeline failed to enter PLAYING")
        worker = threading.Thread(target=self.stimulus, name="stimulus", daemon=True)
        worker.start()
        try:
            self.loop.run()
        finally:
            self.stop.set(); worker.join(timeout=2)
            self.pipeline.set_state(Gst.State.NULL)
            self.gpio.close()
            self.log.write("run_end", mono_ns=time.monotonic_ns(), frames=self.frame_index)
            self.log.close()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--rtsp", default="rtsp://192.168.144.25:8554/main.264")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--events", type=int, default=300)
    p.add_argument("--warmup", type=float, default=3.0)
    p.add_argument("--interval-min", type=float, default=0.35)
    p.add_argument("--interval-max", type=float, default=0.95)
    p.add_argument("--on-time", type=float, default=0.18)
    p.add_argument("--seed", type=int, default=20260830)
    p.add_argument("--gpio-chip", default="/dev/gpiochip0")
    p.add_argument("--gpio-line", type=int, default=144)
    p.add_argument("--recenter", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--recenter-settle", type=float, default=4.0)
    p.add_argument("--recenter-every", type=int, default=50,
                   help="recenter between this many LED pulses; 0 disables periodic recenter")
    p.add_argument("--siyi-ip", default="192.168.144.25")
    p.add_argument("--siyi-port", type=int, default=37260)
    p.add_argument("--navpy-root", type=Path, default=Path("/home/pf/navpy"))
    p.add_argument("--siyi-python", type=Path, default=Path("/home/pf/navpy/.venv/bin/python"))
    p.add_argument("--recenter-helper", type=Path,
                   default=Path("/home/pf/zr10_latency/recenter_siyi.py"))
    p.add_argument("--roi-x", type=int, default=800)
    p.add_argument("--roi-y", type=int, default=290)
    p.add_argument("--roi-w", type=int, default=140)
    p.add_argument("--roi-h", type=int, default=130)
    p.add_argument("--threshold", type=float, default=80.0)
    p.add_argument("--min-red", type=float, default=120.0)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    m = Measurement(args)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: (m.stop.set(), m.loop.quit()))
    m.run()
