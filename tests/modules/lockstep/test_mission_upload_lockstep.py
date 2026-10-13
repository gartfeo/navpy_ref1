"""Prototype: a multi-tick mission upload under both execution models.

The baton model runs the real ``MissionUploader`` and ``MissionInbox`` threads
unchanged (only their ``rt`` is the runtime's). The
synchronous model is a hand-written state machine stepped once per tick.
Both talk real MAVLink bytes to a reference autopilot that answers one tick
later, and both must produce the same per-tick byte journal.
"""

from __future__ import annotations

import hashlib
import logging
import random
import time as host_time
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.plugins.vehicle.ardupilot import mission_inbox, mission_upload
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_sync import BatonLock, BatonQueue, runtime_shim
from navpy.sim.lockstep.mavlink_wire import StreamDecoder, encoder

TICK_US = 20_000
GCS, AUTOPILOT = (255, 190), (1, 1)


class ReferenceAutopilot:
    """ArduPilot's mission receive side, reduced: one reply per message."""

    def __init__(self, *, ignore_counts: int = 0) -> None:
        self._link, self._sink = encoder(*AUTOPILOT)
        self._decoder = StreamDecoder()
        self._ignore_counts = ignore_counts
        self.count = 0
        self.items: list[int] = []

    def step(self, inbound: bytes) -> bytes:
        for message in self._decoder.feed(inbound):
            kind = message.get_type()
            if kind == "MISSION_COUNT":
                if self._ignore_counts:
                    self._ignore_counts -= 1
                    continue
                self.count, self.items = message.count, []
                self._link.mission_request_int_send(GCS[0], GCS[1], 0, 0)
            elif kind == "MISSION_ITEM_INT":
                self.items.append(message.seq)
                if message.seq + 1 < self.count:
                    self._link.mission_request_int_send(GCS[0], GCS[1], message.seq + 1, 0)
                else:
                    self._link.mission_ack_send(GCS[0], GCS[1], 0, 0)
        return self._sink.drain()


def _items(count: int) -> list[mavlink.MAVLink_mission_item_int_message]:
    return [
        mavlink.MAVLink_mission_item_int_message(
            1, 1, seq, 3, 16, 0, 1, 0.0, 0.0, 0.0, 0.0,
            400000000 + seq, 440000000, 40.0, 0,
        )
        for seq in range(count)
    ]


def run_baton_model(monkeypatch, count: int, *, ignore_counts=0, jitter=False):
    runtime = BatonRuntime()
    shim = runtime_shim(runtime)
    monkeypatch.setattr(mission_inbox, "rt", shim)
    monkeypatch.setattr(mission_upload, "rt", shim)
    link, sink = encoder(*GCS)
    inbox = mission_inbox.MissionInbox()
    uploader = mission_upload.MissionUploader(
        1, MavTransport(SimpleNamespace(mav=link), BatonLock(runtime, reentrant=True)),
        store=None, inbox=inbox, logger_ref=LoggerRef(logging.getLogger("lockstep")),
    )
    inbound = BatonQueue(runtime)
    decoder = StreamDecoder()
    result = {}

    def reader() -> None:  # the bus reader: decode and dispatch, in byte order
        while True:
            for message in decoder.feed(inbound.get()):
                inbox.publish(message, runtime.monotonic())

    def upload() -> None:
        result["ok"] = uploader.upload(timeout=0.2, items=_items(count))
        result["tick"] = runtime.now_us // TICK_US

    runtime.spawn(reader, name="reader")
    uploader_thread = runtime.spawn(upload, name="uploader")
    autopilot = ReferenceAutopilot(ignore_counts=ignore_counts)
    journal, pending = [], b""
    runtime.run_until_quiescent()
    for k in range(1, 400):
        runtime.tick(k * TICK_US, (lambda data=pending: inbound.deliver(data)) if pending else None)
        if jitter:
            host_time.sleep(random.random() / 5000)
        outbound = sink.drain()
        pending = autopilot.step(outbound)
        journal.append((k, outbound, pending))
        if uploader_thread.done:
            break
    alive = [thread.name for thread in runtime.threads.alive()]
    runtime.abort()  # unwind the parked reader; nothing outlives a run
    return result, journal, autopilot, alive


class SyncMissionUpload:
    """The same protocol as ``MissionUploader`` as an explicit tick step.

    It must copy the product's float deadline arithmetic (``now + timeout``
    in seconds) to match it: with integer deadlines a retry that the product's
    float drift puts after a tick would fire one tick early.
    """

    def __init__(self, count: int, timeout_s: float, retries: int = 3) -> None:
        self._link, self.sink = encoder(*GCS)
        self._items = _items(count)
        self._count, self._timeout_s = count, timeout_s
        self._retries, self._attempt = retries, 0
        self._deadline_s = 0.0
        self._state = "start"
        self.result: dict = {}

    def step(self, now_us: int, inbound: list) -> None:
        now_s = now_us / 1_000_000
        for message in inbound:  # inputs before timeouts
            if self._state in ("count", "items") and message.get_type() == "MISSION_REQUEST_INT":
                item = self._items[message.seq]
                self._link.mission_item_int_send(
                    1, 0, item.seq, item.frame, item.command, item.current,
                    item.autocontinue, item.param1, item.param2, item.param3,
                    item.param4, item.x, item.y, item.z, 0,
                )
                last = message.seq + 1 >= self._count
                self._state = "ack" if last else "items"
                self._deadline_s = now_s + (3.0 if last else self._timeout_s)
            elif self._state == "ack" and message.get_type() == "MISSION_ACK":
                self._finish(True, now_us)
        if self._state == "start" or (self._state == "count" and now_s >= self._deadline_s):
            if self._attempt == self._retries:
                return self._finish(False, now_us)
            self._attempt += 1
            self._link.mission_count_send(1, 0, self._count, 0)
            self._state, self._deadline_s = "count", now_s + self._timeout_s / 2
        elif self._state in ("items", "ack") and now_s >= self._deadline_s:
            self._finish(False, now_us)

    def _finish(self, ok: bool, now_us: int) -> None:
        self._state = "done"
        self.result = {"ok": ok, "tick": now_us // TICK_US}

    @property
    def done(self) -> bool:
        return self._state == "done"


def run_sync_model(count: int, *, ignore_counts=0):
    model = SyncMissionUpload(count, timeout_s=0.2)
    autopilot = ReferenceAutopilot(ignore_counts=ignore_counts)
    decoder = StreamDecoder()
    model.step(0, [])
    journal, pending = [], b""
    for k in range(1, 400):
        model.step(k * TICK_US, decoder.feed(pending))
        outbound = model.sink.drain()
        pending = autopilot.step(outbound)
        journal.append((k, outbound, pending))
        if model.done:
            break
    return model.result, journal, autopilot


def _digest(journal) -> str:
    digest = hashlib.sha256()
    for k, outbound, inbound in journal:
        digest.update(k.to_bytes(4, "little") + outbound + b"|" + inbound)
    return digest.hexdigest()


def test_the_real_uploader_spans_ticks_and_repeats_byte_for_byte(monkeypatch):
    result, journal, autopilot, alive = run_baton_model(monkeypatch, 10)
    # COUNT at tick 0, item n at tick n + 2 (one tick each way), ACK seen at 12.
    assert result == {"ok": True, "tick": 12}
    assert autopilot.items == list(range(10))
    repeat = run_baton_model(monkeypatch, 10, jitter=True)
    assert _digest(repeat[1]) == _digest(journal)
    assert alive == ["reader"]  # parked on future input when the upload ended


def test_a_lost_count_retries_at_the_timeout_tick(monkeypatch):
    result, _journal, autopilot, _runtime = run_baton_model(monkeypatch, 3, ignore_counts=1)
    # The first MISSION_COUNT is ignored; the retry goes out at deadline
    # 0 + timeout/2 = 100 ms = tick 5.
    assert result == {"ok": True, "tick": 9}
    assert autopilot.items == [0, 1, 2]


@pytest.mark.parametrize("ignore_counts", [0, 1, 3])
def test_the_synchronous_model_writes_the_same_bytes_at_the_same_ticks(monkeypatch, ignore_counts):
    baton = run_baton_model(monkeypatch, 5, ignore_counts=ignore_counts)
    sync = run_sync_model(5, ignore_counts=ignore_counts)
    assert sync[0] == baton[0]
    assert _digest(sync[1]) == _digest(baton[1])
