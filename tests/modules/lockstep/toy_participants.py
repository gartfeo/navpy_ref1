"""Toy participants for the 9d coordinator tests: real processes, product-free.

They run on the participant loop with the 9b bootstrap and baton, as NavPy
and the backend will: product threads are ``rt.Thread``, waits are ``rt``
waits, inputs arrive through ``deliver``. Imports stay within the bootstrap
rule (standard library, ``runtime_time``, ``navpy.sim.lockstep``).

- ``hub`` (backend-like, ``lockstep:serial1``): answers operator requests
  (``echo``; ``spawn`` and ``stop`` become requests in its ``DONE``, a stop is
  answered when its result arrives; ``slow`` waits for a cancel) and forwards
  every lifecycle record it gets as a ``lifecycle`` event.
- ``rover`` (NavPy-like, ``lockstep:serial0``): ready after a bootstrap delay,
  logs every 0.1 s, logs the terminal marker at ``snap_at_s``, stops on a
  lifecycle ``stop`` or the drain ``end``.

Both echo their port bytes reversed behind a tag, on each of their devices
(``devices``, default the class's one: devices are role data). Test knobs (``config``):
``ready_after_s``, ``ready_payload``, ``snap_at_s``, ``crash_before_ready``, ``ignore_stop``,
``ignore_end``, ``fail_flush``, ``exit_code``, ``exit_delay_s``,
``die_after_s``, ``stop_on_end``, ``report_env`` (log these variables),
``crash_message`` (the startup failure's text), ``end_egress_hex`` (bytes
written to the port on the drain ``end``). Either toy logs the terminal
marker at ``snap_at_s``.
"""

from __future__ import annotations

import os
import time

from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.part_wire import Event, Response, SpawnRequest, StopRequest
from navpy.sim.lockstep.participant_hooks import (
    Outbox,
    OutboxBuffer,
    ParticipantContext,
    ParticipantHooks,
)


class ToyLink:
    def __init__(self, tag: bytes) -> None:
        self._tag = tag
        self._inbox = rt.Queue()
        self._egress = bytearray()

    def deliver(self, data: bytes) -> None:
        if data:
            self._inbox.deliver(data)

    def write(self, data: bytes) -> None:
        self._egress += data

    def drain_egress(self) -> bytes:
        data = bytes(self._egress)
        self._egress.clear()
        return data

    def pump(self, stop) -> None:
        while not stop.is_set():
            try:
                data = self._inbox.get(timeout=0.05)
            except rt.Empty:
                continue
            self._egress += self._tag + data[::-1]


class ToyHooks(ParticipantHooks):
    DEVICE = ""
    TAG = b""
    PERIOD_S = 0.1

    def __init__(self, context: ParticipantContext) -> None:
        self.context = context
        self.config = dict(context.config)
        self.links = {device: ToyLink(self.TAG) for device in self.config.get("devices", [self.DEVICE])}
        #: The first device's link: drain-end egress goes there.
        self.link = next(iter(self.links.values()))
        self.out = OutboxBuffer()
        self.stop = rt.Event()
        self.threads: list = []
        self.snapped = False

    def construct(self) -> None:
        for name in self.config.get("report_env", ()):
            self.out.log_lines.append(f"env {name}={os.environ.get(name)}")
        pumps = [(lambda link=link: link.pump(self.stop), "toy-pump") for link in self.links.values()]
        for target, name in ((self.main, "toy-main"), *pumps):
            thread = rt.Thread(target=target, name=name)
            thread.start()
            self.threads.append(thread)

    def main(self) -> None:
        rt.sleep(self.config.get("ready_after_s", 0.0))
        if self.config.get("crash_before_ready"):
            raise RuntimeError(self.config.get("crash_message", "startup failed"))
        self.out.events.append(Event("runtime_ready", {"role": type(self).__name__.lower(),
                                                       **self.config.get("ready_payload", {})}))
        while not self.stop.wait(self.PERIOD_S):
            self.beat()
        self.stopped()

    def beat(self) -> None:
        self.out.log_lines.append(f"{type(self).__name__.lower()} t={rt.monotonic():.2f}")
        if not self.snapped and rt.monotonic() >= self.config.get("snap_at_s", 1e9):
            self.snapped = True
            self.out.log_lines.append(f"SNAP(VISION-NAV) at {rt.monotonic():.2f}")

    def stopped(self) -> None:
        """After the main loop ends: the product's cleanup."""

    def on_end(self) -> None:
        if not self.config.get("ignore_end"):
            self.stop.deliver_set()
        self.link.write(bytes.fromhex(self.config.get("end_egress_hex", "")))

    def finished(self) -> bool:
        return bool(self.threads) and not any(thread.is_alive() for thread in self.threads)

    def flushes(self):
        return [self._flush]

    def _flush(self) -> None:
        if self.config.get("fail_flush"):
            raise OSError("flush failed")
        self.out.log_lines.append(f"{type(self).__name__.lower()} flushed")

    def drain_outbox(self) -> Outbox:
        die_after = self.config.get("die_after_s")
        if die_after is not None and rt.monotonic() >= die_after:
            os._exit(5)  # an exit without an ending DONE
        return self.out.take()

    def exit_code(self, flushed: bool) -> int:
        time.sleep(self.config.get("exit_delay_s", 0.0))  # a host delay after the ending DONE
        return self.config.get("exit_code", 0 if flushed else 1)


class Rover(ToyHooks):
    DEVICE = "lockstep:serial0"
    TAG = b"R"

    def __init__(self, context: ParticipantContext) -> None:
        super().__init__(context)
        self.stop_id = None

    def on_lifecycle(self, entry) -> None:
        if entry.kind == "stop":
            self.stop_id = entry.request
            if not self.config.get("ignore_stop"):
                self.stop.deliver_set()

    def stopped(self) -> None:
        if self.stop_id is not None:
            self.out.events.append(Event("stopped", {"id": list(self.stop_id)}))


class Hub(ToyHooks):
    DEVICE = "lockstep:serial1"
    TAG = b"H"
    PERIOD_S = 0.25

    def __init__(self, context: ParticipantContext) -> None:
        super().__init__(context)
        self.inputs = rt.Queue()
        self.pending: dict = {}

    def construct(self) -> None:
        super().construct()
        worker = rt.Thread(target=self.work, name="toy-worker")
        worker.start()
        self.threads.append(worker)

    def on_request(self, request) -> None:
        self.inputs.deliver(("request", request))

    def on_cancel(self, request_id) -> None:
        self.inputs.deliver(("cancel", request_id))

    def on_lifecycle(self, entry) -> None:
        self.inputs.deliver(("lifecycle", entry))

    def on_end(self) -> None:
        super().on_end()
        if self.config.get("stop_on_end"):  # a stop requested in drain (input phase: no thread runs)
            self.out.stop_requests.append(StopRequest(self.context.ids.next(), 1, 1))

    def work(self) -> None:
        while not self.stop.is_set():
            try:
                kind, item = self.inputs.get(timeout=0.05)
            except rt.Empty:
                continue
            getattr(self, f"_{kind}")(item)

    def _request(self, request) -> None:
        payload = request.payload
        op = payload["op"]
        if op == "echo":
            self.out.responses.append(Response(request.id, {"echo": payload["text"]}))
        elif op == "spawn":
            spawn_id = self.context.ids.next()
            self.out.spawn_requests.append(SpawnRequest(spawn_id, payload["role"], payload["ordinal"]))
            self.out.responses.append(Response(request.id, {"spawn_id": list(spawn_id)}))
        elif op == "stop":
            stop_id = self.context.ids.next()
            self.pending[stop_id] = request.id
            self.out.stop_requests.append(StopRequest(stop_id, payload["ordinal"], payload["generation"]))
        elif op == "slow":
            self.pending[request.id] = request.id  # answered only by its cancel

    def _cancel(self, request_id) -> None:
        if self.pending.pop(request_id, None) is not None:
            self.out.responses.append(Response(request_id, {"cancelled": True}))

    def _lifecycle(self, entry) -> None:
        self.out.events.append(Event("lifecycle", {
            "kind": entry.kind, "request": None if entry.request is None else list(entry.request),
            "ordinal": entry.ordinal, "generation": entry.generation, "result": entry.result,
        }))
        if entry.kind == "stop_result" and entry.request in self.pending:
            self.out.responses.append(Response(self.pending.pop(entry.request), {"result": entry.result}))


def rover(context: ParticipantContext) -> Rover:
    return Rover(context)


def hub(context: ParticipantContext) -> Hub:
    return Hub(context)
