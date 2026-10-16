"""The SIYI command fence of the HITL terminal handoff (docs/design/hitl_final_approach.md sections 2 and 7).

Once closed, every SIYI frame that would move the gimbal, zoom, focus or change mode is dropped and counted at the
SDK's single send point (``sendMsg``, where every request and the SDK's own polling loops end), so a reconnect's
setup (centre, lock, zoom reset) is fenced too. Two kinds still pass:
- read-only queries (firmware, hardware id, gimbal info, attitude, zoom reading, attitude stream), so the readback
  keeps working;
- the two stops, zero rate (0x07 0,0) and zoom hold (0x05 0): a SIYI rate or manual-zoom command keeps acting until
  the next one, so a fence that only dropped could leave the gimbal turning. quiesce_and_stop() sends both once to
  every gimbal endpoint, bounded.
A dropped frame reports success to its caller, as the MAVLink fence does: the hold must not turn into errors in the
gimbal control path.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.gimbal.siyi.hardware.ports import SiyiSdkFactory, SiyiSdkPort
from navpy.plugins.gimbal.siyi.siyi_message import COMMAND

# Frame layout (siyi_message.encodeMsg): STX 0x5566, CTRL 1 byte, data length 2, sequence 2, command id 1, data, CRC 2.
_HEADER = bytes.fromhex("5566")
_CMD_INDEX = 7
_DATA_START = 8
_CRC_BYTES = 2

QUERY_COMMANDS = frozenset(int(cmd, 16) for cmd in (
    COMMAND.ACQUIRE_FW_VER,
    COMMAND.ACQUIRE_HW_ID,
    COMMAND.ACQUIRE_GIMBAL_INFO,
    COMMAND.FUNC_FEEDBACK_INFO,
    COMMAND.ACQUIRE_GIMBAL_ATT,
    COMMAND.CURRENT_ZOOM_VALUE,
    COMMAND.SET_DATA_STREAM,
))
STOP_FRAMES = frozenset({
    (int(COMMAND.GIMBAL_SPEED, 16), bytes(2)),   # yaw 0, pitch 0
    (int(COMMAND.MANUAL_ZOOM, 16), bytes(1)),    # zoom hold
})

StopResult = str   # "sent", "failed" (a send returned False or raised), "timeout" or "no_gimbal" (no SIYI SDK created)

# Bound on quiescence and the stop: a SIYI send is one UDP sendto to a fixed endpoint and returns at once on loopback or
# the camera LAN, so this only catches a stuck send (e.g. a hostname that does not resolve). It keeps the terminal
# record from waiting on the gimbal; the fence stays closed either way.
GIMBAL_STOP_TIMEOUT_S = 1.0


def passes_closed_fence(msg: str) -> bool:
    """True for a query or a stop; False for every other frame, including any that cannot be decoded."""
    try:
        frame = bytes.fromhex(msg)
    except (TypeError, ValueError):
        return False
    if len(frame) < _DATA_START + _CRC_BYTES or not frame.startswith(_HEADER):
        return False
    cmd = frame[_CMD_INDEX]
    data = frame[_DATA_START:-_CRC_BYTES]
    return cmd in QUERY_COMMANDS or (cmd, data) in STOP_FRAMES


@dataclass
class _Endpoint:
    """One gimbal endpoint: its latest SDK and the lock its sends run under (quiescence per endpoint)."""
    lock: rt.LockPort
    sdk: SiyiSdkPort


class SiyiCommandFence:
    """wrap() fences every SDK its factory creates; close() is safe under any lock; the stop is bounded."""

    def __init__(self, stop_timeout_s: float = GIMBAL_STOP_TIMEOUT_S) -> None:
        self._closed = rt.Event()
        self._lock = rt.Lock()                     # guards the endpoint table and the drop count only
        self._endpoints: dict[tuple[str, int], _Endpoint] = {}
        self._stop_timeout_s = float(stop_timeout_s)
        self.dropped = 0

    def wrap(self, factory: SiyiSdkFactory) -> SiyiSdkFactory:
        def create_fenced(*, server_ip: str, port: int) -> SiyiSdkPort:
            sdk = factory(server_ip=server_ip, port=port)
            self._install(sdk, (server_ip, port))
            return sdk

        return create_fenced

    def _install(self, sdk: SiyiSdkPort, key: tuple[str, int]) -> None:
        send: Callable[[str], bool] | None = getattr(sdk, "sendMsg", None)
        if not callable(send):   # fail closed: an SDK without the single send point cannot be fenced
            raise TypeError("SIYI command fence needs the SDK's sendMsg send point")
        with self._lock:
            endpoint = self._endpoints.get(key)
            if endpoint is None:
                endpoint = self._endpoints[key] = _Endpoint(rt.Lock(), sdk)
            else:
                endpoint.sdk = sdk                 # a reconnect: the same endpoint, its latest SDK
        send_lock = endpoint.lock

        def fenced_send(msg: str) -> bool:
            with send_lock:
                if self._closed.is_set() and not passes_closed_fence(msg):
                    with self._lock:
                        self.dropped += 1
                    return True
                return send(msg)

        sdk.sendMsg = fenced_send  # type: ignore[attr-defined]

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def close(self) -> None:
        self._closed.set()

    def quiesce_and_stop(self) -> StopResult:
        """Per endpoint, on its own daemon thread: wait for a send that passed the fence before close(), then send a
        zero rate and a zoom hold. One deadline for all ("timeout" if any endpoint misses it), so a stuck endpoint
        never keeps another from its stop."""
        with self._lock:
            endpoints = list(self._endpoints.values())
        if not endpoints:
            return "no_gimbal"
        results: list[bool] = []
        workers = [rt.Thread(target=lambda e=e: results.append(self._stop_one(e)), name="siyi-fence-stop",
                             daemon=True) for e in endpoints]
        for worker in workers:
            worker.start()
        deadline = rt.monotonic() + self._stop_timeout_s
        for worker in workers:
            worker.join(max(0.0, deadline - rt.monotonic()))
        if any(worker.is_alive() for worker in workers):
            return "timeout"
        return "sent" if all(results) else "failed"

    @staticmethod
    def _stop_one(endpoint: _Endpoint) -> bool:
        with endpoint.lock:       # quiescence: a send in progress at this endpoint holds its lock
            sdk = endpoint.sdk
        try:   # both stops pass the closed fence; the terminal record must still go out
            rate = sdk.requestGimbalSpeed(0, 0)
            zoom = sdk.requestZoomHold()
        except Exception:
            return False
        return rate is True and zoom is True


__all__ = ["GIMBAL_STOP_TIMEOUT_S", "QUERY_COMMANDS", "STOP_FRAMES", "SiyiCommandFence", "StopResult", "passes_closed_fence"]
