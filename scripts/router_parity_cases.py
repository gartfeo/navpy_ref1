"""Router parity cases: the datagrams ``router_parity.py`` sends into each router endpoint.

``lockstep_fleet.md``, Router parity. Each step from an endpoint is followed
by that endpoint's fence (see ``router_parity_wsl.py``).
"""

from __future__ import annotations

from collections.abc import Callable

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.mavlink_wire import encoder

ENDPOINTS = ("sitl1", "sitl2", "sitl3", "backend")
#: A fence packet's source: a sysid per sending endpoint, used by nothing else.
FENCE_SYSID = 200
FENCE_COMPID = 199
#: The cases recorded with the SITL endpoints' filters; the others have no filter.
FILTERED_CASES = ("filtered_broadcast", "filtered_targeted", "filtered_unknown_id",
                  "filtered_mixed", "filtered_image", "filtered_garbage")

Step = tuple[str, bytes, bool]


class _Sources:
    """One encoder per MAVLink source, so sequence numbers run as on a real link."""

    def __init__(self) -> None:
        self._links: dict[tuple[int, int], tuple[mavlink.MAVLink, object]] = {}

    def __call__(self, system: int, component: int, send: str, *args: object) -> bytes:
        if (system, component) not in self._links:
            self._links[(system, component)] = encoder(system, component)
        link, sink = self._links[(system, component)]
        getattr(link, f"{send}_send")(*args)
        return sink.drain()


def _case(steps: list[tuple[str, bytes]]) -> list[Step]:
    """Each step from endpoint E, then E's fence."""
    fences = _Sources()
    out: list[Step] = []
    for endpoint, data in steps:
        out.append((endpoint, data, False))
        index = ENDPOINTS.index(endpoint)
        fence = fences(FENCE_SYSID + index, FENCE_COMPID, "heartbeat", 6, 8, 0, 0, 0)
        out.append((endpoint, fence, True))
    return out


def _unknown_navlink(system: int, component: int, msgid: int = 25500) -> bytes:
    """A MAVLink 2 frame whose id (25500 by default) is in neither the entry table nor the dialect."""
    payload = bytes(range(1, 9))
    header = bytes((0xFD, len(payload), 0, 0, 0, system, component)) + msgid.to_bytes(3, "little")
    return header + payload + b"\x12\x34"


def build_cases() -> dict[str, list[Step]]:
    m = _Sources()
    hb = lambda system, component: m(system, component, "heartbeat", 1, 3, 0, 0, 0)  # noqa: E731
    param = lambda src, system, component: m(  # noqa: E731
        *src, "param_request_read", system, component, b"X", -1)
    learn = [("sitl1", hb(1, 1)), ("sitl2", hb(2, 1)), ("sitl3", hb(3, 1)), ("backend", hb(255, 190))]
    assign = lambda src, target: m(  # noqa: E731
        *src, "task_assign_request", 7, 1, 1000, 5000, target, 4, 1, 2, 1, 2, 3)
    ack = lambda src, target: m(*src, "swarm_ack", 7, 2, 1000, 5000, target, 0, 0, 0, 0)  # noqa: E731
    heartbeat = hb(1, 1)
    corrupt = bytearray(hb(2, 1))
    corrupt[-1] ^= 0xFF
    v1 = bytes((0xFE, 9, 0, 3, 1, 0)) + bytes(9)
    return {
        "broadcast": _case([*learn, ("sitl1", hb(1, 1) + hb(1, 191)), ("sitl3", v1 + _checksum_v1(v1))]),
        "targeted": _case([*learn, ("backend", param((255, 190), 2, 1)), ("backend", param((255, 190), 2, 0)),
                           ("backend", param((255, 190), 2, 5)), ("sitl2", param((2, 1), 1, 1))]),
        "sniffer": _case([*learn, ("sitl1", param((1, 1), 2, 1)), ("sitl3", param((3, 1), 9, 1))]),
        "unknown_target": _case([("sitl1", hb(1, 1)), ("backend", param((255, 190), 9, 1)),
                                 ("backend", hb(255, 190)), ("sitl2", param((2, 1), 9, 1))]),
        "targeted_navlink": _case([*learn, ("sitl1", hb(1, 191)), ("sitl1", assign((1, 191), 2)),
                                   ("sitl2", ack((2, 191), 1)), ("sitl2", ack((2, 191), 0)),
                                   ("sitl1", assign((1, 191), 9)), ("sitl3", _unknown_navlink(3, 191))]),
        "boot_text": _case([("sitl1", b"Init ArduPlane V4.6.0\r\nFree RAM: 4096\r\n" + heartbeat),
                            ("sitl2", b"APM: no marker here\r\n"), ("sitl2", bytes(corrupt) + hb(2, 1)),
                            ("backend", hb(255, 190))]),
        "split_packet": _split_case(hb, assign),
        "ap_echo": _case([*learn, ("sitl1", hb(1, 191)), ("sitl2", hb(1, 191)),
                          ("sitl3", assign((1, 191), 2)), ("backend", hb(1, 191))]),
        **_filtered_cases(m, hb, param, learn, assign, ack),
    }


def _filtered_cases(m: _Sources, hb: Callable[..., bytes], param: Callable[..., bytes],
                    learn: list[tuple[str, bytes]], assign: Callable[..., bytes],
                    ack: Callable[..., bytes]) -> dict[str, list[Step]]:
    """The cases recorded with the SITL endpoints' filters."""
    command = lambda src, system, component, number: m(  # noqa: E731
        *src, "command_long", system, component, number, 0, 0, 0, 0, 0, 0, 0, 0)
    command_ack = lambda src, system, component, number: m(  # noqa: E731
        *src, "command_ack", number, 0, 0, 0, system, component)
    param_set = m(255, 190, "param_set", 1, 1, b"X", 1.0, 9)
    param_value = m(1, 1, "param_value", b"X", 1.0, 9, 1, 0)
    camera_info = m(2, 100, "camera_information", 0, b"V" * 32, b"M" * 32, 0, 0.0, 0.0, 0.0, 0, 0, 0,
                    0, 0, b"")
    request = mavlink.MAV_CMD_REQUEST_MESSAGE
    learn_more = [*learn, ("sitl1", hb(1, 191)), ("sitl2", hb(2, 191)), ("sitl2", hb(2, 100))]
    # A confirmation image as NavPy sends it (image_transfer.py): no target field, so a broadcast.
    handshake = lambda system: m(system, 191, "data_transmission_handshake",  # noqa: E731
                                 mavlink.MAVLINK_DATA_STREAM_IMG_JPEG, 300, 64, 48, 2, 253, 50)
    chunk = lambda system, seq: m(system, 191, "encapsulated_data", seq, bytes(range(seq, seq + 253)))  # noqa: E731
    return {
        "filtered_broadcast": _case([*learn, ("sitl1", hb(1, 1)), ("sitl1", hb(1, 191)),
                                     ("sitl1", hb(1, 100)), ("sitl1", hb(1, 101)),
                                     ("backend", hb(255, 190)), ("backend", hb(255, 0))]),
        "filtered_targeted": _case([*learn_more,
                                    ("sitl1", param((1, 1), 2, 1)),  # filtered at sitl2, sniffed by the backend
                                    ("backend", command((255, 190), 2, 1, 400)),
                                    ("sitl2", command_ack((2, 1), 255, 190, 400)),
                                    ("backend", param_set), ("sitl1", param_value),
                                    ("backend", command((255, 190), 2, 100, request)),
                                    ("sitl2", camera_info),
                                    ("sitl2", command_ack((2, 100), 255, 190, request))]),
        "filtered_unknown_id": _case([*learn, ("sitl1", _unknown_navlink(1, 1)),
                                      ("sitl1", _unknown_navlink(1, 191)),
                                      ("sitl2", _unknown_navlink(2, 100))]),
        "filtered_mixed": _case([*learn_more,
                                 ("sitl1", param((1, 1), 9, 1)),  # filtered and rejected: not unknown
                                 ("backend", param((255, 190), 9, 1)),  # rejected everywhere: unknown
                                 ("sitl3", hb(1, 1)),  # sitl1's source from sitl3: loop, filter, accept
                                 ("sitl1", assign((1, 191), 2)), ("sitl1", assign((1, 191), 9)),
                                 ("sitl2", ack((2, 191), 0)), ("sitl2", param((2, 100), 3, 1)),
                                 # Every endpoint has seen (1, 1): the loop check rejects before
                                 # the filter, so the packet counts as unknown.
                                 ("sitl2", hb(1, 1)), ("backend", param((1, 1), 9, 1))]),
        # The image reaches only the backend; the same companion's NAVLINK still reaches the peers.
        "filtered_image": _case([*learn_more, ("sitl1", handshake(1)), ("sitl1", chunk(1, 0)),
                                 ("sitl1", chunk(1, 1)), ("sitl2", handshake(2)), ("sitl2", chunk(2, 0)),
                                 ("sitl1", assign((1, 191), 0)), ("sitl3", chunk(3, 0))]),
        # SITL byte loss (todo 96): a frame missing its last byte runs into the next; garbage with
        # an unknown id and a known id with a bad CRC, both from the GCS's (255, 0), must not
        # teach a SITL endpoint that source, so the GCS's packets to that UAV still arrive. Two
        # sysids on one feed both still learn.
        "filtered_garbage": _case([*learn_more, ("backend", hb(255, 0)),
                                   ("sitl2", hb(2, 1)[:-1] + hb(2, 1)),
                                   ("sitl2", _unknown_navlink(255, 0, 207)),
                                   ("sitl3", _bad_crc(hb(255, 0))), ("sitl1", hb(7, 1)),
                                   ("backend", param((255, 0), 2, 1)), ("backend", param((255, 0), 3, 1)),
                                   ("backend", param((255, 0), 7, 1))]),
    }


def _bad_crc(frame: bytes) -> bytes:
    broken = bytearray(frame)
    broken[-1] ^= 0xFF
    return bytes(broken)


def _split_case(hb: Callable[[int, int], bytes],
                assign: Callable[[tuple[int, int], int], bytes]) -> list[Step]:
    packet = assign((1, 191), 2)
    steps = _case([("sitl1", hb(1, 1)), ("sitl2", hb(2, 1)), ("backend", hb(255, 190))])
    first, second = packet[:7], packet[7:] + hb(1, 1)
    return [*steps, ("sitl1", first, False), *_case([("sitl1", second)])]


def _checksum_v1(frame: bytes) -> bytes:
    from pymavlink.generator.mavcrc import x25crc

    crc = x25crc(frame[1:])
    crc.accumulate(bytes((50,)))  # HEARTBEAT's CRC extra
    return crc.crc.to_bytes(2, "little")
