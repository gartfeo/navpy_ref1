from __future__ import annotations

from unittest.mock import Mock

import pytest

from gcs.backend.task_confirm_transport import (
    ConfirmationTransport,
    parse_round_uid,
)


def test_round_uid_parser_uses_uint32_pair_semantics() -> None:
    assert parse_round_uid("legacy") == (0, 0)
    assert parse_round_uid("0:1") == (0, 1)
    assert parse_round_uid("1:0") == (1, 0)
    assert parse_round_uid("4294967295:4294967295") == (
        0xFFFFFFFF,
        0xFFFFFFFF,
    )


@pytest.mark.parametrize(
    "uid",
    ("0:0", "-1:2", "1:4294967296", "abc:1", "1", "1:2:3"),
)
def test_round_uid_parser_rejects_malformed_or_legacy_collision(uid: str) -> None:
    with pytest.raises(ValueError, match="confirmation round uid"):
        parse_round_uid(uid)


@pytest.mark.parametrize("operation", ("response", "thumbnail"))
def test_transport_programmer_typeerror_propagates(operation: str) -> None:
    vehicle = Mock()
    vehicle.send_mavlink_message.side_effect = TypeError("bad mavlink contract")
    transport = ConfirmationTransport(lambda _sys_id: vehicle)

    with pytest.raises(TypeError, match="bad mavlink contract"):
        if operation == "response":
            transport.send_response(1, 7, True, "424242:11")
        else:
            transport.request_thumbnail(1, 7, "424242:11")


@pytest.mark.parametrize("operation", ("response", "thumbnail"))
def test_transport_oserror_is_contained(operation: str) -> None:
    vehicle = Mock()
    vehicle.send_mavlink_message.side_effect = OSError("radio down")
    transport = ConfirmationTransport(lambda _sys_id: vehicle)

    if operation == "response":
        transport.send_response(1, 7, True, "424242:11")
    else:
        transport.request_thumbnail(1, 7, "424242:11")
