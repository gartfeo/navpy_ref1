"""The HITL contact channel's datagrams (hitl_final_approach.md section 9, step 26).

Three kinds, each a JSON object with exactly its fields and an HMAC-SHA256 ``mac`` over the rest, keyed by the run's
random key (the link file, ``hitl_contact_link_file.py``):

- ``arm``: navpy to the contact service, NAV entry: the run, navpy's session, a rising sequence number, the armed
  identity (vehicle, task, dock uid, episode) and a fresh random ``arm_id``;
- ``disarm``: navpy to the service, NAV exit: the run, the session, the sequence and the ``arm_id`` it ends;
- ``contact``: the service to navpy, once per arm: the arm's run, session, identity and ``arm_id`` echoed back, and
  the board time (``board_time_us``) of the pose sample that touched the net.

No kind has a coordinate, a distance or any other geometry: a contact says only "this arm touched the net at this
board time". ``decode`` refuses an unknown or missing field, a wrong type, another schema or kind, and a bad MAC, so a
datagram that grew a coordinate is refused, not read.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import re
from collections.abc import Mapping
from typing import Any

SCHEMA = "navpy.hitl.contact.v1"
KIND_ARM, KIND_DISARM, KIND_CONTACT = "arm", "disarm", "contact"
MAC_FIELD = "mac"
KEY_BYTES = 32                      # the run key: 256 random bits
NONCE_HEX = 32                      # session and arm_id: 128 random bits as hex
MAX_DATAGRAM_BYTES = 1024           # every valid datagram is far below this
MAX_JSON_INTEGER = 2 ** 53 - 1      # exact in every JSON reader
RUN_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}")
HEX_NONCE = re.compile(rf"[0-9a-f]{{{NONCE_HEX}}}")

_IDENTITY = ("vehicle", "task_id", "dock_uid", "episode")
FIELDS: Mapping[str, frozenset[str]] = {
    KIND_ARM: frozenset({"schema", "kind", "run_id", "session", "seq", *_IDENTITY, "arm_id"}),
    KIND_DISARM: frozenset({"schema", "kind", "run_id", "session", "seq", "arm_id"}),
    KIND_CONTACT: frozenset({"schema", "kind", "run_id", "session", *_IDENTITY, "arm_id", "board_time_us"}),
}
_INTEGERS = frozenset({"seq", *_IDENTITY, "board_time_us"})
_NONCES = frozenset({"session", "arm_id"})


class WireRefused(ValueError):
    """A datagram the receiver does not read; ``reason`` names why, for the refusal counts."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def canonical(body: Mapping[str, Any]) -> bytes:
    return json.dumps(body, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _mac(body: Mapping[str, Any], key: bytes) -> str:
    return hmac.new(key, canonical(body), hashlib.sha256).hexdigest()


def valid_key(key: bytes) -> bytes:
    if not isinstance(key, bytes) or len(key) != KEY_BYTES:
        raise ValueError(f"the contact key must be {KEY_BYTES} bytes")
    return key


def encode(body: Mapping[str, Any], key: bytes) -> bytes:
    """One signed datagram; ValueError for a body ``decode`` would refuse (never sent)."""
    problem = _body_problem(body, str(body.get("kind")))
    if problem:
        raise ValueError(problem)
    return canonical({**body, MAC_FIELD: _mac(body, valid_key(key))})


def decode(datagram: bytes, key: bytes, kinds: frozenset[str]) -> dict[str, Any]:
    """The authenticated body of one of ``kinds``; WireRefused otherwise. The MAC is checked before any value."""
    if len(datagram) > MAX_DATAGRAM_BYTES:
        raise WireRefused("oversize")
    try:
        data = json.loads(datagram.decode("utf-8"), parse_constant=_no_constant)
    except (UnicodeDecodeError, ValueError):
        raise WireRefused("malformed") from None
    if not isinstance(data, dict) or not isinstance(data.get(MAC_FIELD), str):
        raise WireRefused("malformed")
    body = {name: value for name, value in data.items() if name != MAC_FIELD}
    if not hmac.compare_digest(data[MAC_FIELD].encode("ascii", "replace"), _mac(body, valid_key(key)).encode()):
        raise WireRefused("bad mac")
    kind = body.get("kind")
    if kind not in kinds:
        raise WireRefused("wrong kind")
    problem = _body_problem(body, kind)
    if problem:
        raise WireRefused("malformed")
    return body


def _no_constant(name: str) -> None:
    raise ValueError(f"{name} is not JSON")


def _body_problem(body: Mapping[str, Any], kind: str) -> str:
    fields = FIELDS.get(kind)
    if fields is None:
        return f"unknown kind {kind!r}"
    if set(body) != fields:
        return f"{kind} fields {sorted(set(body) ^ fields)} differ from the schema"
    if body["schema"] != SCHEMA:
        return "another schema"
    if not isinstance(body["run_id"], str) or not RUN_ID.fullmatch(body["run_id"]):
        return "run_id is not a run id"
    for name in fields & _NONCES:
        if not isinstance(body[name], str) or not HEX_NONCE.fullmatch(body[name]):
            return f"{name} is not a {NONCE_HEX}-digit hex nonce"
    for name in fields & _INTEGERS:
        value = body[name]
        if type(value) is not int or not 0 <= value <= MAX_JSON_INTEGER:
            return f"{name} is not an exact non-negative integer"
    return ""


__all__ = ["FIELDS", "KEY_BYTES", "KIND_ARM", "KIND_CONTACT", "KIND_DISARM", "MAC_FIELD", "NONCE_HEX", "SCHEMA",
           "WireRefused", "canonical", "decode", "encode", "valid_key"]
