"""Narrow runtime ports used by the GCS demo evaluator."""

from __future__ import annotations

from typing import Iterable, Protocol, Union


JsonScalar = Union[str, int, float, bool, None]
JsonValue = Union[JsonScalar, list["JsonValue"], dict[str, "JsonValue"]]


class JsonApiPort(Protocol):
    @property
    def websocket_url(self) -> str: ...

    def close(self) -> None: ...

    def get_json(self, path: str, *, timeout: float = 30.0) -> JsonValue: ...

    def post_json(
        self,
        path: str,
        payload: dict[str, JsonValue] | None = None,
        *,
        expected: Iterable[int] = (200,),
        timeout: float = 30.0,
    ) -> tuple[int, JsonValue]: ...

    def put_json(
        self,
        path: str,
        payload: dict[str, JsonValue],
        *,
        timeout: float = 30.0,
    ) -> JsonValue: ...


class TailPort(Protocol):
    def read_new(self) -> str: ...

    @property
    def text(self) -> str: ...


class EventStreamPort(Protocol):
    def __enter__(self) -> EventStreamPort: ...

    def __exit__(self, *args: object) -> object: ...

    def receive(self, timeout_s: float) -> bytes | str: ...


class WebSocketConnectionPort(Protocol):
    def recv(self, timeout: float | None = None) -> bytes | str: ...

    def close(self) -> None: ...


class ClockPort(Protocol):
    def monotonic(self) -> float: ...

    def unix(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


__all__ = [
    "ClockPort",
    "EventStreamPort",
    "JsonApiPort",
    "JsonScalar",
    "JsonValue",
    "TailPort",
    "WebSocketConnectionPort",
]
