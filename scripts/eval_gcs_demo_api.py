"""HTTP, telemetry WebSocket, and append-only log-tail adapters."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import cast

import requests

from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue, WebSocketConnectionPort


class GcsApi:
    def __init__(self, backend_port: int) -> None:
        if type(backend_port) is not int or not 1 <= backend_port <= 65535:
            raise ValueError("backend_port must be an integer in [1, 65535]")
        self.base_url = f"http://127.0.0.1:{backend_port}"
        self.websocket_url = f"ws://127.0.0.1:{backend_port}/ws/telemetry"
        self.session = requests.Session()

    def close(self) -> None:
        self.session.close()

    def request(
        self,
        method: str,
        path: str,
        *,
        expected: Iterable[int] = (200,),
        timeout: float = 30.0,
        json_payload: dict[str, JsonValue] | None = None,
    ) -> requests.Response:
        try:
            response = self.session.request(
                method,
                self.base_url + path,
                timeout=timeout,
                json=json_payload,
            )
        except requests.RequestException as error:
            raise RegressionError(f"{method} {path} failed: {error}") from error
        if response.status_code not in set(expected):
            raise RegressionError(
                f"{method} {path} returned {response.status_code}: "
                f"{response.text[:1000]}"
            )
        return response

    def get_json(self, path: str, *, timeout: float = 30.0) -> JsonValue:
        return cast(JsonValue, self.request("GET", path, timeout=timeout).json())

    def post_json(
        self,
        path: str,
        payload: dict[str, JsonValue] | None = None,
        *,
        expected: Iterable[int] = (200,),
        timeout: float = 30.0,
    ) -> tuple[int, JsonValue]:
        response = self.request(
            "POST",
            path,
            json_payload=payload,
            expected=expected,
            timeout=timeout,
        )
        return response.status_code, cast(JsonValue, response.json())

    def put_json(
        self,
        path: str,
        payload: dict[str, JsonValue],
        *,
        timeout: float = 30.0,
    ) -> JsonValue:
        return cast(
            JsonValue,
            self.request(
                "PUT",
                path,
                json_payload=payload,
                timeout=timeout,
            ).json(),
        )


class TelemetryEventStream:
    """Synchronous `/ws/telemetry` adapter, connected on context entry."""

    def __init__(self, url: str, *, open_timeout_s: float = 10.0) -> None:
        self._url = url
        self._open_timeout_s = open_timeout_s
        self._connection: WebSocketConnectionPort | None = None

    def __enter__(self) -> TelemetryEventStream:
        from websockets.sync.client import connect

        self._connection = connect(
            self._url,
            open_timeout=self._open_timeout_s,
            close_timeout=2.0,
        )
        return self

    def __exit__(self, *_args: object) -> None:
        connection, self._connection = self._connection, None
        if connection is not None:
            connection.close()

    def receive(self, timeout_s: float) -> bytes | str:
        if self._connection is None:
            raise RuntimeError("telemetry stream is not connected")
        return self._connection.recv(timeout=timeout_s)


class FileTail:
    """Incremental UTF-8 tail that survives file creation/truncation."""

    def __init__(self, path: Path, start_offset: int = 0) -> None:
        self.path = path
        self.offset = max(0, start_offset)
        self.parts: list[str] = []

    def read_new(self) -> str:
        try:
            size = self.path.stat().st_size
        except FileNotFoundError:
            return ""
        if size < self.offset:
            self.offset = 0
        with self.path.open("r", encoding="utf-8", errors="replace") as handle:
            handle.seek(self.offset)
            chunk = handle.read()
            self.offset = handle.tell()
        if chunk:
            self.parts.append(chunk)
        return chunk

    @property
    def text(self) -> str:
        return "".join(self.parts)


__all__ = ["FileTail", "GcsApi", "TelemetryEventStream"]
