"""WebSocket client set and broadcast helpers."""
from __future__ import annotations

import asyncio
import logging
from typing import Any

from fastapi import WebSocket
import orjson

log = logging.getLogger(__name__)


class ConnectionManager:
    """Manages active WebSocket connections and broadcasts messages."""

    def __init__(self):
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        async with self._lock:
            self._clients.add(ws)
        log.info("WS client connected (%d total)", len(self._clients))

    async def disconnect(self, ws: WebSocket):
        async with self._lock:
            self._clients.discard(ws)
        log.info("WS client disconnected (%d total)", len(self._clients))

    async def broadcast(self, data: Any):
        """Send JSON-serialized data to all connected clients."""
        if not self._clients:
            return
        payload = orjson.dumps(data)
        async with self._lock:
            stale: list[WebSocket] = []
            for ws in self._clients:
                try:
                    await ws.send_bytes(payload)
                except Exception:
                    stale.append(ws)
            for ws in stale:
                self._clients.discard(ws)

    @property
    def client_count(self) -> int:
        return len(self._clients)


# Singleton instance
manager = ConnectionManager()
