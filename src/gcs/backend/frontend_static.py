"""Serve the built GCS frontend from the backend (single-origin deployments)."""
from __future__ import annotations

import logging
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from starlette.types import Receive, Scope, Send

from gcs.backend.runtime_paths import FRONTEND_DIST_ENV

_FILE_METHODS = ("GET", "HEAD")


def mount_frontend(app: FastAPI, dist: Path | None, logger: logging.Logger) -> bool:
    """Serve ``dist`` for requests that no API route matches.

    The files are the router's fallback, not a catch-all route: the router
    still answers 405 for a wrong method and redirects a trailing slash before
    falling back, and only GET/HEAD requests reach the files. Other unmatched
    requests (an unknown POST, an unknown WebSocket path) keep the router's
    normal not-found reply.

    ``dist`` is None in the desktop layout, where Vite serves the frontend and
    proxies to this backend. A configured ``dist`` without a build fails
    startup instead of leaving the operator on a blank page.
    """
    if dist is None:
        return False
    if not (dist / "index.html").is_file():
        raise RuntimeError(
            f"{FRONTEND_DIST_ENV}={dist} has no index.html; run `npm run build` first",
        )
    files = StaticFiles(directory=dist, html=True)
    not_found = app.router.default

    async def serve_frontend(scope: Scope, receive: Receive, send: Send) -> None:
        if scope["type"] == "http" and scope["method"] in _FILE_METHODS:
            await files(scope, receive, send)
        else:
            await not_found(scope, receive, send)

    app.router.default = serve_frontend
    logger.info("Serving frontend from %s", dist)
    return True
