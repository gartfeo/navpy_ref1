"""Validation of announced RTSP URIs and the WHEP playback URL derived from them.

The URI in VIDEO_STREAM_INFORMATION is untrusted text from the radio link
(docs/design/gcs_video_streams.md, section 2). It is accepted only as
``rtsp://<literal IP>[:port]/<seg>/<seg>...`` with the IP inside an allowed
network, no userinfo, no query and no fragment. Path segments are
``[A-Za-z0-9._-]+``; empty, ``.`` and ``..`` segments are rejected so a browser
cannot normalise the WHEP path to somewhere else. No DNS names are accepted, so
there is nothing to re-resolve later.
"""
from __future__ import annotations

import ipaddress
import re
from dataclasses import dataclass
from typing import Iterable, Optional, Union
from urllib.parse import urlsplit

IpNetwork = Union[ipaddress.IPv4Network, ipaddress.IPv6Network]

_SEGMENT = re.compile(r"[A-Za-z0-9._-]+")
# Printable ASCII without space: urlsplit silently strips tabs and newlines,
# so anything else is rejected before parsing.
_PRINTABLE_MIN, _PRINTABLE_MAX = 0x21, 0x7E


@dataclass(frozen=True)
class RtspLocation:
    """The parts of an accepted URI the WHEP URL is built from."""

    host: str
    path: str  # without the leading "/"


@dataclass(frozen=True)
class UriVerdict:
    location: Optional[RtspLocation]
    reason: str = ""


def _reject(reason: str) -> UriVerdict:
    return UriVerdict(None, reason)


def _path_ok(path: str) -> bool:
    if not path.startswith("/"):
        return False
    segments = path[1:].split("/")
    return all(
        _SEGMENT.fullmatch(segment) and segment not in (".", "..")
        for segment in segments
    )


def validate_rtsp_uri(uri: object, networks: Iterable[IpNetwork]) -> UriVerdict:
    """Accept *uri* only under the rules above; the reason names a rejection."""
    if not isinstance(uri, str) or not uri:
        return _reject("empty")
    if any(not _PRINTABLE_MIN <= ord(ch) <= _PRINTABLE_MAX for ch in uri):
        return _reject("non-printable character")
    if "?" in uri:
        return _reject("query")
    if "#" in uri:
        return _reject("fragment")
    try:
        parts = urlsplit(uri)
    except ValueError:  # e.g. an unclosed "[" in the authority
        return _reject("malformed authority")
    if parts.scheme != "rtsp":
        return _reject("scheme")
    if "@" in parts.netloc:
        return _reject("userinfo")
    try:
        parts.port  # noqa: B018 - raises ValueError on a malformed port
    except ValueError:
        return _reject("port")
    try:
        host = ipaddress.ip_address(parts.hostname or "")
    except ValueError:
        return _reject("host is not a literal IP")
    if not any(host in network for network in networks):
        return _reject("host outside allowed networks")
    if not _path_ok(parts.path):
        return _reject("path")
    text = f"[{host}]" if host.version == 6 else str(host)
    return UriVerdict(RtspLocation(text, parts.path[1:]))


def whep_url(location: RtspLocation, whep_port: int) -> str:
    """MediaMTX's WHEP layout: same host and path, ``/whep`` appended."""
    return f"http://{location.host}:{whep_port}/{location.path}/whep"
