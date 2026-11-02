"""The ``gstreamer+rtsp://`` source: its settings and pipeline, and nothing else."""

from __future__ import annotations

from dataclasses import dataclass
from urllib.parse import parse_qs, urlsplit

GSTREAMER_SCHEME = "gstreamer+rtsp://"
SUPPORTED_DECODERS = ("nvh265dec", "avdec_h265")    # HITL step 7's selection
_MAX_LATENCY_MS = 2000


def is_gstreamer_source(source: object) -> bool:
    return isinstance(source, str) and source.lower().startswith(GSTREAMER_SCHEME)


@dataclass(frozen=True)
class EncodedSourceSettings:
    url: str
    decoder: str
    latency_ms: int

    def pipeline_text(self) -> str:
        return (
            f"rtspsrc name=src location={self.url} protocols=udp buffer-mode=none "
            f"ntp-sync=false rfc7273-sync=false udp-reconnect=false latency={self.latency_ms} "
            "! rtph265depay ! h265parse ! video/x-h265,stream-format=byte-stream,alignment=au "
            f"! {self.decoder} ! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=sink sync=false async=false max-buffers=4 drop=false"
        )


def parse_gstreamer_source(source: str) -> EncodedSourceSettings:
    """The source's settings; ValueError unless all of them are stated and known."""
    if not is_gstreamer_source(source):
        raise ValueError(f"not a {GSTREAMER_SCHEME} source: {source!r}")
    parts = urlsplit(source[len("gstreamer+"):])
    query = parse_qs(parts.query, strict_parsing=bool(parts.query))
    if set(query) != {"decoder", "latency_ms"} or any(len(v) != 1 for v in query.values()):
        raise ValueError(f"{source!r}: needs exactly decoder=NAME and latency_ms=N")
    decoder, latency = query["decoder"][0], query["latency_ms"][0]
    if decoder not in SUPPORTED_DECODERS:
        raise ValueError(f"{source!r}: decoder must be one of {SUPPORTED_DECODERS}")
    if not latency.isdigit() or int(latency) > _MAX_LATENCY_MS:
        raise ValueError(f"{source!r}: latency_ms must be 0 to {_MAX_LATENCY_MS}")
    url = f"rtsp://{parts.netloc}{parts.path}"
    if not parts.hostname or any(c.isspace() or c in '!"\'' for c in url):
        raise ValueError(f"{source!r}: not a usable RTSP address")
    return EncodedSourceSettings(url, decoder, int(latency))


__all__ = [
    "EncodedSourceSettings",
    "GSTREAMER_SCHEME",
    "SUPPORTED_DECODERS",
    "is_gstreamer_source",
    "parse_gstreamer_source",
]
