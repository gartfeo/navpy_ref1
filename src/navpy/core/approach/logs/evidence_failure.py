"""Report a failed evidence write and carry on (todo 290).

Evidence is not on the command path, so a failed write is logged with the
marker and skipped; NavPy keeps running. Evals read the marker and do not
score the run.
"""

from __future__ import annotations

from navpy.core.kernel.logging.cache_logger import ILogger


NAVIGATION_EVIDENCE_FAILURE_MARKER = "NAVIGATION_EVIDENCE_FAILURE"


def report_evidence_failure(logger: ILogger, what: str, error: BaseException) -> None:
    """Log ``error`` with the marker; a failing logger must not stop the caller."""
    try:
        logger.error(
            f"{NAVIGATION_EVIDENCE_FAILURE_MARKER}: {what} "
            f"{type(error).__name__}: {error}",
            error,
        )
    except Exception:  # noqa: BLE001 - nothing left to report to; the record stays
        pass


__all__ = ["NAVIGATION_EVIDENCE_FAILURE_MARKER", "report_evidence_failure"]
