"""Close a cache logger without the worker join bound, for tests that do not test it."""

from unittest.mock import patch


def unbounded_worker_join():
    """Patch the 2 s worker join bound away.

    Close bounds its join against a stuck handler; test_cache_log_worker covers
    that bound. Elsewhere a loaded host can need longer than 2 s to drain the
    queued messages, so the join waits for the worker to finish them.
    """
    return patch("navpy.core.kernel.logging.cache_log_worker.CACHE_WORKER_JOIN_TIMEOUT_S", None)
