"""Focused tests for raw-message duplicate suppression."""

from navpy.logger.cache_log_dedupe import CacheLogDeduplicator


def test_info_dedupes_same_raw_message_per_key_only():
    dedupe = CacheLogDeduplicator()

    assert dedupe.accept_info("a", "same")
    assert not dedupe.accept_info("a", "same")
    assert dedupe.accept_info("b", "same")
    assert dedupe.accept_info("a", "changed")


def test_warning_only_suppresses_current_repeated_raw_message():
    dedupe = CacheLogDeduplicator()

    assert dedupe.accept_warning("a")
    assert not dedupe.accept_warning("a")
    assert dedupe.accept_warning("b")
    assert dedupe.accept_warning("a")


def test_single_warning_accepts_each_key_once():
    dedupe = CacheLogDeduplicator()

    assert dedupe.accept_single_warning("a")
    assert not dedupe.accept_single_warning("a")
    assert dedupe.accept_single_warning("b")
