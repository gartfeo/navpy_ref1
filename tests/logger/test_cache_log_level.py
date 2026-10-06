"""Tests for CacheLogLevel.from_name — the argparse log-level converter."""
import pytest

from navpy.logger.cache_log_level import CacheLogLevel


class TestFromName:
    def test_resolves_uppercase_name(self):
        assert CacheLogLevel.from_name("DEBUG") is CacheLogLevel.DEBUG

    def test_is_case_insensitive(self):
        assert CacheLogLevel.from_name("debug") is CacheLogLevel.DEBUG
        assert CacheLogLevel.from_name("Info") is CacheLogLevel.INFO

    def test_strips_surrounding_whitespace(self):
        assert CacheLogLevel.from_name("  warning  ") is CacheLogLevel.WARNING

    @pytest.mark.parametrize("name,expected", [
        ("verbose", CacheLogLevel.VERBOSE),
        ("debug", CacheLogLevel.DEBUG),
        ("info", CacheLogLevel.INFO),
        ("warning", CacheLogLevel.WARNING),
        ("error", CacheLogLevel.ERROR),
    ])
    def test_all_members_resolve(self, name, expected):
        assert CacheLogLevel.from_name(name) is expected

    def test_idempotent_on_enum_member(self):
        # A non-string default (e.g. CacheLogLevel.INFO) passes through unchanged.
        assert CacheLogLevel.from_name(CacheLogLevel.INFO) is CacheLogLevel.INFO

    def test_rejects_numeric_value_string(self):
        # The old bug: type=CacheLogLevel did a value-lookup, so even "1" failed.
        # Names are the contract now; the integer value string is not a name.
        with pytest.raises(ValueError):
            CacheLogLevel.from_name("1")

    def test_rejects_unknown_name_with_helpful_message(self):
        with pytest.raises(ValueError) as exc:
            CacheLogLevel.from_name("bogus")
        msg = str(exc.value)
        assert "bogus" in msg
        # Every valid name is listed to guide the user.
        for level in CacheLogLevel:
            assert level.name in msg
