"""The concurrency wrapper builds the real case parser's namespace before launch."""

import pytest

from scripts import scratch_plant_concurrency as wrapper


@pytest.mark.parametrize("flags", [
    [],
    ["--engage-wp", "4", "--throttle", "0.6", "--level-s", "4.5",
     "--hold-s", "21.5", "--timeout", "620", "--mission-alt", "180",
     "--gate-offset", "1200", "--target-offset", "250"],
])
def test_case_namespace_matches_direct_parser_without_launch(flags):
    from scratch_plant_id_eval import _parser as case_parser

    # The wrapper intentionally defaults to 600s; the direct case defaults to 400s.
    expected = case_parser().parse_args(["--timeout", "600", *flags])
    actual = wrapper._case_args(wrapper._parser().parse_args(flags))
    assert vars(actual) == vars(expected)
