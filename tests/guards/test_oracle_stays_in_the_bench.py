"""The truth-fed oracle must never become reachable from the flying article.

`scripts/scratch_navigation_oracle.py` deliberately breaks the pure-vision
constraint in AGENTS.md: it reads ground velocity and the commanded wind vector
and feeds the result into the law's gain. That is sanctioned ONLY as a bench
diagnostic measuring a ceiling, and the sanction rests entirely on two
properties that are otherwise nobody's job to preserve.

Both failures are silent, which is why they are pinned here rather than left to
review. Production importing it would fly a truth-fed law that still passes
every navigation test, because the tests measure a miss and the miss would be
BETTER. A default of anything but "none" would treat every aircraft in every
future launch while the artifacts still looked like ordinary runs.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ORACLE_NAMES = ("scratch_navigation_oracle", "substitute_speed")


def test_production_cannot_reach_the_oracle() -> None:
    """No module under src/ may name the oracle, by import or otherwise.

    Text search rather than an import graph: a dynamic import, a string passed
    to importlib, or a copied helper body would all evade a graph walk while
    putting the same truth into the command path.
    """
    offenders = [
        str(path.relative_to(ROOT))
        for path in (ROOT / "src").rglob("*.py")
        for name in ORACLE_NAMES
        if name in path.read_text(encoding="utf-8")
    ]
    assert not offenders, (
        "the truth-fed oracle is referenced from production: "
        f"{offenders}. It reads ground velocity and wind, which AGENTS.md "
        "bars from the command path, and a law that flies on it would still "
        "pass every miss-based test because the miss would improve."
    )


def test_the_bench_default_leaves_every_aircraft_untreated() -> None:
    """`--oracle-gain` must default to "none".

    Parsed from the source rather than by running the parser, because
    importing the child pulls in the whole SITL stack. Any other default would
    silently treat aircraft that no cell asked for.
    """
    tree = ast.parse(
        (ROOT / "scripts" / "scratch_navigation_uav.py").read_text(
            encoding="utf-8"
        )
    )
    defaults = [
        keyword.value.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, "attr", None) == "add_argument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "--oracle-gain"
        for keyword in node.keywords
        if keyword.arg == "default"
    ]
    assert defaults == ["none"], (
        f"--oracle-gain must default to 'none', found {defaults}"
    )


def test_every_result_records_which_mode_flew() -> None:
    """The child must always publish `oracle_gain`, treated or not.

    A treated run that did not say so would be pooled into an accuracy claim as
    an ordinary one. That is not hypothetical: the analysis used to score the
    wind matrix filtered on injected lag, pose delay and estimate source, and
    would have accepted a truth-fed run without noticing.
    """
    source = (ROOT / "scripts" / "scratch_navigation_uav.py").read_text(
        encoding="utf-8"
    )
    assert '"oracle_gain": options.oracle_gain' in source, (
        "the result payload must carry oracle_gain unconditionally, so a "
        "truth-fed run can never be read as a clean one"
    )
