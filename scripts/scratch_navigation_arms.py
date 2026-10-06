"""Two navigation laws, one launch, and proof of which aircraft flew which.

The measurement this exists for compares two variants of the navigation law. It
cannot be done across launches: batch-to-batch spread on IDENTICAL code was
measured at up to 5.5x (calm medians 0.105 / 0.476 / 0.579 m over 44 scored
runs), which is larger than any effect worth measuring. So both variants must
fly inside one launch, where the aircraft share their batch conditions.

One launch, however, runs one copy of the code. An arm is therefore selected by
running the child from a SEPARATE SOURCE TREE, which is where the traps are:

  * The child rebuilds `sys.path` from its own `__file__`, so a tree copy that
    omits `scripts/` is discarded in silence and BOTH arms fly the original
    law. That failure has no error and no symptom -- it reports a true
    difference of zero, indistinguishable from a variant that does nothing.
  * A tree copy that is stale, or edited past the intended change, sits at the
    expected path with the expected layout and invents a difference that
    belongs to the accident rather than to the experiment.
  * Cycling arms and conditions independently can produce an allocation where
    some (arm, condition) pair never occurs, which confounds the arm with the
    condition while looking perfectly balanced in the totals.

Each is refused here, before any aircraft starts, because every one of them
produces a plausible number rather than a visible failure.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

# Loaded BOTH ways: top-level by a launcher that put `scripts/` on the path, and
# as `scripts.<name>` by the test suite and by the child process. Each mode needs
# a different root, and satisfying one has broken the other twice before -- see
# tests/scripts/test_script_entrypoints_load_both_ways.py. Inserting both roots
# here makes the sibling import resolve identically in either mode.
import sys  # noqa: E402

WORKTREE = Path(__file__).resolve().parent.parent
for _root in (WORKTREE, WORKTREE / "src"):
    _path = str(_root)
    while _path in sys.path:
        sys.path.remove(_path)
    sys.path.insert(0, _path)

# A condition is its own shape; re-exported so callers reach one module.
from scripts.scratch_navigation_cells import Cell, parse_cells  # noqa: E402,F401

# The child is launched from the arm's own tree, so the copy must carry it.
CHILD_RELATIVE = Path("scripts/scratch_navigation_uav.py")
SOURCE_RELATIVE = Path("src")
# What the arms are ALLOWED to differ in. Everything else being identical is
# what makes the miss difference attributable to the treatment.
TREATMENT_RELATIVE = Path("src/navpy/modules/navigation/nav/vision_nav/law.py")
# Compared for identity. Bytecode caches and run artifacts are not code and
# differ for reasons that have nothing to do with the experiment.
COMPARED_SUFFIXES = (".py",)
IGNORED_PARTS = ("__pycache__", ".pytest_cache", ".egg-info")


def file_digest(path: Path) -> str:
    """Content identity of one file, for provenance rather than integrity."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_source_root(flew: Path, expected: "Path | None") -> None:
    """Refuse a law imported from outside the declared tree.

    Raised rather than recorded, and checked before the aircraft flies, because
    the failure is not a crash: a child that imported the wrong tree flies
    correctly and returns a good number for the OTHER arm's code. The
    comparison then reports a true difference of zero, which reads as `the
    change had no effect`, and a whole launch is spent discovering it.
    """
    if expected is None:
        return
    root = expected.resolve()
    if not flew.is_relative_to(root):
        raise ValueError(
            f"law source mismatch: expected the navigation law under {root}, "
            f"imported {flew}. Refusing to fly -- this run would produce a "
            "valid-looking result for the wrong arm"
        )


@dataclass(frozen=True)
class Arm:
    """One variant of the navigation law, with the tree that holds it."""

    name: str
    source_root: Path

    @property
    def child_script(self) -> Path:
        return self.source_root / CHILD_RELATIVE

    @property
    def source_path(self) -> Path:
        return self.source_root / SOURCE_RELATIVE

    @property
    def treatment_path(self) -> Path:
        return self.source_root / TREATMENT_RELATIVE


def parse_arms(spec: str | None) -> tuple[Arm, ...]:
    """Parse 'name=root,...'; empty spec means a single-arm launch."""
    if not spec:
        return ()
    arms = []
    for field in spec.split(","):
        name, separator, root = field.partition("=")
        if not separator or not name or not root:
            raise ValueError(f"arm {field!r} must be name=source_root")
        arms.append(Arm(name, Path(root).resolve()))
    names = [arm.name for arm in arms]
    if len(set(names)) != len(names):
        raise ValueError(f"arm names must be unique, got {names}")
    if len(arms) < 2:
        raise ValueError(
            "an arm comparison needs at least two arms; omit --arms for a "
            "single-arm launch"
        )
    return tuple(arms)


def _comparable_files(root: Path) -> dict[Path, Path]:
    """Every code file under a tree, keyed by its path relative to the root."""
    found: dict[Path, Path] = {}
    for base in (root / SOURCE_RELATIVE, root / "scripts"):
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if path.suffix not in COMPARED_SUFFIXES or not path.is_file():
                continue
            if any(part in IGNORED_PARTS for part in path.parts):
                continue
            found[path.relative_to(root)] = path
    return found


def verify_arm_trees(
    arms: tuple[Arm, ...],
    treatment: Path = TREATMENT_RELATIVE,
) -> dict[str, str]:
    """Refuse every way the two trees can lie, and return the arm digests.

    Checked here rather than at scoring time because each failure yields a
    complete, plausible set of results:

      * A tree missing the child script or `src` cannot be selected at all --
        the child would rebuild its import roots from the ORIGINAL tree and
        fly the wrong law while reporting success.
      * Identical treatment bytes across arms make the experiment a null by
        construction: the launch would spend its whole wall time proving that
        code equals itself.
      * A difference OUTSIDE the treatment file means the miss difference has
        at least two candidate causes, and the run cannot say which.
    """
    for arm in arms:
        if not arm.child_script.is_file():
            raise ValueError(
                f"arm {arm.name!r} has no {CHILD_RELATIVE} under "
                f"{arm.source_root} -- a tree without the child cannot be "
                "selected, and the child would silently import the original "
                "law instead"
            )
        if not (arm.source_path / "navpy").is_dir():
            raise ValueError(
                f"arm {arm.name!r} has no {SOURCE_RELATIVE / 'navpy'} under "
                f"{arm.source_root}"
            )
        if not (arm.source_root / treatment).is_file():
            raise ValueError(
                f"arm {arm.name!r} has no treatment file {treatment} under "
                f"{arm.source_root}"
            )

    digests = {
        arm.name: file_digest(arm.source_root / treatment) for arm in arms
    }
    if len(set(digests.values())) != len(digests):
        raise ValueError(
            f"arms share identical {treatment.name} contents "
            f"({sorted(set(digests.values()))[0][:12]}...): the launch would "
            "compare code against itself and report a difference of zero that "
            "reads as 'the change had no effect'"
        )

    reference, *others = arms
    reference_files = _comparable_files(reference.source_root)
    for arm in others:
        arm_files = _comparable_files(arm.source_root)
        unexpected = sorted(
            str(relative)
            for relative in set(reference_files) | set(arm_files)
            if relative != treatment
            and (
                relative not in reference_files
                or relative not in arm_files
                or file_digest(reference_files[relative]) != file_digest(arm_files[relative])
            )
        )
        if unexpected:
            raise ValueError(
                f"arms {reference.name!r} and {arm.name!r} differ outside "
                f"{treatment.name}, so a miss difference would have more than "
                f"one candidate cause: {unexpected[:10]}"
            )
    return digests


@dataclass(frozen=True)
class Case:
    """What one aircraft is: an arm, a condition, or both."""

    arm: "Arm | None"
    cell: "Cell | None"


def assign_cases(
    arms: tuple[Arm, ...],
    cells: tuple[Cell, ...],
    sysids: list[int],
    rotate: int = 0,
) -> dict[int, Case]:
    """Cycle the CROSS PRODUCT of arms and conditions across the aircraft.

    The cross product, not two independent cycles. Cycling arms with period 2
    and conditions with period 2 over consecutive sysids produces only the
    pairs (arm0, cell0) and (arm1, cell1) -- both arms flown, every condition
    flown, and half the combinations missing, so the arm is confounded with the
    condition while every total looks balanced.

    `rotate` shifts which sysid gets which arm. A repeat launch with the
    rotation changed re-tests the same comparison against a different
    arm-to-aircraft mapping, which is what separates a real effect from one
    that belongs to the launch positions.
    """
    if not arms:
        return {
            sysid: Case(None, cells[index % len(cells)] if cells else None)
            for index, sysid in enumerate(sysids)
        }
    ordered = arms[rotate % len(arms):] + arms[:rotate % len(arms)]
    conditions: tuple[Cell | None, ...] = cells or (None,)
    combinations = [
        Case(arm, cell) for cell in conditions for arm in ordered
    ]
    return {
        sysid: combinations[index % len(combinations)]
        for index, sysid in enumerate(sysids)
    }


def check_balanced(
    arms: tuple[Arm, ...], cells: tuple[Cell, ...], instances: int
) -> None:
    """Refuse an aircraft count that cannot fill the matrix evenly.

    An unbalanced tail is not a smaller experiment -- it is a biased one. With
    2 arms across 3 conditions and 8 aircraft, two combinations get 2 aircraft
    and four get 1, so the arm difference partly measures which combinations
    got the extra replicate. Nothing downstream can see that, because every
    aircraft returns a valid result.

    CELLS ALONE COUNT AS A MATRIX. This used to return early without arms, on
    the reasoning that a single-arm launch makes no comparison -- but a
    cells-only launch is exactly how the wind matrix was flown, and there the
    CELLS are the comparison. 10 wind-by-direction cells across 24 aircraft
    would give four cells three replicates and six cells two, biasing the
    symmetry answer by which cells drew the extra flight.
    """
    matrix = max(len(arms), 1) * max(len(cells), 1)
    if instances % matrix:
        raise ValueError(
            f"{instances} aircraft cannot fill {max(len(arms), 1)} arms x "
            f"{max(len(cells), 1)} conditions evenly: use a multiple of "
            f"{matrix}, or the comparison partly measures which combinations "
            "got the extra aircraft"
        )
