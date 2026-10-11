"""ObservableGate contract: PASS/BLOCK/INVALID gates with dwell and composite support.

Introduced in Step 2 of the observable-only navigation redesign. This module
defines the gate contract only; no production wiring, no observables, and no
navigation behavior change. Later steps add concrete gates (passed-POI,
handoff, freeze) parameterised over a real observables context.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import (
    Any,
    Callable,
    Generic,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
    TypeVar,
)


class GateStatus(Enum):
    """Gate evaluation outcome.

    PASS    - condition held this tick.
    BLOCK   - condition did not hold; reason required.
    INVALID - condition could not be evaluated (missing signal, divergent
              estimator, etc.); reason required.
    """

    PASS = "PASS"
    BLOCK = "BLOCK"
    INVALID = "INVALID"


@dataclass
class GateResult:
    """Result of a single gate evaluation.

    Invariants (enforced in __post_init__):
      - PASS must not set reason.
      - BLOCK and INVALID require reason (so every non-pass is diagnosable).
      - details is optional for any status.
    """

    status: GateStatus
    reason: Optional[str] = None
    details: Optional[Mapping[str, Any]] = None

    def __post_init__(self) -> None:
        if self.status == GateStatus.PASS:
            if self.reason is not None:
                raise ValueError("PASS GateResult must not set reason")
        else:
            if self.reason is None:
                raise ValueError(
                    f"{self.status.value} GateResult requires reason"
                )

    @property
    def passed(self) -> bool:
        return self.status == GateStatus.PASS


TContext = TypeVar("TContext")


class ObservableGate(ABC, Generic[TContext]):
    """Base class for an observable gate.

    Concrete gates consume a context (observables, vehicle state, EKF) and
    return a GateResult. The context type is left generic here so Step 5
    can parameterise over a real observables context without this step
    committing to a concrete shape.
    """

    @abstractmethod
    def evaluate(self, ctx: TContext) -> GateResult:
        raise NotImplementedError

    @abstractmethod
    def reset(self) -> None:
        raise NotImplementedError


class DwellGate(ObservableGate[TContext]):
    """Wraps an inner gate and requires continuous PASS for dwell_s.

    Semantics:
      - Inner PASS starts (or continues) the dwell timer.
      - Inner BLOCK clears the dwell timer and propagates BLOCK.
      - Inner INVALID clears the dwell timer and propagates INVALID.
      - dwell_s == 0 passes immediately on the first inner PASS.
      - dwell_s < 0 is rejected at construction.
      - While the dwell window is not yet satisfied the gate returns BLOCK
        with reason="DWELL_NOT_MET" and details {elapsed_s, required_s}.
    """

    _DWELL_BLOCK_REASON = "DWELL_NOT_MET"

    def __init__(
        self,
        inner: ObservableGate[TContext],
        dwell_s: float,
        clock: Callable[[], float] = time.monotonic,
    ):
        if dwell_s < 0:
            raise ValueError(f"dwell_s must be >= 0, got {dwell_s}")
        self._inner = inner
        self._dwell_s = float(dwell_s)
        self._clock = clock
        self._pass_start_time: Optional[float] = None

    def evaluate(self, ctx: TContext) -> GateResult:
        inner_result = self._inner.evaluate(ctx)

        if inner_result.status != GateStatus.PASS:
            self._pass_start_time = None
            return inner_result

        now = self._clock()
        if self._pass_start_time is None:
            self._pass_start_time = now

        elapsed = now - self._pass_start_time
        if elapsed >= self._dwell_s:
            return GateResult(
                status=GateStatus.PASS,
                details={"elapsed_s": elapsed, "required_s": self._dwell_s},
            )

        return GateResult(
            status=GateStatus.BLOCK,
            reason=self._DWELL_BLOCK_REASON,
            details={"elapsed_s": elapsed, "required_s": self._dwell_s},
        )

    def reset(self) -> None:
        self._inner.reset()
        self._pass_start_time = None


class AllGate(ObservableGate[TContext]):
    """Composite gate: PASS iff all named children PASS.

    All children are evaluated on every call (no short-circuit) so that
    stateful children such as DwellGate advance their internal timers
    regardless of earlier failures. The first non-PASS result by
    configured order becomes the composite's primary result; its reason
    is prefixed with the child's name.

    Invariants (enforced at construction):
      - The gate list must be non-empty.
      - Child names must be unique.

    Details on the composite result carry stable keys:
      - gate_results: ordered list of (name, GateResult) pairs (all children).
      - first_blocking: the name of the first non-PASS child, or None if all passed.
    """

    def __init__(
        self, gates: Sequence[Tuple[str, ObservableGate[TContext]]]
    ):
        gates_list = list(gates)
        if not gates_list:
            raise ValueError("AllGate requires at least one child gate")

        seen_names: set[str] = set()
        for name, _ in gates_list:
            if name in seen_names:
                raise ValueError(f"Duplicate gate name: {name!r}")
            seen_names.add(name)

        self._gates: List[Tuple[str, ObservableGate[TContext]]] = gates_list

    def evaluate(self, ctx: TContext) -> GateResult:
        results: List[Tuple[str, GateResult]] = []
        first_failure: Optional[Tuple[str, GateResult]] = None

        for name, gate in self._gates:
            result = gate.evaluate(ctx)
            results.append((name, result))
            if first_failure is None and result.status != GateStatus.PASS:
                first_failure = (name, result)

        details = {"gate_results": results, "first_blocking": None}

        if first_failure is None:
            return GateResult(status=GateStatus.PASS, details=details)

        fail_name, fail_result = first_failure
        details["first_blocking"] = fail_name
        composed_reason = f"{fail_name}:{fail_result.reason}"
        return GateResult(
            status=fail_result.status,
            reason=composed_reason,
            details=details,
        )

    def reset(self) -> None:
        for _, gate in self._gates:
            gate.reset()
