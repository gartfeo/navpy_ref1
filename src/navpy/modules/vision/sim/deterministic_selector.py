"""One source-time selection policy for a predeclared simulator window.

Unwired foundation: no frame/command output, clock reads or live vehicle access.
Owners serialize offer/poll/seal. Each stream must start before the first slot;
complete delivery, boot identity and lossless payload codecs belong to adapters.
Decisions remain provisional until seal(): any feed/association fault invalidates
the entire instance. Capacity is explicit; slow polling can cause invalidity.
The first fault on an invalid feed may depend on poll timing. Valid decisions,
including their first closing samples, depend only on the ordered source inputs.
"""
from __future__ import annotations

from navpy.modules.vision.sim.deterministic_selection_stream import SelectionHistory
from navpy.modules.vision.sim.deterministic_selection_types import (
    ClosingSamples, SelectionConfig, SelectionDecision, SelectionStream,
    SelectionVerdict, SelectionViolation, SlotKey, SourceSample, exact_integer,
)


class DeterministicSelector:
    def __init__(self, config: SelectionConfig) -> None:
        self._config = config
        self._next_slot = config.start_slot
        self._streams = {stream: SelectionHistory(
            stream, config.start_slot * config.period_us, config.capacity_per_stream,
        ) for stream in SelectionStream}
        self._violation: SelectionViolation | None = None
        self._verdict: SelectionVerdict | None = None

    @property
    def violation(self) -> SelectionViolation | None:
        return self._violation

    def offer(self, stream: SelectionStream, sample: SourceSample) -> bool:
        self._require_open()
        if self._violation is not None:
            return False
        if type(stream) is not SelectionStream:
            self._fail("invalid_stream", None, None)
            return False
        if (type(sample) is not SourceSample
                or not exact_integer(sample.boot_epoch)
                or not exact_integer(sample.time_us)
                or type(sample.payload) is not bytes):
            self._fail("invalid_sample", stream, None)
            return False
        if sample.boot_epoch != self._config.boot_epoch:
            self._fail("wrong_epoch", stream, sample.time_us)
            return False
        reason = self._streams[stream].push(sample)
        if reason is not None:
            self._fail(reason, stream, sample.time_us)
            return False
        return True

    def poll(self) -> SelectionDecision | None:
        """One sequential decision; None means pending, invalid, or window done.

        Receiving end_slot_exclusive - start_slot decisions means window done.
        Only seal() can certify that no later offered sample invalidated them.
        """
        self._require_open()
        config = self._config
        if self._violation is not None or self._next_slot >= config.end_slot_exclusive:
            return None
        if any(history.watermark is None for history in self._streams.values()):
            return None
        lower = self._next_slot * config.period_us
        upper = lower + config.period_us
        attitude = self._streams[SelectionStream.ATTITUDE]
        closing = attitude.at_or_after(upper)
        if closing is None:
            return None
        candidates = attitude.within(lower, upper)
        if candidates:
            decision = self._associate(candidates, closing)
            if decision is None:
                return None
        else:
            decision = SelectionDecision(
                SlotKey(config.boot_epoch, self._next_slot), "empty", None, (),
                None, None, ClosingSamples(closing, None, None),
            )
        self._next_slot += 1
        for history in self._streams.values():
            history.retain_from(upper)
        return decision

    def _associate(
        self, candidates: tuple[SourceSample, ...], closing: SourceSample,
    ) -> SelectionDecision | None:
        selected = candidates[-1]
        time_us = selected.time_us
        truth = self._streams[SelectionStream.TRUTH]
        speed = self._streams[SelectionStream.AIRSPEED]
        truth_close = truth.at_or_after(time_us)
        speed_close = speed.at_or_after(time_us)
        if truth_close is None or speed_close is None:
            return None
        truth_lower = truth.at_or_before(time_us)
        airspeed = speed.at_or_before(time_us)
        # Defensive: complete pre-start coverage plus retention preserves both.
        if truth_lower is None:
            self._fail("truth_unbracketed", SelectionStream.TRUTH, time_us)
            return None
        if truth_close.time_us - truth_lower.time_us > self._config.max_truth_gap_us:
            self._fail("truth_gap", SelectionStream.TRUTH, time_us)
            return None
        if airspeed is None:
            self._fail("airspeed_unavailable", SelectionStream.AIRSPEED, time_us)
            return None
        return SelectionDecision(
            SlotKey(self._config.boot_epoch, self._next_slot), "fresh", selected,
            candidates[:-1], (truth_lower, truth_close), airspeed,
            ClosingSamples(closing, truth_close, speed_close),
        )

    def seal(self) -> SelectionVerdict:
        """Freeze the final verdict; caller must finish feeding and polling first."""
        if self._verdict is None:
            config = self._config
            if self._next_slot < config.end_slot_exclusive:
                self._fail("incomplete_window", None, None)
            self._verdict = SelectionVerdict(
                config.boot_epoch, config.start_slot, config.end_slot_exclusive,
                self._violation,
            )
        return self._verdict

    def _fail(self, reason: str, stream: SelectionStream | None, source_us: int | None) -> None:
        if self._violation is None:
            self._violation = SelectionViolation(
                reason, stream, SlotKey(self._config.boot_epoch, self._next_slot), source_us,
            )

    def _require_open(self) -> None:
        if self._verdict is not None:
            raise RuntimeError("selector is sealed")
