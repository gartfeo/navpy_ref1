"""Associate actual projection inputs with their render transaction and publication."""
from __future__ import annotations

from contextlib import contextmanager, nullcontext
from contextvars import ContextVar
from dataclasses import asdict, dataclass
import os
from pathlib import Path
import threading
import uuid
from typing import Any, Callable, Iterator, Optional

from navpy.exception_groups import BaseExceptionGroup
from navpy.modules.vision.sim.finite_projection_evidence import FiniteProjectionEvidence
from navpy.modules.vision.sim.run_evidence_file import EvidenceBudget, RunEvidenceFile
from navpy.modules.vision.sim.sim_frame_types import FrameContext


@dataclass
class FrameEvidence:
    frame_id: int
    epoch: int
    generation: int
    timestamp_s: float
    declared_epoch: Optional[int]
    receipt_timestamp_s: Optional[float]
    publication: str = 'abandoned'
    published_poi_count: Optional[int] = None
    exception_type: Optional[str] = None

    def published(self, accepted: bool, poi_count: int) -> None:
        self.publication = ('successful' if poi_count else 'empty') if accepted else 'rejected'
        self.published_poi_count = poi_count if accepted else None


class ProjectionRunRecorder:
    def __init__(self, output: RunEvidenceFile) -> None:
        self._output = output
        self._frame: ContextVar[Optional[FrameEvidence]] = ContextVar('projection_frame', default=None)
        self._lock = threading.Lock()
        self._frame_count = 0
        self._active = 0

    @property
    def output(self) -> RunEvidenceFile:
        return self._output

    def __call__(self, record: FiniteProjectionEvidence) -> None:
        try:
            frame = self._frame.get()
            if frame is None:
                raise ValueError('projection without frame association')
            self._output.emit('projection', dict(frame=asdict(frame), projection=asdict(record)))
            if not record.outcome.complete or record.source.source_timestamp_s != frame.timestamp_s:
                raise ValueError('incomplete projection or frame timestamp mismatch')
        except Exception as error:
            self.failed_notice(f'projection preparation/emission: {type(error).__name__}: {error}')
            raise

    def failed_notice(self, message: str) -> None:
        if self._output.status['failure'] is None:
            self._output.invalidate(message)

    @contextmanager
    def frame(self, context: FrameContext, declared_epoch: Optional[int]) -> Iterator[FrameEvidence]:
        with self._lock:
            self._frame_count += 1
            self._active += 1
            frame = FrameEvidence(self._frame_count, context.generation.epoch,
                                  context.generation.generation, context.timestamp_s,
                                  declared_epoch, context.receipt_timestamp_s)
        token = self._frame.set(frame)
        try:
            yield frame
        except BaseException as error:
            frame.exception_type = type(error).__name__
            raise
        finally:
            try:
                self._output.emit('publication', dict(frame=asdict(frame)))
            except Exception as error:
                self.failed_notice(f'publication preparation/emission: {type(error).__name__}: {error}')
            self._frame.reset(token)
            with self._lock:
                self._active -= 1

    def close(self) -> None:
        with self._lock:
            if self._active:
                self._output.invalidate('close with active frame transactions')
                raise RuntimeError('cannot close evidence while frame transactions are active')
            self._output.close()


@contextmanager
def projection_frame_scope(recorder: Optional[ProjectionRunRecorder], context: FrameContext,
                           declared_epoch: Optional[int]) -> Iterator[Optional[FrameEvidence]]:
    with (recorder.frame(context, declared_epoch) if recorder else nullcontext(None)) as frame:
        yield frame


def create_projection_recorder(sysid: int, *, ideal_360: bool,
                               name_reader: Optional[Callable[[], str]] = None) -> Optional[ProjectionRunRecorder]:
    if ideal_360 or os.environ.get('NAVPY_SIM_PROJECTION_RECORDER') != '1':
        return None
    directory = Path(os.environ['NAVPY_SIM_EVIDENCE_DIR'])
    run_id = os.environ['NAVPY_SIM_EVIDENCE_RUN_ID']
    budget = EvidenceBudget(int(os.environ['NAVPY_SIM_EVIDENCE_MAX_RECORDS']),
                            int(os.environ['NAVPY_SIM_EVIDENCE_MAX_BYTES']))
    name = name_reader() if name_reader is not None else 'finite'
    return ProjectionRunRecorder(RunEvidenceFile(directory, run_id, sysid, budget,
                                                detector_id=uuid.uuid4().hex, detector_name=name))


def rollback_projection_recorder(recorder: Optional[ProjectionRunRecorder], cause: BaseException) -> None:
    if recorder is None:
        return
    recorder.output.invalidate('detector composition failed')
    try:
        recorder.close()
    except BaseException as cleanup:
        raise BaseExceptionGroup('detector composition and recorder cleanup failed', [cause, cleanup]) from None
