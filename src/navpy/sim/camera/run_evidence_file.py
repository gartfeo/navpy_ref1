"""Bounded exclusive-create evidence file; incomplete captures never certify a run."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import threading
from typing import Any, BinaryIO, Callable, Optional


@dataclass(frozen=True)
class EvidenceBudget:
    max_records: int
    max_bytes: int

    def __post_init__(self) -> None:
        if self.max_records <= 0 or self.max_bytes <= 0:
            raise ValueError('evidence budgets must be positive')


@dataclass
class EvidenceCounts:
    submitted: int = 0
    written: int = 0
    failed: int = 0
    bytes_written: int = 0
    budget_exhausted: bool = False
    closed: bool = False
    failure: Optional[str] = None


FileOpener = Callable[[Path, str], BinaryIO]


def open_evidence_file(path: Path, mode: str) -> BinaryIO:
    return path.open(mode)


class RunEvidenceFile:
    """Serialize allocation and writes under one lock; no rotation or background queue."""
    def __init__(self, directory: Path, run_id: str, sysid: int, budget: EvidenceBudget,
                 *, opener: FileOpener = open_evidence_file, detector_id: str = 'default',
                 detector_name: str = 'finite') -> None:
        if not run_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in run_id):
            raise ValueError('run_id must be a nonempty safe filename component')
        if not detector_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in detector_id):
            raise ValueError('detector_id must be a nonempty safe filename component')
        if not isinstance(detector_name, str) or not detector_name:
            raise ValueError('detector_name must be nonempty text')
        self._identity = dict(run_id=run_id, pid=os.getpid(), vehicle_sysid=sysid,
                              detector_id=detector_id, detector_name=detector_name)
        self._budget = budget
        self._counts = EvidenceCounts()
        self._lock = threading.RLock()
        self._hash = hashlib.sha256()
        self._opener = opener
        directory.mkdir(parents=True, exist_ok=True)
        self._path = directory / f'{run_id}-pid{os.getpid()}-uav{sysid}-detector{detector_id}.jsonl'
        self._file = opener(self._path, 'xb')
        self._pending_manifest: Optional[BinaryIO] = None

    @property
    def path(self) -> Path:
        return self._path

    @property
    def status(self) -> dict[str, Any]:
        with self._lock:
            return self._manifest()

    def emit(self, kind: str, payload: dict[str, Any]) -> int:
        with self._lock:
            if self._counts.closed:
                raise RuntimeError('submission after close')
            self._counts.submitted += 1
            if self._counts.failure is not None:
                self._fail('capture already invalid')
                raise RuntimeError(self._counts.failure)
            try:
                sequence = self._counts.written + 1
                row = dict(self._identity, record_sequence=sequence, kind=kind, **payload)
                data = (json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')
                if sequence > self._budget.max_records or self._counts.bytes_written + len(data) > self._budget.max_bytes:
                    self._counts.budget_exhausted = True
                    raise OverflowError('evidence storage budget exhausted')
                size = self._file.write(data)
                if size != len(data):
                    raise OSError('short evidence write')
            except Exception as error:
                self._fail(f'{type(error).__name__}: {error}')
                raise
            self._counts.written += 1
            self._counts.bytes_written += len(data)
            self._hash.update(data)
            return sequence

    def invalidate(self, reason: str) -> None:
        with self._lock:
            self._fail(reason)

    def close(self) -> None:
        with self._lock:
            if self._counts.closed:
                try:
                    self._close_pending_manifest()
                except Exception as error:
                    self._fail(f'manifest close retry: {type(error).__name__}: {error}')
                    raise
                return
            errors: list[Exception] = []
            if not self._file.closed:
                try:
                    self._file.flush()
                    os.fsync(self._file.fileno())
                except Exception as error:
                    self._fail(f'flush: {type(error).__name__}: {error}')
                    errors.append(error)
                try:
                    self._file.close()
                except Exception as error:
                    self._fail(f'close: {type(error).__name__}: {error}')
                    errors.append(error)
            self._counts.closed = self._file.closed
            if not self._counts.closed:
                if not errors:
                    self._fail('data file did not close')
                raise errors[0] if errors else RuntimeError('data file did not close')
            try:
                data = (json.dumps(self._manifest(), allow_nan=False, sort_keys=True) + '\n').encode('utf-8')
                pending = self._path.with_suffix('.manifest.pending')
                self._pending_manifest = self._opener(pending, 'xb')
                try:
                    if self._pending_manifest.write(data) != len(data):
                        raise OSError('short manifest write')
                    self._pending_manifest.flush()
                    os.fsync(self._pending_manifest.fileno())
                finally:
                    self._close_pending_manifest()
                os.link(pending, self._path.with_suffix('.manifest.json'))
            except Exception as error:
                self._fail(f'manifest: {type(error).__name__}: {error}')
                errors.append(error)
            if errors:
                raise errors[0]

    def _close_pending_manifest(self) -> None:
        if self._pending_manifest is not None:
            if not self._pending_manifest.closed:
                self._pending_manifest.close()
            self._pending_manifest = None

    def _fail(self, reason: str) -> None:
        self._counts.failed += 1
        if self._counts.failure is None:
            self._counts.failure = reason

    def _manifest(self) -> dict[str, Any]:
        return dict(self._identity, **asdict(self._counts), budget=asdict(self._budget),
                    complete=self._counts.closed and self._counts.failure is None,
                    first_sequence=1 if self._counts.written else None,
                    last_sequence=self._counts.written or None, sha256=self._hash.hexdigest())
