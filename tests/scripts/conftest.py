"""Fixtures shared by the scripts tests."""

from __future__ import annotations

import builtins
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

SUMMARY_WRITE_FAILURES = ("second_interrupt", "broken_stdout", "unwritable")


@pytest.fixture
def failing_summary_write(
    monkeypatch: pytest.MonkeyPatch,
) -> Callable[..., list[BaseException]]:
    """Make one determinism evidence file's write fail, the ways review
    round 4 made the summary's fail.

    ``name`` is the file, the summary unless another is named. Every file is
    written under a staging name beside itself and then moved into place,
    so it is that staged write that fails; the file itself is never written
    but by the move.

    ``"second_interrupt"``: the write is itself interrupted, a second Ctrl+C,
    once half the bytes have reached the staged copy.
    ``"broken_stdout"``: the write fails with an OSError, and announcing that
    on stdout fails too, as it does once the harness reading it has gone.
    ``"unwritable"``: the write fails with an OSError and the announcement
    works -- the failure the writer HANDLES, as opposed to one that escapes it.

    Returns the list the injected errors are appended to, in the order they
    were raised. Only the named file's write and the evidence's marker are
    touched; every other write and print goes through.
    """
    from scripts.pixel_pn_determinism_summary import (
        STAGING_SUFFIX,
        SUMMARY_NAME,
        UNWRITABLE_MARKER,
    )

    def arm(how: str, name: str = SUMMARY_NAME) -> list[BaseException]:
        if how not in SUMMARY_WRITE_FAILURES:
            raise ValueError(f"unknown summary write failure: {how!r}")
        injected: list[BaseException] = []
        real_write_bytes = Path.write_bytes
        real_print = builtins.print

        def write_bytes(path: Path, data: bytes) -> int:
            if path.name != name + STAGING_SUFFIX:
                return real_write_bytes(path, data)
            error: BaseException
            if how == "second_interrupt":
                real_write_bytes(path, data[: len(data) // 2])
                error = KeyboardInterrupt("a second ctrl+c, mid-write")
            else:
                error = PermissionError("artifact locked")
            injected.append(error)
            raise error

        def print_(*args: Any, **kwargs: Any) -> None:
            if (
                how == "broken_stdout"
                and args
                and str(args[0]).startswith(UNWRITABLE_MARKER)
            ):
                error = BrokenPipeError("stdout closed")
                injected.append(error)
                raise error
            real_print(*args, **kwargs)

        monkeypatch.setattr(Path, "write_bytes", write_bytes)
        monkeypatch.setattr(builtins, "print", print_)
        return injected

    return arm
