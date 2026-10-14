"""Host paths out of ``DONE`` text: the run's directories become stable tokens.

A participant runs in its host log directory (its cwd); the run
root is that directory's parent. In a log line every spelling of either
(native, POSIX, or with backslashes doubled as a ``repr`` shows them), with
the rest of its path, becomes :data:`RUN_TOKEN` or :data:`ROOT_TOKEN` plus a
POSIX remainder, so ``backend.log`` reads ``<run>/gcs/backend.log`` on every
host. Any other absolute path (a drive or UNC path, or a POSIX path under a
host root, never a URL route) becomes :data:`ABSOLUTE_TOKEN`.
"""

from __future__ import annotations

import re
from pathlib import Path

RUN_TOKEN = "<run>"
ROOT_TOKEN = "<root>"
ABSOLUTE_TOKEN = "<abs>"
#: A path's name characters (log punctuation ends a path), and the rest of a
#: path after a known directory.
_NAME = r"[^\s'\"<>|;,\\/]"
_REST = r"((?:[\\/]+" + _NAME + r"+)*)"
#: Absolute paths that are not the run's.
_OTHER_ABSOLUTE = re.compile(
    r"(?<!\w)[A-Za-z]:[\\/]+" + _NAME + r"*" + _REST
    + r"|\\\\+" + _NAME + r"+" + _REST
    + r"|(?<![\w.:/<])/(?:home|Users|tmp|usr|var|opt|mnt|root|private|etc|srv|proc)/" + _NAME + r"*" + _REST
)
_SEPARATORS = re.compile(r"[\\/]+")


def _spellings(path: Path) -> list[str]:
    native = str(path)
    return sorted({native, path.as_posix(), native.replace("\\", "\\\\")}, key=len, reverse=True)


class HostPathScrubber:
    def __init__(self, run_dir: Path) -> None:
        run_dir = run_dir.resolve()
        known = [(spelling, RUN_TOKEN) for spelling in _spellings(run_dir)]
        known += [(spelling, ROOT_TOKEN) for spelling in _spellings(run_dir.parent)]  # after the run dir
        self._known = [(re.compile(re.escape(spelling) + _REST, re.IGNORECASE), token)
                       for spelling, token in known]

    def __call__(self, text: str) -> str:
        for pattern, token in self._known:
            text = pattern.sub(lambda m, t=token: t + _SEPARATORS.sub("/", m.group(1)), text)
        return _OTHER_ABSOLUTE.sub(ABSOLUTE_TOKEN, text)


__all__ = ["ABSOLUTE_TOKEN", "HostPathScrubber", "ROOT_TOKEN", "RUN_TOKEN"]
