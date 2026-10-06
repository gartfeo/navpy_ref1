"""Shared Node.js runner for the ``tests/gcs`` JS-utility tests.

These tests assemble a stripped JS source plus a test snippet into one string and
execute it with Node. The original ``subprocess.run(["node", "-e", code])`` passes
the whole program on the command line, which on Windows exceeds the ~32767-char
``CreateProcess`` limit (``FileNotFoundError: [WinError 206]``) for large modules
such as ``planner.js``.

Feeding the program to ``node`` via **stdin** avoids that limit while preserving
the same CommonJS / sloppy-mode semantics as ``node -e`` (top-level
``this === globalThis``, implicit globals, no temp file, no module-type ambiguity).
"""
import subprocess


def run_node(code, timeout=10):
    """Execute JS ``code`` with Node via stdin; return the ``CompletedProcess``.

    Passing the program on stdin (not ``node -e <code>``) avoids the Windows
    command-line length limit. Callers keep their own stdout parsing and
    return-code handling, exactly as with the previous ``subprocess.run`` call.
    """
    return subprocess.run(
        ["node"],
        input=code,
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=timeout,
    )
