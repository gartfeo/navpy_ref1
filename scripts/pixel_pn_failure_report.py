"""What a failed direct-pixel child publishes: every failure, and never a raise.

Split out of ``direct_pixel_pn_child`` (a 300-line module limit), and it
imports nothing but the standard library, so it cannot fail to load on the one
path that must always publish.

WHY IT WALKS THE FAILURES ITSELF. Once the determinism summary's own write has
failed, the published result is the only record left of why the run failed
(review round 5). Review round 6 found two ways it still lost failures. On
Python 3.10 NavPy's exception groups are a fallback class
(``navpy.exception_groups``), which ``traceback`` renders as one plain
exception with its members dropped, so the walk expands anything that carries
a tuple of exceptions, native group or not. And formatting can raise (a
SyntaxError whose ``text`` is not a string is enough), which cost the result.

WHY IT PRINTS EACH EXCEPTION ITSELF. Asked for one exception,
``traceback.format_exception_only`` first builds the whole graph under it,
every cause, context and member with its traceback (Python 3.11,
traceback.py:156 and 758-825). Asked for each exception in turn, it read a
chain n deep n(n+1)/2 times, 4000 deep in 15 s, and a member it could not
build cost its healthy group's own message and notes (review round 7). So each
exception is printed from its own fields alone, in the standard formatter's
words; for anything without a group, the tests hold the report to exactly the
standard text.

WHAT IT PROMISES.

- It cannot raise. A read that fails costs only what depended on it, and a
  line says so in its place: one exception's text, its chain, or its members.
  An interrupt raised by a read counts as such a failure (review round 8):
  the report is written after the run has ended, to publish why it failed.
  A type whose name cannot be read as text is named by a literal.
- Each exception is visited once, from the walk's own stack, so no depth is
  too deep. What is a group, a member or a cause is decided from types, never
  by an object's own code, and members come from the tuple's own storage: a
  tuple subclass whose iterator never ended hung the walk (review round 7).
- Reading an exception runs its own code, such as ``__str__``, its notes or a
  property, and that takes as long as it takes, as with the standard
  formatter. Reading ``exceptions`` on an exception that is not a native group
  is the one read the standard formatter does not make: it is how a 3.10
  fallback group is recognised.
"""

from __future__ import annotations

import traceback
from collections.abc import Callable, Sequence
from typing import Any

NOT_FORMATTED = "could not be formatted"
MEMBERS_UNREADABLE = "its members could not be read"
CHAIN_UNREADABLE = "its cause or context could not be read"
RENDERING_FAILED = "traceback rendering failed"
NAME_UNREADABLE = "<type name unreadable>"
STR_FAILED = "<str() failed>"
REPEATED = "<rendered once, elsewhere in this report>"
_TRACEBACK = "Traceback (most recent call last):\n"
_CAUSE = "\nThe above exception was the direct cause of the following exception:\n\n"
_CONTEXT = "\nDuring handling of the above exception, another exception occurred:\n\n"
_INDENT = "    "
# What the walk's stack holds, each with the indent it prints at: an exception
# to expand into its parts, one to print on its own, or finished text.
_EXPAND, _FORMAT, _TEXT = "expand", "format", "text"
_Step = tuple[str, Any, str]


def failure_payload(error: BaseException) -> dict[str, object]:
    """The result a failed run publishes. Cannot raise."""
    return {"passed": False, "error": _one_line(error), "traceback": _rendered(error)}


def render_failure(error: BaseException) -> str:
    """``error`` with its chain and every group member, each exception once."""
    lines: list[str] = []
    seen: set[int] = set()
    stack: list[_Step] = [(_EXPAND, error, "")]
    while stack:
        kind, item, indent = stack.pop()
        if kind == _EXPAND:
            stack += reversed(_steps(item, indent, seen))
        elif kind == _FORMAT:
            lines += _indented(_own_text(item), indent)
        else:
            lines += _indented(item, indent)
    return "".join(lines)


def _one_line(error: BaseException) -> str:
    name = _type_name(error)
    try:
        return f"{name}: {error}"
    except BaseException:  # noqa: BLE001 - the result must still be published
        return f"{name}: {STR_FAILED}"


def _rendered(error: BaseException) -> str:
    try:
        return render_failure(error)
    except BaseException as failed:  # noqa: BLE001 - must still be published
        return _marker(RENDERING_FAILED, failed)


def _type_name(value: object) -> str:
    """``type(value).__name__``, or a literal when it cannot be read as text.

    A metaclass can make the read raise, or answer with something that is not
    text (review round 7); every line that names a type comes through here.
    """
    try:
        name = type(value).__name__
    except BaseException:  # noqa: BLE001 - a name is not worth the result
        return NAME_UNREADABLE
    return name if type(name) is str else NAME_UNREADABLE


def _marker(what: str, failed: BaseException) -> str:
    """The line printed where something could not be read, and why."""
    return f"<{what}: {_type_name(failed)}>\n"


def _steps(error: BaseException, indent: str, seen: set[int]) -> list[_Step]:
    """``error`` in output order: what it was chained to, as the standard
    formatter orders them, then the exception itself, then each member
    indented under it. A part that cannot be read is replaced by its marker."""
    if id(error) in seen:
        # A chain can loop, and one exception can be reached twice.
        return [(_TEXT, f"{_type_name(error)}: {REPEATED}\n", indent)]
    seen.add(id(error))
    steps: list[_Step] = []
    try:
        chained = _chained(error)
    except BaseException as failed:  # noqa: BLE001 - its chain is reported
        steps.append((_TEXT, _marker(CHAIN_UNREADABLE, failed), indent))
        chained = None
    if chained is not None:
        earlier, joiner = chained
        steps += [(_EXPAND, earlier, indent), (_TEXT, joiner, indent)]
    steps.append((_FORMAT, error, indent))
    try:
        members = _members(error)
    except BaseException as failed:  # noqa: BLE001 - its members are reported
        steps.append((_TEXT, _marker(MEMBERS_UNREADABLE, failed), indent))
        members = ()
    for index, member in enumerate(members, 1):
        header = f"+---------------- {index} of {len(members)} ----------------\n"
        steps += [(_TEXT, header, indent), (_EXPAND, member, indent + _INDENT)]
    return steps


def _chained(error: BaseException) -> tuple[BaseException, str] | None:
    """What the standard formatter prints before ``error``, with the line that
    joins the two: its cause, or else its context unless that is suppressed."""
    cause = error.__cause__
    if cause is not None:
        return _exception(cause), _CAUSE
    context = error.__context__
    if context is not None and not error.__suppress_context__:
        return _exception(context), _CONTEXT
    return None


def _exception(value: Any) -> BaseException:
    """``value``, if its type is an exception's: decided by the type, so none
    of the object's own code runs."""
    if issubclass(type(value), BaseException):
        return value
    raise TypeError("a cause or context that is not an exception")


def _members(error: BaseException) -> tuple[BaseException, ...]:
    """A group's members, native or the 3.10 fallback; () for anything else.

    Taken from the tuple's own storage and judged by their types: a tuple
    subclass's iterator can run forever (review round 7), and ``isinstance``
    would run an object's own ``__class__``.
    """
    members = getattr(error, "exceptions", None)
    if not issubclass(type(members), tuple):
        return ()
    stored = tuple.__getitem__(members, slice(None))
    if all(issubclass(type(member), BaseException) for member in stored):
        return stored
    return ()


def _indented(text: str, indent: str) -> list[str]:
    return [indent + line for line in text.splitlines(keepends=True)]


def _own_text(error: BaseException) -> str:
    """One exception's traceback, message and notes, never its chain or
    members: the standard formatter's words, read from this exception alone.

    Joined inside the guard: ``str()`` can hand back a str subclass whose own
    methods raise, and that too costs this exception's text only.
    """
    try:
        trace = error.__traceback__
        frames = [] if trace is None else [_TRACEBACK, *traceback.format_tb(trace)]
        return "".join([*frames, *_message_lines(error), *_note_lines(error)])
    except BaseException as failed:  # noqa: BLE001 - costs this exception's text only
        return f"{_type_name(error)}: <{NOT_FORMATTED}: {_type_name(failed)}>\n"


def _message_lines(error: BaseException) -> list[str]:
    """Its type and message, or a SyntaxError's lines (traceback.py:162-168,
    735, 862-872)."""
    name = _qualified_name(type(error))
    if issubclass(type(error), SyntaxError):
        return _syntax_error_lines(error, name)
    # Converted twice, as the standard formatter does: it keeps str(error),
    # then converts that again before testing it, so text that is a str
    # subclass prints through its own __str__ (review round 8).
    text = _safe_str(_safe_str(error, "exception"), "exception")
    return ["%s: %s\n" % (name, text) if text else "%s\n" % name]


def _qualified_name(cls: type) -> str:
    """The type after its module, unless that is __main__ or builtins."""
    name, module = cls.__qualname__, cls.__module__
    if module in ("__main__", "builtins"):
        return name
    if not isinstance(module, str):
        module = "<unknown>"
    return module + "." + name


def _safe_str(value: object, what: str, show: Callable[[object], str] = str) -> str:
    """``show(value)``, or what the standard formatter prints when that raises
    (traceback.py:170-174)."""
    try:
        return show(value)
    except BaseException:  # noqa: BLE001 - as the standard formatter does
        return f"<{what} {show.__name__}() failed>"


def _note_lines(error: BaseException) -> list[str]:
    """Its notes, as the standard formatter prints them (traceback.py:736-740,
    873-878): each line of each note, or the repr of notes that are not a
    sequence, with no newline after it. A read that raises an interrupt is
    reported in the same words; there the standard formatter would raise."""
    try:
        notes = getattr(error, "__notes__", None)
    except BaseException as failed:  # noqa: BLE001 - printed in the standard words
        shown = _safe_str(failed, "__notes__", repr)
        notes = [f"Ignored error getting __notes__: {shown}"]
    if isinstance(notes, Sequence):
        return [
            line + "\n"
            for note in notes
            for line in _safe_str(note, "note").split("\n")
        ]
    if notes is not None:
        return [_safe_str(notes, "__notes__", repr)]
    return []


def _syntax_error_lines(error: SyntaxError, name: str) -> list[str]:
    """Where a SyntaxError was found, its line, a caret and its message
    (traceback.py:742-752, 880-914)."""
    lineno, filename, text = error.lineno, error.filename, error.text
    lines: list[str] = []
    suffix = ""
    if lineno is not None:
        lines.append(f'  File "{filename or "<string>"}", line {str(lineno)}\n')
    elif filename is not None:
        suffix = f" ({filename})"
    if text is not None:
        rtext = text.rstrip("\n")
        ltext = rtext.lstrip(" \n\f")
        lines.append(f"    {ltext}\n")
        lines += _caret_line(error, ltext, len(rtext) - len(ltext))
    message = error.msg or "<no detail available>"
    lines.append(f"{name}: {message}{suffix}\n")
    return lines


def _caret_line(error: SyntaxError, ltext: str, spaces: int) -> list[str]:
    """The carets under the offending text; none when there is no offset, or
    it falls inside the indent that was stripped."""
    offset, end_offset = error.offset, error.end_offset
    if offset is None:
        return []
    if end_offset in {None, 0}:
        end_offset = offset
    if offset == end_offset or end_offset == -1:
        end_offset = offset + 1
    colno, end_colno = offset - 1 - spaces, end_offset - 1 - spaces
    if colno < 0:
        return []
    # A tab before the caret stays a tab, so the caret lines up under it.
    caretspace = "".join(c if c.isspace() else " " for c in ltext[:colno])
    return [f"    {caretspace}{'^' * (end_colno - colno)}\n"]
