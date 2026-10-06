"""What a failed child publishes: every failure, and never a raise.

Review round 6 found the child's published traceback incomplete in two ways.
On Python 3.10 NavPy's exception groups are a fallback class, which the
traceback module renders as one plain exception with its members dropped; and
a formatter that raised took the whole result with it. The renderer is held to
both here, with the native group class AND the 3.10 fallback, which is
importable on any Python. A probe of the first fix found a third: walked by
recursion, a chain or group deeper than the interpreter's recursion limit was
lost or cut off, so the depth tests go past that limit.

Review round 7 found three more. Asked for one exception, the standard
formatter first builds the whole graph under it, so asking it for each
exception in turn read a chain n deep n(n+1)/2 times, and a member it could not
build cost its healthy group's own text. A type whose name could not be read
took the payload with it. And a tuple subclass whose iterator never ended hung
the walk. The standard formatter stays the reference for everything it can
render without a group: the report must print exactly what it prints.

Review round 8 found two more. The text ``str()`` hands back was tested as it
came, where the standard formatter converts it once more first, so a str
subclass that tests false lost the message. And the report's own guards caught
Exception, so an interrupt raised by one read cost the whole report.
"""

from __future__ import annotations

import itertools
import sys
import traceback
from collections.abc import Callable
from typing import Any

import pytest

from navpy.exception_groups import (
    BaseExceptionGroup,
    _FallbackBaseExceptionGroup,
)
from scripts import pixel_pn_failure_report as report
from scripts.pixel_pn_failure_report import (
    MEMBERS_UNREADABLE,
    NOT_FORMATTED,
    RENDERING_FAILED,
    REPEATED,
    STR_FAILED,
    failure_payload,
    render_failure,
)

GROUPS = {
    "native": BaseExceptionGroup,
    "fallback": _FallbackBaseExceptionGroup,
}


def _raised(error: BaseException) -> BaseException:
    """``error`` carrying a real traceback, as ``main()`` catches it."""
    try:
        raise error
    except BaseException as caught:  # noqa: BLE001 - handed back, not swallowed
        return caught


@pytest.mark.parametrize("groups", sorted(GROUPS))
def test_every_member_of_nested_groups_is_rendered_once_under_its_group(groups):
    group = GROUPS[groups]
    inner = group(
        "the summary's write failed too",
        [KeyboardInterrupt("ctrl+c"), PermissionError("artifact locked")],
    )
    outer = group("teardown failed", [PermissionError("csv locked"), inner])

    text = render_failure(_raised(outer))

    lines = text.splitlines()
    # One indent per level of group, so each member shows whose it is.
    assert "    PermissionError: csv locked" in lines
    assert "        KeyboardInterrupt: ctrl+c" in lines
    assert "        PermissionError: artifact locked" in lines
    # And each only once: the formatter must not expand a group the walk has.
    for member in ("csv locked", "ctrl+c", "artifact locked"):
        assert text.count(member) == 1, member
    # The group was raised, so where it was raised is kept as well.
    assert "Traceback (most recent call last):" in lines
    assert any(line.endswith(", in _raised") for line in lines)


def test_a_cause_comes_first_and_a_suppressed_context_not_at_all():
    try:
        try:
            raise ValueError("hidden context")
        except ValueError:
            raise RuntimeError("the failure") from OSError("the cause")
    except RuntimeError as caught:
        text = render_failure(caught)

    assert text.index("OSError: the cause") < text.index("RuntimeError: the failure")
    assert "direct cause of the following exception" in text
    assert "hidden context" not in text


def test_a_context_suppressed_with_from_none_is_not_rendered():
    """``from None`` sets no cause, so only the suppression flag hides it."""
    try:
        try:
            raise ValueError("hidden context")
        except ValueError:
            raise RuntimeError("the failure") from None
    except RuntimeError as caught:
        text = render_failure(caught)

    assert "RuntimeError: the failure" in text
    assert "hidden context" not in text


def test_a_context_comes_first():
    try:
        try:
            raise ValueError("the first failure")
        except ValueError:
            raise RuntimeError("the second failure")
    except RuntimeError as caught:
        text = render_failure(caught)

    assert text.index("ValueError: the first failure") < text.index(
        "RuntimeError: the second failure"
    )
    assert "During handling of the above exception" in text


def test_a_chain_that_loops_is_rendered_once_and_ends(monkeypatch):
    first, second = ValueError("first"), TypeError("second")
    first.__context__, second.__context__ = second, first
    expanded: list[BaseException] = []
    steps = report._steps

    def bounded(error: BaseException, *rest: object) -> list:
        # A walk that never ended would hang the suite instead of failing it.
        expanded.append(error)
        if len(expanded) > 10:
            raise AssertionError("the walk did not end")
        return steps(error, *rest)

    monkeypatch.setattr(report, "_steps", bounded)

    text = render_failure(first)

    assert text.count("ValueError: first") == 1
    assert text.count("TypeError: second") == 1


def test_a_chain_deeper_than_the_recursion_limit_is_rendered_in_full():
    depth = sys.getrecursionlimit() + 100
    error = ValueError("0")
    for index in range(1, depth):
        later = ValueError(str(index))
        later.__context__ = error
        error = later

    text = render_failure(error)

    assert text.count("ValueError: ") == depth


@pytest.mark.parametrize("groups", sorted(GROUPS))
def test_a_group_nested_deeper_than_the_recursion_limit_keeps_every_level(groups):
    depth = sys.getrecursionlimit() + 100
    error = ValueError("the innermost failure")
    for level in range(depth):
        error = GROUPS[groups](f"level {level}", [error])

    text = render_failure(error)

    assert text.count("+---------------- 1 of 1 ----------------") == depth
    assert "ValueError: the innermost failure" in text


def test_members_that_cannot_be_read_are_reported_not_dropped():
    class Opaque(Exception):
        @property
        def exceptions(self) -> tuple[BaseException, ...]:
            raise RuntimeError("no members today")

    text = render_failure(Opaque("a group of sorts"))

    assert "a group of sorts" in text
    assert f"<{MEMBERS_UNREADABLE}: RuntimeError>" in text


def test_an_exception_that_cannot_be_formatted_costs_only_its_own_text():
    """The standard formatter raises for a SyntaxError whose ``text`` is not
    a string; the rest of the group must still be rendered."""
    malformed = SyntaxError("bad extension syntax")
    malformed.text = 42
    group = BaseExceptionGroup(
        "teardown failed", [malformed, KeyboardInterrupt("ctrl+c")]
    )

    text = render_failure(_raised(group))

    assert f"SyntaxError: <{NOT_FORMATTED}: AttributeError>" in text
    assert "KeyboardInterrupt: ctrl+c" in text


def test_the_payload_survives_a_rendering_that_fails(monkeypatch):
    def rendering_down(*_: object) -> str:
        raise MemoryError("the report is too large")

    monkeypatch.setattr(report, "render_failure", rendering_down)

    assert failure_payload(RuntimeError("the failure")) == {
        "passed": False,
        "error": "RuntimeError: the failure",
        "traceback": f"<{RENDERING_FAILED}: MemoryError>\n",
    }


def test_the_payload_survives_an_exception_whose_str_raises():
    class Unprintable(RuntimeError):
        def __str__(self) -> str:
            raise ValueError("no text")

    payload = failure_payload(_raised(Unprintable()))

    assert payload["passed"] is False
    assert payload["error"] == f"Unprintable: {STR_FAILED}"
    assert "Unprintable" in payload["traceback"]


# --- review round 7 -----------------------------------------------------------


def _without_raising(build: Callable[..., Any], *args: object) -> Any:
    """``build(*args)``, failing the test cleanly if anything escapes.

    Named outside the handler, and by ``type.__repr__``, which reads nothing
    through a metaclass: what escapes can be an exception whose type has no
    readable name, and pytest's own report of it would then fail as well.
    """
    try:
        return build(*args)
    except BaseException as error:  # noqa: BLE001 - reported below, by name
        escaped = type.__repr__(type(error))
    pytest.fail(f"the report raised {escaped}", pytrace=False)


class Counted(ValueError):
    """Counts how often the report asks for its text."""

    reads = 0

    def __str__(self) -> str:
        self.reads += 1
        return super().__str__()


def _chain_of_counted(link: str) -> tuple[BaseException, list[Counted]]:
    counted = [Counted(f"link {index}") for index in range(40)]
    for earlier, later in zip(counted, counted[1:]):
        setattr(later, link, earlier)
    return counted[-1], counted


def _groups_of_counted(group: type) -> tuple[BaseException, list[Counted]]:
    counted = [Counted(f"member {index}") for index in range(40)]
    error: BaseException = counted[0]
    for level, member in enumerate(counted[1:]):
        error = group(f"level {level}", [member, error])
    return error, counted


READ_ONCE = {
    "cause_chain": lambda: _chain_of_counted("__cause__"),
    "context_chain": lambda: _chain_of_counted("__context__"),
    "native_groups": lambda: _groups_of_counted(BaseExceptionGroup),
    "fallback_groups": lambda: _groups_of_counted(_FallbackBaseExceptionGroup),
}


@pytest.mark.parametrize("graph", sorted(READ_ONCE))
def test_each_exception_is_read_once_however_its_failures_are_joined(graph):
    """Review round 7. ``traceback.format_exception_only`` builds the whole
    graph under an exception before it prints that exception's message, so
    asking it for each exception in turn read a chain n deep n(n+1)/2 times.
    A chain 4000 deep took about 15 s; the standard formatter takes 0.015 s."""
    error, counted = READ_ONCE[graph]()

    _without_raising(render_failure, error)

    assert [member.reads for member in counted] == [1] * len(counted)


class BadTrace(Exception):
    """An exception the report cannot format: its traceback cannot be read."""

    def __getattribute__(self, name: str) -> object:
        if name == "__traceback__":
            raise ValueError("traceback unavailable")
        return super().__getattribute__(name)


@pytest.mark.parametrize("groups", sorted(GROUPS))
def test_a_member_that_cannot_be_formatted_costs_its_group_nothing(groups):
    """Review round 7: to print a native group's own message the standard
    formatter first builds its members, so one member it could not build cost
    the healthy group its message and its notes."""
    group = GROUPS[groups](
        "healthy parent metadata", [BadTrace("bad"), ValueError("healthy sibling")]
    )
    group.add_note("healthy parent note")

    lines = _without_raising(render_failure, group).splitlines()

    assert any(
        line.endswith(": healthy parent metadata (2 sub-exceptions)") for line in lines
    )
    assert "healthy parent note" in lines
    assert f"    BadTrace: <{NOT_FORMATTED}: ValueError>" in lines
    assert "    ValueError: healthy sibling" in lines


class BrokenCause(Exception):
    """An exception whose cause cannot be read."""

    @property
    def __cause__(self) -> BaseException | None:
        raise RuntimeError("no cause today")


class CauseNotAnException(Exception):
    """An exception whose cause, once read, is not an exception at all."""

    @property
    def __cause__(self) -> object:
        return "not an exception"


@pytest.mark.parametrize("groups", sorted(GROUPS))
@pytest.mark.parametrize(
    ("broken", "failure"),
    [(BrokenCause, "RuntimeError"), (CauseNotAnException, "TypeError")],
    ids=["raises", "not_an_exception"],
)
def test_a_chain_that_cannot_be_read_costs_only_that_link(groups, broken, failure):
    """Found while fixing round 7: the walk read each cause outside every
    guard, so one member's broken cause cost the whole rendering, its healthy
    group and siblings included."""
    group = GROUPS[groups](
        "healthy parent", [broken("its chain is broken"), ValueError("healthy sibling")]
    )

    lines = _without_raising(render_failure, group).splitlines()

    assert any(line.endswith(": healthy parent (2 sub-exceptions)") for line in lines)
    assert f"    <{report.CHAIN_UNREADABLE}: {failure}>" in lines
    assert any(line.endswith(f"{broken.__name__}: its chain is broken") for line in lines)
    assert "    ValueError: healthy sibling" in lines


class _StorageOnly(tuple):
    """A tuple whose own accessors the report must not use."""

    def __iter__(self):
        raise AssertionError("the report iterated the subclass")

    def __len__(self) -> int:
        raise AssertionError("the report asked the subclass its length")

    def __getitem__(self, index):
        raise AssertionError("the report indexed the subclass")


class _Repeating(tuple):
    """Review round 7's input: one stored member, and an iterator that repeats
    it forever. Bounded here, so that a regression fails instead of hanging
    the suite."""

    def __iter__(self):
        return itertools.repeat(tuple.__getitem__(self, 0), 10_000)

    def __len__(self) -> int:
        return 10_000


@pytest.mark.parametrize(
    "container", [_StorageOnly, _Repeating], ids=["accessors_raise", "repeats_its_member"]
)
def test_members_are_read_from_the_tuples_own_storage(container):
    """Review round 7: the members were checked through the tuple's own
    iterator, so one that never ended hung the walk before any repeat could be
    seen, and with nothing raised there was nothing for a handler to catch."""

    class Duck(Exception):
        exceptions = container((ValueError("one stored member"),))

    text = _without_raising(render_failure, Duck("a group of sorts"))

    assert text.count("ValueError: one stored member") == 1
    assert text.count("+---------------- 1 of 1 ----------------") == 1
    assert REPEATED not in text
    assert MEMBERS_UNREADABLE not in text


class _ClaimsAClass:
    """Neither an exception nor a tuple, and reading its class runs its code."""

    @property
    def __class__(self) -> type:
        raise AssertionError("the report ran this object's code")


@pytest.mark.parametrize("where", ["container", "member"])
def test_what_is_a_member_is_decided_without_running_the_objects_code(where):
    """Found while fixing round 7: ``isinstance`` falls back to an object's own
    ``__class__``, which can run anything, forever included, so the walk reads
    the type instead. A tuple holding anything but exceptions is not a group."""
    stored = (
        _ClaimsAClass() if where == "container" else (ValueError("stored"), _ClaimsAClass())
    )

    class Duck(Exception):
        exceptions = stored

    text = _without_raising(render_failure, Duck("not a group"))

    assert text.endswith("Duck: not a group\n")
    assert MEMBERS_UNREADABLE not in text
    assert "ValueError: stored" not in text


class _NameFails(type):
    """Classes whose ``__name__`` cannot be read (review round 7)."""

    def __getattribute__(cls, name: str) -> object:
        if name == "__name__":
            raise RuntimeError("exception type name unavailable")
        return super().__getattribute__(name)


class Nameless(Exception, metaclass=_NameFails):
    """Its ``__qualname__`` still reads, so its own line can still name it."""


class _NotText:
    def __format__(self, spec: str) -> str:
        raise ValueError("a name that is not text")


class _NameNotText(type):
    def __getattribute__(cls, name: str) -> object:
        if name == "__name__":
            return _NotText()
        return super().__getattribute__(name)


class NameNotText(Exception, metaclass=_NameNotText):
    """Its ``__name__`` reads, but is not text."""


class NamelessUnprintable(Nameless):
    """And its ``__str__`` raises, so the one-line error takes its fallback."""

    def __str__(self) -> str:
        raise ValueError("no text")


@pytest.mark.parametrize(
    ("error", "said", "printed"),
    [
        (Nameless, "run failed", "run failed"),
        (NameNotText, "run failed", "run failed"),
        (NamelessUnprintable, STR_FAILED, "<exception str() failed>"),
    ],
    ids=["raises", "not_text", "and_its_str_raises"],
)
def test_a_type_whose_name_cannot_be_read_costs_only_its_name(error, said, printed):
    """Review round 7: the one-line error's fallback read the type's name
    again, unprotected, so a type whose name could not be read made the
    payload raise instead of being built."""
    payload = _without_raising(failure_payload, _raised(error("run failed")))

    assert payload["passed"] is False
    assert payload["error"] == f"{report.NAME_UNREADABLE}: {said}"
    # The exception's own line uses its __qualname__, which still reads.
    assert f"{error.__qualname__}: {printed}" in payload["traceback"]


def _nameless_failure(*_: object) -> Any:
    raise Nameless("a failure whose type has no readable name")


class _TraceFailsNamelessly(Exception):
    def __getattribute__(self, name: str) -> object:
        if name == "__traceback__":
            _nameless_failure()
        return super().__getattribute__(name)


class _MembersFailNamelessly(Exception):
    @property
    def exceptions(self) -> tuple[BaseException, ...]:
        return _nameless_failure()


class _CauseFailsNamelessly(Exception):
    @property
    def __cause__(self) -> BaseException | None:
        return _nameless_failure()


class _NamelessUnformattable(Nameless):
    def __getattribute__(self, name: str) -> object:
        if name == "__traceback__":
            raise ValueError("traceback unavailable")
        return super().__getattribute__(name)


def _reached_twice() -> BaseException:
    member = Nameless("reached twice")
    return BaseExceptionGroup("twice", [member, member])


# Each case and the line it must print; {unnamed} and {chain} are filled in
# with the report's own markers.
UNNAMED = {
    "reached_twice": (_reached_twice, "    {unnamed}: " + REPEATED),
    "cannot_be_formatted": (
        lambda: _NamelessUnformattable("unformattable"),
        "{unnamed}: <" + NOT_FORMATTED + ": ValueError>",
    ),
    "formatting_failed": (
        lambda: _TraceFailsNamelessly("its traceback"),
        "_TraceFailsNamelessly: <" + NOT_FORMATTED + ": {unnamed}>",
    ),
    "members_failed": (
        lambda: _MembersFailNamelessly("its members"),
        "<" + MEMBERS_UNREADABLE + ": {unnamed}>",
    ),
    "chain_failed": (
        lambda: _CauseFailsNamelessly("its cause"),
        "<{chain}: {unnamed}>",
    ),
}


@pytest.mark.parametrize("case", sorted(UNNAMED))
def test_every_line_that_names_a_type_survives_a_type_without_a_name(case):
    """Review round 7 found the type's name read unprotected in each of the
    report's own lines; any one of them could still fail the rendering."""
    build, line = UNNAMED[case]

    lines = _without_raising(render_failure, build()).splitlines()

    unnamed, chain = report.NAME_UNREADABLE, report.CHAIN_UNREADABLE
    assert line.format(unnamed=unnamed, chain=chain) in lines


def test_a_rendering_that_fails_namelessly_is_still_published(monkeypatch):
    monkeypatch.setattr(report, "render_failure", _nameless_failure)

    payload = _without_raising(failure_payload, RuntimeError("the failure"))

    assert payload["error"] == "RuntimeError: the failure"
    assert payload["traceback"] == f"<{RENDERING_FAILED}: {report.NAME_UNREADABLE}>\n"


class _Qualified(Exception):
    """The standard formatter prints this type with its module in front."""


def _in_main() -> BaseException:
    class InMain(Exception):
        pass

    InMain.__module__ = "__main__"
    return _raised(InMain("printed without its module"))


def _module_not_text() -> BaseException:
    class ModuleNotText(Exception):
        pass

    ModuleNotText.__module__ = 42
    return ModuleNotText("its module is not text")


def _with_notes() -> BaseException:
    error = ValueError("with notes")
    error.add_note("first note")
    error.add_note("a note\nover two lines")
    return error


def _notes_not_a_sequence() -> BaseException:
    error = ValueError("notes that are not a sequence")
    error.__notes__ = 42
    return error


def _note_str_raises() -> BaseException:
    class Unprintable:
        def __str__(self) -> str:
            raise ValueError("no text")

    error = ValueError("a note that cannot be printed")
    error.__notes__ = [Unprintable()]
    return error


def _notes_unreadable() -> BaseException:
    class NotesUnreadable(Exception):
        @property
        def __notes__(self) -> list[str]:
            raise ValueError("no notes")

    return NotesUnreadable("notes that cannot be read")


def _str_raises() -> BaseException:
    class Unprintable(RuntimeError):
        def __str__(self) -> str:
            raise ValueError("no text")

    return _raised(Unprintable())


def _in_context(*, suppressed: bool = False, caused: bool = False) -> BaseException:
    try:
        try:
            raise ValueError("the first failure")
        except ValueError:
            if suppressed:
                raise RuntimeError("the second failure") from None
            if caused:
                raise RuntimeError("the second failure") from OSError("the cause")
            raise RuntimeError("the second failure")
    except RuntimeError as caught:
        return caught


def _compiled(source: str) -> BaseException:
    try:
        compile(source, "<probe>", "exec")
    except SyntaxError as caught:
        return caught
    raise AssertionError(f"{source!r} compiled")


def _syntax(
    text: str | None,
    offset: int | None,
    end_offset: int | None = None,
    *,
    filename: str | None = "probe.py",
    lineno: int | None = 3,
    msg: str = "invalid syntax",
) -> SyntaxError:
    return SyntaxError(msg, (filename, lineno, offset, text, lineno, end_offset))


class _FalseText(str):
    """Text that is not empty, but tests false (review round 8)."""

    def __bool__(self) -> bool:
        return False


class _TruthRaises(str):
    """Text whose truth cannot be tested."""

    def __bool__(self) -> bool:
        raise ValueError("text whose truth cannot be tested")


class _TextRaises(str):
    """Text that cannot be converted to text."""

    def __str__(self) -> str:
        raise ValueError("text that cannot be converted")


class _TextError(Exception):
    """Its ``str()`` is the text it was given, a str subclass included."""

    def __str__(self) -> str:
        return self.args[0]


FORMATTED_AS_STANDARD = {
    "raised": lambda: _raised(ValueError("plain")),
    "not_raised": lambda: ValueError("plain"),
    "empty_message": lambda: _raised(RuntimeError()),
    "module_qualified": lambda: _raised(_Qualified("named with its module")),
    "in_main": _in_main,
    "module_not_text": _module_not_text,
    "notes": _with_notes,
    "notes_not_a_sequence": _notes_not_a_sequence,
    "note_str_raises": _note_str_raises,
    "notes_unreadable": _notes_unreadable,
    "str_raises": _str_raises,
    "in_context": _in_context,
    "context_suppressed": lambda: _in_context(suppressed=True),
    "caused_with_context": lambda: _in_context(caused=True),
    "syntax_compiled": lambda: _compiled("x = = 1"),
    "syntax_indentation": lambda: _compiled("if True:\nx = 1\n"),
    "syntax_indented_text": lambda: _syntax("    foo bar baz\n", 9, 12),
    "syntax_tab": lambda: _syntax("\tfoo = = 1\n", 7),
    "syntax_no_lineno": lambda: _syntax(None, None, lineno=None),
    "syntax_no_filename": lambda: _syntax("x = = 1\n", 5, filename=None),
    "syntax_no_text": lambda: _syntax(None, 5),
    "syntax_no_offset": lambda: _syntax("x = = 1\n", None),
    "syntax_end_offset_zero": lambda: _syntax("x = = 1\n", 5, 0),
    "syntax_end_offset_negative": lambda: _syntax("x = = 1\n", 5, -1),
    "syntax_end_offset_equal": lambda: _syntax("x = = 1\n", 5, 5),
    "syntax_offset_in_indent": lambda: _syntax("    foo\n", 2),
    "syntax_no_message": lambda: _syntax("x = = 1\n", 5, msg=""),
    "str_subclass_false": lambda: _raised(_TextError(_FalseText("message"))),
    "str_subclass_truth_raises": lambda: _raised(_TextError(_TruthRaises("message"))),
    "str_subclass_str_raises": lambda: _raised(_TextError(_TextRaises("message"))),
}


@pytest.mark.parametrize("case", sorted(FORMATTED_AS_STANDARD))
def test_what_the_standard_formatter_can_print_is_printed_as_it_prints_it(case):
    """Reading each exception on its own must not change what is printed for
    it: with no group in it, the whole report is the standard text."""
    error = FORMATTED_AS_STANDARD[case]()

    assert _without_raising(render_failure, error) == "".join(
        traceback.format_exception(error)
    )


# --- review round 8 -----------------------------------------------------------


@pytest.mark.parametrize("groups", sorted(GROUPS))
def test_a_member_whose_text_tests_false_keeps_its_message(groups):
    """Review round 8: a member's message is printed nowhere but on its own
    line, so text that tested false lost it from both payload fields."""
    failure = _TextError(_FalseText("the only failure detail"))

    payload = _without_raising(failure_payload, GROUPS[groups]("run failed", [failure]))

    lines = payload["traceback"].splitlines()
    assert any(line.endswith("_TextError: the only failure detail") for line in lines)


def _interrupted_reading(field: str) -> type:
    class Interrupted(Exception):
        def __getattribute__(self, name: str) -> object:
            if name == field:
                raise KeyboardInterrupt("read interrupted")
            return super().__getattribute__(name)

    return Interrupted


# Each field a read of which is interrupted, the line printed in its place, and
# whether the member's own line survives it.
INTERRUPTED = {
    "__cause__": (f"    <{report.CHAIN_UNREADABLE}: KeyboardInterrupt>", True),
    "__context__": (f"    <{report.CHAIN_UNREADABLE}: KeyboardInterrupt>", True),
    "__traceback__": (f"    Interrupted: <{NOT_FORMATTED}: KeyboardInterrupt>", False),
    "exceptions": (f"    <{MEMBERS_UNREADABLE}: KeyboardInterrupt>", True),
    "__notes__": (
        "    Ignored error getting __notes__: KeyboardInterrupt('read interrupted')",
        True,
    ),
}


@pytest.mark.parametrize("groups", sorted(GROUPS))
@pytest.mark.parametrize("field", sorted(INTERRUPTED))
def test_an_interrupted_read_costs_only_what_it_was_reading(field, groups):
    """Review round 8: the report's own guards caught Exception, and only the
    outer one caught BaseException, so an interrupt raised by one member's read
    replaced the whole report, its healthy parent and sibling included."""
    broken = _interrupted_reading(field)("broken member")
    group = GROUPS[groups]("healthy parent", [broken, ValueError("healthy sibling")])
    group.add_note("healthy parent note")
    marker, own_line_kept = INTERRUPTED[field]

    lines = _without_raising(render_failure, group).splitlines()

    assert any(line.endswith(": healthy parent (2 sub-exceptions)") for line in lines)
    assert "healthy parent note" in lines
    assert marker in lines
    assert any(line.endswith("Interrupted: broken member") for line in lines) is (
        own_line_kept
    )
    assert "    ValueError: healthy sibling" in lines
