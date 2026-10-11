"""String literals for the step-1 rewriter: decoded values mapped to source.

Adjacent literals form one value (implicit concatenation, as in a mock.patch
target split over lines), so a dotted name or path is matched on the joined
value and each edit lands in the literals it spans. Bytes and f-strings stay
opaque: they are never edited, only scanned raw for leftovers.
"""

from __future__ import annotations

import ast
import re
import tokenize
import unicodedata
from dataclasses import dataclass
from typing import Callable

from step1_rewrite_source import Edit, Token

_PREFIX = re.compile(r"[A-Za-z]*")
_SIMPLE_ESCAPES = {"\\": "\\", "'": "'", '"': '"', "a": "\a", "b": "\b", "f": "\f",
                   "n": "\n", "r": "\r", "t": "\t", "v": "\v"}
_HEX_ESCAPES = {"x": 2, "u": 4, "U": 8}
_OCTAL = "01234567"
_BETWEEN_PIECES = (tokenize.NL, tokenize.COMMENT)


@dataclass(frozen=True)
class StringPiece:
    """One str literal token: its value, and the source offset of each value
    character plus one past the last (escapes map to their backslash)."""

    token: Token
    value: str
    index: tuple[int, ...]


def string_groups(tokens: tuple[Token, ...]) -> list[list[Token]]:
    """Runs of STRING tokens joined by implicit concatenation."""
    groups: list[list[Token]] = []
    pending: list[Token] = []
    for token in tokens:
        if token.type == tokenize.STRING:
            pending.append(token)
        elif pending and token.type not in _BETWEEN_PIECES:
            groups.append(pending)
            pending = []
    if pending:
        groups.append(pending)
    return groups


def string_piece(token: Token) -> StringPiece | None:
    """The decoded str literal; None for bytes and f-strings."""
    prefix = _PREFIX.match(token.string).group(0).lower()
    if "b" in prefix or "f" in prefix:
        return None
    rest = token.string[len(prefix):]
    quote = rest[:3] if rest[:3] in ('"""', "'''") else rest[:1]
    body = rest[len(quote):len(rest) - len(quote)]
    value, index = _decode(body, raw="r" in prefix)
    try:
        if value != ast.literal_eval(token.string):
            return None
    except (SyntaxError, ValueError):
        return None
    body_start = token.start + len(prefix) + len(quote)
    return StringPiece(token, value, tuple(body_start + offset for offset in index))


def piece_edits(
    pieces: list[StringPiece],
    renames: Callable[[str], list[tuple[int, int, str]]],
) -> list[Edit]:
    """Edits applying renames to the joined value of adjacent pieces."""
    starts: list[int] = []
    position = 0
    for piece in pieces:
        starts.append(position)
        position += len(piece.value)
    value = "".join(piece.value for piece in pieces)
    edits: list[Edit] = []
    for start, end, new in renames(value):
        touched = [k for k, piece in enumerate(pieces)
                   if piece.value and starts[k] < end and starts[k] + len(piece.value) > start]
        parts = split_replacement(value[start:end], new, [starts[k] - start for k in touched[1:]])
        for k, part in zip(touched, parts):
            low = max(start, starts[k]) - starts[k]
            high = min(end, starts[k] + len(pieces[k].value)) - starts[k]
            edits.append(Edit(pieces[k].index[low], pieces[k].index[high], part))
    return edits


def split_replacement(old: str, new: str, cuts: list[int]) -> list[str]:
    """new cut where old is cut. A cut in the head or tail old and new share
    keeps its distance from that end; one in the changed middle moves to the
    middle's end, so a module prefix and its unchanged symbol stay apart."""
    head = _common_prefix(old, new)
    tail = _common_prefix(old[head:][::-1], new[head:][::-1])
    positions: list[int] = []
    for cut in cuts:
        if cut <= head:
            position = cut
        elif len(old) - cut <= tail:
            position = len(new) - (len(old) - cut)
        else:
            position = len(new) - tail
        positions.append(max([position, *positions]))
    bounds = [0, *positions, len(new)]
    return [new[low:high] for low, high in zip(bounds, bounds[1:])]


def _common_prefix(left: str, right: str) -> int:
    count = 0
    for a, b in zip(left, right):
        if a != b:
            break
        count += 1
    return count


def _decode(body: str, raw: bool) -> tuple[str, list[int]]:
    """Value of a literal body and each value character's body offset."""
    value: list[str] = []
    index: list[int] = []
    position = 0
    while position < len(body):
        if body.startswith("\r\n", position):
            value.append("\n")
            index.append(position)
            position += 2
        elif body[position] != "\\" or raw:
            value.append(body[position])
            index.append(position)
            position += 1
        else:
            position = _decode_escape(body, position, value, index)
    index.append(len(body))
    return "".join(value), index


def _decode_escape(body: str, position: int, value: list[str], index: list[int]) -> int:
    """Decode the escape at body[position] (a backslash); the next position."""
    code = body[position + 1:position + 2]
    if code == "\n":
        return position + 2
    if body.startswith("\r\n", position + 1):
        return position + 3
    end = position + 2
    if code in _SIMPLE_ESCAPES:
        char = _SIMPLE_ESCAPES[code]
    elif code and code in _OCTAL:
        end = position + 1
        while end < min(position + 4, len(body)) and body[end] in _OCTAL:
            end += 1
        char = chr(int(body[position + 1:end], 8))
    elif code in _HEX_ESCAPES:
        end = position + 2 + _HEX_ESCAPES[code]
        char = chr(int(body[position + 2:end], 16))
    elif code == "N":
        end = body.index("}", position) + 1
        char = unicodedata.lookup(body[position + 3:end - 1])
    else:
        char, end = "\\", position + 1
    value.append(char)
    index.append(position)
    return end
