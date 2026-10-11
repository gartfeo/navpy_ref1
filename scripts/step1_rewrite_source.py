"""A parsed Python source for the step-1 rewriter: AST, tokens, offsets, edits.

Offsets are character offsets into the exact file text, so edits keep every
byte they do not replace, CRLF line endings included.
"""

from __future__ import annotations

import ast
import bisect
import io
import re
import tokenize
from dataclasses import dataclass
from typing import Iterable

BYTE_ORDER_MARK = chr(0xFEFF)
# A carriage return not ending a CRLF: ast and tokenize would count lines differently.
_LONE_CR = re.compile("\r(?!\n)")


class SourceError(ValueError):
    """The text cannot be parsed or tokenized for rewriting."""


@dataclass(frozen=True)
class Edit:
    start: int
    end: int
    text: str


@dataclass(frozen=True)
class Token:
    type: int
    string: str
    start: int
    end: int
    line: int

    def is_op(self, text: str) -> bool:
        return self.type == tokenize.OP and self.string == text


@dataclass(frozen=True)
class Source:
    text: str
    tree: ast.Module
    tokens: tuple[Token, ...]
    token_starts: tuple[int, ...]
    line_starts: tuple[int, ...]

    @property
    def newline(self) -> str:
        return "\r\n" if "\r\n" in self.text else "\n"

    def ast_offset(self, line: int, byte_col: int) -> int:
        """Character offset of an ast (line, UTF-8 byte column) position."""
        start = self.line_starts[line - 1]
        line_text = self.text[start:start + byte_col]
        if not line_text.isascii():
            end = self.line_starts[line] if line < len(self.line_starts) else len(self.text)
            line_text = self.text[start:end].encode("utf-8")[:byte_col].decode("utf-8")
        return start + len(line_text)

    def span(self, node: ast.stmt) -> tuple[int, int]:
        assert node.end_lineno is not None and node.end_col_offset is not None
        return (
            self.ast_offset(node.lineno, node.col_offset),
            self.ast_offset(node.end_lineno, node.end_col_offset),
        )

    def tokens_in(self, start: int, end: int) -> tuple[Token, ...]:
        starts = self.token_starts
        return self.tokens[bisect.bisect_left(starts, start):bisect.bisect_left(starts, end)]

    def line_head(self, offset: int) -> str:
        """Text from the start of offset's line up to offset."""
        line = bisect.bisect_right(self.line_starts, offset) - 1
        return self.text[self.line_starts[line]:offset]


def parse_source(text: str) -> Source:
    if text.startswith(BYTE_ORDER_MARK) or _LONE_CR.search(text):
        raise SourceError("a byte-order mark or a bare carriage return")
    try:
        tree = ast.parse(text)
        raw_tokens = list(tokenize.generate_tokens(io.StringIO(text).readline))
    except (SyntaxError, tokenize.TokenError, ValueError) as error:
        raise SourceError(str(error)) from error
    line_starts = (0, *(index + 1 for index, char in enumerate(text) if char == "\n"))
    tokens = tuple(_token(raw, line_starts, len(text)) for raw in raw_tokens)
    return Source(text, tree, tokens, tuple(token.start for token in tokens), line_starts)


def apply_edits(text: str, edits: Iterable[Edit]) -> str:
    """text with every edit applied; edits must not overlap."""
    pieces: list[str] = []
    position = 0
    for edit in sorted(edits, key=lambda item: (item.start, item.end)):
        if edit.start < position:
            raise ValueError(f"overlapping edits at offset {edit.start}")
        pieces += [text[position:edit.start], edit.text]
        position = edit.end
    pieces.append(text[position:])
    return "".join(pieces)


def _token(raw: tokenize.TokenInfo, line_starts: tuple[int, ...], size: int) -> Token:
    return Token(raw.type, raw.string, _offset(raw.start, line_starts, size),
                 _offset(raw.end, line_starts, size), raw.start[0])


def _offset(position: tuple[int, int], line_starts: tuple[int, ...], size: int) -> int:
    row, column = position
    return line_starts[row - 1] + column if row <= len(line_starts) else size
