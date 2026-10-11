"""Rewrite one Python source for a step-1 move, with minimal text edits.

It renames, per RewriteMaps: from-imports (absolute and relative, per
imported name), dotted names in code (`import a.b`, `a.b.attr`), dotted
names and src/navpy paths in string literals (joined across implicit
concatenation) and in comments. A qualified reference to a name that leaves
its module (`navpy.a.b.Name`, or `mod.Name` through a module alias bound by
`import navpy.a.b as mod` or `from navpy.a import b`) is reported as a hand
fix. Everything else keeps its exact text,
line endings included. c01 and c02 reuse it with their own maps.
"""

from __future__ import annotations

import ast
import tokenize
from dataclasses import dataclass
from functools import partial
from typing import Collection, Dict, Iterator

from step1_rewrite_imports import Importer, import_edits, import_target
from step1_rewrite_maps import (
    DOTTED_NAME,
    ROOT_PACKAGE,
    RewriteMaps,
    rename_module,
    rename_reference,
    text_renames,
    unroutable,
)
from step1_rewrite_source import Edit, Source, SourceError, Token, apply_edits, parse_source
from step1_rewrite_strings import piece_edits, string_groups, string_piece


@dataclass(frozen=True)
class RewriteResult:
    text: str
    edits: int
    problems: tuple[str, ...]


def rewrite_source(text: str, maps: RewriteMaps, importer: Importer | None = None) -> RewriteResult:
    """The source with every reference the maps rename rewritten. importer
    places the file (needed for relative imports); None outside src/."""
    try:
        source = parse_source(text)
    except SourceError as error:
        problems = (f"cannot parse: {error}",) if ROOT_PACKAGE in text else ()
        return RewriteResult(text, 0, problems)
    edits, problems = import_edits(source, importer, maps)
    spans = [source.span(node) for node in ast.walk(source.tree) if isinstance(node, ast.ImportFrom)]
    edits += _chain_edits(source, maps, spans, problems)
    problems += _alias_problems(source, maps, importer, spans)
    edits += _string_edits(source, maps, problems)
    edits += _comment_edits(source, maps, spans)
    new_text = apply_edits(text, edits)
    try:
        ast.parse(new_text)
    except SyntaxError as error:
        return RewriteResult(text, 0, (*problems, f"rewrite would not parse ({error}); left as is"))
    return RewriteResult(new_text, len(edits), tuple(problems))


def dotted_chains(
    tokens: tuple[Token, ...], heads: Collection[str] = (ROOT_PACKAGE,)
) -> Iterator[tuple[Token, Token, str]]:
    """(first, last, dotted name) for each `navpy.a.b` name chain in code, or
    each chain starting with one of heads."""
    for index, token in enumerate(tokens):
        if token.type != tokenize.NAME or token.string not in heads:
            continue
        if index and tokens[index - 1].is_op("."):
            continue
        names = [token.string]
        last = index
        while (last + 2 < len(tokens) and tokens[last + 1].is_op(".")
               and tokens[last + 2].type == tokenize.NAME):
            names.append(tokens[last + 2].string)
            last += 2
        yield token, tokens[last], ".".join(names)


def _inside(offset: int, spans: list[tuple[int, int]]) -> bool:
    return any(start <= offset < end for start, end in spans)


def _chain_edits(source: Source, maps: RewriteMaps, spans: list[tuple[int, int]],
                 problems: list[str]) -> list[Edit]:
    edits: list[Edit] = []
    for first, last, dotted in dotted_chains(source.tokens):
        new = rename_module(dotted, maps)
        if _inside(first.start, spans):
            continue
        if rename_reference(dotted, maps) != new or unroutable(dotted, maps):
            problems.append(f"line {first.line}: {dotted} names a name that leaves its module;"
                            " rewrite it and its import by hand")
            continue
        if new == dotted:
            continue
        if source.text[first.start:last.end] != dotted:
            problems.append(f"line {first.line}: dotted name {dotted} spans spaces or lines")
            continue
        edits.append(Edit(first.start, last.end, new))
    return edits


def module_aliases(source: Source, maps: RewriteMaps,
                   importer: Importer | None) -> Dict[str, str]:
    """Name -> old navpy module it binds: `import navpy.a.b as name` and
    `from navpy.a import b [as name]` when navpy.a.b is a module."""
    aliases: Dict[str, str] = {}
    for node in ast.walk(source.tree):
        if isinstance(node, ast.Import):
            aliases.update((alias.asname, alias.name) for alias in node.names
                           if alias.asname and alias.name.split(".")[0] == ROOT_PACKAGE)
        elif isinstance(node, ast.ImportFrom):
            target = import_target(node, importer, "old")
            if target is not None and target.split(".")[0] == ROOT_PACKAGE:
                aliases.update((alias.asname or alias.name, f"{target}.{alias.name}")
                               for alias in node.names if f"{target}.{alias.name}" in maps.known)
    return aliases


def _alias_problems(source: Source, maps: RewriteMaps, importer: Importer | None,
                    spans: list[tuple[int, int]]) -> list[str]:
    """`mod.Name` through a module alias, where Name leaves the module."""
    aliases = module_aliases(source, maps, importer)
    problems: list[str] = []
    for first, _, chain in dotted_chains(source.tokens, frozenset(aliases)):
        head, _, rest = chain.partition(".")
        dotted = f"{aliases[head]}.{rest}"
        if rest and not _inside(first.start, spans) and (
                rename_reference(dotted, maps) != rename_module(dotted, maps)
                or unroutable(dotted, maps)):
            problems.append(f"line {first.line}: {chain} ({dotted}) names a name that leaves"
                            " its module; rewrite it and its import by hand")
    return problems


def _string_edits(source: Source, maps: RewriteMaps, problems: list[str]) -> list[Edit]:
    renames = partial(text_renames, maps=maps)
    edits: list[Edit] = []
    for group in string_groups(source.tokens):
        pieces = [string_piece(token) for token in group]
        known = [piece for piece in pieces if piece is not None]
        runs = [known] if len(known) == len(pieces) else [[piece] for piece in known]
        for run in runs:
            edits += piece_edits(run, renames)
            value = "".join(piece.value for piece in run)
            problems += [f"line {run[0].token.line}: {match.group(0)} is looked up by several"
                         " split modules; patch the right one by hand"
                         for match in DOTTED_NAME.finditer(value) if unroutable(match.group(0), maps)]
    return edits


def _comment_edits(source: Source, maps: RewriteMaps, spans: list[tuple[int, int]]) -> list[Edit]:
    return [
        Edit(token.start + start, token.start + end, new)
        for token in source.tokens
        if token.type == tokenize.COMMENT and not _inside(token.start, spans)
        for start, end, new in text_renames(token.string, maps)
    ]
