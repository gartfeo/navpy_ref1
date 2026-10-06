"""Guards the .claude/agents -> .codex/agents generator.

The headline test is `test_codex_agents_are_in_sync`: it is the drift check that
makes `.claude/agents/*.md` the single source of truth. The rest cover the
generator's own edge cases, including the literal `\\r` escapes the hand-written
TOML files used to carry.
"""
import importlib.util
import sys
import tomllib
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "scripts" / "gen_codex_agents.py"

spec = importlib.util.spec_from_file_location("gen_codex_agents", SCRIPT_PATH)
gen = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = gen
spec.loader.exec_module(gen)


def _write_agent(dir_path: Path, stem: str, description: str = "Does a thing.", body: str = "Body text.") -> Path:
    path = dir_path / f"{stem}.md"
    path.write_text(
        f"---\nname: {stem}\ndescription: {description}\ntools: Read\nmodel: sonnet\n---\n\n{body}\n",
        encoding="utf-8",
    )
    return path


# --- the drift check -------------------------------------------------------

def test_codex_agents_are_in_sync():
    """Regenerating .codex/agents/ must produce no change.

    If this fails, someone edited a .toml by hand or changed a .md without
    regenerating. Fix it by running:  python scripts/gen_codex_agents.py
    """
    problems = gen.check(gen.collect())
    assert not problems, "\n\n".join(problems)


def test_generated_toml_parses_and_round_trips():
    for path in sorted(gen.OUT_DIR.glob("*.toml")):
        parsed = tomllib.loads(path.read_text(encoding="utf-8"))
        meta, body = gen.parse_agent_md(gen.SRC_DIR / f"{path.stem}.md")
        assert parsed["name"] == meta["name"], path.name
        assert parsed["description"] == meta["description"], path.name
        assert parsed["developer_instructions"].rstrip("\n") == body, path.name


def test_no_carriage_returns_survive_toml_parsing():
    """The old hand-written files escaped every newline as `\\r` + newline."""
    for path in sorted(gen.OUT_DIR.glob("*.toml")):
        parsed = tomllib.loads(path.read_text(encoding="utf-8"))
        assert "\r" not in parsed["developer_instructions"], path.name


def test_excluded_agents_have_no_toml():
    for stem in gen.CODEX_EXCLUDE:
        assert (gen.SRC_DIR / f"{stem}.md").exists(), f"{stem} is excluded but has no .md source"
        assert not (gen.OUT_DIR / f"{stem}.toml").exists(), f"{stem} must not be exposed to Codex"


def test_agent_name_comes_from_frontmatter_not_filename():
    """product-manager.md declares `name: pm`; the TOML must follow the frontmatter."""
    parsed = tomllib.loads((gen.OUT_DIR / "product-manager.toml").read_text(encoding="utf-8"))
    assert parsed["name"] == "pm"


# --- generator behaviour ---------------------------------------------------

def test_write_is_idempotent(tmp_path):
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    _write_agent(src, "alpha")

    rendered = gen.collect(src)
    assert gen.write(rendered, out) == [out / "alpha.toml"]
    assert gen.write(rendered, out) == []
    assert gen.check(rendered, out) == []


def test_check_reports_hand_edit(tmp_path):
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    _write_agent(src, "alpha", body="Original body.")
    rendered = gen.collect(src)
    gen.write(rendered, out)

    target = out / "alpha.toml"
    target.write_text(target.read_text(encoding="utf-8").replace("Original", "Tampered"), encoding="utf-8")

    problems = gen.check(rendered, out)
    assert len(problems) == 1
    assert "Tampered" in problems[0] and "Original" in problems[0]


@pytest.mark.parametrize("eol", [b"\r\n", b"\n"])
def test_check_tolerates_either_checkout_line_ending(tmp_path, eol):
    """CRLF (Windows, autocrlf=true) and LF (Linux) checkouts must both read clean."""
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    out.mkdir()
    _write_agent(src, "alpha", body="Line one.\nLine two.")
    rendered = gen.collect(src)

    (out / "alpha.toml").write_bytes(rendered[0][1].encode("utf-8").replace(b"\n", eol))

    assert gen.check(rendered, out) == []


def test_check_catches_lone_cr_corruption(tmp_path):
    """A CR-mangled file must NOT pass: tomllib rejects it, so the gate must too."""
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    out.mkdir()
    _write_agent(src, "alpha", body="Line one.\nLine two.")
    rendered = gen.collect(src)

    path = out / "alpha.toml"
    path.write_bytes(rendered[0][1].encode("utf-8").replace(b"\n", b"\r"))

    with pytest.raises(tomllib.TOMLDecodeError):
        tomllib.loads(path.read_bytes().decode("utf-8"))
    assert gen.check(rendered, out), "drift gate passed a file tomllib cannot parse"


def test_check_reports_missing_and_stale(tmp_path):
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    out.mkdir()
    _write_agent(src, "alpha")
    (out / "ghost.toml").write_text('name = "ghost"\n', encoding="utf-8")

    problems = gen.check(gen.collect(src), out)
    assert any("alpha.toml: MISSING" in p for p in problems)
    assert any("ghost.toml: STALE" in p for p in problems)


def test_excluded_agents_are_not_collected(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    _write_agent(src, "alpha")
    for stem in gen.CODEX_EXCLUDE:
        _write_agent(src, stem)

    assert [stem for stem, _ in gen.collect(src)] == ["alpha"]


@pytest.mark.parametrize("body", [
    r"Path is C:\repos\navpy and a regex \d+.",   # backslashes must not become escapes
    'He said "hi" then "bye".',                    # single quotes are legal inside """..."""
    'Ends with a quote"',                          # would otherwise collide with the delimiter
    'Ends with two quotes""',
    'Ends with a fence"""',                        # run-escape must not double-escape the last quote
    'Ends with four""""',
    'A """fence""" inside the body.',              # runs of three must be escaped
    'Escaped backslash then quote \\"',
    "Trailing backslash \\",
    "Backslash pair \\\\",
    "\tLeading tab is legal in TOML",
])
def test_body_survives_escaping(tmp_path, body):
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    _write_agent(src, "alpha", body=body)
    gen.write(gen.collect(src), out)

    parsed = tomllib.loads((out / "alpha.toml").read_text(encoding="utf-8"))
    assert parsed["developer_instructions"].rstrip("\n") == body


@pytest.mark.parametrize("char", ["\x00", "\x0b", "\x0c", "\x1b", "\x7f"])
def test_control_chars_are_rejected_not_silently_emitted(tmp_path, char):
    """Without this the generator writes a .toml that tomllib refuses to read."""
    path = _write_agent(tmp_path, "alpha", body=f"before{char}after")
    with pytest.raises(gen.AgentParseError, match="control character"):
        gen.parse_agent_md(path)


def test_control_char_in_description_is_rejected(tmp_path):
    path = _write_agent(tmp_path, "alpha", description="bad\x0cdesc")
    with pytest.raises(gen.AgentParseError, match="control character"):
        gen.parse_agent_md(path)


def test_description_with_quotes_and_backslashes(tmp_path):
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    _write_agent(src, "alpha", description=r'Use "quotes" and C:\path here.')
    gen.write(gen.collect(src), out)

    parsed = tomllib.loads((out / "alpha.toml").read_text(encoding="utf-8"))
    assert parsed["description"] == r'Use "quotes" and C:\path here.'


@pytest.mark.parametrize("text, expected", [
    ("no frontmatter here\n", "missing `---` YAML frontmatter"),
    ("---\nname: alpha\n---\n\nbody\n", "missing `description`"),
    ("---\ndescription: d\n---\n\nbody\n", "missing `name`"),
    ("---\nname: alpha\ndescription: d\n---\n\n\n", "body is empty"),
    ("---\nname: alpha\nbroken line\n---\n\nbody\n", "not `key: value`"),
    # YAML this parser does not implement: raise rather than emit a wrong value
    ("---\nname: alpha\nname: beta\ndescription: d\n---\n\nbody\n", "duplicate frontmatter key"),
    ("---\nname: alpha\ndescription: >\n---\n\nbody\n", "block scalar"),
    ('---\nname: alpha\ndescription: "quoted"\n---\n\nbody\n', "YAML-quoted"),
    ("---\nname: alpha\ndescription: 'quoted'\n---\n\nbody\n", "YAML-quoted"),
])
def test_malformed_agent_md_is_rejected(tmp_path, text, expected):
    path = tmp_path / "alpha.md"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(gen.AgentParseError, match=expected):
        gen.parse_agent_md(path)


def test_colon_inside_a_value_is_kept_whole(tmp_path):
    """`partition` splits once, so a colon in prose must survive."""
    path = _write_agent(tmp_path, "alpha", description="Use for X: do Y, then Z")
    meta, _ = gen.parse_agent_md(path)
    assert meta["description"] == "Use for X: do Y, then Z"


def test_collect_rejects_empty_source_dir(tmp_path):
    (tmp_path / "src").mkdir()
    with pytest.raises(gen.AgentParseError, match="no agent definitions"):
        gen.collect(tmp_path / "src")


def test_crlf_source_parses_the_same_as_lf(tmp_path):
    lf, crlf = tmp_path / "lf.md", tmp_path / "crlf.md"
    text = "---\nname: alpha\ndescription: d\n---\n\nLine one.\nLine two.\n"
    lf.write_text(text, encoding="utf-8", newline="\n")
    crlf.write_text(text, encoding="utf-8", newline="\r\n")

    assert gen.parse_agent_md(lf) == gen.parse_agent_md(crlf)


def test_main_check_exit_codes(tmp_path, monkeypatch, capsys):
    src, out = tmp_path / "src", tmp_path / "out"
    src.mkdir()
    _write_agent(src, "alpha")
    monkeypatch.setattr(gen, "SRC_DIR", src)
    monkeypatch.setattr(gen, "OUT_DIR", out)

    assert gen.main(["--check"]) == 1
    assert gen.main([]) == 0
    assert gen.main(["--check"]) == 0
    assert "OK: 1 Codex agent file(s)" in capsys.readouterr().out
