"""The media runtime check (tools/hitl/check_media_runtime.py) over synthetic installs and a fake gi/Gst: a pass,
and a specific blocker for each missing or mismatched installer, layout, binding, import, version and element."""
from __future__ import annotations

import ast
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from tests.tools.hitl import media_fakes as fakes
from tools.hitl import check_media_runtime as media
from tools.hitl import media_profiles as profiles


@pytest.fixture
def pc(tmp_path, monkeypatch):
    """A synthetic pc install, installer stand-ins with pinned sums, and a run of the check over them."""
    layout = fakes.install(tmp_path / "gstreamer")
    folder, contents = fakes.installers(tmp_path / "msi")
    fakes.pinned_installers(monkeypatch, contents)

    def run(loader=None, **changes):
        kwargs = dict(root=layout.root, installers=folder, encoder="nvh265enc", decoder="nvh265dec",
                      interpreter=fakes.INTERPRETER, loader=loader or fakes.Loader(), driver=lambda: "GPU, 1.0")
        kwargs.update(changes)
        return media.check_runtime(profiles.PROFILES["pc"], **kwargs)
    run.layout, run.folder = layout, folder
    return run


def blockers(result) -> str:
    assert result["verdict"] == "blocked"
    return "; ".join(result["blockers"])


def test_pass_records_versions_origins_and_launch(pc):
    loader = fakes.Loader()
    result = pc(loader)
    assert result["verdict"] == "pass" and result["blockers"] == []
    assert loader.calls == [pc.layout]
    assert set(result["elements"]) == {*profiles.PROFILES["pc"].elements, "nvh265enc", "nvh265dec"}
    assert result["elements"]["rtph265pay"]["version"] == "1.26.11"
    assert result["gstreamer"]["version"] == "1.26.11" and result["gstreamer"]["nano"] == 0
    assert result["gstreamer"]["pygobject_version"] == "3.50.0"
    assert result["python"]["gi_extensions"] == [fakes.BINDING]
    assert result["launch"] == {"dll_directory": str(pc.layout.bin), "sys_path": str(pc.layout.site_packages)}
    assert result["installers"] == dict(profiles.PROFILES["pc"].installers)
    assert result["selected"] == {"encoder": "nvh265enc", "decoder": "nvh265dec"}
    assert result["gpu_driver"] == "GPU, 1.0"
    json.dumps(result, allow_nan=False)


def test_official_installer_sums_are_pinned():
    assert dict(profiles.PROFILES["pc"].installers) == {
        "gstreamer-1.0-msvc-x86_64-1.26.11.msi": "31cbc21fa0950b5c1e79c80959b2799805cb05a7a35953a13a9f790776137605",
        "gstreamer-1.0-devel-msvc-x86_64-1.26.11.msi":
            "af0a0720692052aaf5fb0cf2847ef34a87016ef0f914328d516f049e6c86f587"}


def test_missing_or_changed_installers_block_but_import_still_reports(pc):
    name = sorted(profiles.PROFILES["pc"].installers)[0]
    (pc.folder / name).write_bytes(b"tampered")
    loader = fakes.Loader()
    assert f"installer {name} SHA-256" in blockers(pc(loader))
    assert loader.calls == [pc.layout]          # independent diagnostics still run
    (pc.folder / name).unlink()
    assert f"installer {name} is not in" in blockers(pc())
    assert "--installers is required" in blockers(pc(installers=None))


def test_missing_root_or_layout_blocks_before_import(pc, tmp_path):
    loader = fakes.Loader()
    assert "no GStreamer root" in blockers(pc(loader, root=None))
    assert "Python bindings folder" in blockers(pc(loader, root=tmp_path / "nowhere"))
    assert loader.calls == []


def test_missing_bindings_block_before_import(pc):
    (pc.layout.gi / fakes.BINDING).unlink()
    loader = fakes.Loader()
    assert "no gi extension (_gi.*)" in blockers(pc(loader))
    assert loader.calls == []


@pytest.mark.parametrize("bindings, blocked", [
    (("_gi.cp313-win_amd64.pyd",), False),
    (("_gi.cp39-win_amd64.pyd", "_gi.cp313-win_amd64.pyd"), False),     # one compatible among several
    (("_gi.cp39-win_amd64.pyd",), True),
    (("_gi.cp313t-win_amd64.pyd",), True),                                # free-threaded build
    (("_gi.cp313-win32.pyd",), True),                                     # 32-bit build
])
def test_binding_must_match_this_interpreter(tmp_path, bindings, blocked):
    layout = fakes.install(tmp_path / "gst", bindings)
    loadable, problems = media.check_binding(layout, fakes.INTERPRETER)
    assert bool(problems) == blocked
    if blocked:
        assert "loads only _gi{.cp313-win_amd64.pyd, .pyd}" in problems[0]
    else:
        assert loadable == ["_gi.cp313-win_amd64.pyd"]


def test_navpy_python_is_named_against_cp313_bindings(pc):
    py311 = media.Interpreter("3.11.9", "CPython", "win-amd64", (".cp311-win_amd64.pyd", ".pyd"), "py311")
    loader = fakes.Loader()
    text = blockers(pc(loader, interpreter=py311))
    assert f"the bindings are {fakes.BINDING}; Python 3.11.9 loads only" in text
    assert loader.calls == []


@pytest.mark.parametrize("interpreter", [
    media.Interpreter("3.13.9", "PyPy", "win-amd64", (".cp313-win_amd64.pyd",), "pypy"),
    media.Interpreter("3.13.9", "CPython", "win32", (".cp313-win_amd64.pyd",), "py32"),
])
def test_bindings_need_64_bit_cpython(pc, interpreter):
    assert "the official MSVC bindings need CPython win-amd64" in blockers(pc(interpreter=interpreter))


@pytest.mark.parametrize("error", [ImportError("DLL load failed while importing _gi"),
                                   ValueError("Namespace Gst not available"), OSError("missing gstreamer-1.0-0.dll")])
def test_import_failures_are_blockers(pc, error):
    text = blockers(pc(fakes.Loader(error=error)))
    assert f"importing gi/Gst failed: {type(error).__name__}: {error}" in text


@pytest.mark.parametrize("field", ["gi_file", "gi_extension_file", "core_dll"])
def test_bindings_and_core_dll_from_another_install_are_refused(pc, tmp_path, field):
    other = fakes.install(tmp_path / "other")
    gi, core_dll = fakes.fake_gi(pc.layout), "install"
    if field == "gi_file":
        gi.__file__ = str(other.gi / "__init__.py")
    elif field == "gi_extension_file":
        gi._gi.__file__ = str(other.gi / fakes.BINDING)
    else:
        core_dll = str(other.bin / profiles.CORE_DLL)
    loader = fakes.Loader(gi=gi, core_dll=core_dll)
    text = blockers(pc(loader))
    assert f"gstreamer.{field} " in text and "is not in" in text
    assert text.count("is not in") == 1


def test_unknown_core_dll_is_refused(pc):
    text = blockers(pc(fakes.Loader(core_dll=None)))     # GetModuleHandleW found no gstreamer-1.0-0.dll
    assert "gstreamer.core_dll None is not in" in text


@pytest.mark.parametrize("version", [(1, 26, 10, 0), (1, 28, 0, 0), (1, 26, 11, 1)])
def test_other_gstreamer_versions_stop_before_elements(pc, version):
    gst = fakes.fake_gst(pc.layout)
    gst.version_tuple = version
    result = pc(fakes.Loader(gst=gst))
    assert "not the pinned release 1.26.11" in blockers(result)
    assert result["elements"] == {}


@pytest.mark.parametrize("name", [*profiles.PROFILES["pc"].elements, "nvh265enc", "nvh265dec"])
def test_each_missing_element_is_named(pc, name):
    gst = fakes.fake_gst(pc.layout)
    del gst.elements[name]
    assert f"element {name} is not registered" in blockers(pc(fakes.Loader(gst=gst)))


def test_element_that_cannot_be_created_is_refused(pc):
    gst = fakes.fake_gst(pc.layout)
    version, filename, _ = gst.elements["nvh265enc"]
    gst.elements["nvh265enc"] = (version, filename, False)       # nvcodec without a working GPU/driver
    assert "element nvh265enc is registered but cannot be created" in blockers(pc(fakes.Loader(gst=gst)))


def test_element_from_another_folder_or_version_is_refused(pc, tmp_path):
    gst = fakes.fake_gst(pc.layout)
    gst.elements["rtspsrc"] = ("1.26.11", str(tmp_path / "GST_PLUGIN_PATH" / "gstrtsp.dll"), True)
    gst.elements["h265parse"] = ("1.24.0", gst.elements["h265parse"][1], True)
    text = blockers(pc(fakes.Loader(gst=gst)))
    assert "elements.rtspsrc.filename" in text and "is not in" in text
    assert "element h265parse plugin plugin-h265parse is version 1.24.0" in text


def test_selection_outside_the_profile_blocks_before_import(pc):
    loader = fakes.Loader()
    assert "encoder openh264enc is not one of nvh265enc, x265enc" in blockers(pc(loader, encoder="openh264enc"))
    assert "decoder nvv4l2decoder is not one of" in blockers(pc(loader, decoder="nvv4l2decoder"))
    assert loader.calls == []


def test_alternative_encoder_and_decoder_are_inspected(pc):
    result = pc(encoder="x265enc", decoder="avdec_h265")
    assert result["verdict"] == "pass" and {"x265enc", "avdec_h265"} <= set(result["elements"])
    assert "nvh265enc" not in result["elements"]


def test_cli_writes_the_result_once_and_exits_by_verdict(tmp_path, monkeypatch, capsys):
    out = tmp_path / "media_runtime.json"
    monkeypatch.delenv(profiles.ROOT_ENV, raising=False)
    assert media.main(["--host", "pc", "--out", str(out)]) == 1
    written = json.loads(out.read_text(encoding="utf-8"))
    assert written["verdict"] == "blocked" and "no GStreamer root" in "; ".join(written["blockers"])
    assert "BLOCKED: no GStreamer root" in capsys.readouterr().err
    assert media.main(["--host", "pc", "--out", str(out)]) == 1
    assert "refused:" in capsys.readouterr().err
    assert json.loads(out.read_text(encoding="utf-8")) == written


def test_cli_reads_the_installer_root_variable(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv(profiles.ROOT_ENV, str(tmp_path / "from-env"))
    media.main(["--host", "pc"])
    assert json.loads(capsys.readouterr().out)["gstreamer"]["root"] == str(tmp_path / "from-env")


@pytest.mark.parametrize("module", [media, profiles])
def test_probe_imports_only_the_standard_library(module):
    """It must run from a separate Python that has only the bindings, without navpy's dependencies."""
    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    imported = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    imported |= {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.level == 0}
    allowed = set(sys.stdlib_module_names) | {"__future__", "gi", "tools.hitl.media_profiles"}
    assert {name for name in imported if name.split(".")[0] not in allowed and name not in allowed} == set()


def test_real_loader_restores_sys_path_on_failure(tmp_path, monkeypatch):
    layout = fakes.install(tmp_path / "gst")
    before = list(sys.path)
    added = []

    class Handle:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            added.append("closed")

    monkeypatch.setattr(media.os, "add_dll_directory", lambda path: added.append(path) or Handle(), raising=False)
    monkeypatch.setitem(sys.modules, "gi", None)        # import gi raises ImportError
    with pytest.raises(ImportError):
        with media.load_gst(layout):
            pass
    assert sys.path == before and added == [str(layout.bin), "closed"]


def test_real_loader_keeps_the_dll_folder_until_inspection_ends(tmp_path, monkeypatch):
    """The DLL directory and the bindings path stay in place while Gst is initialised and inspected."""
    import types
    layout = fakes.install(tmp_path / "gst")
    events = []

    class Handle:
        def __enter__(self):
            events.append("dll folder added")
            return self

        def __exit__(self, *exc):
            events.append("dll folder removed")

    gst = types.SimpleNamespace(init=lambda _: events.append(f"Gst.init, path has bindings: "
                                                             f"{str(layout.site_packages) in sys.path}"))
    gi = types.ModuleType("gi")
    gi.require_version = lambda name, version: events.append(f"require {name} {version}")
    repository = types.ModuleType("gi.repository")
    repository.Gst = gst
    gi.repository = repository
    monkeypatch.setitem(sys.modules, "gi", gi)
    monkeypatch.setitem(sys.modules, "gi.repository", repository)
    monkeypatch.setattr(media.os, "add_dll_directory", lambda path: Handle(), raising=False)
    monkeypatch.setattr(media, "loaded_dll_path", lambda name: str(layout.bin / name))
    with media.load_gst(layout) as (loaded_gi, loaded_gst, core_dll):
        events.append("inspecting")
        assert loaded_gi is gi and loaded_gst is gst and core_dll == str(layout.bin / profiles.CORE_DLL)
    assert events == ["dll folder added", "require Gst 1.0", "Gst.init, path has bindings: True", "inspecting",
                      "dll folder removed"]
    assert str(layout.site_packages) not in sys.path


@pytest.mark.skipif(sys.platform != "win32", reason="Windows module lookup")
def test_loaded_dll_path_finds_a_loaded_module_and_not_a_missing_one():
    assert Path(media.loaded_dll_path("kernel32.dll")).name.lower() == "kernel32.dll"
    assert media.loaded_dll_path("no-such-module-navpy.dll") is None


def test_documented_launch_runs_without_navpy_on_the_path(tmp_path):
    """python -E -s -m from the repository root: no PYTHONPATH, no user site; blocked here (no GStreamer root)."""
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", profiles.ROOT_ENV)}
    result = subprocess.run([sys.executable, "-E", "-s", "-m", "tools.hitl.check_media_runtime", "--host", "pc"],
                            cwd=Path(media.__file__).resolve().parents[2], env=env, capture_output=True, text=True,
                            timeout=60, check=False)
    assert result.returncode == 1, result.stderr
    assert json.loads(result.stdout)["verdict"] == "blocked"
    assert "BLOCKED: no GStreamer root" in result.stderr
