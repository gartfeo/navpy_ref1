"""The media runtime check (tools/hitl/check_media_runtime.py) over a synthetic package database and a fake gi/Gst:
a pass, and a specific blocker for each missing or mismatched package, binding, import, version, element, file origin
and roundtrip."""
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
from tools.hitl import media_roundtrip

PC = profiles.PROFILES["pc-wsl"]


def blockers(result) -> str:
    assert result["verdict"] == "blocked"
    return "; ".join(result["blockers"])


def test_pass_records_versions_origins_and_roundtrip():
    loader, roundtrip = fakes.Loader(), fakes.Roundtrip()
    result = fakes.run(loader=loader, roundtrip=roundtrip)
    assert result["verdict"] == "pass" and result["blockers"] == []
    assert loader.calls == 1 and roundtrip.calls == [("nvcudah265enc", "nvh265dec")]
    assert set(result["elements"]) == {*PC.elements, "nvcudah265enc", "nvh265dec"}
    assert result["gstreamer"]["version"] == "1.24.2" and result["gstreamer"]["nano"] == 0
    assert result["gstreamer"]["core_library"] == fakes.CORE
    assert result["packages"] == dict(PC.packages) and result["package_files_verified"] is True
    assert result["file_packages"][fakes.CORE] == "libgstreamer1.0-0"
    assert result["python"]["gi_extensions"] == [fakes.BINDING]
    assert result["roundtrip"] == fakes.PASSING_ROUNDTRIP
    assert result["selected"] == {"encoder": "nvcudah265enc", "decoder": "nvh265dec"}
    assert result["gpu_driver"] == "GPU, 1.0"
    json.dumps(result, allow_nan=False)


def test_noble_packages_are_pinned_exactly():
    assert dict(PC.packages) == {
        "libgstreamer1.0-0": "1.24.2-1ubuntu0.1", "gstreamer1.0-plugins-base": "1.24.2-1ubuntu0.5",
        "gstreamer1.0-plugins-good": "1.24.2-1ubuntu1.8", "gstreamer1.0-plugins-bad": "1.24.2-1ubuntu4",
        "gstreamer1.0-libav": "1.24.1-1build1", "gstreamer1.0-rtsp": "1.24.2-1", "python3-gi": "3.48.2-1",
        "python3-gst-1.0": "1.24.1-1", "gir1.2-gstreamer-1.0": "1.24.2-1ubuntu0.1",
        "gir1.2-gst-plugins-base-1.0": "1.24.2-1ubuntu0.5", "libgstreamer-plugins-base1.0-0": "1.24.2-1ubuntu0.5",
        "libgstreamer-plugins-bad1.0-0": "1.24.2-1ubuntu4", "libgstreamer-gl1.0-0": "1.24.2-1ubuntu0.5",
        "libgstrtspserver-1.0-0": "1.24.2-1"}


def test_missing_or_other_package_versions_block_but_import_still_reports():
    packages = dict(PC.packages)
    del packages["gstreamer1.0-rtsp"]
    packages["gstreamer1.0-plugins-good"] = "1.24.2-1ubuntu1.9"
    loader = fakes.Loader()
    text = blockers(fakes.run(loader=loader, command=fakes.Dpkg(packages=packages)))
    assert "packages.gstreamer1.0-rtsp is not installed, not the pinned 1.24.2-1" in text
    assert "packages.gstreamer1.0-plugins-good is 1.24.2-1ubuntu1.9, not the pinned 1.24.2-1ubuntu1.8" in text
    assert loader.calls == 1                      # independent diagnostics still run


def test_changed_package_files_block():
    dpkg = fakes.Dpkg(verify_output="??5?????? c /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstrtp.so\n")
    result = fakes.run(command=dpkg)
    assert "dpkg --verify of the pinned packages reports: ??5??????" in blockers(result)
    assert result["package_files_verified"] is False


def test_missing_bindings_block_before_import():
    loader = fakes.Loader()
    assert "Python bindings folder" in blockers(fakes.run(loader=loader, list_bindings=lambda folder: None))
    assert "no gi extension (_gi.*)" in blockers(fakes.run(loader=loader, list_bindings=lambda folder: []))
    assert loader.calls == 0


@pytest.mark.parametrize("listed, blocked", [
    ((fakes.BINDING,), False),
    (("_gi.cpython-311-x86_64-linux-gnu.so", fakes.BINDING), False),       # one compatible among several
    (("_gi.cpython-311-x86_64-linux-gnu.so",), True),
    (("_gi.cpython-312t-x86_64-linux-gnu.so",), True),                      # free-threaded build
    (("_gi.cpython-312-aarch64-linux-gnu.so",), True),                      # the Orin's build
])
def test_binding_must_match_this_interpreter(listed, blocked):
    loadable, problems = media.check_binding(PC, fakes.INTERPRETER, list(listed))
    assert bool(problems) == blocked
    if blocked:
        assert "loads only _gi{.cpython-312-x86_64-linux-gnu.so, .abi3.so, .so}" in problems[0]
    else:
        assert loadable == [fakes.BINDING]


@pytest.mark.parametrize("interpreter", [
    media.Interpreter("3.12.3", "PyPy", "linux-x86_64", (".cpython-312-x86_64-linux-gnu.so",), "pypy"),
    media.Interpreter("3.11.9", "CPython", "win-amd64", (".cp311-win_amd64.pyd", ".pyd"), "navpy venv"),
])
def test_bindings_need_the_distro_cpython(interpreter):
    loader = fakes.Loader()
    assert "the distro bindings need CPython linux-x86_64" in blockers(fakes.run(loader=loader,
                                                                                 interpreter=interpreter))
    assert loader.calls == 0


@pytest.mark.parametrize("error", [ImportError("No module named 'gi'"), ValueError("Namespace Gst not available"),
                                   OSError("libgstreamer-1.0.so.0: cannot open shared object file")])
def test_import_failures_are_blockers(error):
    roundtrip = fakes.Roundtrip()
    text = blockers(fakes.run(loader=fakes.Loader(error=error), roundtrip=roundtrip))
    assert f"importing gi/Gst failed: {type(error).__name__}: {error}" in text
    assert roundtrip.calls == []


@pytest.mark.parametrize("field", ["gi_file", "gi_extension_file", "core_library"])
def test_files_from_outside_the_layout_or_no_pinned_package_are_refused(field):
    gi, core = fakes.fake_gi(), fakes.CORE
    stray = "/home/user/.local/lib/python3.12/site-packages/gi/" + ("__init__.py" if field == "gi_file"
                                                                     else fakes.BINDING)
    if field == "gi_file":
        gi.__file__ = stray
    elif field == "gi_extension_file":
        gi._gi.__file__ = stray
    else:
        core = stray = "/opt/gstreamer/lib/libgstreamer-1.0.so.0"
    roundtrip = fakes.Roundtrip()
    text = blockers(fakes.run(loader=fakes.Loader(gi=gi, core_library=core), roundtrip=roundtrip))
    assert f"gstreamer.{field} {stray} is not in" in text
    assert f"gstreamer.{field} {stray} is not from a pinned package (owner none)" in text
    assert roundtrip.calls == []


def test_unknown_core_library_is_refused():
    text = blockers(fakes.run(loader=fakes.Loader(core_library=None)))   # /proc/self/maps had no libgstreamer
    assert "gstreamer.core_library None is not in" in text


def test_plugin_owned_by_an_unpinned_package_or_at_another_version_is_refused():
    owners = fakes.default_owners()
    owners[fakes.plugin_file("rtspsrc")] = "gstreamer1.0-plugins-good-extra"
    gst = fakes.fake_gst()
    gst.elements["h265parse"] = ("1.24.1", fakes.plugin_file("h265parse"), True)
    text = blockers(fakes.run(loader=fakes.Loader(gst=gst), command=fakes.Dpkg(owners=owners)))
    assert "elements.rtspsrc.filename" in text and "owner gstreamer1.0-plugins-good-extra" in text
    assert "elements.h265parse.version 1.24.1 is not gstreamer1.0-plugins-bad's 1.24.2" in text


@pytest.mark.parametrize("version", [(1, 24, 1, 0), (1, 26, 0, 0), (1, 24, 2, 1)])
def test_other_gstreamer_versions_stop_before_elements(version):
    gst = fakes.fake_gst()
    gst.version_tuple = version
    result = fakes.run(loader=fakes.Loader(gst=gst))
    assert "not the pinned release 1.24.2" in blockers(result)
    assert result["elements"] == {}


@pytest.mark.parametrize("name", [*PC.elements, "nvcudah265enc", "nvh265dec"])
def test_each_missing_element_is_named(name):
    gst = fakes.fake_gst()
    del gst.elements[name]
    assert f"element {name} is not registered" in blockers(fakes.run(loader=fakes.Loader(gst=gst)))


def test_element_that_cannot_be_created_is_refused():
    gst = fakes.fake_gst()
    version, filename, _ = gst.elements["nvcudah265enc"]
    gst.elements["nvcudah265enc"] = (version, filename, False)      # nvcodec without a working GPU/driver
    roundtrip = fakes.Roundtrip()
    text = blockers(fakes.run(loader=fakes.Loader(gst=gst), roundtrip=roundtrip))
    assert "element nvcudah265enc is registered but cannot be created" in text
    assert roundtrip.calls == []


def test_roundtrip_problems_block():
    roundtrip = fakes.Roundtrip({"frames_pushed": 30, "frames_decoded": 0, "caps": None},
                                ["roundtrip decoded 0 of 30 frames"])
    result = fakes.run(roundtrip=roundtrip)
    assert "roundtrip decoded 0 of 30 frames" in blockers(result)
    assert result["roundtrip"]["frames_decoded"] == 0


def test_selection_outside_the_profile_blocks_before_import():
    loader = fakes.Loader()
    assert "encoder nvh265enc is not one of nvcudah265enc, x265enc" in blockers(
        fakes.run(loader=loader, encoder="nvh265enc"))
    assert "decoder nvv4l2decoder is not one of" in blockers(fakes.run(loader=loader, decoder="nvv4l2decoder"))
    assert loader.calls == 0


def test_alternative_encoder_and_decoder_are_inspected_at_their_package_versions():
    roundtrip = fakes.Roundtrip()
    result = fakes.run(encoder="x265enc", decoder="avdec_h265", roundtrip=roundtrip)
    assert result["verdict"] == "pass", result["blockers"]
    assert result["elements"]["avdec_h265"]["version"] == "1.24.1"            # gstreamer1.0-libav 1.24.1
    assert "nvcudah265enc" not in result["elements"] and roundtrip.calls == [("x265enc", "avdec_h265")]


def test_file_owners_parse_dpkg_output():
    out = ("python3-gi: /usr/lib/python3/dist-packages/gi/__init__.py\n"
           "gstreamer1.0-plugins-bad:amd64: /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnvcodec.so\n"
           "diversion by foo from: /usr/lib/x\n"
           "libfoo:amd64, libbar:amd64: /usr/share/shared\n")
    owners = media.file_owners(["/a"], lambda args: (0, out, ""))
    assert owners == {"/usr/lib/python3/dist-packages/gi/__init__.py": "python3-gi",
                      "/usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnvcodec.so": "gstreamer1.0-plugins-bad"}


@pytest.mark.parametrize("stray, reason", [
    ("/home/u/typelibs/Gst-1.0.typelib", "is not from a pinned package (owner none)"),        # GI_TYPELIB_PATH
    ("/opt/gst/lib/libgstvideo-1.0.so.0", "is not from a pinned package (owner none)"),        # LD_LIBRARY_PATH
    ("/home/u/.local/lib/libz.so.1", "is not from a distro package"),
])
def test_loaded_files_outside_the_packages_block(stray, reason):
    result = fakes.run(inventory=fakes.inventory(stray))
    assert f"loaded_files: {stray} {reason}" in blockers(result)
    assert stray in result["loaded_files"]


def test_loaded_media_file_of_an_unpinned_package_blocks_and_the_wsl_driver_is_allowed():
    override = "/usr/local/lib/python3.12/dist-packages/gi/overrides/GstExtra.py"      # gi.overrides extends its path
    owners = {**fakes.default_owners(), "/usr/lib/x86_64-linux-gnu/libgstvideo-1.0.so.0.2402.0": "libgst-custom",
              override: "python3-gst-extra"}
    text = blockers(fakes.run(command=fakes.Dpkg(owners=owners), inventory=fakes.inventory(override)))
    assert "libgstvideo-1.0.so.0.2402.0 is not from a pinned package (owner libgst-custom)" in text
    assert f"{override} is not from a pinned package (owner python3-gst-extra)" in text
    assert "/usr/lib/wsl/" not in text and "libc.so.6" not in text


def test_loaded_library_path_reads_the_process_maps(tmp_path):
    maps = tmp_path / "maps"
    maps.write_text("7f00-7f01 r--p 00000000 08:20 1 /usr/lib/x86_64-linux-gnu/libglib-2.0.so.0.8000.0\n"
                    "7f02-7f03 r-xp 00001000 08:20 2 /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so.0.2402.0\n"
                    "7f06-7f07 r--p 00000000 08:20 3 /usr/lib/x86_64-linux-gnu/girepository-1.0/Gst-1.0.typelib\n"
                    "7f08-7f09 r--p 00000000 08:20 4 /usr/lib/locale/locale-archive\n"
                    "7f0a-7f0b r--p 00000000 08:20 5 /tmp/a dir/libx.so (deleted)\n"
                    "7f04-7f05 rw-p 00000000 00:00 0 [heap]\n"
                    "7f0c-7f0d rw-p 00000000 00:00 0\n", encoding="utf-8")
    assert media.mapped_files(maps) == ["/tmp/a dir/libx.so (deleted)",
                                        "/usr/lib/x86_64-linux-gnu/girepository-1.0/Gst-1.0.typelib",
                                        "/usr/lib/x86_64-linux-gnu/libglib-2.0.so.0.8000.0", fakes.CORE]
    assert media.loaded_library_path("libgstreamer-1.0.so", maps) == fakes.CORE
    assert media.loaded_library_path("libgstnothing.so", maps) is None
    assert media.loaded_library_path("libgstreamer-1.0.so", tmp_path / "absent") is None


def test_cli_writes_the_result_once_and_exits_by_verdict(tmp_path, monkeypatch, capsys):
    real = media.check_runtime
    monkeypatch.setattr(media, "check_runtime", lambda profile, **kwargs: real(
        profile, **kwargs, loader=fakes.Loader(), command=fakes.Dpkg(packages={}), roundtrip=fakes.Roundtrip(),
        list_bindings=fakes.bindings(), driver=lambda: "GPU, 1.0"))
    out = tmp_path / "media_runtime.json"
    assert media.main(["--host", "pc-wsl", "--out", str(out)]) == 1
    written = json.loads(out.read_text(encoding="utf-8"))
    assert written["verdict"] == "blocked" and "not installed" in "; ".join(written["blockers"])
    assert "BLOCKED: packages." in capsys.readouterr().err
    assert media.main(["--host", "pc-wsl", "--out", str(out)]) == 1
    assert "refused:" in capsys.readouterr().err
    assert json.loads(out.read_text(encoding="utf-8")) == written


@pytest.mark.parametrize("module", [media, profiles, media_roundtrip])
def test_probe_imports_only_the_standard_library(module):
    """It must run with the distro's own Python, without navpy's dependencies."""
    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    imported = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    imported |= {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.level == 0}
    allowed = set(sys.stdlib_module_names) | {"__future__", "gi", "gi.repository", "tools.hitl",
                                              "tools.hitl.media_profiles"}
    assert {name for name in imported if name.split(".")[0] not in allowed and name not in allowed} == set()


def test_documented_launch_runs_without_navpy_on_the_path():
    """python -E -s -m from the repository root: no PYTHONPATH, no user site; the verdict sets the exit code."""
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    result = subprocess.run([sys.executable, "-E", "-s", "-m", "tools.hitl.check_media_runtime", "--host", "pc-wsl",
                             "--encoder", "x265enc", "--decoder", "avdec_h265"],
                            cwd=Path(media.__file__).resolve().parents[2], env=env, capture_output=True, text=True,
                            timeout=300, check=False)
    verdict = json.loads(result.stdout)["verdict"]
    assert result.returncode == (0 if verdict == "pass" else 1), result.stderr
