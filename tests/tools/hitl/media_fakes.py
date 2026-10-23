"""A synthetic GStreamer install and a fake gi/Gst for tools/hitl/check_media_runtime.py; nothing real is loaded."""
from __future__ import annotations

import contextlib
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

from tools.hitl import check_media_runtime as media
from tools.hitl import media_profiles as profiles

PC = profiles.PROFILES["pc"]
INTERPRETER = media.Interpreter(version="3.13.9", implementation="CPython", platform="win-amd64",
                                extension_suffixes=(".cp313-win_amd64.pyd", ".pyd"), executable="C:/py313/python.exe")
BINDING = "_gi.cp313-win_amd64.pyd"


def install(root: Path, bindings: tuple[str, ...] = (BINDING,)) -> profiles.Layout:
    """An official-looking install folder (empty files where DLLs, bindings and plugins would be)."""
    layout = profiles.layout_of(root)
    for folder in (layout.bin, layout.gi, layout.plugins):
        folder.mkdir(parents=True, exist_ok=True)
    for name in bindings:
        (layout.gi / name).write_bytes(b"")
    (layout.gi / "__init__.py").write_bytes(b"")
    return layout


def installers(folder: Path, profile: profiles.HostProfile = PC) -> tuple[Path, dict[str, bytes]]:
    """Installer stand-ins whose pinned sums are patched to match (see pinned_installers)."""
    folder.mkdir(parents=True, exist_ok=True)
    contents = {name: f"synthetic {name}".encode() for name in profile.installers}
    for name, data in contents.items():
        (folder / name).write_bytes(data)
    return folder, contents


class Plugin:
    def __init__(self, name: str, version: str, filename: Optional[str]) -> None:
        self._name, self._version, self._filename = name, version, filename

    def get_name(self) -> str:
        return self._name

    def get_version(self) -> str:
        return self._version

    def get_filename(self) -> Optional[str]:
        return self._filename


class Factory:
    def __init__(self, plugin: Optional[Plugin], rank: int = 256) -> None:
        self._plugin, self._rank = plugin, rank

    def get_plugin(self) -> Optional[Plugin]:
        return self._plugin

    def get_rank(self) -> int:
        return self._rank


class FakeGst:
    """Gst with a registry of elements: name -> (plugin version, plugin file, creatable)."""

    def __init__(self, version: tuple[int, int, int, int], elements: dict[str, tuple[str, Optional[str], bool]]):
        self._version, self.elements = version, elements
        registry = self

        class ElementFactory:
            @staticmethod
            def find(name):
                if name not in registry.elements:
                    return None
                plugin_version, filename, _ = registry.elements[name]
                return Factory(Plugin(f"plugin-{name}", plugin_version, filename))

            @staticmethod
            def make(name, _):
                return object() if name in registry.elements and registry.elements[name][2] else None

        self.ElementFactory = ElementFactory

    def version(self):
        return self._version


def fake_gst(layout: Optional[profiles.Layout], profile: profiles.HostProfile = PC,
             names: Optional[list[str]] = None) -> FakeGst:
    """A Gst holding every element the profile could select, each from layout's plugin folder."""
    names = names or [*profile.elements, *profile.encoders, *profile.decoders]
    folder = layout.plugins if layout is not None else Path("/usr/lib/aarch64-linux-gnu/gstreamer-1.0")
    return FakeGst((*profile.gst_version, 0),
                   {name: (profile.version_text, str(folder / f"gst{name}.dll"), True) for name in names})


def fake_gi(layout: Optional[profiles.Layout]) -> SimpleNamespace:
    folder = layout.gi if layout is not None else Path("/usr/lib/python3/dist-packages/gi")
    return SimpleNamespace(__file__=str(folder / "__init__.py"), __version__="3.50.0")


class Loader:
    """A loader that records its calls and yields the given fakes, or raises."""

    def __init__(self, gi=None, gst=None, error: Optional[Exception] = None) -> None:
        self.gi, self.gst, self.error, self.calls = gi, gst, error, []

    @contextlib.contextmanager
    def __call__(self, layout):
        self.calls.append(layout)
        if self.error is not None:
            raise self.error
        yield (self.gi if self.gi is not None else fake_gi(layout),
               self.gst if self.gst is not None else fake_gst(layout))


def pinned_installers(monkeypatch, contents: dict[str, bytes], profile: profiles.HostProfile = PC) -> None:
    """Pin the stand-ins' sums in place of the official ones, for this test only."""
    import hashlib
    sums = {name: hashlib.sha256(data).hexdigest() for name, data in contents.items()}
    monkeypatch.setitem(profiles.PROFILES, profile.name, profiles.HostProfile(**{**profile.__dict__, "installers": sums}))


def passing_pc_result(tmp_path: Path, monkeypatch) -> dict:
    """A passing pc result over a synthetic install, with the stand-ins' sums pinned."""
    layout = install(tmp_path / "gstreamer")
    folder, contents = installers(tmp_path / "msi")
    pinned_installers(monkeypatch, contents)
    result = media.check_runtime(profiles.PROFILES["pc"], root=layout.root, installers=folder, encoder="nvh265enc",
                                 decoder="nvh265dec", interpreter=INTERPRETER, loader=Loader(),
                                 driver=lambda: "Synthetic GPU, 1.0")
    assert result["verdict"] == "pass", result["blockers"]
    return result
