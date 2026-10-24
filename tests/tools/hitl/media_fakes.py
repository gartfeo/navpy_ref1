"""A synthetic distro install, package database and fake gi/Gst for tools/hitl/check_media_runtime.py; nothing real
is loaded."""
from __future__ import annotations

from types import SimpleNamespace
from typing import Optional

from tools.hitl import check_media_runtime as media
from tools.hitl import media_profiles as profiles

PC = profiles.PROFILES["pc-wsl"]
INTERPRETER = media.Interpreter(version="3.12.3", implementation="CPython", platform="linux-x86_64",
                                extension_suffixes=(".cpython-312-x86_64-linux-gnu.so", ".abi3.so", ".so"),
                                executable="/usr/bin/python3")
BINDING = "_gi.cpython-312-x86_64-linux-gnu.so"
CORE = f"{PC.layout.libraries}/libgstreamer-1.0.so.0.2402.0"
# The package each element's plugin file comes from, as on the real distro.
ELEMENT_PACKAGES = {"appsrc": "gstreamer1.0-plugins-base", "appsink": "gstreamer1.0-plugins-base",
                    "videoconvert": "gstreamer1.0-plugins-base", "rtspsrc": "gstreamer1.0-plugins-good",
                    "rtph265depay": "gstreamer1.0-plugins-good", "rtph265pay": "gstreamer1.0-plugins-good",
                    "h265parse": "gstreamer1.0-plugins-bad", "rtspclientsink": "gstreamer1.0-rtsp",
                    "nvcudah265enc": "gstreamer1.0-plugins-bad", "x265enc": "gstreamer1.0-plugins-bad",
                    "nvh265dec": "gstreamer1.0-plugins-bad", "avdec_h265": "gstreamer1.0-libav"}
PASSING_ROUNDTRIP = {"frames_pushed": 30, "frames_decoded": 30, "caps": "video/x-raw, format=(string)BGR",
                     "format": "BGR", "width": 640, "height": 360}
# Loaded after the roundtrip besides the named files: a support library, a typelib, an override, a system library
# and the WSL GPU driver.
EXTRA_LOADED = {"/usr/lib/x86_64-linux-gnu/libgstvideo-1.0.so.0.2402.0": "libgstreamer-plugins-base1.0-0",
                "/usr/lib/x86_64-linux-gnu/girepository-1.0/Gst-1.0.typelib": "gir1.2-gstreamer-1.0",
                f"{PC.layout.gi}/overrides/Gst.py": "python3-gst-1.0",
                "/usr/lib/x86_64-linux-gnu/libc.so.6": "libc6",
                "/usr/lib/wsl/lib/libnvidia-encode.so.1": None}


def plugin_file(name: str) -> str:
    return f"{PC.layout.plugins}/libgst{name}.so"


def bindings(*names: str):
    """A bindings-folder listing holding names (the pinned binding by default)."""
    listed = list(names) if names else [BINDING]
    return lambda folder: listed


class Dpkg:
    """dpkg-query/dpkg answers from a package table and a file -> package table."""

    def __init__(self, packages: Optional[dict] = None, owners: Optional[dict] = None, verify_output: str = ""):
        self.packages = dict(PC.packages) if packages is None else packages
        self.owners = owners
        self.verify_output = verify_output
        self.calls: list[list[str]] = []

    def __call__(self, args):
        args = list(args)
        self.calls.append(args)
        if args[:2] == ["dpkg-query", "-W"]:
            return 0, "".join(f"{name}\t{self.packages[name]}\tii \n" for name in args[3:] if name in self.packages), ""
        if args[:2] == ["dpkg", "--verify"]:
            return (1 if self.verify_output else 0), self.verify_output, ""
        if args[:2] == ["dpkg-query", "-S"]:
            owners = self.owners if self.owners is not None else default_owners()
            return 0, "".join(f"{owners[f]}:amd64: {f}\n" for f in args[2:] if f in owners), ""
        raise AssertionError(f"unexpected command {args}")


def default_owners() -> dict[str, str]:
    owners = {f"{PC.layout.gi}/__init__.py": "python3-gi", f"{PC.layout.gi}/{BINDING}": "python3-gi",
              CORE: "libgstreamer1.0-0"}
    owners.update({plugin_file(name): package for name, package in ELEMENT_PACKAGES.items()})
    owners.update({file: package for file, package in EXTRA_LOADED.items() if package is not None})
    return owners


def inventory(*extra: str):
    """What the process loaded: the pinned files, EXTRA_LOADED and extra."""
    files = [*default_owners(), *EXTRA_LOADED, *extra]
    return lambda: sorted(set(files))


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
    def __init__(self, plugin: Plugin) -> None:
        self._plugin = plugin

    def get_plugin(self) -> Plugin:
        return self._plugin

    def get_rank(self) -> int:
        return 256


class FakeGst:
    """Gst with a registry of elements: name -> (plugin version, plugin file, creatable)."""

    def __init__(self, version: tuple[int, int, int, int], elements: dict[str, tuple[str, Optional[str], bool]]):
        self.version_tuple, self.elements = version, elements
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
        return self.version_tuple


def fake_gst() -> FakeGst:
    """A Gst holding every element the profile could select, each from its package's plugin file and version."""
    return FakeGst((*PC.gst_version, 0),
                   {name: (profiles.upstream_version(PC.packages[package]), plugin_file(name), True)
                    for name, package in ELEMENT_PACKAGES.items()})


def fake_gi() -> SimpleNamespace:
    return SimpleNamespace(__file__=f"{PC.layout.gi}/__init__.py", __version__="3.48.2",
                           _gi=SimpleNamespace(__file__=f"{PC.layout.gi}/{BINDING}"))


class Loader:
    """A loader that counts its calls and returns the given fakes (by default the pinned ones), or raises."""

    def __init__(self, gi=None, gst=None, core_library: Optional[str] = CORE,
                 error: Optional[Exception] = None) -> None:
        self.gi, self.gst, self.core_library, self.error, self.calls = gi, gst, core_library, error, 0

    def __call__(self):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return (self.gi if self.gi is not None else fake_gi(), self.gst if self.gst is not None else fake_gst(),
                self.core_library)


class Roundtrip:
    """A roundtrip that records its selection and returns the given counts and problems."""

    def __init__(self, found: Optional[dict] = None, problems: Optional[list] = None) -> None:
        self.found = dict(PASSING_ROUNDTRIP) if found is None else found
        self.problems, self.calls = problems or [], []

    def __call__(self, gst, encoder, decoder):
        self.calls.append((encoder, decoder))
        return dict(self.found), list(self.problems)


def run(**changes) -> dict:
    kwargs = dict(encoder="nvcudah265enc", decoder="nvh265dec", interpreter=INTERPRETER, loader=Loader(),
                  command=Dpkg(), roundtrip=Roundtrip(), list_bindings=bindings(), inventory=inventory(),
                  driver=lambda: "GPU, 1.0")
    kwargs.update(changes)
    return media.check_runtime(PC, **kwargs)


def passing_pc_result() -> dict:
    """A passing pc-wsl result over a synthetic bindings listing and package database."""
    result = run(driver=lambda: "Synthetic GPU, 1.0")
    assert result["verdict"] == "pass", result["blockers"]
    return result
