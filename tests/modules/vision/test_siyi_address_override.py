"""--siyi-ip (HITL runner only): the SIYI gimbal's address over the profile's, the profile itself untouched."""
from __future__ import annotations

import argparse

import pytest

from navpy.app.perception import vision_profile_composition
from navpy.app.perception.siyi_address_override import with_siyi_ip
from navpy.app.vision_args import VisionArgs
from navpy.core.perception.profile import vision_profile_loader


class _Logger:
    def info(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


def _hitl_profile() -> dict:
    return vision_profile_loader.resolve_profile("siyi_zr10_hitl_encoded", _Logger())[1]


def test_the_flag_reaches_vision_args():
    parser = argparse.ArgumentParser()
    VisionArgs.add_args(parser)
    assert VisionArgs.from_args(parser.parse_args(["--siyi-ip", "172.23.112.1"])).siyi_ip == "172.23.112.1"
    assert VisionArgs.from_args(parser.parse_args([])).siyi_ip == ""


def test_the_address_replaces_the_profiles_and_leaves_the_profile_alone():
    profile = _hitl_profile()
    moved = with_siyi_ip(profile, "172.23.112.1")
    assert [d["gimbal"]["siyi_ip"] for d in vision_profile_loader.get_devices(moved)] == ["172.23.112.1"]
    assert [d["gimbal"]["siyi_ip"] for d in vision_profile_loader.get_devices(profile)] == ["127.0.0.1"]
    assert with_siyi_ip(profile, "") is profile


def test_a_malformed_address_or_a_profile_without_siyi_is_refused():
    with pytest.raises(ValueError):
        with_siyi_ip(_hitl_profile(), "host.local")
    with pytest.raises(ValueError, match="no SIYI gimbal"):
        with_siyi_ip({"devices": [{"gimbal": {"type": "sim"}}]}, "172.23.112.1")


def test_the_profile_assembly_builds_its_mounts_from_the_moved_address(monkeypatch):
    seen = []

    def mounts(raw_profile, *_args, **_kwargs):
        seen.extend(d["gimbal"]["siyi_ip"] for d in vision_profile_loader.get_devices(raw_profile))
        return []

    monkeypatch.setattr(vision_profile_composition, "build_camera_mounts", mounts)
    with pytest.raises(ValueError, match="No camera mounts"):        # stops right after the mounts are built
        vision_profile_composition.build_vision_profile("siyi_zr10_hitl_encoded", object(), _Logger(), None,
                                                        use_sim_gimbal=False, siyi_ip="172.23.112.1")
    assert seen == ["172.23.112.1"]
