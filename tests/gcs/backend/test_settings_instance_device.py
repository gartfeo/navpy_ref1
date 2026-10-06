"""Regression test for B-1: the per-chat monitor device that _with_instance_device
injects into GET responses must NOT be persisted when the frontend round-trips
the whole draft back on Save. See routes/settings.py._drop_injected_instance_device.
"""
import os
import unittest
from unittest import mock

from gcs.backend import instance_ports
from gcs.backend.routes.settings import _drop_injected_instance_device

_ENV = "GCS_CHAT_INDEX"


class DropInjectedInstanceDeviceTest(unittest.TestCase):
    def test_injected_device_dropped_in_launcher_mode(self):
        with mock.patch.dict(os.environ, {_ENV: "1"}):
            injected = instance_ports.monitor_device(1)   # udp:0.0.0.0:15551
            body = {"connection": {"default_device": injected, "auto_connect": True}}
            _drop_injected_instance_device(body)
            # The injected per-chat device is stripped so it can't be persisted...
            self.assertNotIn("default_device", body["connection"])
            # ...but unrelated edits in the same draft are preserved.
            self.assertEqual(body["connection"]["auto_connect"], True)

    def test_genuine_user_device_preserved(self):
        with mock.patch.dict(os.environ, {_ENV: "1"}):
            body = {"connection": {"default_device": "udp:0.0.0.0:14550"}}
            _drop_injected_instance_device(body)
            self.assertEqual(body["connection"]["default_device"], "udp:0.0.0.0:14550")

    def test_injected_sitl_presets_dropped_in_launcher_mode(self):
        with mock.patch.dict(os.environ, {_ENV: "1"}):
            injected = [instance_ports.companion_device(s)
                        for s in instance_ports.sysids_for_chat(1)]
            body = {"simulation": {"sitl_presets": injected, "sim_mode": True}}
            _drop_injected_instance_device(body)
            # Injected per-chat companion presets stripped so they aren't persisted...
            self.assertNotIn("sitl_presets", body["simulation"])
            # ...unrelated edits in the same draft are preserved.
            self.assertEqual(body["simulation"]["sim_mode"], True)

    def test_genuine_user_sitl_presets_preserved(self):
        with mock.patch.dict(os.environ, {_ENV: "1"}):
            custom = ["udp:0.0.0.0:14560", "udp:0.0.0.0:14570"]
            body = {"simulation": {"sitl_presets": custom}}
            _drop_injected_instance_device(body)
            self.assertEqual(body["simulation"]["sitl_presets"], custom)

    def test_noop_without_chat_index(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop(_ENV, None)
            dev = instance_ports.monitor_device(1)
            body = {"connection": {"default_device": dev}}
            _drop_injected_instance_device(body)
            self.assertEqual(body["connection"]["default_device"], dev)


if __name__ == "__main__":
    unittest.main()
