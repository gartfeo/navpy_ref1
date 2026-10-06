import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from navpy.modules.vision.device import (
    resolve_auto_device,
    resolve_reid_device,
    uses_cuda_device,
)


def _torch(available):
    return SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: bool(available)))


class TestUsesCudaDevice(unittest.TestCase):
    def test_index_zero_and_positive_are_cuda(self):
        self.assertTrue(uses_cuda_device(0))
        self.assertTrue(uses_cuda_device(1))  # second GPU (regression: was False)
        self.assertTrue(uses_cuda_device("0"))
        self.assertTrue(uses_cuda_device("1"))

    def test_cuda_strings_are_cuda(self):
        self.assertTrue(uses_cuda_device("cuda"))
        self.assertTrue(uses_cuda_device("cuda:1"))

    def test_cpu_and_bool_are_not_cuda(self):
        self.assertFalse(uses_cuda_device("cpu"))
        self.assertFalse(uses_cuda_device(-1))
        self.assertFalse(uses_cuda_device(True))
        self.assertFalse(uses_cuda_device(False))


class TestResolveAutoDevice(unittest.TestCase):
    def test_auto_resolves_to_index_zero_when_cuda(self):
        with patch.dict(sys.modules, {"torch": _torch(True)}):
            self.assertEqual(resolve_auto_device("auto"), 0)

    def test_auto_resolves_to_cpu_without_cuda(self):
        with patch.dict(sys.modules, {"torch": _torch(False)}):
            self.assertEqual(resolve_auto_device("auto"), "cpu")

    def test_auto_resolves_to_cpu_when_torch_missing(self):
        # Simulate torch import failure.
        real_import = __builtins__["__import__"] if isinstance(__builtins__, dict) else __builtins__.__import__

        def fake_import(name, *args, **kwargs):
            if name == "torch":
                raise ImportError("no torch")
            return real_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=fake_import):
            self.assertEqual(resolve_auto_device("auto"), "cpu")

    def test_explicit_device_passes_through(self):
        self.assertEqual(resolve_auto_device("cpu"), "cpu")
        self.assertEqual(resolve_auto_device(1), 1)


class TestResolveReidDevice(unittest.TestCase):
    def test_auto_auto_resolves_cuda_when_available(self):
        with patch.dict(sys.modules, {"torch": _torch(True)}):
            self.assertEqual(resolve_reid_device("auto", "auto"), "cuda:0")

    def test_auto_auto_resolves_cpu_when_unavailable(self):
        with patch.dict(sys.modules, {"torch": _torch(False)}):
            self.assertEqual(resolve_reid_device("auto", "auto"), "cpu")

    def test_inherits_detector_device(self):
        self.assertEqual(resolve_reid_device("auto", "cpu"), "cpu")
        self.assertEqual(resolve_reid_device("auto", 1), "cuda:1")

    def test_explicit_reid_device_wins(self):
        self.assertEqual(resolve_reid_device("cpu", 0), "cpu")
        self.assertEqual(resolve_reid_device(1, "cpu"), "cuda:1")
        self.assertEqual(resolve_reid_device("cuda:0", "cpu"), "cuda:0")


if __name__ == "__main__":
    unittest.main()
