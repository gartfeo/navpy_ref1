"""Source-level check that mission upload includes DOCK type."""
import os
import unittest


_HOOK_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks", "useMissionUpload.js",
))


class TestMissionUploadDockType(unittest.TestCase):
    def test_default_delivery_hub_includes_type(self):
        with open(_HOOK_FILE, encoding="utf-8") as f:
            src = f.read()
        self.assertIn(
            "default_delivery_hub: assignedDeliveryHub ? { lat: assignedDeliveryHub.lat, lon: assignedDeliveryHub.lon, type: assignedDeliveryHub.type } : null",
            src,
        )


if __name__ == "__main__":
    unittest.main()
