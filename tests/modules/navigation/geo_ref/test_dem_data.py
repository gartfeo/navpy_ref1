import unittest
from pathlib import Path

from navpy.modules.navigation.geo.zc_util import DemData

DEM_FILE = Path(__file__).resolve().parents[4] / '.terrain' / 'dem' / 'N40E044.hgt'


class DemDataTestCase(unittest.TestCase):
    def setUp(self):
        self.dem = DemData(str(DEM_FILE))

    def test_get_hgt_file_name(self):
        self.assertEqual(DemData.get_hgt_file_name([40.4, 44.5]), 'N40E044.hgt')
        self.assertEqual(DemData.get_hgt_file_name([-5.5, -1.2]), 'S06W002.hgt')

    def test_gps_to_dem_coords(self):
        x, z = self.dem.gps_to_dem_coords(41.0, 44.0)
        self.assertAlmostEqual(x, 0.5, places=2)
        self.assertAlmostEqual(z, 0.5, places=2)

    def test_get_height(self):
        alt = self.dem.get_height((40.31148910522461, 44.455230712890625))
        self.assertIsNotNone(alt)
        self.assertAlmostEqual(alt, 1295.75, places=1)


if __name__ == '__main__':
    unittest.main()
