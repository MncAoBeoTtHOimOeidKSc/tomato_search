import math
import unittest

import numpy as np

from main import mount_matrix
from search_geometry import approach, checked_matrix, plant_map, rigid


class GeometryTest(unittest.TestCase):
    def test_camera_to_map_composes_translation_and_rotation(self):
        handeye = rigid([.1, 0, 0], [0, 0, 0])
        mount = rigid([.3, .2, .5], [0, 0, 90])
        point = plant_map([.2, .1, .5], [.5, 0, .2, 0, 0, 0], handeye, mount, [2, 3, math.pi/2])
        np.testing.assert_allclose(point, [1., 3.2, 1.2])

    def test_approach_stays_short_of_plant_and_faces_it(self):
        self.assertEqual(approach([0, 0, 0], [0, 2, .3], .65), [0., 1.35, math.pi/2])
        self.assertIsNone(approach([0, 0, 0], [0, .7, .3], .65))

    def test_unmeasured_mount_and_reflection_fail(self):
        with self.assertRaisesRegex(ValueError, 'SET_HIGH_ARM'):
            mount_matrix(dict(arm_base_in_base_link_xyz_m=None, arm_base_in_base_link_rpy_deg=None))
        with self.assertRaisesRegex(ValueError, 'INVALID_RIGID'):
            checked_matrix(np.diag([-1., 1., 1., 1.]))
