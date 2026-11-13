import unittest

import numpy as np

from robot_control.trajectory import build_waypoints


class TrajectoryTest(unittest.TestCase):
    def test_square_closes_each_lap(self):
        points = build_waypoints("square", 1.2, 2)
        self.assertEqual(len(points), 9)
        np.testing.assert_allclose(points[4], [0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(points[-1], [0.0, 0.0], atol=1e-12)

    def test_octagon_has_eight_edges_and_closes(self):
        points = build_waypoints("octagon", 0.7, 1)
        self.assertEqual(len(points), 9)
        np.testing.assert_allclose(points[-1], [0.0, 0.0], atol=1e-12)
        lengths = [
            np.linalg.norm(points[index + 1] - points[index])
            for index in range(8)
        ]
        np.testing.assert_allclose(lengths, np.full(8, 0.7), atol=1e-12)

    def test_invalid_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            build_waypoints("circle", 1.0, 1)


if __name__ == "__main__":
    unittest.main()
