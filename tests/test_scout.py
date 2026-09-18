import unittest
from types import SimpleNamespace

from src.scout import matches, moved


class ScoutTests(unittest.TestCase):
    def test_horizontal_movement_ignores_pitch(self):
        p = [0, 0, 30, -45, 90, 0]
        result = moved(p, "w", 5)
        self.assertAlmostEqual(result[0], 0)
        self.assertAlmostEqual(result[1], 5)
        self.assertEqual(result[2:], p[2:])
        self.assertEqual(p[0:3], [0, 0, 30])

    def test_up_and_right(self):
        self.assertEqual(moved([0, 0, 30, -45, 0, 0], "up", 5)[2], 35)
        self.assertEqual(moved([0, 0, 30, -45, 0, 0], "d", 5)[1], 5)

    def test_reject_nonfinite(self):
        with self.assertRaises(ValueError):
            moved([0, 0, 30, -45, 0, 0], "w", float("nan"))

    def test_old_pose_rejected(self):
        t = SimpleNamespace(
            location=SimpleNamespace(x=0, y=0, z=30),
            rotation=SimpleNamespace(pitch=-45, yaw=0, roll=0),
        )
        self.assertTrue(matches(t, [0, 0, 30, -45, 360, 0]))
        self.assertFalse(matches(t, [5, 0, 30, -45, 0, 0]))


if __name__ == "__main__":
    unittest.main()
