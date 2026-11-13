import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools" / "estimate_noise.py"
SPEC = importlib.util.spec_from_file_location("estimate_noise", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class NoiseEstimationTest(unittest.TestCase):
    def test_motion_trials_are_summarized_by_primitive_kind(self):
        fieldnames = [
            "kind",
            "commanded_distance_m",
            "actual_distance_m",
            "commanded_delta_yaw_rad",
            "actual_delta_yaw_rad",
            "forward_std_m",
            "yaw_std_rad",
        ]
        rows = [
            ["straight", 0.05, 0.051, 0.0, 0.001, 0.002, 0.003],
            ["straight", 0.05, 0.049, 0.0, -0.001, 0.002, 0.003],
            ["left", 0.0, 0.001, 0.052, 0.030, 0.001, 0.002],
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motion.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(fieldnames)
                writer.writerows(rows)
            result = MODULE.estimate_motion(path)

        self.assertEqual(result["trial_count"], 3)
        self.assertEqual(result["group_summaries"]["straight"]["trial_count"], 2)
        self.assertEqual(result["group_summaries"]["left"]["trial_count"], 1)
        self.assertIsNotNone(
            result["group_summaries"]["straight"]["sample_covariance"]
        )
        self.assertIsNone(result["group_summaries"]["left"]["sample_covariance"])


if __name__ == "__main__":
    unittest.main()
