import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools" / "replay_measurement_noise.py"
SPEC = importlib.util.spec_from_file_location("replay_measurement_noise", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MeasurementReplayTest(unittest.TestCase):
    def test_replay_accepts_repeat_and_rejects_large_jump(self):
        fieldnames = [
            "tag_id",
            "range_m",
            "z_forward_m",
            "z_left_m",
        ]
        rows = [
            {"tag_id": 0, "range_m": 1.0, "z_forward_m": 1.0, "z_left_m": 0.0},
            {"tag_id": 0, "range_m": 1.0, "z_forward_m": 1.01, "z_left_m": 0.0},
            {"tag_id": 0, "range_m": 3.0, "z_forward_m": 3.0, "z_left_m": 0.0},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "observations.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            result = MODULE.replay([path], 0.01, 0.005, 9.21034037197618)

        self.assertEqual(result["observation_count"], 3)
        self.assertEqual(result["accepted"], 2)
        self.assertEqual(result["rejected"], 1)
        self.assertGreater(result["nis_max"], result["gate_chi2"])


if __name__ == "__main__":
    unittest.main()
