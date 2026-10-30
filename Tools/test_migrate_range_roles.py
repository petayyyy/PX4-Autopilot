#!/usr/bin/env python3
import unittest
import subprocess
import sys
import tempfile
from pathlib import Path
from migrate_range_roles import device_id, migrate


class RangeRoleMigrationTest(unittest.TestCase):
    export = """# QGC export
1 1 EKF2_RNG_CAN_MAX 1.0 9
1 1 EKF2_RNG_CAN_HYS 0.2 9
1 1 EKF2_RNG_CAN_TO 0.3 9
1 1 EKF2_RNG_RAD_TO 0.25 9
1 1 EKF2_RNG_MAV_TO 2.0 9
1 1 CUSTOM_PARAMETER 42 6
"""

    def test_preserves_values_and_unrelated_parameters(self):
        result = migrate(self.export, 123, 456, "radar")
        self.assertTrue(result.startswith(self.export))
        for name, value in [("S_MAX", "1.0"), ("HYST", "0.2"), ("S_TO", "0.3"), ("L_TO", "0.25")]:
            self.assertIn(f"EKF2_RNG_{name}\t{value}\t9", result)
        self.assertIn("EKF2_RNG_MAV_MAX absent", result)
        self.assertIn("EKF2_RNG_S_ID\t123\t6", result)

    def test_long_timeout_is_explicit(self):
        result = migrate(self.export, 123, 456, "mavlink")
        self.assertIn("EKF2_RNG_L_TO\t2.0\t9", result)
        with self.assertRaises(ValueError):
            migrate(self.export, 123, 456, "auto")

    def test_full_width_ids_and_single_sensor(self):
        self.assertEqual(device_id("-1"), 0xFFFFFFFF)
        self.assertEqual(device_id("0xffffffff"), 0xFFFFFFFF)
        result = migrate(self.export, 0xFFFFFFFF, 0, "radar")
        self.assertIn("EKF2_RNG_S_ID\t-1\t6", result)
        self.assertIn("EKF2_RNG_L_ID\t0\t6", result)

    def test_conflicts_fail_instead_of_overwriting(self):
        for ids in [(123, 123), (0, 0)]:
            with self.assertRaises(ValueError):
                migrate(self.export, *ids, "radar")
        with self.assertRaises(ValueError):
            migrate(self.export + "1 1 EKF2_RNG_L_TO 3.0 9\n", 123, 456, "radar")

    def test_idempotent(self):
        first = migrate(self.export, 123, 456, "radar")
        second = migrate(first, 123, 456, "radar")
        self.assertEqual(second.count("EKF2_RNG_S_ID\t"), 1)

    def test_invalid_export_rejected(self):
        for text in ["", "param set A 3", self.export + "2 1 OTHER 0 6",
                     self.export + "1 1 EKF2_RNG_RAD_TO 1 9", self.export.replace("0.25", "nan"),
                     self.export.replace("0.25", "-0.25")]:
            with self.assertRaises(ValueError):
                migrate(text, 123, 456, "radar")

    def test_cli_does_not_overwrite_files(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/"old.params", Path(directory)/"new.params"
            source.write_text(self.export)
            command = [sys.executable, str(Path(__file__).with_name("migrate_range_roles.py")),
                       str(source), str(output), "--short-id", "123", "--long-id", "456",
                       "--long-timeout-from", "radar"]
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 0)
            first = output.read_bytes()
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            self.assertEqual(output.read_bytes(), first)
            self.assertEqual(source.read_text(), self.export)


if __name__ == "__main__":
    unittest.main()
