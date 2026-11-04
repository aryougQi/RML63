"""Explicit operator placement confirmation updates only a temporary config."""
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import confirm_place


class ConfirmationTests(unittest.TestCase):
    def files(self, directory):
        config = Path(directory)/'fixed.yaml'
        config.write_text(yaml.safe_dump(dict(release_mode='taught_place', velocity_scaling=.35,
            taught_place_verification=dict(verified=False))))
        points = Path(directory)/'waypoints.yaml'
        points.write_text(yaml.safe_dump(dict(schema=1, identity={'test': True}, waypoints={
            name: {'measured': True} for name in ('Home', 'Grasp', 'Preplace', 'Place', 'Retreat')})))
        return config, points

    def test_records_explicit_source_and_current_digest_with_backup(self):
        with tempfile.TemporaryDirectory() as directory:
            config, points = self.files(directory)
            previous = config.read_bytes()
            backup = confirm_place.confirm(config, points, '现场实测确认')
            data = yaml.safe_load(config.read_text())
            self.assertEqual(backup.read_bytes(), previous)
            self.assertEqual(data['velocity_scaling'], .35)
            self.assertTrue(data['taught_place_verification']['verified'])
            self.assertEqual(data['taught_place_verification']['source'], '现场实测确认')
            self.assertEqual(data['taught_place_verification']['waypoints_sha256'],
                             hashlib.sha256(points.read_bytes()).hexdigest())

    def test_failed_atomic_write_preserves_existing_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            config, points = self.files(directory)
            previous = config.read_bytes()
            with patch.object(confirm_place.os, 'replace', side_effect=OSError('write failed')):
                with self.assertRaises(OSError): confirm_place.confirm(config, points, '现场实测确认')
            self.assertEqual(config.read_bytes(), previous)

    def test_command_requires_explicit_operator_flag(self):
        with patch.object(sys, 'argv', ['confirm_place.py', '--source', '现场实测确认']), \
                patch('sys.stderr'), patch.object(confirm_place, 'confirm') as write:
            with self.assertRaises(SystemExit): confirm_place.main()
        write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
