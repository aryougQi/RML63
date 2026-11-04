"""Independent grasp selection must preserve shared placement and measured poses."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from grasp_profiles import action_name, load_task_waypoints, new_grasp_profile, read_shared
from waypoint_offsets import derive_from_grasp


class ProfileTests(unittest.TestCase):
    def shared(self):
        pose = np.eye(4).tolist()
        record = dict(joints_rad=[0.]*6, tcp_base=pose)
        data = dict(schema=1, identity={'model': 'same'}, waypoints={
            name: deepcopy(record) for name in ('Home', 'Grasp', 'Preplace', 'Place', 'Retreat')})
        return derive_from_grasp(data)

    def files(self, directory):
        first, second = Path(directory)/'waypoints.yaml', Path(directory)/'waypoints_2.yaml'
        first.write_text(yaml.safe_dump(self.shared()))
        shared, digest = read_shared(first)
        profile = new_grasp_profile(shared, digest)
        pose = np.eye(4)
        pose[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
        pose[:3, 3] = [.2, -.3, .1]
        profile['waypoints']['Grasp'] = dict(joints_rad=[.1]*6, tcp_base=pose.tolist())
        profile = derive_from_grasp(profile)
        second.write_text(yaml.safe_dump(profile))
        return first, second, shared, profile, digest

    def test_second_grasp_changes_only_three_targets_and_preserves_shared_digest(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second, shared, profile, digest = self.files(directory)
            original = first.read_bytes()
            chosen, actual_digest = load_task_waypoints(first, second)
            self.assertEqual(actual_digest, digest)
            for name in ('Home', 'Preplace', 'Place', 'Retreat'):
                self.assertEqual(chosen['waypoints'][name], shared['waypoints'][name])
            self.assertEqual(chosen['waypoints']['Grasp'], profile['waypoints']['Grasp'])
            self.assertNotEqual(chosen['waypoints']['Grasp'], shared['waypoints']['Grasp'])
            grasp = np.asarray(chosen['waypoints']['Grasp']['tcp_base'])
            for name in ('Pregrasp', 'Lift'):
                target = np.asarray(chosen['waypoints'][name]['tcp_base'])
                np.testing.assert_array_equal(target[:3, :3], grasp[:3, :3])
                np.testing.assert_allclose(target[:3, 3]-grasp[:3, 3], [0., 0., .15])
            self.assertEqual(first.read_bytes(), original)
            original_choice, _ = load_task_waypoints(first)
            self.assertEqual(original_choice, shared)

    def test_missing_second_grasp_never_falls_back_to_first(self):
        with tempfile.TemporaryDirectory() as directory:
            first, second, *_ = self.files(directory)
            second.unlink()
            with self.assertRaisesRegex(ValueError, 'not taught'):
                load_task_waypoints(first, second)

    def test_shared_change_model_mismatch_or_place_override_rejected(self):
        for change in ('shared', 'identity', 'Place', 'derived', 'height', 'measured'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                first, second, shared, profile, _ = self.files(directory)
                if change == 'shared':
                    shared['waypoints']['Place']['tcp_base'][2][3] = -.1
                    first.write_text(yaml.safe_dump(shared))
                elif change == 'identity':
                    profile['identity'] = {'model': 'different'}
                elif change == 'Place':
                    profile['waypoints']['Place'] = shared['waypoints']['Place']
                elif change == 'derived':
                    profile['waypoints']['Lift']['tcp_base'][0][3] += .01
                elif change == 'height':
                    profile = derive_from_grasp(profile, .2)
                else:
                    profile['waypoints']['Grasp']['derived_from'] = 'Home'
                second.write_text(yaml.safe_dump(profile))
                with self.assertRaises(ValueError):
                    load_task_waypoints(first, second)

    def test_two_actions_keep_legacy_first_command(self):
        self.assertEqual(action_name(1), '/fixed_grasp/task')
        self.assertEqual(action_name(2), '/fixed_grasp/task_2')
        for value in (0, 3, '2', None):
            with self.assertRaises(ValueError): action_name(value)


if __name__ == '__main__':
    unittest.main()
