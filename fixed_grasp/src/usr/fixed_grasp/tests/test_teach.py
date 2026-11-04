"""Interactive teaching with simulated input/feedback; no ROS node or hardware."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import teach
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source workspace and built fixed_grasp local_setup.bash')
class TeachingTests(unittest.TestCase):
    def test_confirmation_before_read_and_early_exit_keeps_save(self):
        data = dict(schema=1, identity={}, waypoints={})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'poses.yaml'
            capture = Mock(return_value=dict(joints_rad=[0.]*6))
            def record(name):
                teach.save_pose(path, data, name, capture())
            calls = []
            def answer(prompt):
                calls.append(prompt)
                if len(calls) == 1:
                    capture.assert_not_called()
                    return ''
                self.assertIn('Home', yaml.safe_load(path.read_text())['waypoints'])
                return 'q'
            teach.interactive(['Home', 'Pregrasp'], data, record, answer, lambda text: None)
            capture.assert_called_once()
            self.assertEqual(list(data['waypoints']), ['Home'])

    def test_lift_prompt_has_no_height_cap(self):
        messages = []
        teach.interactive(['Lift'], dict(waypoints={}), Mock(), lambda prompt: 'q',
                          messages.append)
        self.assertTrue(any('Lift' in text and '所需位置' in text for text in messages))
        self.assertFalse(any('上限' in text for text in messages))

    def test_failed_capture_retries_same_position(self):
        data = dict(waypoints={})
        names = []
        def record(name):
            names.append(name)
            if len(names) == 1: raise RuntimeError('moving')
            data['waypoints'][name] = {}
        answers = iter(['', ''])
        teach.interactive(['Home'], data, record, lambda prompt: next(answers), lambda text: None)
        self.assertEqual(names, ['Home', 'Home'])

    def test_existing_record_kept_until_explicit_r(self):
        old = dict(joints_rad=[1.]*6)
        data = dict(waypoints=dict(Home=old))
        record = Mock()
        teach.interactive(['Home'], data, record, lambda prompt: '', lambda text: None)
        record.assert_not_called()
        self.assertEqual(data['waypoints']['Home'], old)
        teach.interactive(['Home'], data, record, lambda prompt: 'r', lambda text: None)
        record.assert_called_once_with('Home')

    def test_skip_and_eof_do_not_capture(self):
        data = dict(waypoints={})
        record = Mock()
        answers = iter(['s'])
        def answer(prompt):
            try: return next(answers)
            except StopIteration: raise EOFError()
        teach.interactive(['Home', 'Grasp'], data, record, answer, lambda text: None)
        record.assert_not_called()
        self.assertEqual(data['waypoints'], {})

    def test_failed_atomic_save_preserves_previous_data(self):
        data = dict(schema=1, identity={}, waypoints=dict(Home={'old': True}))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'poses.yaml'
            path.write_text(yaml.safe_dump(data))
            previous = path.read_text()
            with patch.object(teach.os, 'replace', side_effect=OSError('write failed')):
                with self.assertRaises(OSError): teach.save_pose(path, data, 'Home', {'old': False})
            self.assertEqual(path.read_text(), previous)
            self.assertEqual(data['waypoints']['Home'], {'old': True})

    def test_second_capture_saves_actual_grasp_and_derives_both_offsets_atomically(self):
        from grasp_profiles import new_grasp_profile
        record = dict(joints_rad=[.1]*6, tcp_base=np.eye(4).tolist(), stationary_samples=5)
        data = new_grasp_profile({'identity': {'model': 'same'}}, 'shared-digest')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints_2.yaml'
            teach.save_second_grasp(path, data, record)
            saved = yaml.safe_load(path.read_text())
            self.assertEqual(saved['waypoints']['Grasp'], record)
            self.assertEqual(set(saved['waypoints']), {'Grasp', 'Pregrasp', 'Lift'})
            self.assertEqual(saved['shared_waypoints_sha256'], 'shared-digest')
            for name in ('Pregrasp', 'Lift'):
                self.assertEqual(saved['waypoints'][name]['offset_base_m'], [0., 0., .15])
                self.assertNotIn('joints_rad', saved['waypoints'][name])
            previous = path.read_bytes()
            with patch.object(teach.os, 'replace', side_effect=OSError('write failed')):
                with self.assertRaises(OSError): teach.save_second_grasp(path, data, dict(record, joints_rad=[.2]*6))
            self.assertEqual(path.read_bytes(), previous)
            self.assertEqual(data, saved)

    def test_first_grasp_capture_derives_offsets_and_preserves_all_shared_targets(self):
        shared = {name: {'unchanged': name} for name in ('Home', 'Preplace', 'Place', 'Retreat')}
        data = dict(schema=1, identity={'model': 'same'}, waypoints=dict(shared))
        pose = np.eye(4); pose[:3, 3] = [.2, -.1, .05]
        record = dict(joints_rad=[.1]*6, tcp_base=pose.tolist())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.yaml'
            teach.save_first_grasp(path, data, record)
            saved = yaml.safe_load(path.read_text())
        self.assertEqual(saved['waypoints']['Grasp'], record)
        for name, target in shared.items():
            self.assertEqual(saved['waypoints'][name], target)
        for name in ('Pregrasp', 'Lift'):
            derived = np.array(saved['waypoints'][name]['tcp_base'])
            np.testing.assert_allclose(derived[:3, 3]-pose[:3, 3], [0., 0., .15])
            np.testing.assert_array_equal(derived[:3, :3], pose[:3, :3])
            self.assertNotIn('joints_rad', saved['waypoints'][name])

    def test_second_teaching_command_preserves_primary_file(self):
        shared = dict(schema=1, identity={'model': 'same'}, waypoints={'Home': {'unchanged': True}})
        record = dict(joints_rad=[.1]*6, tcp_base=np.eye(4).tolist())
        geom = Mock(); geom.at_width.return_value = {'flange_tcp': np.eye(4)}
        with tempfile.TemporaryDirectory() as directory:
            first = Path(directory)/'waypoints.yaml'; first.write_text(yaml.safe_dump(shared))
            original = first.read_bytes()
            argv = ['teach.py', 'Grasp', '--grasp', '2', '--interactive', '--shared-waypoints', str(first)]
            with patch.object(teach.rospy, 'myargv', return_value=argv), patch.object(teach.rospy, 'init_node'), \
                    patch.object(teach.moveit_commander, 'roscpp_initialize'), patch.object(teach, 'Session'), \
                    patch.object(teach, 'geometry', return_value=geom), patch.object(teach, 'identity', return_value=shared['identity']), \
                    patch.object(teach, 'capture', return_value=record), patch('builtins.print'), \
                    patch.object(teach, 'interactive', side_effect=lambda names, data, capture: capture('Grasp')):
                teach.main()
            self.assertEqual(first.read_bytes(), original)
            selected = yaml.safe_load((first.parent/'waypoints_2.yaml').read_text())
            self.assertEqual(selected['waypoints']['Grasp'], record)
            self.assertEqual(selected['waypoints']['Lift']['offset_base_m'], [0., 0., .15])

    def session(self, moving=False, duplicate=False):
        session = Mock()
        values = []
        for i in range(5):
            q = np.ones(6)*(.01*i if moving else 0.)
            stamp = Mock(); stamp.to_nsec.return_value = 1 if duplicate else i+1
            values.append((q, stamp))
        session.measured.side_effect = values
        return session

    def test_unstable_or_duplicate_samples_rejected(self):
        for moving, duplicate in [(True, False), (False, True)]:
            session = self.session(moving, duplicate)
            with patch.object(teach.time, 'sleep'):
                with self.assertRaisesRegex(RuntimeError, 'stationary'):
                    teach.capture(session, dict(max_capture_motion_deg=.1), np.eye(4))
            session.chain.fk.assert_not_called()

    def test_stationary_samples_keep_actual_joints(self):
        session = self.session()
        session.chain.fk.return_value = np.eye(4)
        session.offsets = np.zeros(6)
        with patch.object(teach.time, 'sleep'):
            value = teach.capture(session, dict(max_capture_motion_deg=.1), np.eye(4))
        session.verify_hardware.assert_called_once_with(require_execution=False)
        self.assertEqual(value['stationary_samples'], 5)
        self.assertEqual(value['joint_stamp_ns'], 5)
        self.assertEqual(value['joints_rad'], [0.]*6)


if __name__ == '__main__': unittest.main()
