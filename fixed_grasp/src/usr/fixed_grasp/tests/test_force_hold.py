"""Fixed grasp contact uses force feedback, independent of aperture."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import backend


class ForceHoldTests(unittest.TestCase):
    def test_continuous_grip_uses_configured_force(self):
        config = backend.load_settings(Path(__file__).resolve().parents[1] / 'config/fixed.yaml')
        with patch.object(backend.rospy, 'Publisher') as publisher:
            gripper = backend.Gripper(config, Mock())
        self.assertEqual([call.args[0] for call in publisher.call_args_list],
                         ['/rm_driver/Gripper_Set', '/rm_driver/Gripper_Pick_On'])
        gripper._connected = Mock()
        gripper.query = Mock(return_value=self.state())
        with patch.object(backend.rospy, 'sleep'):
            gripper.close()
        command = gripper.pick_pub.publish.call_args.args[0]
        self.assertEqual(command.force, config['gripper_force'])
        self.assertEqual(command.speed, config['gripper_speed'])

    def test_grip_force_override_validated_before_use(self):
        for value in (0, 1001, -1, 500.5, True, '500', None, float('nan')):
            with self.subTest(value=value), tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as config:
                yaml.safe_dump(dict(gripper_force=value), config)
                config.flush()
                with self.assertRaisesRegex(ValueError, 'gripper_force'):
                    backend.load_settings(config.name)
        for value in (1, 500, 1000):
            with self.subTest(value=value), tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as config:
                yaml.safe_dump(dict(gripper_force=value), config)
                config.flush()
                self.assertEqual(backend.load_settings(config.name)['gripper_force'], value)

    def state(self, **updates):
        state = dict(status=1, enable=1, error=0, mode=6, current_force=478)
        state.update(updates)
        return state

    def test_aperture_is_not_required_or_restricted(self):
        for extra in ({}, {'actpos': 0}, {'actpos': 558}, {'actpos': 1000}):
            self.assertTrue(backend.force_contact(self.state(**extra), 50))

    def test_invalid_force_and_status_are_rejected(self):
        for extra in ({'current_force': None}, {'current_force': float('nan')},
                      {'current_force': float('inf')}, {'current_force': 49},
                      {'current_force': 0}, {'status': 0}, {'enable': 0},
                      {'error': 1}, {'mode': 2}):
            self.assertFalse(backend.force_contact(self.state(**extra), 50))

    def test_loaded_regrip_is_only_accepted_after_initial_contact(self):
        state = dict(actpos=550, command='get_gripper_state', current_force=964,
                     enable=1, error=0, mode=4, status=1, temperature=32)
        self.assertFalse(backend.force_contact(state, 50))
        self.assertTrue(backend.force_contact(state, 50, allow_regrip=True))

    def test_regrip_keeps_force_and_hardware_gates(self):
        for mode in (4, 6):
            for extra in ({'current_force': 49}, {'current_force': 0},
                          {'current_force': None}, {'current_force': float('nan')},
                          {'current_force': float('inf')}, {'enable': 0},
                          {'status': 0}, {'error': 1}):
                with self.subTest(mode=mode, extra=extra):
                    self.assertFalse(backend.force_contact(self.state(mode=mode, **extra),
                                                           50, allow_regrip=True))
        for mode in (None, 0, 1, 2, 3, 5, 7):
            with self.subTest(mode=mode):
                self.assertFalse(backend.force_contact(self.state(mode=mode, current_force=964),
                                                       50, allow_regrip=True))

    def test_close_requires_three_consecutive_force_samples(self):
        gripper = backend.Gripper.__new__(backend.Gripper)
        gripper.config = dict(gripper_speed=100, gripper_force=200,
                              gripper_timeout_sec=12, gripper_min_hold_force=50)
        gripper._connected = Mock()
        gripper.pick_pub = Mock()
        gripper.query = Mock(side_effect=[self.state(), self.state(current_force=0),
                                         self.state(mode=4, current_force=964),
                                         self.state(actpos=0), self.state(actpos=558), self.state()])
        with patch.object(backend.rospy, 'sleep'):
            gripper.close()
        self.assertEqual(gripper.query.call_count, 6)
        gripper.pick_pub.publish.assert_called_once()

    def test_hold_checks_force_without_aperture_stability(self):
        b = backend.Backend.__new__(backend.Backend)
        b.config = dict(gripper_min_hold_force=50)
        b.gripper = Mock()
        b.gripper.query.side_effect = [self.state(actpos=0), self.state(actpos=558), self.state()]
        with patch.object(backend.time, 'sleep'):
            b.verify_hold()
        b.gripper.query.side_effect = None
        b.gripper.query.return_value = self.state(current_force=0)
        with self.assertRaisesRegex(RuntimeError, 'force contact lost'):
            b.quick_hold()

    def test_hold_accepts_force_stop_regrip_transition_and_rejects_loss(self):
        b = backend.Backend.__new__(backend.Backend)
        b.config = dict(gripper_min_hold_force=50)
        b.gripper = Mock()
        b.gripper.query.side_effect = [self.state(), self.state(mode=4, actpos=550, current_force=964),
                                      self.state()]
        with patch.object(backend.time, 'sleep'):
            b.verify_hold()
        self.assertEqual(b.gripper.query.call_count, 3)
        b.gripper.query.side_effect = None
        for extra in ({'mode': 4, 'current_force': 49}, {'mode': 5, 'current_force': 964},
                      {'mode': 4, 'error': 1}, {'mode': 4, 'status': 0}, {'mode': 4, 'enable': 0}):
            with self.subTest(extra=extra):
                b.gripper.query.return_value = self.state(**extra)
                with self.assertRaisesRegex(RuntimeError, 'force contact lost'):
                    b.quick_hold()


if __name__ == '__main__':
    unittest.main()
