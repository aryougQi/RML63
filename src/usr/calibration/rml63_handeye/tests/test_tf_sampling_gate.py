"""Exercise the camera latency/repeated-frame guards without connecting to a robot."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import rospy
from geometry_msgs.msg import TransformStamped

spec = importlib.util.spec_from_file_location('auto_calibrate_under_test',
    Path(__file__).resolve().parents[1] / 'scripts/auto_calibrate.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def transform(stamp):
    result = TransformStamped()
    result.header.stamp = rospy.Time.from_sec(stamp)
    result.transform.rotation.w = 1.0
    return result


class SamplingGateTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.runner = module.AutomaticCalibration.__new__(module.AutomaticCalibration)
        self.runner.tf_max_age = .8
        self.runner.tf_wait_timeout = .12
        self.runner.settle_timeout = 3.0
        self.runner.marker = 'marker'
        self.patches = [patch.object(module.time, 'monotonic', lambda: self.now),
                        patch.object(module.rospy.Time, 'now', lambda: rospy.Time.from_sec(self.now)),
                        patch.object(module.rospy, 'sleep', self.sleep),
                        patch.object(module.rospy, 'is_shutdown', lambda: False)]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)

    def sleep(self, seconds):
        self.now += seconds

    def buffer(self, callback):
        class Buffer:
            def lookup_transform(_, parent, child, *args):
                return callback(parent, child)
        self.runner.buffer = Buffer()

    def test_accepts_measured_point_six_second_latency(self):
        self.buffer(lambda p, c: transform(self.now-.6))
        result = self.runner.fresh_tf('optical', 'marker')
        self.assertAlmostEqual(result.header.stamp.to_sec(), 99.4)

    def test_stale_frame_is_retried_until_new_frame_arrives(self):
        stamps = iter([99.0, 99.5])
        self.buffer(lambda p, c: transform(next(stamps)))
        result = self.runner.fresh_tf('optical', 'marker')
        self.assertEqual(result.header.stamp.to_sec(), 99.5)
        self.assertGreater(self.now, 100)

    def test_frozen_frame_is_rejected_even_if_younger_than_age_limit(self):
        self.buffer(lambda p, c: transform(99.5))
        with self.assertRaisesRegex(RuntimeError, 'Timed out'):
            self.runner.fresh_tf('optical', 'marker', rospy.Time.from_sec(99.5))

    def test_genuinely_stale_frames_still_fail(self):
        self.buffer(lambda p, c: transform(90.0))
        with self.assertRaisesRegex(RuntimeError, 'Timed out'):
            self.runner.fresh_tf('optical', 'marker')

    def test_settling_waits_for_exposures_after_motion_and_distinct_frames(self):
        self.runner.tf_wait_timeout = 1.0
        camera_stamps = []
        def callback(parent, child):
            stamp = self.now-(.6 if child == 'marker' else .005)
            if child == 'marker':
                camera_stamps.append(stamp)
            return transform(stamp)
        self.buffer(callback)
        self.runner.stable()
        self.assertGreaterEqual(self.now, 101.5)
        self.assertGreater(camera_stamps[-1], 100)


if __name__ == '__main__':
    unittest.main()
