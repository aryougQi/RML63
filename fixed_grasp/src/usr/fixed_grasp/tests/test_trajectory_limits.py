"""Long fixed-task paths keep real joint bounds, collisions and timing checks."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    import rospy
    from core import Cancelled
    from moveit_msgs.msg import RobotState, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class TrajectoryLimitsTests(unittest.TestCase):
    def session(self):
        session = backend.Session.__new__(backend.Session)
        session.config = dict(max_joint_travel_deg=90, max_joint_jump_deg=5)
        session.active = ['joint%d' % i for i in range(1, 7)]
        session.offsets = np.zeros(6)
        session.robot = Mock()
        session.robot.get_joint.return_value.bounds.return_value = (-np.pi, np.pi)
        session.cancel_check = Mock()
        return session

    def path(self, angle=135):
        session = self.session()
        state = RobotState()
        state.joint_state.name = session.active
        state.joint_state.position = [0.]*6
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = session.active
        count = max(2, int(np.ceil(abs(angle)/2))+1)
        for i, value in enumerate(np.linspace(0., np.radians(angle), count)):
            positions = [0.]*6; positions[4] = float(value)
            trajectory.joint_trajectory.points.append(JointTrajectoryPoint(
                positions=positions, time_from_start=rospy.Duration(.1*i)))
        return session, state, trajectory

    def test_joint5_135_degree_path_passes_and_every_point_is_collision_checked(self):
        session, state, trajectory = self.path()
        with patch.object(backend.RobotSession, 'check_state') as collision:
            positions = session.check_trajectory(trajectory, state)
        self.assertAlmostEqual(positions[-1, 4], np.radians(135))
        self.assertEqual(collision.call_count, len(trajectory.joint_trajectory.points))
        # The shared visual pipeline still has its own original travel cap.
        with self.assertRaisesRegex(RuntimeError, 'trajectory joint travel too large'):
            backend.RobotSession.check_trajectory(session, trajectory, state)
        self.assertEqual(session.config['max_joint_travel_deg'], 90)

    def test_real_joint_bounds_and_calibration_offset_still_stop_long_paths(self):
        for angle, offset in ((190, 0.), (175, .2)):
            with self.subTest(angle=angle, offset=offset):
                session, state, trajectory = self.path(angle)
                session.offsets[4] = offset
                with patch.object(backend.RobotSession, 'check_state'):
                    with self.assertRaisesRegex(RuntimeError, 'joint limit: joint5'):
                        session.check_trajectory(trajectory, state)

    def test_collision_after_90_degrees_still_rejects_trajectory(self):
        session, state, trajectory = self.path()
        def collision(q):
            if q[4] > np.radians(100):
                raise RuntimeError('state collision')
        with patch.object(backend.RobotSession, 'check_state', side_effect=collision):
            with self.assertRaisesRegex(RuntimeError, 'state collision'):
                session.check_trajectory(trajectory, state)

    def test_start_mismatch_nonfinite_jump_and_bad_timing_are_still_rejected(self):
        for failure, message in (('start', 'start mismatch'), ('nonfinite', 'invalid joint'),
                                 ('jump', 'joint jump'), ('time', 'timing invalid')):
            with self.subTest(failure=failure):
                session, state, trajectory = self.path()
                points = trajectory.joint_trajectory.points
                if failure == 'start': points[0].positions[0] = .1
                elif failure == 'nonfinite': points[-1].positions[4] = float('nan')
                elif failure == 'jump': points[1].positions[4] = .5
                else: points[-1].time_from_start = points[-2].time_from_start
                with self.assertRaisesRegex(RuntimeError, message):
                    session.check_trajectory(trajectory, state)

    def test_cancel_stops_validation_between_points(self):
        session, state, trajectory = self.path()
        session.cancel_check.side_effect = [None]*5+[Cancelled('cancelled')]
        with patch.object(backend.RobotSession, 'check_state') as collision:
            with self.assertRaises(Cancelled): session.check_trajectory(trajectory, state)
        self.assertEqual(collision.call_count, 5)


if __name__ == '__main__': unittest.main()
