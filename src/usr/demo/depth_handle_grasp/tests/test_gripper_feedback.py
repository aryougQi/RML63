"""No hardware: old open feedback must not be classified as empty closure."""
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from grasp_ros import GripperIO


def state(mode,position):
    return dict(status=1,enable=1,error=0,mode=mode,actpos=position)


class GripperFeedbackTests(unittest.TestCase):
    def io(self,states):
        io=GripperIO.__new__(GripperIO)
        io.config=dict(gripper_timeout_sec=12,gripper_speed=100,gripper_force=150)
        io.pick_pub=MagicMock();io._connected=MagicMock()
        io.query=MagicMock(side_effect=states)
        return io

    @patch('grasp_ros.rospy.sleep')
    def test_old_open_mode_two_waits_for_stable_contact(self,sleep):
        io=self.io([state(2,633),state(2,633),state(1,400),state(6,237),state(6,235),state(6,235)])
        result=io.close()
        self.assertEqual(result['actpos'],235)
        self.assertEqual(len(result['stable_feedback_samples']),3)
        self.assertEqual(io.pick_pub.publish.call_count,1)
        self.assertEqual(io.query.call_count,6)

    @patch('grasp_ros.rospy.sleep')
    def test_true_empty_closure_requires_low_opening_and_three_samples(self,sleep):
        io=self.io([state(2,633),state(1,100),state(2,2),state(2,1),state(2,0)])
        with self.assertRaisesRegex(RuntimeError,'fully closed') as context:io.close()
        self.assertEqual(len(context.exception.gripper_feedback),5)
        self.assertEqual(io.pick_pub.publish.call_count,1)

    @patch('grasp_ros.rospy.sleep')
    def test_transient_contact_must_settle(self,sleep):
        io=self.io([state(6,100),state(6,200),state(6,235),state(6,235),state(6,235)])
        self.assertEqual(io.close()['actpos'],235)
        self.assertEqual(io.query.call_count,5)

    def test_fault_stops_without_another_command(self):
        io=self.io([dict(state(1,500),error=1)])
        with self.assertRaisesRegex(RuntimeError,'gripper error'):io.close()
        self.assertEqual(io.pick_pub.publish.call_count,1)

if __name__=='__main__':unittest.main()
