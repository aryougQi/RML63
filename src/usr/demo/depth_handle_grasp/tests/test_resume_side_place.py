"""Continuation only accepts the exact known supported-contact checkpoint."""
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resume_side_place import validate_record,read_contact,load_trajectory


class ResumeSidePlaceTests(unittest.TestCase):
    def test_only_known_failure_with_operator_confirmation(self):
        status=dict(error='gripper fully closed without detecting an object')
        feedback=[dict(stage='pregrasp'),dict(stage='descent')]
        confirmation=dict(gripped_handle_confirmed=True,basket_supported=True)
        plans={name:{} for name in ('pregrasp','descent','lift','transfer','place','retreat')}
        validate_record(status,feedback,confirmation,plans)
        for bad in (dict(error='collision'),dict(error='timeout')):
            with self.assertRaises(RuntimeError):validate_record(bad,feedback,confirmation,plans)
        with self.assertRaises(RuntimeError):validate_record(status,feedback,{},plans)
        with self.assertRaises(RuntimeError):validate_record(status,feedback+[dict(stage='lift')],confirmation,plans)
        with self.assertRaises(RuntimeError):validate_record(status,feedback,confirmation,{'lift':{}})

    @patch('resume_side_place.rospy.sleep')
    def test_read_only_contact_verification_sends_no_commands(self,sleep):
        hardware=MagicMock()
        hardware.query.return_value=dict(mode=6,actpos=235,error=0,status=1,enable=1)
        self.assertEqual(read_contact(hardware,{'contact_width_m':.016})['actpos'],235)
        self.assertEqual(hardware.query.call_count,3)
        hardware.open.assert_not_called();hardware.close.assert_not_called()

    @patch('resume_side_place.rospy.sleep')
    def test_lost_contact_aborts_without_release(self,sleep):
        hardware=MagicMock();hardware.query.return_value=dict(mode=2,actpos=0,error=0,status=1,enable=1)
        with self.assertRaises(RuntimeError):read_contact(hardware,{'contact_width_m':.016})
        hardware.open.assert_not_called();hardware.close.assert_not_called()

    def test_saved_timing_and_joint_data_restored(self):
        saved=dict(joint_names=['joint1'],points=[dict(positions=[.1],velocities=[.02],accelerations=[0.],time_sec=1.25)])
        result=load_trajectory(saved)
        self.assertEqual(result.joint_trajectory.joint_names,['joint1'])
        self.assertAlmostEqual(result.joint_trajectory.points[0].time_from_start.to_sec(),1.25)

if __name__=='__main__':unittest.main()
