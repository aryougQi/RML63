"""ROS message decoding tests, no camera/robot/master required."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ros_depth import decode_depth, stationary_evidence

class DepthTransport(unittest.TestCase):
    def fixture(self):
        image=Image();image.width=2;image.height=2;image.step=6
        image.encoding='16UC1';image.header.frame_id='camera_depth_optical_frame'
        image.header.stamp=rospy.Time(10,20);image.header.seq=123
        image.data=np.array([0,2000,999,3200,4000,999],dtype='<u2').tobytes()
        info=CameraInfo();info.header=deepcopy(image.header);info.width=2;info.height=2
        info.K=[100.,0,1.,0,100.,1.,0,0,1.];info.D=[0.]*5
        meta=dict(ros_frame_id=image.header.frame_id,ros_stamp_ns=image.header.stamp.to_nsec(),ros_header_seq=123,
            intrinsics=dict(width=2,height=2,fx=100.,fy=100.,ppx=1.,ppy=1.,coefficients=[0.]*5),depth_scale_m_per_unit=.00025)
        return image,meta,info

    def test_native_units_and_stride(self):
        image,meta,info=self.fixture();raw=decode_depth(image,meta,info)
        np.testing.assert_array_equal(raw,[[0,2000],[3200,4000]])
        self.assertAlmostEqual(raw[0,1]*meta['depth_scale_m_per_unit'],.5)
        self.assertEqual(raw.dtype,np.uint16)

    def test_big_endian(self):
        image,meta,info=self.fixture();image.is_bigendian=True
        image.data=np.array([0,2000,999,3200,4000,999],dtype='>u2').tobytes()
        np.testing.assert_array_equal(decode_depth(image,meta,info),[[0,2000],[3200,4000]])

    def test_invalid_encoding(self):
        image,meta,info=self.fixture();image.encoding='mono8'
        with self.assertRaises(ValueError):decode_depth(image,meta,info)

    def test_wrong_frame_metadata(self):
        image,meta,info=self.fixture();meta['ros_stamp_ns']+=1
        with self.assertRaises(ValueError):decode_depth(image,meta,info)

    def test_wrong_camera_intrinsics(self):
        image,meta,info=self.fixture();info.K[0]=200
        with self.assertRaises(ValueError):decode_depth(image,meta,info)

    def test_bad_scale_and_truncated_data(self):
        image,meta,info=self.fixture();meta['depth_scale_m_per_unit']=float('nan')
        with self.assertRaises(ValueError):decode_depth(image,meta,info)
        meta['depth_scale_m_per_unit']=.00025;image.data=image.data[:-1]
        with self.assertRaises(ValueError):decode_depth(image,meta,info)

class StationaryAcquisition(unittest.TestCase):
    def fixture(self):
        samples=[(int(i*1e8),np.zeros(6)) for i in range(5)]
        frames=[(None,dict(ros_stamp_ns=int(i*1e8+5e7))) for i in range(4)]
        config=dict(max_capture_motion_deg=.1,joint_feedback_max_age_sec=.3)
        return samples,frames,config

    def test_static_sequence(self):
        samples,frames,config=self.fixture()
        self.assertEqual(stationary_evidence(samples,frames,config)['sample_count'],5)

    def test_motion_returning_to_same_endpoints_is_rejected(self):
        samples,frames,config=self.fixture();samples[2][1][0]=np.radians(.2)
        with self.assertRaises(RuntimeError):stationary_evidence(samples,frames,config)

    def test_feedback_gap_rejected(self):
        samples,frames,config=self.fixture();samples=[samples[0],samples[-1]]
        with self.assertRaises(RuntimeError):stationary_evidence(samples,frames,config)

    def test_missing_final_bracket_rejected(self):
        samples,frames,config=self.fixture()
        with self.assertRaises(RuntimeError):stationary_evidence(samples[:-1],frames,config)

if __name__=='__main__':unittest.main()
