from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest
import rospy
from sensor_msgs.msg import Image,CameraInfo
from realsense2_camera.msg import Metadata
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from realsense_source import normalize, validate_device

class OriginalCameraAdapter(unittest.TestCase):
    def test_sanitized_and_sdk_model_names(self):
        for name in ('intel_realsense_l515','Intel RealSense L515'):
            validate_device(name,'f1380095','f1380095')

    def test_wrong_serial_is_still_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'serial mismatch'):
            validate_device('intel_realsense_l515','another','f1380095')

    def test_wrong_model_is_still_rejected(self):
        for name in ('intel_realsense_d435','intel_realsense_l5150'):
            with self.assertRaisesRegex(RuntimeError,'model mismatch'):
                validate_device(name,'f1380095','f1380095')

    def fixture(self):
        image=Image();image.header.stamp=rospy.Time(12,30);image.header.frame_id='camera_depth_optical_frame'
        image.width=320;image.height=240
        info=CameraInfo();info.header=deepcopy(image.header);info.width=320;info.height=240
        info.K=[200.,0,160.,0,200.,120.,0,0,1.];info.R=[1.,0,0,0,1.,0,0,0,1.];info.D=[0.]*5
        meta=Metadata();meta.header=deepcopy(image.header)
        meta.json_data=json.dumps(dict(frame_number=15,frame_timestamp=12345.,clock_domain='global_time',time_of_arrival=12000.))
        context=dict(expected_frame=image.header.frame_id,serial='TEST',device='L515',usb_type='synthetic',stream_id='test-node',
            depth_to_color=dict(rotation_matrix=[[1,0,0],[0,1,0],[0,0,1]],translation_m=[.01,0,0]))
        return image,info,meta,context

    def test_ros_millimetres_are_not_sdk_units(self):
        image,info,meta,context=self.fixture();result=normalize(image,info,meta,context)
        self.assertEqual(result['depth_scale_m_per_unit'],.001)
        self.assertEqual(result['frame_number'],15)
        self.assertEqual(result['timestamp_ms'],12345.)
        self.assertEqual(result['depth_to_color'],context['depth_to_color'])

    def test_aligned_depth_rejected(self):
        image,info,meta,context=self.fixture();image.header.frame_id='camera_color_optical_frame'
        with self.assertRaises(ValueError):normalize(image,info,meta,context)

    def test_wrong_metadata_timestamp_rejected(self):
        image,info,meta,context=self.fixture();meta.header.stamp=rospy.Time(11)
        with self.assertRaises(ValueError):normalize(image,info,meta,context)

    def test_nonzero_distortion_rejected(self):
        image,info,meta,context=self.fixture();info.D[0]=.1
        with self.assertRaises(ValueError):normalize(image,info,meta,context)

    def test_unsynchronized_hardware_clock_rejected(self):
        image,info,meta,context=self.fixture()
        data=json.loads(meta.json_data);data['clock_domain']='hardware_clock'
        meta.json_data=json.dumps(data)
        with self.assertRaisesRegex(ValueError,'clock is not synchronized'):
            normalize(image,info,meta,context)

    def test_missing_hardware_frame_number_rejected(self):
        image,info,meta,context=self.fixture();meta.json_data=json.dumps(dict(clock_domain='global_time'))
        with self.assertRaises(KeyError):normalize(image,info,meta,context)

if __name__=='__main__':unittest.main()
