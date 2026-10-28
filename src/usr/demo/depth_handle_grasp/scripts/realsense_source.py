"""Read-only adapter for installed ROS1 realsense2_camera 2.3.2 messages.

That driver rescales SDK Z16 into uint16 millimetres in fix_depth_scale().
Never attach the SDK's native scale to an image published by this driver.
"""
import json
import re
import numpy as np
import rospy
from realsense2_camera.msg import Extrinsics
from realsense2_camera.srv import DeviceInfo

TIMESTAMP_BASIS='librealsense global_time synchronized to host clock; stationary acquisition only'


def validate_device(name,serial,expected_serial):
    if serial!=expected_serial:
        raise RuntimeError('RealSense serial mismatch: expected %r, received %r (model %r)'%
                           (expected_serial,serial,name))
    # ROS1 driver sanitizes device names to lower-case graph resource names.
    if 'l515' not in re.split(r'[^a-z0-9]+',name.lower()):
        raise RuntimeError('RealSense model mismatch: expected L515, received %r (serial %r)'%(name,serial))


def camera_context(root,config,master,owner):
    service=root+'/realsense2_camera/device_info'
    rospy.wait_for_service(service,timeout=3.)
    device=rospy.ServiceProxy(service,DeviceInfo)()
    validate_device(device.device_name,device.serial_number,config['camera_serial'])
    topic=root+'/extrinsics/depth_to_color'
    pubs=dict(master.getSystemState()[0])
    if pubs.get(topic)!=[owner]:raise RuntimeError('depth-to-color extrinsics missing or from another publisher')
    extrinsic=rospy.wait_for_message(topic,Extrinsics,timeout=3.)
    rotation=np.array(extrinsic.rotation,dtype=float).reshape(3,3,order='F')
    translation=np.array(extrinsic.translation,dtype=float)
    if (not np.isfinite(rotation).all() or not np.isfinite(translation).all()
        or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-5)
        or abs(np.linalg.det(rotation)-1)>1e-5 or np.linalg.norm(translation)>.2):
        raise RuntimeError('invalid factory depth-to-color extrinsics')
    filters=rospy.get_param(root+'/realsense2_camera/filters','')
    if filters not in ('',None):raise RuntimeError('disable depth filters for the validated native-depth detector')
    return dict(serial=device.serial_number,device=device.device_name,usb_type=device.usb_type_descriptor,
        stream_id=master.lookupNode(owner),owner=owner,
        expected_frame=config.get('ros_depth_optical_frame','camera_depth_optical_frame'),
        depth_to_color=dict(rotation_matrix=rotation.tolist(),translation_m=translation.tolist(),
            convention='P_color_optical = R @ P_depth_optical + t',
            source='realsense2_camera/extrinsics/depth_to_color; SDK column-major array'))


def normalize(message,camera_info,metadata_message,context):
    if message.header.frame_id!=context['expected_frame']:
        raise ValueError('expected native depth optical grid; aligned/color depth is not accepted')
    if (metadata_message.header.stamp!=message.header.stamp
        or metadata_message.header.frame_id!=message.header.frame_id):
        raise ValueError('RealSense Metadata belongs to another depth frame')
    data=json.loads(metadata_message.json_data)
    if data.get('clock_domain')!='global_time':
        raise ValueError('L515 clock is not synchronized: enable global_time_enabled on /camera/l500_depth_sensor (use configure_camera_clock.py)')
    number=data['frame_number'];timestamp=data['frame_timestamp']
    arrival=data['time_of_arrival']
    if not isinstance(arrival,(int,float)) or not np.isfinite(arrival):
        raise ValueError('invalid RealSense host arrival timestamp')
    if (isinstance(number,bool) or not isinstance(number,int) or number<0
        or not isinstance(timestamp,(int,float)) or not np.isfinite(timestamp)):
        raise ValueError('invalid RealSense hardware frame identity')
    if not np.allclose(camera_info.R,np.eye(3).ravel(),atol=1e-8):
        raise ValueError('unsupported rectification for the native L515 depth grid')
    # L515 depth currently has zero distortion. Do not invent an SDK distortion
    # enum from a ROS model if a future camera/profile reports nonzero distortion.
    if not np.allclose(camera_info.D,0,atol=1e-12):
        raise ValueError('nonzero ROS depth distortion requires a dedicated projection adapter')
    k=camera_info.K
    if not np.isfinite(k).all() or min(k[0],k[4])<=0:
        raise ValueError('invalid ROS camera calibration')
    intr=dict(width=camera_info.width,height=camera_info.height,fx=k[0],fy=k[4],ppx=k[2],ppy=k[5],
              coefficients=list(camera_info.D),distortion_model_id=0)
    return dict(intrinsics=intr,depth_scale_m_per_unit=.001,
        depth_scale_source='realsense2_camera 2.3.2 fix_depth_scale(): uint16 millimetres',
        depth_storage='unmodified ROS 16UC1 values; SDK sub-millimetre quantization may already be lost in driver',
        frame_number=number,timestamp_ms=float(timestamp),timestamp_domain=data['clock_domain'],
        host_arrival_timestamp_ms=float(arrival),
        origin='real_l515_realsense_ros',coordinate_frame='depth_optical',
        ros_frame_id=message.header.frame_id,ros_stamp_ns=message.header.stamp.to_nsec(),
        ros_timestamp_basis=TIMESTAMP_BASIS,serial=context['serial'],device=context['device'],
        usb_type=context['usb_type'],stream_id=context['stream_id'],
        depth_to_color=context['depth_to_color'],realsense_metadata=data)
