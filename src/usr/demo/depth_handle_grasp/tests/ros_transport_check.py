#!/usr/bin/env python3
"""Synthetic image transport check on an existing ROS master. No robot topics used."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import numpy as np
import rospy
from sensor_msgs.msg import Image,CameraInfo,JointState
from realsense2_camera.msg import Metadata,Extrinsics
from realsense2_camera.srv import DeviceInfo,DeviceInfoResponse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ros_depth import capture_ros


def check(mode):
    namespace='/depth_handle_transport_test_'+mode
    publishers=[rospy.Publisher(namespace+'/depth/'+topic,typ,queue_size=10)
        for topic,typ in [('image_rect_raw',Image),('camera_info',CameraInfo),('metadata',Metadata)]]
    extrinsics=rospy.Publisher(namespace+'/extrinsics/depth_to_color',Extrinsics,queue_size=1,latch=True)
    e=Extrinsics();e.rotation=[0.,1.,0,-1.,0,0,0,0,1.];e.translation=[.01,0,0];extrinsics.publish(e)
    service=rospy.Service(namespace+'/realsense2_camera/device_info',DeviceInfo,
        lambda request:DeviceInfoResponse(device_name='intel_realsense_l515',serial_number='SYNTHETIC_TRANSPORT_FIXTURE',usb_type_descriptor='synthetic'))
    stop=threading.Event()
    joint_pub=rospy.Publisher(namespace+'/joints',JointState,queue_size=10) if mode=='joint_startup' else None
    first_joint=[]
    def publish_joints():
        while not stop.is_set():
            if joint_pub.get_num_connections():
                msg=JointState();msg.header.stamp=rospy.Time.now()+rospy.Duration(.02)
                msg.name=['test_joint'];msg.position=[0.]
                if not first_joint:first_joint.append(msg.header.stamp.to_sec())
                joint_pub.publish(msg)
            stop.wait(.02)
    joint_worker=threading.Thread(target=publish_joints) if joint_pub else None
    if joint_worker:joint_worker.start()
    def publish():
        deadline=time.monotonic()+5
        while not all(p.get_num_connections() for p in publishers):
            if stop.wait(.01) or time.monotonic()>deadline:return
        for i in range(12):
            if stop.is_set():return
            im=Image();im.width=2;im.height=2;im.step=4;im.encoding='16UC1'
            im.header.stamp=rospy.Time.now();im.header.seq=i
            if mode=='joint_startup' and i==0:
                im.header.stamp=rospy.Time.from_sec(first_joint[0]-.001)
            im.header.frame_id='camera_depth_optical_frame';im.data=np.array([[0,2000],[1000,4000]],dtype='<u2').tobytes()
            info=CameraInfo();info.header=im.header;info.width=2;info.height=2
            info.K=[100.,0,1.,0,100.,1.,0,0,1.];info.D=[0.]*5;info.R=[1.,0,0,0,1.,0,0,0,1.]
            number=i+(1 if mode=='dropped' and i>=2 else 0)
            if mode=='restart' and i>=2:number=i-2
            meta=Metadata();meta.header=im.header
            meta.json_data=json.dumps(dict(frame_number=number,frame_timestamp=i*100.,clock_domain='global_time',time_of_arrival=(rospy.Time.now().to_sec()-(2 if mode=='stale_arrival' else 0))*1000))
            # Deliberately change publication order to exercise timestamp pairing.
            publishers[0].publish(im);publishers[2].publish(meta);publishers[1].publish(info)
            if stop.wait(.1):return
    worker=threading.Thread(target=publish);worker.start()
    try:
        with tempfile.TemporaryDirectory(prefix='synthetic_ros_depth_') as directory:
            config=dict(ros_camera_namespace=namespace,camera_serial='SYNTHETIC_TRANSPORT_FIXTURE',ros_capture_timeout_sec=4,joint_feedback_max_age_sec=.3,max_capture_motion_deg=.1)
            try:
                result=capture_ros(Path(directory)/'frames',config,count=5,
                    joint_names=['test_joint'] if joint_pub else None,joint_topic=namespace+'/joints')
            except RuntimeError as error:
                if mode in ('normal','joint_startup'):raise
                expected='host arrival is stale' if mode=='stale_arrival' else 'restarted, dropped'
                assert expected in str(error),str(error)
                print('PASS rejected '+mode)
            else:
                assert mode in ('normal','joint_startup'),'bad sequence accepted'
                assert len(list((Path(directory)/'frames').glob('frame_*')))==5
                raw=np.load(str(Path(directory)/'frames/frame_000000/depth_raw.npy'))
                assert raw[0,1]*.001==2.0
                metadata=json.loads((Path(directory)/'frames/frame_000000/metadata.json').read_text())
                assert metadata['depth_scale_m_per_unit']==.001
                assert metadata['depth_to_color']['translation_m']==[.01,0.,0.]
                assert metadata['depth_to_color']['rotation_matrix']==[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]
                if mode=='joint_startup':
                    assert metadata['frame_number']>=1,'frame preceding first joint was not discarded'
                    assert result['joint_evidence']['first_stamp_ns']<=result['first_depth_stamp_ns']
                    print('PASS startup race: pre-joint depth skipped; contiguous sequence bracketed by joints')
                else:print('PASS original RealSense message transport, millimetre scale and factory extrinsics')
    finally:
        stop.set();worker.join(2)
        for p in publishers:p.unregister()
        extrinsics.unregister();service.shutdown()
        if joint_worker:joint_worker.join(2);joint_pub.unregister()

if __name__=='__main__':
    rospy.init_node('synthetic_depth_transport_check',anonymous=True)
    for mode in ('normal','dropped','restart','stale_arrival','joint_startup'):check(mode)
