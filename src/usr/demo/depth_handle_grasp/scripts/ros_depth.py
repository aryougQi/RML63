"""Collect synchronized native ROS depth, calibration, and stationary joint evidence."""
import json
from realsense2_camera.srv import DeviceInfo
import threading
import time
from collections import deque
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState
from realsense2_camera.msg import Metadata
from realsense_source import camera_context, normalize, TIMESTAMP_BASIS


def decode_depth(message, metadata, camera_info):
    if message.encoding!='16UC1' or message.header.frame_id!=metadata['ros_frame_id']:
        raise ValueError('expected native 16UC1 depth in declared optical frame')
    k=metadata['intrinsics']
    if (message.width,message.height)!=(k['width'],k['height']):raise ValueError('depth dimensions mismatch')
    if camera_info.header.stamp!=message.header.stamp:
        raise ValueError('CameraInfo belongs to another frame')
    if camera_info.header.frame_id!=message.header.frame_id or (camera_info.width,camera_info.height)!=(message.width,message.height):
        raise ValueError('CameraInfo grid/frame mismatch')
    if not np.allclose([camera_info.K[0],camera_info.K[4],camera_info.K[2],camera_info.K[5]],
                       [k['fx'],k['fy'],k['ppx'],k['ppy']],rtol=0,atol=1e-6):
        raise ValueError('CameraInfo intrinsics differ from capture metadata')
    if not np.allclose(camera_info.D,k.get('coefficients',[]),rtol=0,atol=1e-8):
        raise ValueError('CameraInfo distortion differs from capture metadata')
    scale=metadata['depth_scale_m_per_unit']
    if not np.isfinite(scale) or scale<=0:raise ValueError('invalid raw depth scale')
    # rospy rewrites Header.seq per publisher; pair by exact timestamp instead.
    if message.header.stamp.to_nsec()!=metadata['ros_stamp_ns']:
        raise ValueError('metadata belongs to another frame')
    if message.step<message.width*2 or len(message.data)!=message.step*message.height:
        raise ValueError('invalid depth data stride/length')
    dtype=np.dtype('>u2' if message.is_bigendian else '<u2')
    return np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                      strides=(message.step,2)).astype(np.uint16,copy=True)


def capture_ros(output,config,count=30,joint_names=None,joint_topic='/joint_states'):
    import rosgraph
    root=config.get('ros_camera_namespace','/camera').rstrip('/')
    topics=[root+'/depth/image_rect_raw',root+'/depth/camera_info',root+'/depth/metadata']
    master=rosgraph.Master(rospy.get_name())
    pubs=dict(master.getSystemState()[0])
    owners=[pubs.get(t,[]) for t in topics]
    if any(len(o)!=1 for o in owners) or len({o[0] for o in owners})!=1:
        raise RuntimeError('start the original realsense2_camera node; duplicate/missing depth publishers: '+str(owners))
    owner=owners[0][0]
    context=camera_context(root,config,master,owner)
    output=Path(output)
    if output.exists() and any(output.iterdir()):raise ValueError('capture output must be empty')
    output.mkdir(parents=True,exist_ok=True)
    lock=threading.Condition();cache={};ready=deque();joint_samples=[];errors=[]
    start=rospy.Time.now().to_nsec();max_age=float(config.get('ros_frame_max_age_sec',.5))
    def receive(kind,message):
        try:
            value=message
            stamp=message.header.stamp.to_nsec()
            if stamp<start:return
            with lock:
                entry=cache.setdefault(stamp,{})
                if kind in entry:raise ValueError('duplicate frame component')
                entry[kind]=value
                if len(entry)==3:
                    ready.append(entry);del cache[stamp]
                if len(cache)>120 or len(ready)>120:raise ValueError('ROS frame synchronization backlog')
                lock.notify_all()
        except Exception as error:
            with lock:errors.append(str(error));lock.notify_all()
    def joints(message):
        try:
            age=(rospy.Time.now()-message.header.stamp).to_sec()
            if not -.05<=age<=config['joint_feedback_max_age_sec']:
                raise ValueError('stale joint state during depth acquisition')
            values=dict(zip(message.name,message.position))
            q=np.array([values[n] for n in joint_names])
            if not np.isfinite(q).all():raise ValueError('invalid joints')
            with lock:joint_samples.append((message.header.stamp.to_nsec(),q));lock.notify_all()
        except Exception as error:
            with lock:errors.append(str(error));lock.notify_all()
    subscribers=[]
    frames=[];deadline=time.monotonic()+float(config.get('ros_capture_timeout_sec',15))
    try:
        if joint_names:
            subscribers.append(rospy.Subscriber(joint_topic,JointState,joints,queue_size=200))
            with lock:
                while not joint_samples:
                    if errors:raise RuntimeError(errors[0])
                    if time.monotonic()>=deadline:raise RuntimeError('no initial joint feedback')
                    lock.wait(.05)
                # First received joints may be newer than the request start.
                # Only admit images stamped at/after real joint coverage begins.
                start=max(start,joint_samples[0][0])
        for topic,kind,typ in zip(topics,('image','info','metadata'),(Image,CameraInfo,Metadata)):
            subscribers.append(rospy.Subscriber(topic,typ,lambda m,k=kind:receive(k,m),queue_size=90,buff_size=2**24))
        previous=None
        while len(frames)<count:
            if rospy.is_shutdown():raise RuntimeError('ROS shutdown during capture')
            with lock:
                if errors:raise RuntimeError(errors[0])
                if not ready:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise RuntimeError('timed out waiting for synchronized ROS depth')
                    lock.wait(min(.1,remaining));continue
                entry=ready.popleft()
            message=entry['image']
            metadata=normalize(message,entry['info'],entry['metadata'],context)
            age=(rospy.Time.now()-message.header.stamp).to_sec()
            if not -.05<=age<=max_age:
                raise RuntimeError('ROS depth header age %.3f s outside [-0.05, %.3f] s'%(age,max_age))
            arrival_age=rospy.Time.now().to_sec()-metadata['host_arrival_timestamp_ms']/1000.
            if not -.05<=arrival_age<=config.get('ros_arrival_max_age_sec',.5):
                raise RuntimeError('RealSense host arrival is stale or future dated: %.3f s'%arrival_age)
            if metadata.get('serial')!=config['camera_serial'] or metadata.get('origin')!='real_l515_realsense_ros':
                raise RuntimeError('unexpected camera identity/source')
            if metadata.get('ros_timestamp_basis')!=TIMESTAMP_BASIS:
                raise RuntimeError('unsupported timestamp basis')
            if previous and (metadata['stream_id']!=previous['stream_id'] or metadata['frame_number']!=previous['frame_number']+1
                or metadata['timestamp_domain']!=previous['timestamp_domain'] or metadata['timestamp_ms']<=previous['timestamp_ms'] or metadata['ros_stamp_ns']<=previous['ros_stamp_ns']):
                raise RuntimeError('camera restarted, dropped, repeated or reordered depth frame; retry acquisition')
            raw=decode_depth(message,metadata,entry['info'])
            frames.append((raw,metadata));previous=metadata
        if joint_names:
            with lock:
                while not joint_samples or joint_samples[-1][0]<frames[-1][1]['ros_stamp_ns']:
                    if errors:raise RuntimeError(errors[0])
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise RuntimeError('joint feedback did not bracket captured depth')
                    lock.wait(min(.05,remaining))
                if errors:raise RuntimeError(errors[0])
                samples=list(joint_samples)
            evidence=stationary_evidence(samples,frames,config)
        else:evidence=None
        if master.lookupNode(owner)!=context['stream_id']:
            raise RuntimeError('RealSense publisher restarted during acquisition')
        after=dict(master.getSystemState()[0])
        if any(after.get(t)!=[owner] for t in topics):
            raise RuntimeError('RealSense publishers changed during acquisition')
        device=rospy.ServiceProxy(root+'/realsense2_camera/device_info',DeviceInfo)()
        if device.serial_number!=config['camera_serial']:
            raise RuntimeError('camera identity changed during acquisition')
        # Save after acquisition to avoid disk IO causing frame loss.
        for i,(raw,m) in enumerate(frames):
            directory=output/('frame_%06d'%i);directory.mkdir()
            np.save(str(directory/'depth_raw.npy'),raw)
            (directory/'metadata.json').write_text(json.dumps(m,indent=2)+'\n')
        return dict(first_depth_stamp_ns=frames[0][1]['ros_stamp_ns'],last_depth_stamp_ns=frames[-1][1]['ros_stamp_ns'],
                    stream_id=frames[0][1]['stream_id'],joint_evidence=evidence,topics=topics,
                    acquisition_not_before_ns=start)
    finally:
        for subscriber in subscribers:subscriber.unregister()


def stationary_evidence(samples,frames,config):
    if len(samples)<2 or not frames:
        raise RuntimeError('insufficient stationary joint evidence')
    if samples[-1][0]<frames[-1][1]['ros_stamp_ns']:
        raise RuntimeError('joint feedback does not bracket last depth frame')
    if samples[0][0]>frames[0][1]['ros_stamp_ns']:
        raise RuntimeError('no joint feedback before first depth frame; retry')
    q=np.array([s[1] for s in samples])
    if not np.isfinite(q).all():raise RuntimeError('invalid joint evidence')
    if np.max(np.ptp(q,axis=0))>np.radians(config['max_capture_motion_deg']):
        raise RuntimeError('robot moved during ROS depth sequence')
    times=np.array([s[0] for s in samples],dtype=np.int64)
    if np.any(np.diff(times)<=0) or np.max(np.diff(times))>config['joint_feedback_max_age_sec']*1e9:
        raise RuntimeError('joint feedback has a gap or nonmonotonic timestamps')
    for _,m in frames:
        if np.min(abs(times-m['ros_stamp_ns']))>config['joint_feedback_max_age_sec']*1e9:
            raise RuntimeError('depth has no nearby joint feedback')
    evidence=dict(sample_count=len(samples),max_joint_range_deg=float(np.degrees(np.max(np.ptp(q,axis=0)))),
        first_stamp_ns=int(times[0]),last_stamp_ns=int(times[-1]),
        measured_joints_mean_rad=np.mean(q,axis=0).tolist())
    return evidence
