#!/usr/bin/env python3
"""Place one cuboid at the selected recorded TCP and publish the same MoveIt scene."""
import json, math, sys
from pathlib import Path
import numpy as np
import rospkg, rospy, yaml, actionlib
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import SpawnModel, GetModelState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
import moveit_commander


def box_sdf(name, size, static, color):
    sx,sy,sz=size
    return '''<sdf version="1.6"><model name="%s"><static>%s</static><link name="body">
<inertial><mass>0.15</mass><inertia><ixx>%s</ixx><iyy>%s</iyy><izz>%s</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name="collision"><geometry><box><size>%s %s %s</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>%s %s %s</size></box></geometry><material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>
</link></model></sdf>'''%(name,str(static).lower(),.15*(sy*sy+sz*sz)/12,.15*(sx*sx+sz*sz)/12,.15*(sx*sx+sy*sy)/12,sx,sy,sz,sx,sy,sz,color,color)


def main():
    rospy.init_node('fixed_scene')
    pkg=Path(rospkg.RosPack().get_path('fixed_grasp'))
    profile=int(rospy.get_param('~profile',1))
    if profile not in (1,2): raise ValueError('profile must be 1 or 2')
    shared=yaml.safe_load((pkg/'config/waypoints.yaml').read_text())
    data=shared if profile==1 else yaml.safe_load((pkg/'config/waypoints_2.yaml').read_text())
    tcp=np.array(data['waypoints']['Grasp']['tcp_base'])
    # Explicit assumed dimensions: the source does not contain object CAD.
    size=[.12,.03,.06]
    xyz=tcp[:3,3].copy(); xyz[2]+=.5
    # A level cuboid: project TCP Y onto the table, leaving its short X across
    # the gripper opening (EG2-4C2 closes along TCP X).
    yaw=math.atan2(tcp[1,1],tcp[0,1])
    pose=Pose(); pose.position.x,pose.position.y,pose.position.z=xyz.tolist()
    pose.orientation.z=math.sin(yaw/2); pose.orientation.w=math.cos(yaw/2)
    rospy.wait_for_service('/gazebo/spawn_sdf_model',60)
    spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
    top=xyz[2]-size[2]/2
    models=[('target_cuboid',size,False,pose,'0.95 0.45 0.06 1')]
    support=Pose(); support.position.x=xyz[0]; support.position.y=xyz[1]
    support.position.z=top/2; support.orientation.w=1
    models.append(('target_support',[.20,.16,top],True,support,'0.35 0.4 0.45 1'))
    pedestal=Pose(); pedestal.position.z=.235; pedestal.orientation.w=1
    models.append(('robot_pedestal',[.18,.18,.47],True,pedestal,'0.25 0.28 0.32 1'))
    for name,dim,static,p,c in reversed(models):
        result=spawn(name,box_sdf(name,dim,static,c),'',p,'world')
        if not result.success: raise RuntimeError(result.status_message)
    scene=moveit_commander.PlanningSceneInterface(synchronous=True)
    for name,dim,_,p,_ in models:
        stamp=PoseStamped(); stamp.header.frame_id='world'; stamp.pose=p
        scene.add_box(name,stamp,size=dim)
    pub=rospy.Publisher('/fixed_sim/grasp_marker',Marker,queue_size=1,latch=True)
    marker=Marker(); marker.header.frame_id='base_link'; marker.ns='taught_grasp'; marker.id=1
    marker.type=Marker.SPHERE; marker.action=Marker.ADD
    marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=tcp[:3,3].tolist()
    marker.pose.orientation.w=1; marker.scale.x=marker.scale.y=marker.scale.z=.012
    marker.color.r=1; marker.color.g=.1; marker.color.a=1
    pub.publish(marker)
    client=actionlib.SimpleActionClient('/arm/arm_joint_controller/follow_joint_trajectory',FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(60)): raise RuntimeError('No simulated trajectory controller')
    correction=yaml.safe_load((pkg/'config/runtime_kinematics.yaml').read_text())
    names=shared['identity']['joint_names']
    # Gazebo uses nominal URDF coordinates, while recorded joints are controller
    # readings. Apply the paired correction to the simulation command only.
    q=[v+math.radians(correction['joint_offsets_deg'][n]) for n,v in zip(names,shared['waypoints']['Home']['joints_rad'])]
    goal=FollowJointTrajectoryGoal(); goal.trajectory.joint_names=names
    goal.trajectory.points=[JointTrajectoryPoint(positions=q,time_from_start=rospy.Duration(4))]
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(30)) or client.get_result().error_code!=0: raise RuntimeError('Simulated Home failed')
    rospy.sleep(1)
    rospy.wait_for_service('/gazebo/get_model_state',30)
    actual=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)('target_cuboid','world')
    position=np.array([actual.pose.position.x,actual.pose.position.y,actual.pose.position.z])
    if np.linalg.norm(position-xyz)>.002: raise RuntimeError('Cuboid placement drift exceeds 2 mm')
    report={'profile':profile,'frame':'base_link','grasp_tcp_base_m':tcp[:3,3].tolist(),'cuboid_size_m':size,'cuboid_world_m':xyz.tolist(),'actual_world_m':position.tolist(),'placement_error_m':float(np.linalg.norm(position-xyz)),'home_action_error_code':client.get_result().error_code,'physics_grasp_validated':False,'assumptions':'120 x 30 x 60 mm cuboid centered at grasp TCP; level yaw from TCP Y; support is approximate; frozen gripper, no robot contact physics'}
    out=Path('/workspace/outputs'); out.mkdir(exist_ok=True)
    (out/'scene_report.json').write_text(json.dumps(report,indent=2)+'\n')
    rospy.loginfo('SCENE READY: %s',json.dumps(report))
    rospy.spin()

if __name__=='__main__': main()
