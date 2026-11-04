#!/usr/bin/env python3
"""Validate live Gazebo placement, controller feedback and MoveIt geometry."""
import json, math
from pathlib import Path
import numpy as np
import rospkg, rospy, yaml
from gazebo_msgs.srv import GetModelState
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
import moveit_commander
rospy.init_node('check_fixed_scene',anonymous=True)
report=json.loads(Path('/workspace/outputs/scene_report.json').read_text())
state=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)('target_cuboid','world')
assert state.success
actual=np.array([state.pose.position.x,state.pose.position.y,state.pose.position.z])
error=float(np.linalg.norm(actual-report['cuboid_world_m']))
assert error<.002, ('cuboid drift',error)
controllers=rospy.ServiceProxy('/arm/controller_manager/list_controllers',ListControllers)()
assert all(any(c.name==name and c.state=='running' for c in controllers.controller) for name in ['joint_state_controller','arm_joint_controller'])
js=rospy.wait_for_message('/arm/joint_states',JointState,timeout=10)
pkg=Path(rospkg.RosPack().get_path('fixed_grasp'))
shared=yaml.safe_load((pkg/'config/waypoints.yaml').read_text())
offsets=yaml.safe_load((pkg/'config/runtime_kinematics.yaml').read_text())['joint_offsets_deg']
q=[dict(zip(js.name,js.position))[n] for n in shared['identity']['joint_names']]
expected=[v+math.radians(offsets[n]) for n,v in zip(shared['identity']['joint_names'],shared['waypoints']['Home']['joints_rad'])]
assert np.max(np.abs(np.array(q)-expected))<.01, ('home feedback mismatch',q,expected)
scene=moveit_commander.PlanningSceneInterface(synchronous=True)
assert {'target_cuboid','target_support','robot_pedestal'} <= set(scene.get_known_object_names())
# Compare the recorded grasp against paired calibrated forward kinematics.
import sys
sys.path.insert(0,str(pkg/'scripts'))
from kinematics_chain import RobotChain
from kinematics_config import load_config
xml=(Path(rospkg.RosPack().get_path('rml63_tools_description'))/'urdf/fixed_runtime.urdf').read_text()
_,chain,delta,_=load_config(pkg/'config/runtime_kinematics.yaml',xml)
data=shared if report['profile']==1 else yaml.safe_load((pkg/'config/waypoints_2.yaml').read_text())
grasp=data['waypoints']['Grasp']
fk=chain.fk(grasp['joints_rad'],delta) @ np.array(shared['identity']['flange_tcp'])
fk_error=float(np.linalg.norm(fk[:3,3]-np.array(report['grasp_tcp_base_m'])))
assert fk_error<1e-6, ('TCP calibration mismatch',fk_error)
report['live_check']={'placement_error_m':error,'calibrated_grasp_fk_error_m':fk_error,'controllers_running':True,'home_feedback_verified':True,'moveit_objects_verified':True}
Path('/workspace/outputs/scene_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
