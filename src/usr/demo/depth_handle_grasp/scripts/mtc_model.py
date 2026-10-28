"""Load an isolated articulated model for planning; never change /robot_description."""
from pathlib import Path
import xacro
import xml.etree.ElementTree as ET
import yaml
import rospy

HERE = Path(__file__).resolve().parents[1]
SRC = HERE.parents[2]


def load_planning_model(arm_cfg):
    if rospy.get_namespace() == '/':
        raise RuntimeError('MTC model must run in its own ROS namespace (__ns:=/basket_mtc)')
    mount = yaml.safe_load(Path(arm_cfg['mount_file']).read_text())['launch_defaults']
    xml = xacro.process_file(str(SRC/'rml63_tools_description/urdf/rml63_with_gripper.urdf.xacro'),
        mappings={'fixed_gripper': 'false', 'mount_xyz': ' '.join(map(str, mount['mount_xyz'])),
                  'mount_rpy': ' '.join(map(str, mount['mount_rpy']))}).toxml()
    semantic = ET.fromstring((SRC/'rml63_tools_description/config/rml63_with_gripper.srdf').read_text())
    hand = ET.SubElement(semantic, 'group', name='eg2_planning')
    ET.SubElement(hand, 'joint', name='4C2_Joint1')
    # Mimic joints are included through the gripper links for complete state updates.
    for name in ('4C2_Link1', '4C2_Link2', '4C2_Link3', '4C2_Link4', '4C2_Link5', '4C2_Link6'):
        ET.SubElement(hand, 'link', name=name)
    rospy.set_param('robot_description', xml)
    rospy.set_param('robot_description_semantic', ET.tostring(semantic, encoding='unicode'))
    moveit_cfg = SRC/'rm_robot/rm_moveit_config/rml_63_moveit_config/config'
    rospy.set_param('robot_description_kinematics', yaml.safe_load((moveit_cfg/'kinematics.yaml').read_text()))
    limits = yaml.safe_load((moveit_cfg/'joint_limits.yaml').read_text())
    # Explicit conservative acceleration for timing, including the planning-only hand.
    for name in ['joint%d' % i for i in range(1, 7)]+['4C2_Joint1']:
        limits['joint_limits'].setdefault(name, {})
        limits['joint_limits'][name].update(has_acceleration_limits=True, max_acceleration=.5)
    rospy.set_param('robot_description_planning', limits)
    ompl = yaml.safe_load((moveit_cfg/'ompl_planning.yaml').read_text())
    ompl.update(planning_plugin='ompl_interface/OMPLPlanner',
                request_adapters='default_planner_request_adapters/AddTimeOptimalParameterization',
                start_state_max_bounds_error=.001)
    rospy.set_param('move_group/planning_pipelines/ompl', ompl)
    return xml
