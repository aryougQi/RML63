"""The open CAD replacement must retain obstacle checking for the real tool."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
import numpy as np
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from moveit_msgs.msg import AttachedCollisionObject, RobotState
from copy import deepcopy
from shape_msgs.msg import Mesh, SolidPrimitive
from geometry_msgs.msg import Pose

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from grasp_ros import open_finger_collision_matrix, RobotSession


class OpenFingerModelTests(unittest.TestCase):
    def test_joint_only_diff_retains_scene_attachment_collision(self):
        # Exercise MoveIt's actual conversion semantics, not a mock service.
        from moveit.core import load_robot_model,planning_scene,collision_detection
        here=Path(__file__).resolve().parents[1]
        src=here.parents[2]
        model=load_robot_model(str(here/'reports/robot.urdf'),
                              str(src/'rml63_tools_description/config/rml63_with_gripper.srdf'))
        scene=planning_scene.PlanningScene(model)
        state=RobotState();state.joint_state.name=['joint%d'%i for i in range(1,7)]
        state.joint_state.position=[0.]*6
        attachment=AttachedCollisionObject();attachment.link_name='base_link';attachment.touch_links=['base_link']
        attachment.object.id='diff_retention_test';attachment.object.header.frame_id='base_link'
        attachment.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.15,.15,.15])]
        pose=Pose();pose.position.z=.17;pose.orientation.w=1.
        attachment.object.primitive_poses=[pose]
        if hasattr(attachment.object,'pose'):attachment.object.pose.orientation.w=1.
        state.attached_collision_objects=[attachment];scene.setCurrentState(state)
        diff=RobotState();diff.is_diff=True;diff.joint_state=deepcopy(state.joint_state)
        scene.setCurrentState(diff)
        request=collision_detection.CollisionRequest();request.contacts=True;request.max_contacts=100
        result=collision_detection.CollisionResult();scene.checkCollision(request,result)
        self.assertTrue(any('diff_retention_test' in pair for pair in result.contacts),
                        'joint-only diff must retain attached collision geometry')

    def test_nominal_and_corrected_checks_keep_independent_joint_values(self):
        session=RobotSession.__new__(RobotSession)
        template=RobotState();template.joint_state.name=['j1','j2'];template.joint_state.position=[0.,0.]
        reads=[];requests=[]
        def current():reads.append(True);return deepcopy(template)
        session.robot=SimpleNamespace(get_current_state=current)
        session.active=['j1','j2'];session.offsets=np.array([.01,.02])
        session.config={'planning_group':'arm'}
        session.open_tool_attachment=AttachedCollisionObject();session.open_tool_attachment.object.id='tool'
        def valid(request):
            requests.append(request.robot_state)
            return SimpleNamespace(valid=True)
        session.validity=valid
        session.check_state(np.array([.1,.2]))
        self.assertEqual(len(reads),0)
        np.testing.assert_allclose(requests[0].joint_state.position,[.1,.2])
        np.testing.assert_allclose(requests[1].joint_state.position,[.11,.22])
        self.assertTrue(requests[0].is_diff)
        self.assertTrue(requests[1].is_diff)
        self.assertEqual(requests[0].attached_collision_objects,[])
        self.assertEqual(requests[1].attached_collision_objects,[])
        self.assertEqual(template.joint_state.position,[0.,0.])

    def test_phase_width_updates_geometry_without_hardware(self):
        session=RobotSession.__new__(RobotSession)
        session.current_tool_width_m=.032
        def link(name,angle):
            t=np.eye(4);t[1,3]=angle/2*(1 if name=='left' else -1);return t
        session.tool_geometry=SimpleNamespace(meshes={'left':None,'right':None},
            at_width=lambda width:{'linkage_angle_rad':width},link=link)
        session.tool_local_samples={'left':np.zeros((1,3)),'right':np.zeros((1,3))}
        attachment=AttachedCollisionObject();attachment.object.id='tool'
        attachment.object.meshes=[Mesh(),Mesh()];attachment.object.mesh_poses=[Pose(),Pose()]
        attachment.object.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.045])]
        attachment.object.primitive_poses=[Pose()]
        session.open_tool_attachment=attachment;updates=[]
        session.apply=lambda **kw:updates.append(kw)
        session.set_tool_width(.016)
        self.assertEqual(session.current_tool_width_m,.016)
        np.testing.assert_allclose(session.tool_samples[:,1],[.008,-.008])
        self.assertEqual(session.open_tool_attachment.object.primitives[0].dimensions,[.045])
        self.assertEqual(len(updates),1)
        session.set_tool_width(.032)
        np.testing.assert_allclose(session.tool_samples[:,1],[.016,-.016])
        session.set_tool_width(.032)
        self.assertEqual(len(updates),2)

    def test_only_duplicate_finger_tests_are_replaced(self):
        names=['finger5','finger6','basket','open_tool','arm']
        matrix=AllowedCollisionMatrix(entry_names=names,
            entry_values=[AllowedCollisionEntry(enabled=[False]*5) for _ in names])
        matrix.entry_values[0].enabled[4]=True
        matrix.entry_values[4].enabled[0]=True
        # A stale permissive entry for the real tool must be corrected.
        matrix.entry_values[2].enabled[3]=True
        matrix.entry_values[3].enabled[2]=True
        updated=open_finger_collision_matrix(matrix,['finger5','finger6'],'basket','open_tool')
        self.assertTrue(updated.entry_values[0].enabled[2])
        self.assertTrue(updated.entry_values[1].enabled[2])
        self.assertFalse(updated.entry_values[2].enabled[3])
        self.assertFalse(updated.entry_values[3].enabled[2])
        self.assertFalse(updated.entry_values[4].enabled[2])
        self.assertTrue(updated.entry_values[0].enabled[4])
        self.assertFalse(matrix.entry_values[0].enabled[2])
        self.assertTrue(matrix.entry_values[2].enabled[3])

    def test_new_objects_and_distinct_model_required(self):
        matrix=AllowedCollisionMatrix()
        updated=open_finger_collision_matrix(matrix,['finger5','finger6'],'basket','open_tool')
        self.assertEqual(len(updated.entry_names),4)
        self.assertTrue(all(len(row.enabled)==4 for row in updated.entry_values))
        self.assertEqual(updated,open_finger_collision_matrix(updated,['finger5','finger6'],'basket','open_tool'))
        with self.assertRaises(ValueError):
            open_finger_collision_matrix(matrix,['finger5'],'basket','basket')


if __name__=='__main__':unittest.main()
