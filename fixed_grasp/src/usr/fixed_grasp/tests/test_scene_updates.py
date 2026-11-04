"""MoveIt rejects removing missing bodies; all tests use scene service doubles."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    import return_home
    from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
    ROS_READY = True
except ImportError:
    ROS_READY = False


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class SceneTests(unittest.TestCase):
    def scene(self, world=(), attached=()):
        return SimpleNamespace(world=SimpleNamespace(collision_objects=list(world)),
                               robot_state=SimpleNamespace(attached_collision_objects=list(attached)))

    def model(self):
        b = backend.Backend.__new__(backend.Backend)
        b.config = dict(scene_mode='robot_only', flange_frame='link6')
        b.s = Mock(); b.scene_touched = False
        b.original = self.scene(); b.current_scene = Mock(return_value=self.scene())
        return b

    def test_failed_preflight_has_no_scene_cleanup(self):
        b = self.model(); b.restore_preview()
        b.current_scene.assert_not_called(); b.s.apply.assert_not_called()

    def test_empty_scene_cleanup_is_idempotent(self):
        b = self.model(); b.scene_touched = True
        b.restore_preview(); b.restore_preview()
        b.current_scene.assert_called_once(); b.s.apply.assert_not_called()

    def test_cleanup_removes_only_existing_owned_objects(self):
        b = self.model(); b.scene_touched = True
        tool = AttachedCollisionObject(link_name='link6', object=CollisionObject(id='fixed_grasp_tool_envelope'))
        other = AttachedCollisionObject(link_name='link6', object=CollisionObject(id='other_tool'))
        b.current_scene.return_value = self.scene([CollisionObject(id='table')], [tool, other])
        b.restore_preview()
        kwargs = b.s.apply.call_args.kwargs
        self.assertEqual(kwargs.get('world', []), [])
        self.assertEqual([a.object.id for a in kwargs['attached']], ['fixed_grasp_tool_envelope'])
        self.assertEqual(kwargs['attached'][0].object.operation, CollisionObject.REMOVE)

    def test_cleanup_removes_world_object_created_by_detach(self):
        b = self.model(); b.scene_touched = True
        tool = AttachedCollisionObject(link_name='link6', object=CollisionObject(id='fixed_grasp_tool_envelope'))
        table = CollisionObject(id='user_table')
        b.current_scene.side_effect = [self.scene([table], [tool]), self.scene([table, tool.object])]
        b.restore_preview()
        self.assertEqual(b.s.apply.call_count, 2)
        detach, remove = b.s.apply.call_args_list
        self.assertEqual([a.object.id for a in detach.kwargs['attached']], ['fixed_grasp_tool_envelope'])
        self.assertEqual([o.id for o in remove.kwargs['world']], ['fixed_grasp_tool_envelope'])
        self.assertEqual(remove.kwargs['world'][0].operation, CollisionObject.REMOVE)

    def test_cleanup_restores_previous_tool_without_missing_removals(self):
        b = self.model(); b.scene_touched = True
        tool = AttachedCollisionObject(link_name='link6', object=CollisionObject(id='fixed_grasp_tool_envelope'))
        b.original = self.scene(attached=[tool])
        b.restore_preview()
        b.s.apply.assert_called_once_with(world=[], attached=[tool])

    def test_initial_basket_does_not_remove_nonexistent_attachment(self):
        b = self.model(); b.config['scene_mode'] = 'measured'
        obj = CollisionObject(id='fixed_grasp_basket', operation=CollisionObject.ADD)
        b.basket = Mock(return_value=obj)
        b.set_basket(False)
        b.s.apply.assert_called_once_with(world=[obj], attached=[])

    def test_detach_removes_attachment_only_when_present(self):
        b = self.model(); b.config['scene_mode'] = 'measured'
        obj = CollisionObject(id='fixed_grasp_basket', operation=CollisionObject.ADD)
        b.basket = Mock(return_value=obj)
        existing = AttachedCollisionObject(link_name='link6', object=obj)
        b.current_scene.return_value = self.scene(attached=[existing])
        b.set_basket(False)
        attached = b.s.apply.call_args.kwargs['attached']
        self.assertEqual(len(attached), 1)
        self.assertEqual(attached[0].object.operation, CollisionObject.REMOVE)

    def test_cleanup_failure_preserves_original_return_home_error(self):
        b = self.model(); b.restore_preview = Mock(side_effect=RuntimeError('cleanup failed'))
        with patch.object(return_home.rospy, 'myargv', return_value=['return_home.py']), patch.object(return_home.rospy, 'init_node'), patch.object(return_home.rospy, 'logwarn') as log, patch.object(return_home.moveit_commander, 'roscpp_initialize'), patch.object(return_home.fcntl, 'flock'), patch.object(return_home, 'load_settings', return_value={}), patch.object(return_home, 'Backend', return_value=b), patch.object(return_home, 'return_home', side_effect=RuntimeError('original preflight error')):
            with self.assertRaisesRegex(RuntimeError, 'original preflight error'):
                return_home.main()
        log.assert_called_once()


if __name__ == '__main__': unittest.main()
