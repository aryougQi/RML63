"""Exact CAD coverage and immutable state sharing; no ROS node or hardware."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
try:
    import backend
    from tool_envelope import build_mesh_union
    from moveit_msgs.msg import RobotState, RobotTrajectory, AttachedCollisionObject, CollisionObject
    from trajectory_msgs.msg import JointTrajectoryPoint
    ROS_READY = True
except ImportError:
    ROS_READY = False


class TinyGripper:
    def __init__(self):
        self.meshes = {name: {'vertices': np.array([[[0.,0.,0.],[scale,0.,0.],[0.,scale,0.]]])}
                       for name, scale in [('static',1.),('moving',2.),('near',3.)]}
    def at_width(self, width): return {'linkage_angle_rad': width}
    def link(self, name, angle):
        t = np.eye(4)
        if name == 'moving': t[0,3] = angle
        if name == 'near': t[0,3] = angle*1e-10
        return t


@unittest.skipUnless(ROS_READY, 'source ROS workspace')
class EnvelopeTests(unittest.TestCase):
    def test_exact_duplicates_only_and_mesh_conversion_reused(self):
        meshes, transforms, counts = build_mesh_union(TinyGripper(), .045)
        self.assertEqual(counts['tool_input_instances'], 138)
        self.assertEqual(counts['tool_unique_instances'], 93)
        self.assertEqual(len(set(id(m) for m in meshes)), 3)
        self.assertEqual(counts['tool_mesh_conversions'], 3)
        # Even tiny nonzero translations survive; no allclose-based pruning.
        self.assertEqual(sum(0 < t[0,3] < 1e-10 for t in transforms), 45)

    def test_cancellation_is_checked_while_building(self):
        check = Mock(side_effect=RuntimeError('cancelled'))
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            build_mesh_union(TinyGripper(), .045, check)

    def test_all_real_cad_openings_preserved_exactly(self):
        config = backend.load_settings(Path(__file__).resolve().parents[1]/'config/fixed.yaml')
        geom = backend.geometry(config)
        meshes, transforms, counts = build_mesh_union(geom, config['open_width_m'])
        def key(vertices, triangles):
            return np.asarray(vertices, dtype=np.float64).tobytes()+np.asarray(triangles, dtype=np.int64).tobytes()
        originals = {}
        for name, data in geom.meshes.items():
            vertices, indices = np.unique(data['vertices'].reshape(-1,3), axis=0, return_inverse=True)
            originals[name] = key(vertices, indices.reshape(-1,3))
        expected = set()
        for width in np.linspace(0., config['open_width_m'], 46):
            angle = geom.at_width(float(width))['linkage_angle_rad']
            for name in geom.meshes:
                expected.add((originals[name], np.asarray(geom.link(name, angle), dtype=np.float64).tobytes()))
        encoded = {}
        for mesh in meshes:
            if id(mesh) not in encoded:
                encoded[id(mesh)] = key([[p.x,p.y,p.z] for p in mesh.vertices],
                                        [t.vertex_indices for t in mesh.triangles])
        actual = {(encoded[id(mesh)], t.tobytes()) for mesh,t in zip(meshes,transforms)}
        self.assertEqual(actual, expected)
        self.assertLess(counts['tool_unique_triangles'], counts['tool_input_triangles'])
        self.assertEqual(counts['tool_mesh_conversions'], len(geom.meshes))

    def test_state_advance_matches_original_without_copying_geometry(self):
        state = RobotState()
        state.joint_state.name = ['j1','j2']; state.joint_state.position = [.1,.2]
        state.joint_state.velocity = [.3,.4]
        state.attached_collision_objects = [AttachedCollisionObject(object=CollisionObject(id='held'))]
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['j2','j1']
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[.5,.6])]
        expected = backend.state_after(state, trajectory)
        actual = backend.state_after_shared_geometry(state, trajectory)
        self.assertEqual(actual, expected)
        self.assertIs(actual.attached_collision_objects[0], state.attached_collision_objects[0])
        self.assertIsNot(actual.joint_state, state.joint_state)
        self.assertIsNot(actual.attached_collision_objects, state.attached_collision_objects)
        self.assertEqual(state.joint_state.position, [.1,.2])


if __name__ == '__main__': unittest.main()
