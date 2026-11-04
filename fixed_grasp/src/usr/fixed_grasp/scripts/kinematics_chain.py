"""Runtime FK only; no calibration fitting or acquisition."""
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from tf.transformations import euler_matrix
JOINT_NAMES = ['joint%d' % i for i in range(1, 7)]


def rotation(vector):
    return cv2.Rodrigues(np.asarray(vector, dtype=float))[0]


class RobotChain:
    def __init__(self, xml, base='base_link', tip='link6'):
        root = ET.fromstring(xml)
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, current = [], tip
        while current != base:
            if current not in parents or len(chain) > 100:
                raise ValueError('URDF has no valid base-to-tip chain')
            joint = parents[current]
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute', 'continuous') or joint.find('mimic') is not None:
                raise ValueError('Unsupported joint in robot chain')
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            transform = euler_matrix(*rpy)
            transform[:3, 3] = xyz
            axis_node = joint.find('axis')
            axis = np.fromstring(axis_node.get('xyz', '1 0 0') if axis_node is not None else '1 0 0', sep=' ')
            axis /= np.linalg.norm(axis)
            index = None if kind == 'fixed' else JOINT_NAMES.index(joint.get('name'))
            chain.append((transform, axis, index))
            current = joint.find('parent').get('link')
        self.chain = list(reversed(chain))
        if sorted(i for _, _, i in self.chain if i is not None) != list(range(6)):
            raise ValueError('Expected exactly joint1 ... joint6 in chain')

    def fk(self, q, offsets=None):
        q = np.asarray(q) + (np.zeros(6) if offsets is None else np.asarray(offsets))
        result = np.eye(4)
        for origin, axis, index in self.chain:
            result = result @ origin
            if index is not None:
                motion = np.eye(4)
                motion[:3, :3] = rotation(axis * q[index])
                result = result @ motion
        return result
