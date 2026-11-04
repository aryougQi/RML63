"""Compiled KDL IK for the existing calibrated chain; no ROS or robot commands."""
import math

import numpy as np
import PyKDL as kdl
from scipy.spatial.transform import Rotation


def frame(matrix):
    matrix = np.asarray(matrix, float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('invalid IK pose')
    return kdl.Frame(kdl.Rotation(*matrix[:3, :3].ravel()), kdl.Vector(*matrix[:3, 3]))


class CalibratedIK:
    def __init__(self, source, offsets, bounds):
        self.source = source
        self.offsets = np.asarray(offsets, float)
        bounds = np.asarray(bounds, float)
        if (self.offsets.ndim != 1 or bounds.shape != (len(self.offsets), 2) or
                not np.isfinite(bounds).all() or not np.isfinite(self.offsets).all()):
            raise ValueError('invalid IK offsets/bounds')
        # Both reported and calibrated joint coordinates must remain in bounds.
        self.lower = np.maximum(bounds[:, 0], bounds[:, 0]-self.offsets)
        self.upper = np.minimum(bounds[:, 1], bounds[:, 1]-self.offsets)
        self.chain = kdl.Chain()
        self.indices = []
        for number, (origin, axis, index) in enumerate(source.chain):
            self.chain.addSegment(kdl.Segment('origin_%d' % number, kdl.Joint(), frame(origin)))
            if index is not None:
                self.indices.append(index)
                joint = kdl.Joint('joint%d' % (index+1), kdl.Vector.Zero(), kdl.Vector(*axis), kdl.Joint.RotAxis)
                self.chain.addSegment(kdl.Segment('rotation_%d' % number, joint))
        if sorted(self.indices) != list(range(len(self.offsets))) or np.any(self.lower >= self.upper):
            raise ValueError('unsupported calibrated IK chain/bounds')
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        # KDL solvers hold native references; keep their dependencies alive.
        self.velocity_solver = kdl.ChainIkSolverVel_pinv(self.chain)
        self.nominal_lower = self.joints(self.lower+self.offsets)
        self.nominal_upper = self.joints(self.upper+self.offsets)
        self.bounded_solver = kdl.ChainIkSolverPos_NR_JL(self.chain,
            self.nominal_lower, self.nominal_upper,
            self.fk_solver, self.velocity_solver, 80, 1e-8)
        self.lm_solver = kdl.ChainIkSolverPos_LMA(self.chain, 1e-8, 100)
        # Reject a construction/order mismatch before allowing any fast solution.
        probe = np.clip(np.zeros(len(self.offsets)), self.lower, self.upper)
        if not np.allclose(self.fk(probe), source.fk(probe, self.offsets), atol=1e-10):
            raise ValueError('KDL chain disagrees with calibrated FK')

    def joints(self, values):
        result = kdl.JntArray(len(self.indices))
        for number, index in enumerate(self.indices):
            result[number] = float(values[index])
        return result

    def fk(self, reported):
        result = kdl.Frame()
        self.fk_solver.JntToCart(self.joints(np.asarray(reported)+self.offsets), result)
        matrix = np.eye(4)
        matrix[:3, :3] = [[result.M[i, j] for j in range(3)] for i in range(3)]
        matrix[:3, 3] = [result.p[i] for i in range(3)]
        return matrix

    def solutions(self, desired, seed):
        desired = np.asarray(desired, float)
        seed = np.asarray(seed, float)
        if seed.shape != self.offsets.shape or not np.isfinite(seed).all():
            raise ValueError('invalid IK seed')
        initial = self.joints(np.clip(seed, self.lower, self.upper)+self.offsets)
        target = frame(desired)
        seen = []
        for solver in (self.bounded_solver, self.lm_solver):
            answer = kdl.JntArray(len(self.indices))
            solver.CartToJnt(initial, target, answer)
            q = np.empty(len(self.indices))
            for number, index in enumerate(self.indices):
                q[index] = answer[number]-self.offsets[index]
            if not np.isfinite(q).all():
                continue
            # KDL LMA can return an equivalent revolution outside the URDF range.
            # Choose the nearest equivalent that respects both coordinate bounds.
            for index, value in enumerate(q):
                low = math.ceil((self.lower[index]-value)/(2*math.pi))
                high = math.floor((self.upper[index]-value)/(2*math.pi))
                if low > high:
                    break
                turn = min(high, max(low, round((seed[index]-value)/(2*math.pi))))
                q[index] += turn*2*math.pi
            else:
                q = np.clip(q, self.lower, self.upper)
                achieved = self.source.fk(q, self.offsets)
                error = np.linalg.norm(Rotation.from_dcm(desired[:3, :3].T@achieved[:3, :3]).as_rotvec())
                if (np.linalg.norm(achieved[:3, 3]-desired[:3, 3]) <= .001 and error <= np.radians(.5) and
                        not any(np.max(abs(q-old)) < 1e-7 for old in seen)):
                    seen.append(q.copy())
                    yield q
