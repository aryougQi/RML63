#!/usr/bin/env python3
"""Offline FK and identifiable joint-offset/hand-eye fitting. Never controls a robot."""
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from tf.transformations import euler_matrix

JOINT_NAMES = ['joint%d' % i for i in range(1, 7)]
FREE_JOINTS = [1, 2, 3, 4]  # Zero based; J1 and J6 are gauge freedoms, not measured zeros.


def rotation(vector):
    return cv2.Rodrigues(np.asarray(vector, dtype=float))[0]


def rotation_vector(value):
    return cv2.Rodrigues(np.asarray(value, dtype=float))[0].ravel()


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


def mean_transform(values):
    values = np.asarray(values)
    result = np.eye(4)
    result[:3, 3] = values[:, :3, 3].mean(axis=0)
    u, _, vt = np.linalg.svd(values[:, :3, :3].mean(axis=0))
    result[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return result


def pose_errors(predictions, target):
    return {'position_rms_mm': float(np.sqrt(np.mean([
        np.linalg.norm(p[:3, 3] - target[:3, 3]) ** 2 for p in predictions])) * 1000),
            'position_max_mm': float(max(np.linalg.norm(p[:3, 3] - target[:3, 3]) for p in predictions) * 1000),
            'rotation_rms_deg': float(np.degrees(np.sqrt(np.mean([
                np.linalg.norm(rotation_vector(target[:3, :3].T @ p[:3, :3])) ** 2 for p in predictions]))))}


def fit(chain, samples, initial_x, with_offsets=True, max_offset_deg=3., prior_deg=1.,
        position_sigma_mm=1., rotation_sigma_deg=.5):
    """Fit only on supplied training poses. delta is added to measured q, in radians."""
    if min(max_offset_deg, prior_deg, position_sigma_mm, rotation_sigma_deg) <= 0:
        raise ValueError('Bounds, prior and measurement scales must be positive')
    qs = [np.asarray(s['joint_positions_rad']) for s in samples]
    cs = [np.asarray(s['camera_marker_matrix']) for s in samples]
    if len(samples) < 8 or not all(q.shape == (6,) and np.isfinite(q).all() for q in qs):
        raise ValueError('Need at least eight poses with finite six-joint readings')
    y0 = mean_transform([chain.fk(q) @ initial_x @ c for q, c in zip(qs, cs)])
    count = 4 if with_offsets else 0

    def unpack(z):
        delta = np.zeros(6)
        if with_offsets:
            delta[FREE_JOINTS] = z[:count]
        x, y = initial_x.copy(), y0.copy()
        x[:3, :3] = rotation(z[count:count+3]) @ initial_x[:3, :3]
        x[:3, 3] += z[count+3:count+6]
        y[:3, :3] = rotation(z[count+6:count+9]) @ y0[:3, :3]
        y[:3, 3] += z[count+9:count+12]
        return delta, x, y

    def data_residual(z):
        delta, x, y = unpack(z)
        residual = []
        for q, c in zip(qs, cs):
            p = chain.fk(q, delta) @ x @ c
            residual.extend((p[:3, 3] - y[:3, 3]) / (position_sigma_mm / 1000))
            residual.extend(rotation_vector(y[:3, :3].T @ p[:3, :3]) / np.radians(rotation_sigma_deg))
        return np.asarray(residual)

    def objective(z):
        data = data_residual(z)
        return np.r_[data, z[:count] / np.radians(prior_deg)] if with_offsets else data

    z0 = np.zeros(count+12)
    lower, upper = np.full_like(z0, -np.inf), np.full_like(z0, np.inf)
    lower[:count], upper[:count] = -np.radians(max_offset_deg), np.radians(max_offset_deg)
    result = least_squares(objective, z0, bounds=(lower, upper), loss='soft_l1',
                           x_scale='jac', max_nfev=250, ftol=1e-9, xtol=1e-9, gtol=1e-9)
    delta, x, y = unpack(result.x)
    # Data-only Jacobian: priors must never falsely appear to make a parameter observable.
    steps = np.r_[np.full(count, np.radians(1.)), np.full(3, np.radians(1.)),
                  np.full(3, .001), np.full(3, np.radians(1.)), np.full(3, .001)]
    jac = []
    for i, scale in enumerate(steps):
        h = np.zeros_like(result.x)
        h[i] = scale * 1e-4
        jac.append((data_residual(result.x+h)-data_residual(result.x-h)) / 2e-4)
    sv = np.linalg.svd(np.asarray(jac).T, compute_uv=False)
    prediction = [chain.fk(q, delta) @ x @ c for q, c in zip(qs, cs)]
    return {'success': bool(result.success), 'offsets_rad': delta.tolist(),
            'offsets_deg': np.degrees(delta).tolist(), 'link6_camera_matrix': x.tolist(),
            'base_tag_matrix': y.tolist(), 'training': pose_errors(prediction, y),
            'data_jacobian_rank': int(np.sum(sv > sv[0]*1e-6)), 'parameter_count': len(result.x),
            'scaled_data_jacobian_singular_values': sv.tolist(),
            'scaled_data_condition': float(sv[0]/max(sv[-1], 1e-15)),
            'offset_bound_reached': bool(np.any(np.abs(np.degrees(delta)) >= max_offset_deg * .98))}


def evaluate(chain, samples, result):
    x, y = np.asarray(result['link6_camera_matrix']), np.asarray(result['base_tag_matrix'])
    return pose_errors([chain.fk(s['joint_positions_rad'], result['offsets_rad']) @ x @
                        np.asarray(s['camera_marker_matrix']) for s in samples], y)
