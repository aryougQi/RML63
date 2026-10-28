"""Geometry shared by the ROS node and offline tests; no ROS node or motion calls."""
import math

import numpy as np
from tf.transformations import euler_matrix, quaternion_matrix


def finite_vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('%s must contain %d finite numbers' % (name, size))
    return result


def transform(xyz, xyzw):
    xyz = finite_vector(xyz, 3, 'translation')
    q = finite_vector(xyzw, 4, 'quaternion')
    if np.linalg.norm(q) < 1e-8:
        raise ValueError('Zero quaternion')
    result = quaternion_matrix(q / np.linalg.norm(q))
    result[:3, 3] = xyz
    return result


def tcp_transform(xyz, rpy):
    xyz = finite_vector(xyz, 3, 'tcp_xyz')
    rpy = finite_vector(rpy, 3, 'tcp_rpy')
    result = euler_matrix(*rpy)
    result[:3, 3] = xyz
    return result


def mean_transform(values):
    values = np.asarray(values)
    result = np.eye(4)
    result[:3, 3] = values[:, :3, 3].mean(axis=0)
    u, _, vt = np.linalg.svd(values[:, :3, :3].mean(axis=0))
    result[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return result


def delta(a, b):
    distance = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    cosine = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1)
    return distance, math.degrees(math.acos(float(cosine)))


def calibration_matrix(data, base, tool, camera, tag):
    params = data['parameters']
    if params['eye_on_hand'] is not True:
        raise ValueError('Expected eye-on-hand calibration')
    for key, expected in [('robot_base_frame', base), ('robot_effector_frame', tool),
                          ('tracking_base_frame', camera), ('tracking_marker_frame', tag)]:
        if params[key] != expected:
            raise ValueError('Calibration %s=%s; expected %s' % (key, params[key], expected))
    value = data['transformation']
    return transform([value[k] for k in 'xyz'], [value['q' + k] for k in 'xyzw'])


def joint_calibration_matrix(data, base, tool, camera):
    if data.get('schema') != 1 or data.get('joint_names') != ['joint%d' % i for i in range(1, 7)]:
        raise ValueError('Unsupported joint calibration schema or joint order')
    for key, expected in [('base_frame', base), ('nominal_effector_frame', tool),
                          ('camera_optical_frame', camera)]:
        if data[key] != expected:
            raise ValueError('Joint calibration %s=%s; expected %s' % (key, data[key], expected))
    corrected = data['calibrated_effector_frame']
    if not corrected or corrected.startswith('/') or corrected in (base, tool, camera):
        raise ValueError('Invalid calibrated_effector_frame')
    finite_vector([data['joint_offsets_deg'][n] for n in data['joint_names']], 6, 'joint_offsets_deg')
    value = data['effector_to_camera_optical']
    q = finite_vector(value['quaternion_xyzw'], 4, 'quaternion_xyzw')
    if abs(np.linalg.norm(q) - 1) > 1e-5:
        raise ValueError('Joint calibration quaternion must have unit length')
    return transform(value['translation_m'], q), corrected


def touch_targets(base_tag, base_camera, base_tool, tool_tcp, approach, clearance, align):
    if not np.isfinite([approach, clearance]).all() or not approach > clearance >= 0:
        raise ValueError('Require approach_distance_m > clearance_m >= 0')
    normal = base_tag[:3, 2].copy()
    camera_side = float(np.dot(base_camera[:3, 3] - base_tag[:3, 3], normal))
    if abs(camera_side) < .01:
        raise ValueError('Camera is too close to the Tag plane to determine the approach side')
    normal *= 1 if camera_side > 0 else -1
    current_tcp = base_tool @ tool_tcp
    # Do not approach through the board from the opposite half-space.
    if np.dot(current_tcp[:3, 3] - base_tag[:3, 3], normal) <= clearance:
        raise ValueError('Current TCP is behind/inside the Tag plane or closer than the requested gap')
    rotation = current_tcp[:3, :3].copy()
    if align:
        z_axis = -normal  # Closed gripper extends along TCP +Z toward the board.
        x_axis = rotation[:, 0] - z_axis * np.dot(rotation[:, 0], z_axis)
        if np.linalg.norm(x_axis) < 1e-6:
            x_axis = rotation[:, 1] - z_axis * np.dot(rotation[:, 1], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        rotation = np.column_stack([x_axis, np.cross(z_axis, x_axis), z_axis])
    tcp_goals, tool_goals = [], []
    for distance in (approach, clearance):
        goal = np.eye(4)
        goal[:3, :3] = rotation
        goal[:3, 3] = base_tag[:3, 3] + distance * normal
        tcp_goals.append(goal)
        tool_goals.append(goal @ np.linalg.inv(tool_tcp))
    return tcp_goals, tool_goals, normal
