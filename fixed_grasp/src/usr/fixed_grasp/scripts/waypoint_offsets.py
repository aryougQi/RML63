"""Pure pose derivation; generated targets never masquerade as measured joints."""
from copy import deepcopy
import time
import numpy as np


def offset_pose(source, offset):
    pose = np.asarray(source, float).copy()
    delta = np.asarray(offset, float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError('invalid source TCP pose')
    if delta.shape != (3,) or not np.isfinite(delta).all() or delta[2] <= 0 or np.any(delta[:2] != 0):
        raise ValueError('derived Pregrasp/Lift offset must be positive along base +Z')
    pose[:3, 3] += delta
    return pose


def derive_from_grasp(data, height_m=.15):
    if data.get('schema') != 1 or 'Grasp' not in data.get('waypoints', {}):
        raise ValueError('schema 1 teaching file with measured Grasp required')
    source = data['waypoints']['Grasp']
    if 'derived_from' in source or 'joints_rad' not in source:
        raise ValueError('Grasp must remain an actual measured joint/TCP record')
    delta = [0., 0., float(height_m)]
    pose = offset_pose(source['tcp_base'], delta)
    result = deepcopy(data)
    for name in ('Pregrasp', 'Lift'):
        result['waypoints'][name] = dict(derived_from='Grasp', offset_base_m=delta.copy(),
            tcp_base=pose.tolist(), generated_unix=time.time())
    return result
