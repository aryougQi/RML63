"""Two grasp selections with one authoritative Home and placement teaching file."""
from copy import deepcopy
import hashlib
from pathlib import Path

import numpy as np
import yaml

from waypoint_offsets import derive_from_grasp

GRASP_NAMES = {'Grasp', 'Pregrasp', 'Lift'}


def action_name(grasp):
    if grasp not in (1, 2):
        raise ValueError('grasp must be 1 or 2')
    return '/fixed_grasp/task' if grasp == 1 else '/fixed_grasp/task_2'


def read_shared(path):
    raw = Path(path).read_bytes()
    data = yaml.safe_load(raw)
    if (not isinstance(data, dict) or data.get('schema') != 1 or
            not isinstance(data.get('identity'), dict) or not isinstance(data.get('waypoints'), dict)):
        raise ValueError('schema 1 shared teaching file required')
    return data, hashlib.sha256(raw).hexdigest()


def new_grasp_profile(shared, digest):
    return dict(schema=1, identity=deepcopy(shared['identity']),
                shared_waypoints_sha256=digest, waypoints={})


def validate_profile_reference(profile, shared, digest):
    if (not isinstance(profile, dict) or profile.get('schema') != 1 or
            profile.get('identity') != shared['identity'] or
            profile.get('shared_waypoints_sha256') != digest):
        raise ValueError('second grasp model/shared teaching changed; teach grasp 2 again')
    if not isinstance(profile.get('waypoints'), dict) or set(profile['waypoints']) - GRASP_NAMES:
        raise ValueError('second grasp file may contain only Grasp, Pregrasp and Lift')


def load_task_waypoints(shared_path, grasp_path=None):
    shared, digest = read_shared(shared_path)
    if grasp_path is None:
        return shared, digest
    path = Path(grasp_path)
    if not path.is_file():
        raise ValueError('second grasp not taught; run teach.py Grasp --grasp 2 --interactive')
    profile = yaml.safe_load(path.read_text())
    validate_profile_reference(profile, shared, digest)
    if set(profile['waypoints']) != GRASP_NAMES:
        raise ValueError('second grasp requires measured Grasp and derived Pregrasp/Lift')
    expected = derive_from_grasp(profile, .15)['waypoints']
    for name in ('Pregrasp', 'Lift'):
        record = profile['waypoints'][name]
        if (not isinstance(record, dict) or record.get('derived_from') != 'Grasp' or 'joints_rad' in record or
                not np.array_equal(record.get('offset_base_m'), [0., 0., .15]) or
                not np.array_equal(record.get('tcp_base'), expected[name]['tcp_base'])):
            raise ValueError('second Pregrasp/Lift must be Grasp + base Z 0.15 m; teach grasp 2 again')
    result = deepcopy(shared)
    result['waypoints'].update(deepcopy(profile['waypoints']))
    return result, digest
