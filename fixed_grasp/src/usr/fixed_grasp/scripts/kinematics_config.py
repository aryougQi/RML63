"""Read frozen joint corrections and validate the matching runtime model."""
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from tf.transformations import quaternion_matrix
from kinematics_chain import RobotChain, JOINT_NAMES


def chain_signature(chain):
    payload = [(origin.tolist(), axis.tolist(), index) for origin, axis, index in chain.chain]
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def load_config(path, xml):
    config = yaml.safe_load(Path(path).expanduser().read_text())
    if config.get('schema') != 1 or config.get('joint_names') != JOINT_NAMES:
        raise ValueError('Unsupported calibration config or joint order')
    frames = [config[k] for k in ('base_frame', 'nominal_effector_frame', 'calibrated_effector_frame',
                                  'camera_link_frame', 'camera_optical_frame')]
    if len(set(frames)) != len(frames) or any(not f or f.startswith('/') for f in frames):
        raise ValueError('Frame names must be distinct and have no leading slash')
    chain = RobotChain(xml, config['base_frame'], config['nominal_effector_frame'])
    if chain_signature(chain) != config['robot_chain_sha256']:
        raise ValueError('Running robot kinematic chain differs from calibrated chain; review model before applying')
    delta = np.radians([config['joint_offsets_deg'][n] for n in JOINT_NAMES])
    if not np.isfinite(delta).all() or np.max(np.abs(delta)) > np.radians(3.000001):
        raise ValueError('Joint corrections must be finite degrees within the fitted +/-3 degree bound')
    if abs(delta[0]) > 1e-12 or abs(delta[5]) > 1e-12:
        raise ValueError('This calibration fixes J1/J6 gauge offsets to zero; changing these needs a new consistent calibration')
    transform = config['effector_to_camera_optical']
    q, t = np.asarray(transform['quaternion_xyzw'], dtype=float), np.asarray(transform['translation_m'], dtype=float)
    if q.shape != (4,) or t.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(t).all():
        raise ValueError('Invalid camera transform shape/value')
    if abs(np.linalg.norm(q)-1) > 1e-5:
        raise ValueError('Camera quaternion must have unit length')
    x = quaternion_matrix(q)
    x[:3, 3] = t
    return config, chain, delta, x
