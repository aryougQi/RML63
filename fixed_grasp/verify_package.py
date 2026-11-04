#!/usr/bin/env python3
"""Read-only relocation/model/teaching validation. Never start nodes or move hardware."""
import hashlib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
import yaml


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'src/usr/fixed_grasp/scripts'))
from backend import load_settings, geometry
from grasp_profiles import load_task_waypoints
from kinematics_config import load_config


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    pack = rospkg.RosPack()
    expected = {'fixed_grasp', 'rm_driver', 'rm_control', 'rm_msgs',
                'rm_description', 'rml_63_moveit_config', 'rml63_tools_description'}
    local = {ET.parse(path).getroot().findtext('name') for path in (ROOT/'src').rglob('package.xml')}
    assert local == expected, (local, expected)
    for name in expected:
        assert ROOT in Path(pack.get_path(name)).resolve().parents, (name, pack.get_path(name))
    for name in ('rml63_handeye', 'depth_handle_grasp', 'easy_handeye', 'apriltag_ros'):
        try:
            path = pack.get_path(name)
        except rospkg.ResourceNotFound:
            continue
        raise AssertionError('Validation environment still exposes removed package '+name+': '+path)
    fixed = Path(pack.get_path('fixed_grasp'))
    tools = Path(pack.get_path('rml63_tools_description'))
    config = load_settings(fixed/'config/fixed.yaml')
    xml = (tools/'urdf/fixed_runtime.urdf').read_text()
    _, chain, offsets, _ = load_config(config['calibration_file'], xml)
    mount = Path(config['mount_file'])
    shared, digest = load_task_waypoints(fixed/'config/waypoints.yaml')
    profile, _ = load_task_waypoints(fixed/'config/waypoints.yaml', fixed/'config/waypoints_2.yaml')
    identity = shared['identity']
    assert sha(Path(config['calibration_file'])) == identity['calibration_sha256']
    assert sha(mount) == identity['mount_sha256']
    assert hashlib.sha256(xml.encode()).hexdigest() == identity['urdf_sha256']
    assert sha(tools/'config/rml63_with_gripper.srdf') == identity['srdf_sha256']
    assert digest == config['taught_place_verification']['waypoints_sha256']
    assert profile['identity'] == identity
    assert np.allclose(geometry(config).at_width(config['contact_width_m'])['flange_tcp'], identity['flange_tcp'])
    for node in ET.fromstring(xml).iter('mesh'):
        uri = node.get('filename')
        assert uri.startswith('package://'), uri
        name, relative = uri[len('package://'):].split('/', 1)
        assert (Path(pack.get_path(name))/relative).is_file(), uri
    from fast_ik import CalibratedIK
    urdf = ET.fromstring(xml)
    bounds = [[float(urdf.find("joint[@name='%s']/limit" % name).get(key))
               for key in ('lower', 'upper')] for name in identity['joint_names']]
    solver = CalibratedIK(chain, offsets, bounds)
    q = np.array([0., -.5, .7, .1, -.3, .2])
    assert np.allclose(solver.fk(q), chain.fk(q, offsets), atol=1e-10)
    assert list(solver.solutions(chain.fk(q, offsets), q)), 'KDL IK did not return a valid solution'
    for path in ROOT.rglob('*.py'):
        if any(part in ('build', 'devel') for part in path.relative_to(ROOT).parts):
            continue
        compile(path.read_bytes(), str(path), 'exec')
    print('PASS: 7 local packages; no removed application packages visible.')
    print('PASS: model, joint corrections, mount, TCP, two grasp profiles and release hashes.')
    print('PASS: mesh resources, Python syntax, compiled KDL FK and IK.')
    print('Motion scaling:', config['velocity_scaling'], config['acceleration_scaling'])


if __name__ == '__main__':
    main()
