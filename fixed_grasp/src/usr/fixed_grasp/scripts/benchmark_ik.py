#!/usr/bin/env python3
"""Offline IK comparison on taught poses; no ROS nodes, collision RPC or motion."""
import argparse
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml

from backend import Session, RobotSession, load_settings, geometry
from planner_metrics import PlannerMetrics
from kinematics_config import load_config


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root/'reports/ik_comparison.json')
    args = parser.parse_args()
    config = load_settings(root/'config/fixed.yaml')
    import rospkg
    model = Path(rospkg.RosPack().get_path('rml63_tools_description'))/'urdf/rml63_with_gripper.urdf.xacro'
    xml = xacro.process_file(str(model), mappings={'fixed_gripper': 'true'}).toxml()
    _, chain, offsets, _ = load_config(config['calibration_file'], xml)
    joints = {joint.get('name'): joint for joint in ET.fromstring(xml).findall('joint')}
    names = ['joint%d' % i for i in range(1, 7)]
    bounds = {name: (float(joints[name].find('limit').get('lower')),
                     float(joints[name].find('limit').get('upper'))) for name in names}
    shared = yaml.safe_load((root/'config/waypoints.yaml').read_text())
    second = yaml.safe_load((root/'config/waypoints_2.yaml').read_text())
    tcp = geometry(config).at_width(config['contact_width_m'])['flange_tcp']
    inv_tcp = np.linalg.inv(tcp)

    def session():
        result = Session.__new__(Session)
        result.config = config.copy()
        result.chain = chain; result.offsets = offsets; result.active = names
        result.metrics = PlannerMetrics()
        result.cancel_check = lambda: None
        result.robot = SimpleNamespace(get_joint=lambda name: SimpleNamespace(bounds=lambda: bounds[name]))
        def check_state(q):
            for values in (np.asarray(q), np.asarray(q)+offsets):
                if not np.isfinite(values).all() or any(not bounds[n][0] <= v <= bounds[n][1] for n, v in zip(names, values)):
                    raise RuntimeError('joint bounds')
        result.check_state = check_state
        return result

    records = [shared['waypoints'][name] for name in ('Home', 'Grasp', 'Preplace', 'Place', 'Retreat')]
    records.append(second['waypoints']['Grasp'])
    random = np.random.RandomState(63)
    endpoints = []
    stress = []
    for record in records:
        q = np.asarray(record['joints_rad'], float)
        target = chain.fk(q, offsets)
        for _ in range(10):
            endpoints.append((target, q+random.uniform(-.15, .15, 6)))
            stress.append((target, q+random.uniform(-1., 1., 6)))

    def run(optimized):
        s = session()
        solve = (lambda target, seed: s.solve(target, seed)) if optimized else (
            lambda target, seed: RobotSession.solve(s, target, seed))
        cold = time.monotonic()
        solve(endpoints[0][0], endpoints[0][1])
        cold = time.monotonic()-cold
        timings = []; errors = []; failures = []
        groups = {}
        def query(target, seed, label, local=False):
            s._local_ik = local
            started = time.monotonic()
            try:
                q = solve(target, seed)
                achieved = chain.fk(q, offsets)
                position = float(np.linalg.norm(achieved[:3, 3]-target[:3, 3]))
                angle = float(np.degrees(np.linalg.norm(Rotation.from_dcm(target[:3, :3].T@achieved[:3, :3]).as_rotvec())))
                if position > .001 or angle > .5:
                    raise RuntimeError('FK residual')
                if local and np.max(abs(q-seed)) > np.radians(config['max_joint_jump_deg']):
                    raise RuntimeError('Cartesian branch jump')
                errors.append((position, angle))
                groups[label]['successes'] += 1
                return q
            except Exception as error:
                failures.append(dict(group=label, reason=str(error)))
                return None
            finally:
                elapsed = time.monotonic()-started
                timings.append(elapsed)
                groups[label]['queries'] += 1
                groups[label]['wall_sec'] += elapsed
        groups['endpoints'] = dict(queries=0, successes=0, wall_sec=0.)
        for target, seed in endpoints:
            query(target, seed, 'endpoints')
        groups['stress_endpoints'] = dict(queries=0, successes=0, wall_sec=0.)
        for target, seed in stress:
            query(target, seed, 'stress_endpoints')
        for number, data in enumerate((shared, second), 1):
            label = 'grasp_%d_vertical_150mm' % number
            groups[label] = dict(queries=0, successes=0, wall_sec=0.)
            q = np.asarray(data['waypoints']['Grasp']['joints_rad'], float)
            start = chain.fk(q, offsets)@tcp
            for height in np.linspace(0., .15, 51)[1:]:
                target = start.copy(); target[2, 3] += height
                answer = query(target@inv_tcp, q, label, local=True)
                if answer is None: break
                q = answer
        return dict(queries=len(timings), successes=len(errors), failures=failures, groups=groups,
                    total_solver_wall_sec=sum(timings), median_query_ms=float(np.median(timings))*1000,
                    p95_query_ms=float(np.percentile(timings, 95))*1000, cold_first_query_sec=cold,
                    max_position_error_m=max((e[0] for e in errors), default=None),
                    max_rotation_error_deg=max((e[1] for e in errors), default=None),
                    counters=dict(s.metrics.counts))

    baseline = run(False)
    optimized = run(True)
    report = dict(scope='Offline IK only; actual model/calibration/taught poses. No collision RPC, MoveIt search, ROS nodes or motion. Not a full-planning speed or success-rate measurement.',
        reference='rml63_tag_touch uses compiled MoveIt KDL; fixed_grasp now uses compiled KDL with calibrated joint conversion and original numerical fallback.',
        calibration_sha256=hashlib.sha256(Path(config['calibration_file']).read_bytes()).hexdigest(),
        baseline=baseline, optimized=optimized,
        ik_speedup=baseline['total_solver_wall_sec']/optimized['total_solver_wall_sec'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), baseline_successes=baseline['successes'],
                         optimized_successes=optimized['successes'], ik_speedup=report['ik_speedup']), indent=2))


if __name__ == '__main__': main()
