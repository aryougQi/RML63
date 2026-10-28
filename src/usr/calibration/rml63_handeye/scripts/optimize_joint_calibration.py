#!/usr/bin/env python3
"""Offline baseline vs joint-offset fits, with pose-level cross-validation. No application."""
import argparse
import hashlib
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joint_calibration_model import RobotChain, JOINT_NAMES, fit, evaluate
from validate_calibration import matrix


def load(path):
    data = yaml.safe_load(path.read_text())
    if data.get('schema') != 1 or data.get('joint_names') != JOINT_NAMES:
        raise ValueError('Need new synchronized joint dataset, not old easy_handeye TF-only samples')
    data['samples'] = [s for s in data.get('samples', []) if s.get('used_for_fit', True)]
    if not data['samples']:
        raise ValueError('Empty dataset')
    return data, (path.parent/'robot.urdf').read_text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--validation', type=Path)
    parser.add_argument('--max-offset-deg', type=float, default=3.)
    parser.add_argument('--prior-deg', type=float, default=1.)
    args = parser.parse_args()
    data, xml = load(args.dataset)
    if data['role'] != 'train' or len(data['samples']) < 20:
        raise ValueError('Need a training dataset with at least 20 independent poses')
    chain = RobotChain(xml, data['frames'][0], data['frames'][1])
    t = data['initial_calibration']['transformation']
    initial_x = matrix({'translation': t, 'rotation': {k: t['q'+k] for k in 'xyzw'}})
    report = {'status': 'diagnostic_only_not_applied', 'convention': 'q_model = q_reported + delta',
              'dataset_file': str(args.dataset.resolve()),
              'dataset_sha256': hashlib.sha256(args.dataset.read_bytes()).hexdigest(),
              'robot_urdf_sha256': hashlib.sha256(xml.encode()).hexdigest(),
              'fit_pose_count': len(data['samples']),
              'joint_span_deg': np.degrees(np.ptp([s['joint_positions_rad'] for s in data['samples']], axis=0)).tolist(),
              'fixed_gauges': ['joint1', 'joint6'],
              'note': 'Fixed gauges are not verified physical zero offsets. Other fitted offsets may absorb visual/model errors.',
              'max_offset_deg': args.max_offset_deg, 'offset_prior_sigma_deg': args.prior_deg,
              'position_sigma_mm': 1., 'rotation_sigma_deg': .5, 'models': {}}
    samples = data['samples']
    groups = np.arange(len(samples))
    np.random.RandomState(2026).shuffle(groups)
    for name, offsets in [('handeye_only', False), ('handeye_and_joint_offsets', True)]:
        print('拟合 '+name+'（离线，不控制机械臂）', flush=True)
        kwargs = dict(with_offsets=offsets, max_offset_deg=args.max_offset_deg, prior_deg=args.prior_deg)
        result = fit(chain, samples, initial_x, **kwargs)
        folds = []
        for fold in range(5):
            test_indices = set(groups[fold::5].tolist())
            training = [s for i, s in enumerate(samples) if i not in test_indices]
            test = [s for i, s in enumerate(samples) if i in test_indices]
            fitted = fit(chain, training, initial_x, **kwargs)
            folds.append({'test_pose_count': len(test), 'test': evaluate(chain, test, fitted),
                          'offsets_deg': fitted['offsets_deg'], 'success': fitted['success'],
                          'offset_bound_reached': fitted['offset_bound_reached']})
            print('  留出验证 %d/5：%.3f mm RMS' % (fold+1, folds[-1]['test']['position_rms_mm']), flush=True)
        result['pose_level_cross_validation'] = folds
        result['cross_validation_position_rms_mm'] = float(np.sqrt(sum(
            f['test']['position_rms_mm']**2 * f['test_pose_count'] for f in folds)/len(samples)))
        result['offset_fold_std_deg'] = np.std([f['offsets_deg'] for f in folds], axis=0).tolist()
        report['models'][name] = result
    if args.validation:
        validation, validation_xml = load(args.validation)
        if validation['role'] != 'validation' or len(validation['samples']) < 8:
            raise ValueError('Independent validation needs at least 8 newly captured poses')
        if xml != validation_xml or data['frames'] != validation['frames'] or data['tag_config'] != validation['tag_config']:
            raise ValueError('Model, frames or target configuration changed between train and validation')
        # Exact duplicates must not be presented as independent validation.
        if any(np.max(np.abs(np.asarray(a['joint_positions_rad'])-b['joint_positions_rad'])) < 1e-8
               for a in samples for b in validation['samples']):
            raise ValueError('Validation contains copied training poses')
        for result in report['models'].values():
            result['independent_validation'] = evaluate(chain, validation['samples'], result)
        report['validation_note'] = 'Tag and camera mounting must remain physically fixed between datasets; software cannot verify this.'
    baseline = report['models']['handeye_only']['cross_validation_position_rms_mm']
    joint = report['models']['handeye_and_joint_offsets']['cross_validation_position_rms_mm']
    report['cross_validation_relative_improvement'] = float(1-joint/baseline) if baseline > 1e-12 else None
    output = args.dataset.parent/'joint_optimization_report.yaml'
    output.write_text(yaml.safe_dump(report, sort_keys=False))
    print('报告：'+str(output))
    print('结果只用于诊断；未修改 URDF、joint_states、MoveIt 或控制器零点。')


if __name__ == '__main__':
    main()
