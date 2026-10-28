#!/usr/bin/env python3
"""Export a matched joint/hand-eye profile after rechecking independent validation."""
import argparse
from datetime import datetime
import hashlib
from pathlib import Path
import sys

import numpy as np
import yaml
from tf.transformations import quaternion_from_matrix

sys.path.insert(0, str(Path(__file__).resolve().parent))
from joint_calibration_model import RobotChain, JOINT_NAMES, evaluate
from publish_joint_calibration import chain_signature, load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('training_report', type=Path)
    parser.add_argument('validation_dataset', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = yaml.safe_load(args.training_report.read_text())
    train_path = Path(report['dataset_file'])
    if hashlib.sha256(train_path.read_bytes()).hexdigest() != report['dataset_sha256']:
        raise ValueError('Training data changed since fitting')
    train = yaml.safe_load(train_path.read_text())
    validation = yaml.safe_load(args.validation_dataset.read_text())
    xml = (train_path.parent/'robot.urdf').read_text()
    if xml != (args.validation_dataset.parent/'robot.urdf').read_text():
        raise ValueError('Training and validation models differ')
    if train['frames'] != validation['frames'] or train['tag_config'] != validation['tag_config']:
        raise ValueError('Frames or tag configuration differ')
    samples = [s for s in validation['samples'] if s.get('used_for_fit', True)]
    if train['role'] != 'train' or validation['role'] != 'validation' or len(samples) < 8:
        raise ValueError('Need a separate validation dataset with at least eight poses')
    if any(np.max(np.abs(np.asarray(a['joint_positions_rad'])-b['joint_positions_rad'])) < 1e-8
           for a in train['samples'] for b in samples):
        raise ValueError('Validation contains copied training poses')
    fit = report['models']['handeye_and_joint_offsets']
    if not fit['success'] or fit['offset_bound_reached'] or fit['data_jacobian_rank'] != fit['parameter_count']:
        raise ValueError('Joint fit failed, hit its bounds or was not observable')
    chain = RobotChain(xml, train['frames'][0], train['frames'][1])
    stats = evaluate(chain, samples, fit)
    baseline = evaluate(chain, samples, report['models']['handeye_only'])
    if stats['position_rms_mm'] >= baseline['position_rms_mm']:
        raise ValueError('Joint correction did not improve independent position validation')
    x = np.asarray(fit['link6_camera_matrix'])
    config = {'schema': 1, 'base_frame': train['frames'][0], 'nominal_effector_frame': train['frames'][1],
              'calibrated_effector_frame': 'rml63_link6_calibrated', 'camera_link_frame': 'camera_link',
              'camera_optical_frame': train['frames'][2], 'joint_names': JOINT_NAMES,
              'robot_chain_sha256': chain_signature(chain),
              'joint_offsets_deg': dict(zip(JOINT_NAMES, fit['offsets_deg'])),
              'effector_to_camera_optical': {'translation_m': x[:3, 3].tolist(),
                                            'quaternion_xyzw': quaternion_from_matrix(x).tolist()},
              'provenance': {'training_report': str(args.training_report.resolve()),
                             'training_sha256': report['dataset_sha256'],
                             'validation_dataset': str(args.validation_dataset.resolve()),
                             'validation_sha256': hashlib.sha256(args.validation_dataset.read_bytes()).hexdigest(),
                             'independent_validation': stats,
                             'scope': 'camera localization TF only; controller and MoveIt unchanged'}}
    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix('.yaml.tmp')
    temporary.write_text('# Active paired calibration. Angles: degrees; translations: metres.\n'
                         '# Restart publish.launch after changes. See JOINT_CALIBRATION.md.\n'+yaml.safe_dump(config, sort_keys=False))
    try:
        load_config(temporary, xml)
        if output.exists():
            backup = Path.home()/'.ros/rml63_handeye_deployments'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            backup.mkdir(parents=True)
            (backup/'previous_config.yaml').write_text(output.read_text())
            print('Previous config: '+str(backup/'previous_config.yaml'))
        temporary.replace(output)
    finally:
        if temporary.exists():
            temporary.unlink()
    print('Exported '+str(output))
    print('Independent validation: '+str(stats))
    print('Restart publisher to apply. No motion or controller zero changes were commanded.')


if __name__ == '__main__':
    main()
