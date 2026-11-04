#!/usr/bin/env python3
"""Record an operator's explicit placement verification; no ROS or robot commands."""
import argparse
from copy import deepcopy
import os
from pathlib import Path
import shutil
import time

import yaml
from grasp_profiles import read_shared


def confirm(config_path, waypoint_path, source):
    if not isinstance(source, str) or not source.strip():
        raise ValueError('placement verification source must not be empty')
    shared, digest = read_shared(waypoint_path)
    if not {'Home', 'Grasp', 'Preplace', 'Place', 'Retreat'} <= set(shared['waypoints']):
        raise ValueError('measured grasp and shared placement waypoints required')
    config_path = Path(config_path)
    config = yaml.safe_load(config_path.read_text())
    if not isinstance(config, dict) or config.get('release_mode') != 'taught_place':
        raise ValueError('this confirmation is only for release_mode: taught_place')
    updated = deepcopy(config)
    updated['taught_place_verification'] = dict(verified=True, source=source.strip(),
        waypoints_sha256=digest, confirmed_unix=time.time())
    backup = config_path.with_name(config_path.name+'.before_place_confirmation_'+str(time.time_ns())+'.bak')
    shutil.copy2(str(config_path), str(backup))
    temporary = config_path.with_suffix(config_path.suffix+'.tmp')
    temporary.write_text(yaml.safe_dump(updated, sort_keys=False))
    os.replace(str(temporary), str(config_path))
    return backup


def main():
    defaults = Path(__file__).resolve().parents[1]/'config'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=defaults/'fixed.yaml')
    parser.add_argument('--waypoints', type=Path, default=defaults/'waypoints.yaml')
    parser.add_argument('--source', required=True, help='操作者的实测确认记录')
    parser.add_argument('--confirm-supported', action='store_true',
                        help='明确确认当前物体在 Place 已实测能稳定支撑并松爪')
    args = parser.parse_args()
    if not args.confirm_supported:
        parser.error('必须实测确认后显式传入 --confirm-supported；该程序不检测物理支撑')
    backup = confirm(args.config, args.waypoints, args.source)
    print('已记录操作者的实测放置确认；未发送机械臂或夹爪命令。')
    print('Backup:', backup)


if __name__ == '__main__':
    main()
