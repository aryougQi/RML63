#!/usr/bin/env python3
"""Acquire fresh data through the existing ROS depth/calibration pipeline. No motion."""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import json
import numpy as np
import yaml
import rospy
import moveit_commander
from grasp_demo import acquire, write
from grasp_ros import RobotSession
HERE = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE/'config/grasp.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    cfg = yaml.safe_load(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    moveit_commander.roscpp_initialize(sys.argv); rospy.init_node('capture_multigrasp')
    session = RobotSession(cfg)
    # Construction and acquire only read hardware state. Do not call gripper IO,
    # session.execute_one(), add_scene(), or require trajectory execution enabled.
    record, q, stamp, depth, meta, mask = acquire(session, cfg, args.output,
        'multigrasp_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    base_depth = np.asarray(record['base_depth'])
    z = depth.astype(float)*meta['depth_scale_m_per_unit']
    v, u = np.nonzero((depth > 0) & (z >= .25) & (z <= 1.))
    k = meta['intrinsics']; zz = z[v, u]
    points = np.c_[(u-k['ppx'])*zz/k['fx'], (v-k['ppy'])*zz/k['fy'], zz]
    points = points @ base_depth[:3, :3].T+base_depth[:3, 3]
    outside = session.self_filter(points, q)
    keep = outside & ~mask[v, u].astype(bool)
    # All captured non-target samples are preserved; planner later voxelizes
    # conservatively. Unobserved basket surfaces require explicit box geometry.
    record['observed_obstacle_points_base_m'] = points[keep].tolist()
    record['hardware_motion_sent'] = False
    write(args.output/'capture_context.json', record)
    print(args.output/'capture_context.json')
    return 0

if __name__ == '__main__': sys.exit(main())
