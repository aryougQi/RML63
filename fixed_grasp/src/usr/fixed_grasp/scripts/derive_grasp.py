#!/usr/bin/env python3
"""Offline: back up teaching file and derive Pregrasp/Lift above Grasp."""
import argparse
from pathlib import Path
import os
import shutil
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parent))
import yaml
from waypoint_offsets import derive_from_grasp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints', type=Path, required=True)
    parser.add_argument('--height-m', type=float, default=.15)
    args = parser.parse_args()
    original = args.waypoints.read_text()
    result = derive_from_grasp(yaml.safe_load(original), args.height_m)
    backup = args.waypoints.with_name(args.waypoints.name+'.before_offsets_'+str(time.time_ns())+'.bak')
    shutil.copy2(str(args.waypoints), str(backup))
    temporary = args.waypoints.with_suffix(args.waypoints.suffix+'.tmp')
    temporary.write_text(yaml.safe_dump(result, sort_keys=False))
    os.replace(str(temporary), str(args.waypoints))
    print('Backup:', backup)
    print('Pregrasp = Lift = Grasp + base Z %.3f m; orientation unchanged' % args.height_m)
    print('Updated:', args.waypoints)


if __name__ == '__main__': main()
