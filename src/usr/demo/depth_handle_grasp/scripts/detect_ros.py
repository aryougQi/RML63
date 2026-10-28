#!/usr/bin/env python3
"""Acquire native ROS depth and run offline stability/detection, without robot control."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import rospy
import yaml
from ros_depth import capture_ros

HERE=Path(__file__).resolve().parents[1]
VISION=HERE.parents[2]/'usr/depth_handle_detect'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=HERE/'config/grasp.yaml')
    args=parser.parse_args()
    config=yaml.safe_load(args.config.read_text())
    rospy.init_node('depth_handle_detect_ros',anonymous=True)
    run='ros_'+datetime.now().strftime('%Y%m%d_%H%M%S')
    raw=VISION/'data/captures'/run
    visual=VISION/'outputs'/run
    try:
        evidence=capture_ros(raw,config,count=30)
        (raw/'ros_capture.json').write_text(json.dumps(evidence,indent=2)+'\n')
        result=subprocess.run([str(VISION/'.venv/bin/python'),str(VISION/'run.py'),'stability',
            '--input',str(raw),'--config',config['visual_config'],'--output',str(visual)])
        print('Detection image:',visual/'handle_detection.png')
        print('Results:',VISION/'reports/runs'/run)
        return result.returncode
    except Exception as error:
        print('ROS detection failed:',error)
        return 2

if __name__=='__main__':raise SystemExit(main())
