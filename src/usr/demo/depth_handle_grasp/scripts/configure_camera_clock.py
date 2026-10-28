#!/usr/bin/env python3
"""Enable SDK host clock synchronization on an already-running RealSense ROS node."""
import argparse
import rospy
from dynamic_reconfigure.client import Client


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-namespace',default='/camera')
    args=parser.parse_args()
    rospy.init_node('configure_camera_clock',anonymous=True)
    for sensor in ('l500_depth_sensor','rgb_camera'):
        name=args.camera_namespace.rstrip('/')+'/'+sensor
        client=Client(name,timeout=5)
        before=client.get_configuration(timeout=3)
        if 'global_time_enabled' not in before:raise RuntimeError(name+' has no global_time_enabled option')
        after=client.update_configuration({'global_time_enabled':True})
        if after['global_time_enabled'] is not True:raise RuntimeError(name+' did not enable global time')
        print(name+': global_time_enabled '+str(before['global_time_enabled'])+' -> True')
    print('Clock option enabled. Allow a few seconds to settle, then verify detection. No robot commands sent.')

if __name__=='__main__':main()
