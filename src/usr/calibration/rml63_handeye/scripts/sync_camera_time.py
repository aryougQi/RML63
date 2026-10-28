#!/usr/bin/env python3
"""Enable the L515 SDK's global clock before taking hand-eye samples."""
import rospy
from dynamic_reconfigure.client import Client


def configure():
    namespace = rospy.get_param('~camera_namespace', '/camera').rstrip('/')
    for sensor in ['rgb_camera', 'l500_depth_sensor', 'motion_module']:
        name = namespace+'/'+sensor
        try:
            client = Client(name, timeout=10 if sensor == 'rgb_camera' else 2)
            config = client.get_configuration(timeout=3)
            if 'global_time_enabled' not in config:
                raise RuntimeError('global_time_enabled is not supported by '+name)
            if not config['global_time_enabled']:
                config = client.update_configuration({'global_time_enabled': True})
            if not config['global_time_enabled']:
                raise RuntimeError('SDK did not enable global time for '+name)
            rospy.loginfo('%s: global_time_enabled=true', name)
        except Exception as error:
            if sensor == 'rgb_camera':
                raise
            rospy.logwarn('Optional sensor time configuration skipped: %s', error)
    rospy.loginfo('L515 global clock configured. Start a fresh sample set after a clock-mode change.')


if __name__ == '__main__':
    rospy.init_node('sync_camera_time')
    try:
        configure()
        rospy.spin()
    except Exception as error:
        rospy.logfatal('Camera clock synchronization failed: %s', error)
        raise SystemExit(1)
