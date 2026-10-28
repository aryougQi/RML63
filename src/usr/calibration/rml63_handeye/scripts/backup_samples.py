#!/usr/bin/env python3
"""Back up manual GUI samples by reading the list service; never takes samples or moves."""
import datetime
from pathlib import Path

import rospy
from easy_handeye_msgs.srv import TakeSample


def main():
    rospy.init_node('rml63_manual_sample_backup')
    namespace = rospy.get_param('~calibration_namespace').rstrip('/')
    service = namespace + '/get_sample_list'
    directory = Path.home() / '.ros/rml63_handeye_manual_runs' / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    (directory / 'namespace.txt').write_text(namespace + '\n')
    rospy.loginfo('Manual sample backup: %s', directory)
    rospy.wait_for_service(service)
    get_samples = rospy.ServiceProxy(service, TakeSample)
    previous = None
    while not rospy.is_shutdown():
        try:
            samples = get_samples().samples
            content = str(samples)
            if content != previous:
                # Replace atomically, keeping the last complete list on interruption.
                temporary = directory / 'samples.yaml.tmp'
                temporary.write_text(content)
                temporary.replace(directory / 'samples.yaml')
                previous = content
                rospy.loginfo('Manual samples backed up: %d', len(samples.hand_world_samples))
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5, 'Manual sample backup waiting for server: %s', exc)
        rospy.sleep(.5)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
