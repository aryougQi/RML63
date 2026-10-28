#!/usr/bin/env python3
"""Publish saved candidate transforms to RViz, with no planning/model changes."""
import argparse
import json
from pathlib import Path
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from grasp_ros import pose_msg


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('candidates', type=Path)
    p.add_argument('--frame', default='base_link')
    a = p.parse_args(rospy.myargv()[1:])
    data = json.loads(a.candidates.read_text())
    rospy.init_node('publish_multigrasp_candidates')
    pub = rospy.Publisher('/depth_handle_grasp/candidates', PoseArray, queue_size=1, latch=True)
    markers = rospy.Publisher('/depth_handle_grasp/candidate_scores', MarkerArray, queue_size=1, latch=True)
    poses = PoseArray(); poses.header.frame_id = a.frame; poses.header.stamp = rospy.Time.now()
    array = MarkerArray(); scores = {s['id']: s for s in data['scores']}
    for c in data['candidates']:
        pose = pose_msg(np.asarray(c['tcp']))
        poses.poses.append(pose)
        marker = Marker(); marker.header = poses.header
        marker.ns = 'TCP_X_along_rod'; marker.id = c['id']; marker.type = Marker.ARROW
        marker.pose = pose; marker.scale.x = .035; marker.scale.y = .003; marker.scale.z = .005
        marker.color.a = 1.; marker.color.r = float(not scores[c['id']]['accepted'])
        marker.color.g = scores[c['id']]['score']
        array.markers.append(marker)
    pub.publish(poses); markers.publish(array)
    rospy.spin()

if __name__ == '__main__': main()
