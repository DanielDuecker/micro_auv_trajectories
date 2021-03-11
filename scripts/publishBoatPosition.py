#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

publisher_marker = rospy.Publisher('/pose_boat_NED', Marker, queue_size=1)

def callback(msg):
    """"""
    global rate

    marker = Marker()
    marker.header.frame_id = "global_tank"
    marker.id = 0
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1  # transparency
    marker.pose.orientation.w = msg.pose.orientation.w
    marker.pose.orientation.x = msg.pose.orientation.x
    marker.pose.orientation.y = msg.pose.orientation.y
    marker.pose.orientation.z = msg.pose.orientation.z
    marker.pose.position.x = msg.pose.position.x # x
    marker.pose.position.y = msg.pose.position.y # y
    marker.pose.position.z = msg.pose.position.z # z
   # marker.mesh_resource = "package://hippocampus_test/models/uuv_hippocampus.stl"
    marker.mesh_resource = "package://hippocampus_common/models/uuv_hippocampus.stl"
    publisher_marker.publish(marker)



def main():
    rospy.init_node('rviz_boatpose')
    global rate
    rate = rospy.Rate(30)

    rospy.Subscriber("/uuv00/pose_px4", PoseStamped, callback)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()