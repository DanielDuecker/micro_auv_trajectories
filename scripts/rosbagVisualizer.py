#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import  HippocampusControl,HippocampusDesired,AttitudeControlExt,HippocampusOutput
class rosbag:

    def __init__(self):
        self.boatposition  = [0.0, 0.0, 0.0]
        self.desiredposition = [0.0, 0.0, 0.0]
        self.desiredvelocity = [0.0, 0.0, 0.0]
        self.quat_orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self.goal=[0.5, 1.0, 0.5]
        self.start=[2.8,1.0,0.5]
        self.desiredthrust = 0.0
        self.evalTime=0.0
        self.counter=0.0
        rospy.Subscriber("/uuv00/pose_px4", PoseStamped, self.poseCallback)
        rospy.Subscriber("hippocampus/output", HippocampusOutput, self.output_callback)
        self.desired_pub = rospy.Publisher("/hippocampus/desired", HippocampusDesired, queue_size=1)
        self.publisher_marker = rospy.Publisher('/pose_boat_NED', Marker, queue_size=1)
        self.publish_goal = rospy.Publisher('/goal_visualization', Marker, queue_size=1)
        self.publisher_trajectory = rospy.Publisher('/traject_point', Marker, queue_size=1)
        self.publisher_position = rospy.Publisher('/driven_point', Marker, queue_size=1)
        self.data_info = rospy.Publisher('/data_info', MarkerArray, queue_size=1)
        self.publish_start = rospy.Publisher('/start', Marker, queue_size=1)

    def poseCallback(self,pose):
        print("poseCallback")
        self.boatposition[0] = pose.pose.position.x
        self.boatposition[1] = pose.pose.position.y
        self.boatposition[2] = pose.pose.position.z
        self.quat_orientation =pose.pose.orientation

    def output_callback(self, output):
        print("outputCallback")
        self.desiredposition[0] = output.des_position.x
        self.desiredposition[1] = output.des_position.y
        self.desiredposition[2] = output.des_position.z
        self.desiredvelocity[0] = output.des_velocity.x
        self.desiredvelocity[1] = output.des_velocity.y
        self.desiredvelocity[2] = output.des_velocity.z

        self.desiredthrust = output.thrust_time.x
        self.evalTime = output.thrust_time.y



    def publishPose(self):
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
        marker.pose.orientation.w = self.quat_orientation.w
        marker.pose.orientation.x = self.quat_orientation.x
        marker.pose.orientation.y = self.quat_orientation.y
        marker.pose.orientation.z = self.quat_orientation.z
        marker.pose.position.x = self.boatposition[0]  # x
        marker.pose.position.y = self.boatposition[1]  # y
        marker.pose.position.z = self.boatposition[2] # z
        marker.mesh_resource = "package://hippocampus_test/models/uuv_hippocampus.stl"
        self.publisher_marker.publish(marker)
    def publishGoal(self):
        goal_pose = Marker()
        r = 0.2
        goal_pose.header.frame_id = "global_tank"
        goal_pose.id = 0
        goal_pose.type = goal_pose.CYLINDER
        goal_pose.action = goal_pose.ADD
        goal_pose.scale.x = r
        goal_pose.scale.y = r
        goal_pose.scale.z = 0.01
        goal_pose.color.r = 1.0
        goal_pose.color.g = 1.0
        goal_pose.color.b = 0.6
        goal_pose.color.a = 0.5
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.7071068
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.7071068
        goal_pose.pose.position.x = self.goal[0]
        goal_pose.pose.position.y = self.goal[1]
        goal_pose.pose.position.z = self.goal[2]
        self.publish_goal.publish(goal_pose)

    def publishStart(self):
        start_pose = Marker()
        r = 0.1
        start_pose.header.frame_id = "global_tank"
        start_pose.id = 0
        start_pose.type = start_pose.SPHERE
        start_pose.action = start_pose.ADD
        start_pose.scale.x = r
        start_pose.scale.y = r
        start_pose.scale.z = r
        start_pose.color.r = 1.0
        start_pose.color.g = 1.0
        start_pose.color.b = 0
        start_pose.color.a = 0.5
        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.0
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 1.0
        start_pose.pose.position.x = self.start[0]
        start_pose.pose.position.y = self.start[1]
        start_pose.pose.position.z = self.start[2]
        self.publish_start.publish(start_pose)
    def publish_position(self):
        current_pose = Marker()
        r = 0.1
        current_pose.header.frame_id = "global_tank"
        current_pose.id = self.counter +1.0
        current_pose.type = current_pose.SPHERE
        current_pose.action = current_pose.ADD
        current_pose.scale.x = r
        current_pose.scale.y = r
        current_pose.scale.z = r
        current_pose.color.r = 1.6
        current_pose.color.g = 1.0
        current_pose.color.b = 1.0
        current_pose.color.a = 0.5
        current_pose.pose.orientation.w = 1.0
        current_pose.pose.position.x = self.boatposition[0]
        current_pose.pose.position.y = self.boatposition[1]
        current_pose.pose.position.z = self.boatposition[2]
        self.publisher_position.publish(current_pose)


    def publish_trajectory(self):
        desired_pose = Marker()
        r = 0.1
        desired_pose.header.frame_id = "global_tank"
        desired_pose.id = self.counter +1.0
        desired_pose.type = desired_pose.SPHERE
        desired_pose.action = desired_pose.ADD
        desired_pose.scale.x = r
        desired_pose.scale.y = r
        desired_pose.scale.z = r
        desired_pose.color.r = 1.0
        desired_pose.color.g = 1.0
        desired_pose.color.b = 1.6
        desired_pose.color.a = 0.5
        desired_pose.pose.orientation.w = 1.0
        desired_pose.pose.position.x = self.desiredposition[0]
        desired_pose.pose.position.y = self.desiredposition[1]
        desired_pose.pose.position.z = self.desiredposition[2]
        self.publisher_trajectory.publish(desired_pose)
    def publishDataText(self):
        data_info = MarkerArray()
        for i in range(3):
            marker = Marker()
            marker.header.frame_id = "global_tank"
            marker.id = i
            # print("MarkerID", marker.id)
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.ns ="basic_shapes"
            marker.scale.x = 0.3  # r*2 of distance to camera from tag_14
            marker.scale.y = 0.3
            marker.scale.z = 0.1
            marker.color.b = 0.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.a = 1  # transparency
            #if id==1:
                #marker.text = str(self.desiredthrust)
            #if id ==2:
                #marker.text = str(self.desiredvelocity[0])
            #else:
            marker.text = str(self.desiredthrust)

            marker.pose.orientation.w = 1.0
            marker.pose.position.x = 0.0 # x
            marker.pose.position.y = 2.0  # y
            marker.pose.position.z = 1.0- (i*0.15)  # z
            data_info.markers.append(marker)
        self.data_info.publish(data_info)

    def main_function(self):
        print("main")
        self.publishPose()
        self.publishGoal()
        self.publishStart()
        self.publish_position()
        self.publish_trajectory()
        self.publishDataText()


def main():
    rospy.init_node('rosbag_visual')
    rate = rospy.Rate(50)
    rb=rosbag()
    while not rospy.is_shutdown():
        rb.main_function()
        rate.sleep()

if __name__ == '__main__':
    main()