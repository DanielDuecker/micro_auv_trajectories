#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import  HippocampusControl,HippocampusDesired
from pyquaternion import Quaternion
import controlpy
class test_bodyrate():
    def __init__(self):
        self.start = True
        self.desiredAxis = np.array([0.0,1.0,0.0])
        self.current_axis = np.array([1.0, 0.0, 0.0])
        # Subscriber
        #rospy.Subscriber("/uuv00/mavros/local_position/velocity_bodyNED2", TwistStamped, self.bodyRateCallback)
        #rospy.Subscriber("uuv00/mavros/local_position/pose_NED2", PoseStamped, self.orientationCallback)
        #rospy.Subscriber("/mavros/local_position/pose_NED2", PoseStamped, self.orientationCallback)
        rospy.Subscriber("/uuv00/pose_px4", PoseStamped, self.orientationCallback)
        self.desired_pub = rospy.Publisher("/hippocampus/desired", HippocampusDesired, queue_size=1)

    def orientationCallback(self, orientation_message):
        #print("OrientCallback")
        tmpQuat = Quaternion(w=orientation_message.pose.orientation.w,
                             x=orientation_message.pose.orientation.x,
                             y=orientation_message.pose.orientation.y,
                             z=orientation_message.pose.orientation.z)
        self.orientation = tmpQuat
        self.current_axis = self.normalize(self.orientation.rotate(np.array([1, 0, 0])))
        #print("Test Current Axis : ",self.current_axis)

   # def bodyRateCallback(self, body_rate_message):
       # print("Yo")

    def publishDesiredValues(self):
        #print("Publish Data :", self.desiredAxis)
        hdes = HippocampusDesired()
        hdes.frame_stamp = rospy.Time.now()

        hdes.thrust = 0.0
        hdes.rollrate = self.desiredAxis[0]
        hdes.pitchrate = self.desiredAxis[1]
        hdes.yawrate = self.desiredAxis[2]
        self.desired_pub.publish(hdes)

    def testGoalReached(self):
       # print("Subtract", self.current_axis)
        if np.linalg.norm(np.subtract(self.desiredAxis, self.current_axis)) <= 0.2:
            self.desiredAxis = -self.desiredAxis
            print("GOAL REACHED+")
            rospy.sleep(2.0)

    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v

        return v / norm
def main():
    rospy.init_node('test_bodyrate')
    rate = rospy.Rate(30)
    test = test_bodyrate()

    while not rospy.is_shutdown():

        test.publishDesiredValues()
        test.testGoalReached()
        rate.sleep()

if __name__ == '__main__':
    main()
