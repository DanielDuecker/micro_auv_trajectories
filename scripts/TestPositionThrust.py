#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import  HippocampusControl,HippocampusDesired
from tf.transformations import euler_from_quaternion, quaternion_from_euler, unit_vector
from pyquaternion import Quaternion
import controlpy
class test_bodyrate():
    def __init__(self):
        self.start = True
        self.desiredAxis = np.array([1.0, 0.0,0.0])
        self.current_axis = np.array([1.0, 0.0, 0.0])
        self.desiredThrust = 0.0
        self.switch=1.0
        self.goal_position = np.array([2.0, 1.0,0.5])
        self.gazebo_position = [0.0, 0.0, 0.0]
        self.camera_position=[0.0, 0.0, 0.0]
        self.camera_orientation = Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
        self.euler_camera = [0.0, 0.0, 0.0]
        # Subscriber
        #rospy.Subscriber("/uuv00/mavros/local_position/velocity_bodyNED2", TwistStamped, self.bodyRateCallback)
        #rospy.Subscriber("/uuv00/mavros/local_position/pose_NED2", PoseStamped, self.orientationCallback)
        rospy.Subscriber("/uuv00/pose_px4", PoseStamped, self.orientationCallback)
        #rospy.Subscriber("/uuv00/mavros/local_position/pose_NED", PoseStamped, self.cameraPoseCallback)
        #rospy.Subscriber("/estimated_twist", TwistStamped, self.cameraVeloCallback)
        self.desired_pub = rospy.Publisher("/hippocampus/desired", HippocampusDesired, queue_size=1)

    def cameraPoseCallback(self, camera_pose):
        self.camera_position[0] = camera_pose.pose.position.x
        self.camera_position[1] = camera_pose.pose.position.y
        self.camera_position[2] = camera_pose.pose.position.z
        self.camera_orientation = Quaternion(w=camera_pose.pose.orientation.w,
                                             x=camera_pose.pose.orientation.x,
                                             y=camera_pose.pose.orientation.y,
                                             z=camera_pose.pose.orientation.z)
        self.euler_camera = euler_from_quaternion(np.array([camera_pose.pose.orientation.x,camera_pose.pose.orientation.y,
                                                            camera_pose.pose.orientation.z,camera_pose.pose.orientation.w]))
    def orientationCallback(self, orientation_message):
        #print("OrientCallback")
        self.gazebo_position[0] = orientation_message.pose.position.x
        self.gazebo_position[1] = orientation_message.pose.position.y
        self.gazebo_position[2] = orientation_message.pose.position.z
        tmpQuat = Quaternion(w=orientation_message.pose.orientation.w,
                             x=orientation_message.pose.orientation.x,
                             y=orientation_message.pose.orientation.y,
                             z=orientation_message.pose.orientation.z)
        self.orientation = tmpQuat
        self.current_axis = self.normalize(self.orientation.rotate(np.array([1, 0, 0])))

    def goToPosition(self):
        print("Move To Start")
        print("GoalPosition:",self.goal_position)
        self.desiredAxis = unit_vector(np.subtract(self.goal_position, self.gazebo_position))
        self.desiredThrust = 0.08

    def publishDesiredValues(self):
        # 2222print("Publish Data :", self.desiredAxis)
        hdes = HippocampusDesired()
        hdes.frame_stamp = rospy.Time.now()

        hdes.thrust = self.desiredThrust
        hdes.rollrate = self.desiredAxis[0]
        hdes.pitchrate = self.desiredAxis[1]
        hdes.yawrate = self.desiredAxis[2]
        self.desired_pub.publish(hdes)

    def testGoalReached(self):

        if np.linalg.norm(np.subtract(self.goal_position, self.gazebo_position)) <= 0.3:
            print("Goal Reached")
            self.desiredThrust = 0.0
            self.switch = -self.switch
            if self.switch <0:
                self.goal_position = np.array([2.0, 0.6,0.5])
            if self.switch > 0:
                self.goal_position = np.array([0.5, 0.5, 0.5])
            self.desiredAxis = np.array([1.0, 0.0, 0.0])
            self.publishDesiredValues()
            rospy.sleep(2.0)

    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v

        return v / norm
def main():
    rospy.init_node('test_positionthrust')
    rate = rospy.Rate(30)
    test = test_bodyrate()

    while not rospy.is_shutdown():
        test.goToPosition()
        test.publishDesiredValues()
        test.testGoalReached()
        rate.sleep()

if __name__ == '__main__':
    main()