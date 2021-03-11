#!/usr/bin/env python
import numpy as np
import rospy
from pyquaternion import Quaternion
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mavros_msgs.msg import  HippocampusControl,HippocampusCurrentaxis
#from Utilities.utils import RPYToRot, RotToQuat, RotToRPY
import random

class BoatData:
    qx_180 = Quaternion(axis=[1, 0, 0], angle=np.pi)
    qz_90p = Quaternion(axis=[0, 0, 1], angle=np.pi / 2)
    q_rot = qz_90p * qx_180

    def __init__(self):
        self.position  = [0.0, 0.0, 0.0]
        self.velocity =  [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.velocity_rot = [0.0, 0.0, 0.0]
        self.acceleration_rot = [0.0, 0.0, 0.0]
        self.bodyAxis= [0.0, 0.0, 0.0]
        #self.RotMatrix = RPYToRot(0.0, 0.0, 0.0)
        self.quat_orientation = Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
        self.euler = [0.0, 0.0, 0.0]
        self.angular_velo = [0.0, 0.0, 0.0]
        self.angular_velo_rot = [0.0, 0.0, 0.0]
        self.accelMedianX = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.accelMedianY = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.accelMedianZ = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.veloMedianX = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.veloMedianY = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.veloMedianZ = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.filteredAccelValue=[0.0, 0.0, 0.0]
        self.filteredVeloValue = [0.0, 0.0, 0.0]

        self.camera_position=[0.0, 0.0, 0.0]
        self.camera_velocity= [0.0, 0.0, 0.0]
        self.camera_orientation = Quaternion(w=1.0,x=0.0,y=0.0,z=0.0)
        self.euler_camera = [0.0, 0.0, 0.0]



        #Publisher
        self.position_publish = rospy.Publisher('/uuv00/mavros/local_position/pose_NED2', PoseStamped, queue_size=1)
        self.velocity_publish = rospy.Publisher('/uuv00/mavros/local_position/velocity_localNED2', TwistStamped, queue_size=1)
        self.accel_publish = rospy.Publisher('/uuv00/mavros/imu/data_NED2', Imu, queue_size=1)
        self.angular_velocity_publish = rospy.Publisher('/uuv00/mavros/local_position/velocity_bodyNED2', TwistStamped, queue_size=1)
        self.axis_publish = rospy.Publisher('/hippocampus/currentaxis', HippocampusCurrentaxis, queue_size=1)
        #self.current_axis_pub = rospy.Publisher('/hippocampus/current_axis', HippocampusControl,
          #                                              queue_size=1)
        # Subscriber

        rospy.Subscriber("/uuv00/mavros/local_position/pose", PoseStamped, self.poseCallback)
        rospy.Subscriber("/uuv00/mavros/local_position/velocity_body", TwistStamped, self.angularVeloCallback)
        rospy.Subscriber("/uuv00/mavros/local_position/velocity_local", TwistStamped, self.veloCallback)
        rospy.Subscriber("/uuv00/mavros/imu/data", Imu, self.imuCallback)

        rospy.Subscriber("/uuv00/mavros/local_position/pose_NED", PoseStamped, self.cameraPoseCallback)
        rospy.Subscriber("/uuv00/estimated_twist", TwistStamped, self.cameraVeloCallback)

    def poseCallback(self, msg):
        #print("POSE CALLBACK", msg.pose.position.x)
        #print("POSE CALLBACK2", self.position)
        tmpQuat = Quaternion(w=msg.pose.orientation.w,
                                           x=msg.pose.orientation.x,
                                           y=msg.pose.orientation.y,
                                           z=msg.pose.orientation.z)

        orientation_quat_ned = self.q_rot * (tmpQuat * self.qx_180)
        euler_orientation = euler_from_quaternion(np.array([orientation_quat_ned.x,orientation_quat_ned.y,orientation_quat_ned.z,orientation_quat_ned.w]))
        self.quat_orientation = orientation_quat_ned #Orientation in NED as Quaternion

        self.euler = euler_orientation #Orientation in NED as Euler
        #Generate Random Errors if desired
        randx = random.uniform(-1.,1.) *0.1 *0
        randy = random.uniform(-1.,1.) *0.1 *0
        randz = random.uniform(-1.,1.) *0.1 *0
        randdistance=np.linalg.norm(np.array([randx,randy,randz]))
        #print("RandomErrors :",randx,randy,randz,randdistance)


        self.position = self.q_rot.rotate(np.array([msg.pose.position.x +  randx,
                                                    msg.pose.position.y + randy,
                                                    msg.pose.position.z + randz]))


        #Publish Transformed Position and Orientation to ROS
        NED = PoseStamped()
        NED.header = msg.header  #TIMESTAMP?
        NED.header.frame_id = 'global_tank'
        NED.pose.position.x = self.position[0]
        NED.pose.position.y = self.position[1]
        NED.pose.position.z = self.position[2]
        current_axis = self.getBodyAxisQuat()
        NED.pose.orientation.w = self.quat_orientation.w
        NED.pose.orientation.x = self.quat_orientation.x
        NED.pose.orientation.y = self.quat_orientation.y
        NED.pose.orientation.z = self.quat_orientation.z
        self.position_publish.publish(NED)
        #current_axis = self.normalize(self.quat_orientation.rotate(np.array([1, 0, 0])))
        currAxis = HippocampusCurrentaxis()
        currAxis.frame_stamp = rospy.Time.now()
        currAxis.axis_x = current_axis[0]
        currAxis.axis_y =current_axis[1]
        currAxis.axis_z = current_axis[2]

        self.position_publish.publish(NED)
        self.axis_publish.publish(currAxis)

    def veloCallback(self,velo):
        self.velocity[0] = velo.twist.linear.x
        self.velocity[1] = velo.twist.linear.y
        self.velocity[2] = velo.twist.linear.z
        randx = random.uniform(-1.,1.) * 0.1*0
        randy = random.uniform(-1.,1.) * 0.1*0
        randz = random.uniform(-1.,1.) * 0.1*0
        randdistance = np.linalg.norm(np.array([randx, randy, randz]))
        #print("RandomErrors Velo:", randx, randy, randz, randdistance)
        self.velocity_rot = self.q_rot.rotate(np.array([velo.twist.linear.x+randx, velo.twist.linear.y+randy, velo.twist.linear.z+randz]))
        #print("Velocity", self.velocity)
        self.veloMedianX = np.roll(self.veloMedianX, 1)
        self.veloMedianY = np.roll(self.veloMedianY, 1)
        self.veloMedianZ = np.roll(self.veloMedianZ, 1)

        self.veloMedianX[0] = self.velocity_rot[0]
        self.veloMedianY[1] = self.velocity_rot[1]
        self.veloMedianZ[2] = self.velocity_rot[2]

        self.filteredVeloValue[0] = (np.sum(self.veloMedianX) / len(self.veloMedianX))
        self.filteredVeloValue[1] = (np.sum(self.veloMedianY) / len(self.veloMedianY))
        self.filteredVeloValue[2] = (np.sum(self.veloMedianZ) / len(self.veloMedianZ))

        veloNED =TwistStamped()
        veloNED.header = velo.header  #TIMESTAMP?
        veloNED.header.frame_id = 'global_tank'
        veloNED.twist.linear.x = self.filteredVeloValue[0]
        veloNED.twist.linear.y = self.filteredVeloValue[1]
        veloNED.twist.linear.z = self.filteredVeloValue[2]
        self.velocity_publish.publish(veloNED)

    def imuCallback(self, imu):
        self.acceleration[0] = imu.linear_acceleration.x
        self.acceleration[1] = imu.linear_acceleration.y
        self.acceleration[2] = imu.linear_acceleration.z

        #print("Accel", self.acceleration)
        accel_temp = np.array([-imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z])
        #print("AccelRaw",accel_temp)
        accel_temp_trans = self.getQuaternionOrientation().rotate(accel_temp)
        accel_temp_trans[0]=  -accel_temp_trans[0]
        accel_temp_trans[1]=  -accel_temp_trans[1]
        accel_temp_trans[2] = -(accel_temp_trans[2] - 9.81)
        self.acceleration_rot = accel_temp_trans

        self.accelMedianX = np.roll(self.accelMedianX, 1)
        self.accelMedianY = np.roll(self.accelMedianY, 1)
        self.accelMedianZ = np.roll(self.accelMedianZ, 1)

        self.accelMedianX[0] = accel_temp_trans[0]
        self.accelMedianY[1] = accel_temp_trans[1]
        self.accelMedianZ[2] = accel_temp_trans[2]

        self.filteredAccelValue[0]= (np.sum(self.accelMedianX) / len(self.accelMedianX))
        self.filteredAccelValue[1] = (np.sum(self.accelMedianY) / len(self.accelMedianY))
        self.filteredAccelValue[2] = (np.sum(self.accelMedianZ) / len(self.accelMedianZ))
       # print(imu.linear_acceleration.x)
        #print("X",self.accelMedianX)
        #print("X Filtered", self.filteredAccelValue[0])
        accelNED=Imu()
        accelNED.header=imu.header
        accelNED.linear_acceleration.x=self.filteredAccelValue[0]
        accelNED.linear_acceleration.y = self.filteredAccelValue[1]
        accelNED.linear_acceleration.z = self.filteredAccelValue[2]
        self.accel_publish.publish(accelNED)

#test

    def angularVeloCallback(self, ang_velo):
        self.angular_velo[0] = ang_velo.twist.angular.x
        self.angular_velo[1] = ang_velo.twist.angular.y
        self.angular_velo[2] = ang_velo.twist.angular.z
        tmp_angular_velo_rot = np.array([ang_velo.twist.angular.x, ang_velo.twist.angular.y, ang_velo.twist.angular.z])
        self.angular_velo_rot = self.qx_180.rotate(tmp_angular_velo_rot)
        ang_veloNED = TwistStamped()
        ang_veloNED.header = ang_velo.header  # TIMESTAMP?
        ang_veloNED.header.frame_id = 'global_tank'
        ang_veloNED.twist.angular.x=  self.angular_velo_rot[0]
        ang_veloNED.twist.angular.y = self.angular_velo_rot[1]
        ang_veloNED.twist.angular.z = self.angular_velo_rot[2]
        self.angular_velocity_publish.publish(ang_veloNED)

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

    def cameraVeloCallback(self, camera_velocity):
        self.camera_velocity[0]=  camera_velocity.twist.linear.x
        self.camera_velocity[1]=  camera_velocity.twist.linear.y
        self.camera_velocity[2] = camera_velocity.twist.linear.z

        
    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v

        return v / norm
    def getFilteredVelo(self):
        return self.filteredVeloValue
    def getFilteredAccel(self):
        return self.filteredAccelValue

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity

    def getAcceleration(self):
        return self.acceleration

    def getVelocityRotated(self):
        return self.velocity_rot

    def getAccelerationRotated(self):
        return self.acceleration_rot

    def getAngularVelocityRotated(self):
        return self.angular_velo_rot

    def getQuaternionOrientation(self):
        return self.quat_orientation

    def getQuaternionOrientationCamera(self):
        return self.camera_orientation

    def getEulerOrientation(self):
        return self.euler

    def getEulerOrientationCamera(self):
        return self.euler_camera

    def getBodyAxisQuat(self):
        return self.normalize(self.quat_orientation.rotate(np.array([1, 0, 0])))

    def getBodyAxisQuatCamera(self):
        return self.normalize(self.camera_orientation.rotate(np.array([1, 0, 0])))

    def getPositionCamera(self):
        return self.camera_position

    def getVelocityCamera(self):
        return self.camera_velocity


def main():
    rospy.init_node('transform_data')
    rate = rospy.Rate(50)
    ros_data = BoatData()

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()

