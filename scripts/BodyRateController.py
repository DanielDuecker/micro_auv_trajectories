#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import  HippocampusControl,HippocampusDesired,AttitudeControlExt,HippocampusOutput,HippocampusCurrentaxis,HippocampusOutput_2,AttitudeTarget
from pyquaternion import Quaternion
import controlpy
import scipy.sparse as sparse
from tf.transformations import euler_from_quaternion, quaternion_from_euler, rotation_matrix, quaternion_from_matrix


class controller():
    def __init__(self):
        self.body_rate_k_p = np.array([250., 60., 60.]) #1000 80 80
        self.body_rate_k_d = np.array([10., 1., 1.])

        self.Inertia = np.array([0.002408, 0.010717, 0.010717])
        self.Inertia_added = np.array([0.00451, 0.0163, 0.0162])
        self.Mass_added=np.array([1.11, 2.80, 2.80])
        self.damping=np.array([0.0114,0.070,0.070])

        self.body_rate=[0.0, 0.0, 0.0]

        self.taus=np.array([0.,0.,0.])
        self.MAX_TORQUE = 10
        self.current_axis = np.array([1.0,0.0,0.0])
        self.desired_axis = np.array([1.0,0.0,0.0])
        self.desired_rates = np.array([0.0,0.0,0.0])
        self.desired_axis_body = np.array([1.0, 0.0, 0.0])
        self.desiredThrust=0.0
        self.p_pitch= 1.0
        self.p_yaw= 1.0
        self.p_roll=1.0
        self.KLQR = self.bodyRateLQR()
        factor = 2.5
        #self.gain = np.array([self.KLQR.item(0) * 0.7, self.KLQR.item(1) * 1.4, self.KLQR.item(2) * 1.4])#Gazebo
        self.gain=np.array([self.KLQR.item(0)* 0.3*factor,self.KLQR.item(1)*0.70*factor,self.KLQR.item(2)* 0.95*factor]) #tank
        #self.gain = np.array([self.KLQR.item(0) * 1.8, self.KLQR.item(1) * 1.2, self.KLQR.item(2) * 0.9])
        self.orientation =  Quaternion(w=1.0,
                             x=0.0,
                             y=0.0,
                             z=0.0)

        #Subscriber
        #rospy.Subscriber("/mavros/local_position/velocity_bodyNED2", TwistStamped, self.bodyRateCallback)
        #rospy.Subscriber("/mavros/local_position/pose_NED2", PoseStamped, self.orientationCallback)
        rospy.Subscriber("/uuv00/mavros/local_position/velocity_bodyNED2", TwistStamped, self.bodyRateCallback)
        #rospy.Subscriber("/mavros/local_position/pose_NED2", PoseStamped, self.orientationCallback)
       #New Firmware
        rospy.Subscriber("/uuv00/pose_px4", PoseStamped, self.orientationCallback)
        rospy.Subscriber("/hippocampus/desired", HippocampusDesired, self.desiredValuesCallback)
        self.control_pub2 = rospy.Publisher('hippocampus/control', HippocampusControl, queue_size=1)
        self.control_pub = rospy.Publisher('/uuv00/mavros/hippocampus/attitude_control_ext', AttitudeControlExt, queue_size=1)
        self.control_pub3 = rospy.Publisher('/uuv00/mavros/setpoint_raw/attitude', AttitudeTarget,
                                           queue_size=1)
        self.output_pub = rospy.Publisher('hippocampus/output', HippocampusOutput, queue_size=1)
        self.output_2_pub = rospy.Publisher('hippocampus/output_2', HippocampusOutput_2, queue_size=1)
    def publishControlInputs(self):
        hcc = AttitudeControlExt()
        hcc.header.stamp = rospy.Time.now()

        hcc.thrust = self.desiredThrust
        #gain = 100.0 *0.5
        hcc.roll = self.taus[0]
        hcc.pitch = -self.taus[1]
        hcc.yaw = -self.taus[2]
    #####################################################
        target = AttitudeTarget()
        target.header.stamp=rospy.Time.now()
        target.body_rate.x = self.taus[0]
        target.body_rate.y = -self.taus[1]
        target.body_rate.z = -self.taus[2]
        target.thrust = self.desiredThrust


#########################################################
        output = HippocampusOutput_2()
        output.frame_stamp = rospy.Time.now()
        output.des_rates.x = self.desired_rates[0]
        output.des_rates.y = self.desired_rates[1]
        output.des_rates.z = self.desired_rates[2]

        output.current_rates.x = self.body_rate[0]
        output.current_rates.y = self.body_rate[1]
        output.current_rates.z = self.body_rate[2]

        outputx = HippocampusOutput()
        outputx.frame_stamp = rospy.Time.now()
        outputx.current_orientation.x = self.current_axis[0]
        outputx.current_orientation.y = self.current_axis[1]
        outputx.current_orientation.z = self.current_axis[2]
        outputx.des_orientation.x = self.desired_axis[0]
        outputx.des_orientation.y = self.desired_axis[1]
        outputx.des_orientation.z = self.desired_axis[2]
        outputx.current_velocity.x =  self.desired_rates[0]
        outputx.current_velocity.y =  self.desired_rates[1]
        outputx.current_velocity.z =  self.desired_rates[2]

       # self.output_pub.publish(outputx)
       # self.output_2_pub.publish(output)
       # self.control_pub.publish(hcc)
        self.control_pub3.publish(target)

    def publishControlInputs2(self):
        hcc = HippocampusControl()
        #hcc.header.stamp = rospy.Time.now()

        hcc.thrust = self.desiredThrust
        gain = 100.0 *0.5
        hcc.roll_effort = self.taus[0]
        hcc.pitch_effort = self.taus[1]
        hcc.yaw_effort = self.taus[2]

       # output = HippocampusOutput()
       # output.frame_stamp = rospy.Time.now()
       # output.current_orientation.x = self.current_axis[0]
       # output.current_orientation.y = self.current_axis[1]
       # output.current_orientation.z = self.current_axis[2]
       # output.des_orientation.x = self.desired_axis[0]
       # output.des_orientation.y = self.desired_axis[1]
       # output.des_orientation.z = self.desired_axis[2]
       # output.current_velocity.x =  self.desired_rates[0]
       # output.current_velocity.y =  self.desired_rates[1]
        #output.current_velocity.z =  self.desired_rates[2]

        #self.output_pub.publish(output)
        self.control_pub2.publish(hcc)

    def desiredValuesCallback(self, desiredValues):
       # print("DesiredCallback")
        self.desiredThrust = desiredValues.thrust
        self.desired_axis =np.array([desiredValues.rollrate,desiredValues.pitchrate,desiredValues.yawrate])#check this

        #print("DesValues Callback",  self.desired_axis )

    def orientationCallback(self, orientation_message):
        #print("OrientCallback")
        tmpQuat = Quaternion(w=orientation_message.pose.orientation.w,
                             x=orientation_message.pose.orientation.x,
                             y=orientation_message.pose.orientation.y,
                             z=orientation_message.pose.orientation.z)
        self.orientation=tmpQuat
        self.desired_axis_body = self.orientation.inverse.rotate(self.desired_axis)
        self.current_axis=self.normalize(self.orientation.rotate(np.array([1, 0, 0])))

       # print("Controller Current Axis : ", self.current_axis)#check this


    def bodyRateCallback(self, body_rate_message):
        self.body_rate[0] = body_rate_message.twist.angular.x
        self.body_rate[1] = body_rate_message.twist.angular.y
        self.body_rate[2] = body_rate_message.twist.angular.z #check these
        #print("Controller Body Rates : ", self.body_rate)
        #print("BodyRate")

    def body_rate_control(self):
        #print("BodyRateControl")
        angle_between_axes = np.arccos(np.dot(self.current_axis, self.desired_axis))
        normal = self.normalize(np.cross(self.current_axis, self.desired_axis))
        normal_body = self.orientation.inverse.rotate(normal)
        quaternionxyz= normal_body.dot(np.sin(angle_between_axes*0.5))
        error_quaternion = Quaternion (w=np.cos(angle_between_axes*0.5),x=quaternionxyz[0],y=quaternionxyz[1],z=quaternionxyz[2])
        pitch_desired=0
        yaw_desired=0

        #print("ErrorQuaternion", error_quaternion)
        if error_quaternion.elements[0] >= 0.0:

            pitch_desired= 2* self.p_pitch * error_quaternion.elements[2]
            yaw_desired  = 2* self.p_yaw * error_quaternion.elements[3]
        if error_quaternion.elements[0] < 0.0:

            pitch_desired = -2 * self.p_pitch * error_quaternion.elements[2]
            yaw_desired = -2 * self.p_yaw * error_quaternion.elements[3]


        roll_desired = self.rollRateCalc(error_quaternion)
        body_rate_des = np.array([roll_desired,pitch_desired,yaw_desired])
       # body_rate_des = np.array([0.0, 0.0,  0.5])

        self.desired_rates =  body_rate_des

        term= np.multiply(self.Inertia,self.body_rate)
        feedForward = np.cross(self.body_rate, term)
        feedForward2= np.multiply(self.damping,self.body_rate)

        control_error = self.Inertia * np.multiply(self.gain, (body_rate_des - self.body_rate))
        self.taus =  control_error + feedForward #+ feedForward2
        #print("Error : ", round(control_error[2],4))
        #print("FF_J : ",    round(feedForward[2],4))
        #print("FF_D : ", round(feedForward2[2], 4))
        taus_mod = np.linalg.norm(self.taus)
        #return self.taus, body_rate_des


    def rollRateCalc(self,errorQuat):
        desired_roll_angle = 0.0
        z_inter= np.array([0.0, -np.sin(desired_roll_angle),np.cos(desired_roll_angle)])


        ex_desired = self.normalize(self.desired_axis)
        ey_desired = self.normalize(np.cross(z_inter, ex_desired))
        ez_desired = self.normalize(np.cross(ex_desired, ey_desired))
        R = np.matrix([ex_desired,ey_desired,ez_desired]).T
        #print("DesiredAxis",self.desired_axis)
       # print("Matrix", R)

        quat_des = Quaternion(matrix=R)
        #print("Quat_Des", quat_des)
        q_rotate_roll = (self.orientation*errorQuat).inverse * quat_des

        roll_rate = 0.0
        if q_rotate_roll.elements[0] >= 0.0:
            roll_rate = 2 * self.p_roll * q_rotate_roll.elements[1]
        if q_rotate_roll.elements[0] < 0.0:
            roll_rate = -2 * self.p_roll * q_rotate_roll.elements[1]


        return roll_rate

    def bodyRateLQR(self):
        inertia_matrix=np.diag(self.Inertia + self.Inertia_added)
        damping_matrix =np.diag(self.damping)
        inertia_inv= np.linalg.inv(inertia_matrix)
        #print("Inertia INv", inertia_inv)
        A = -(inertia_inv.dot(damping_matrix))#np.matrix([[0, 1,1], [0, 0,1],[0, 0,1]])
        B = np.matrix([[1], [1],[1]])
        # Define our costs:
        cost = np.array([4,1,1])   #np.array([5,2,1])
        Q = np.diag(cost)
        R = np.matrix([[0.0001]]) #np.matrix([[0.0001]])

        # Compute the LQR controller
        gain, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(A, B, Q, R)
        #print("EigenVal", closedLoopEigVals)
        print("Gain", gain.T)
        return gain

    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v

        return v / norm


def main():
    rospy.init_node('bodyrate_control')
    print("Starting Body Rate Controller ... ")
    rate = rospy.Rate(30)
    control = controller()
    rospy.sleep(5.)
    while not rospy.is_shutdown():
        control.body_rate_control()
        control.publishControlInputs()
        #control.publishControlInputs2()
        rate.sleep()

if __name__ == '__main__':
    main()
