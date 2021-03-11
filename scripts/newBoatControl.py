#!/usr/bin/env python
import numpy as np
from pyquaternion import Quaternion
import controlpy
import scipy.integrate as integrate
from timeit import default_timer as timer
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from mavros_msgs.msg import  HippocampusControl,HippocampusDesired,AttitudeControlExt,HippocampusOutput
from pyquaternion import Quaternion
import math
class LQRThrustMixingController():
    def __init__(self):
        self.Inertia = np.array([0.002408, 0.010717, 0.010717])
        self.Inertia_added = np.array([0.00451, 0.0163, 0.0162])
        self.damping = np.array([0.0114, 0.070, 0.070])
        self.alphaUp=11e-2
        self.alphaDown = 27e-2
        self.alpha=(self.alphaUp+self.alphaDown) / 2
        self.controllerUpdateRate = 1./30.
        self.current_axis = np.array([1, 0, 0])
        self.desired_axis = np.array([0, 1, 0])
        self.fdes = 0.3
        self.gainMatrix = self.bodyRateLQR()
#---------------------------------------------------------
        self.current_omega=np.array([0,0,0])
        self.desired_omega=np.array([0,0,0])
        self.desired_omega_accel=np.array([0,0,0])
        self.torque_reference=np.array([0,0,0])
        self.torque_desired=np.array([0,0,0])
        self.torque_estimated=np.array([0,0,0])
        self.desired_collective_thrust= 0.0
        self.previous_thrust=np.array([0.0,0.0,0.0,0.0])
        self.singleMotorThrust= np.array([0.0,0.0,0.0,0.0])

    def mainControl(self,omega_desired,thrust_desired,current_omega):
        self.desired_omega=omega_desired
        self.current_omega=current_omega
        sT=thrust_desired /4.
        self.desired_collective_thrust=thrust_desired
        self.singleMotorThrust = np.array([sT,sT,sT,sT])
        #self.previous_thrust=self.singleMotorThrust
        #Calculate Nref
        self.calculateRefTorque()

        #Calculate Nestimated
        self.estimateTorque()
        #Calculate Ndesired
        self.calculateDesAngAccel()
        self.calculateDesiredTorque()
        self.solveforThrust()#
        return self.torque_desired, sum(self.previous_thrust)

    def calculateRefTorque(self):
        term = np.multiply(self.Inertia+self.Inertia_added, self.desired_omega)
        self.torque_reference = np.cross(self.desired_omega, term)+np.multiply(self.Inertia+self.Inertia_added, self.desired_omega_accel)
        #print("TorqueRef", self.desired_omega,term)

    def estimateTorque(self):
        f=self.previous_thrust
        #print("f", f)
        fdesi=self.desired_collective_thrust/4
        fdes = np.array([fdesi,fdesi,fdesi,fdesi])
        estimated_thrust =integrate.odeint(self.estimateThrust, f, [0,self.controllerUpdateRate],args=(fdes,) )

        #print("EstimateThrust", estimated_thrust[1])
        self.torque_estimated = self.solveforTorque(estimated_thrust[1])
        #print("EstTorques", self.torque_estimated)

    def calculateDesiredTorque(self):
        term = np.multiply(self.Inertia, self.current_omega)
        feedforward= np.cross(self.desired_omega, term)

        error_omega=self.desired_omega - self.current_omega
        error_torque = self.torque_reference - self.torque_estimated[0:3]

        state = np.concatenate((error_omega, error_torque), axis=0)
        self.torque_desired = self.gainMatrix.dot(state) + feedforward + np.multiply(self.Inertia+self.Inertia_added, self.desired_omega_accel)
        #print("TDes",self.torque_desired)



    def solveforTorque(self,estimatedthrusts):
        kappa1 = 0.02
        kappa2 = 0.02
        kappa3 = 0.02
        kappa4 = 0.02
        rotor_length = 0.01
        coefficient = (np.sqrt(2) / 2) * rotor_length
        collectiveThrust = 1
        m_list = [[coefficient, -coefficient, -coefficient, coefficient],
                  [-coefficient, -coefficient, coefficient, coefficient],
                  [kappa1, -kappa2, kappa3, -kappa4],
                  [1, 1, 1, 1]]
        Am = np.array(m_list)
        Bv = estimatedthrusts
        torques=Am.dot(Bv)
        return torques

    def solveforThrust(self):
        kappa1= 0.02
        kappa2= 0.02
        kappa3= 0.02
        kappa4= 0.02
        rotor_length= 0.01
        coefficient= (np.sqrt(2)/2) * rotor_length
        collectiveThrust= 1
        m_list = [[coefficient, -coefficient, -coefficient, coefficient],
                  [-coefficient, -coefficient, coefficient, coefficient],
                  [kappa1,  -kappa2, kappa3, -kappa4],
                  [1,  1, 1, 1]]
        Am = np.array(m_list)
        Bv = np.array([self.torque_desired[0], self.torque_desired[1],self.torque_desired[2],self.desired_collective_thrust ])
        singleMotorThrust=np.linalg.inv(Am).dot(Bv)

        self.previous_thrust[0]=  self.calculateThrust(singleMotorThrust[0])
        self.previous_thrust[1] = self.calculateThrust(singleMotorThrust[1])
        self.previous_thrust[2] = self.calculateThrust(singleMotorThrust[2])
        self.previous_thrust[3] = self.calculateThrust(singleMotorThrust[3])

        if singleMotorThrust[0] < 0:
            self.previous_thrust[0]=-1*self.previous_thrust[0]
        if singleMotorThrust[1] < 0:
            self.previous_thrust[0]=-1*self.previous_thrust[1]
        if singleMotorThrust[2] < 0:
            self.previous_thrust[0]=-1*self.previous_thrust[2]
        if singleMotorThrust[3] < 0:
            self.previous_thrust[0]=-1*self.previous_thrust[3]

    def calculateDesAngAccel(self):
        term = np.multiply(self.Inertia, self.desired_omega)
        term2 = np.cross(self.desired_omega, term)
        term3= np.multiply(self.damping, self.desired_omega)

        self.desired_omega_accel =-np.linalg.inv((np.diag(self.Inertia+self.Inertia_added))).dot(-term3-term2)
        #print("DesiredAccels", self.desired_omega_accel)



    def bodyRateLQR(self):
        inertia_matrix = np.diag(self.Inertia + self.Inertia_added)
        damping_matrix = np.diag(self.damping)
        torque_matrix=np.diag( np.array([(-1/self.alpha),(-1/self.alpha),(-1/self.alpha)]) )
        inertia_inv = np.linalg.inv(inertia_matrix)
        zeromatrix=np.zeros([3, 3])

        # print("Inertia INv", inertia_inv)
        upperPartA=np.concatenate((zeromatrix, inertia_inv), axis=1)
        lowerPartA=np.concatenate((zeromatrix, torque_matrix), axis=1)
        A = np.concatenate((upperPartA, lowerPartA), axis=0)

        B=np.concatenate((zeromatrix, torque_matrix), axis=0)

        # Define our costs:
        cost = np.array([1, 1, 1, 1, 1, 1])*0.1  # np.array([5,2,1])
        Q = np.diag(cost)
        R = np.eye(3)  # np.matrix([[0.0001]])

        # Compute the LQR controller
        gain, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(A, B, Q, R)
        #print("EigenVal", closedLoopEigVals)
        #print("Gain", gain)
        return gain

    def estimateThrust(self,f,t,fdes):
        dfdt = self.alphaDown * (fdes - f)
        return dfdt

    def calculateThrust(self, force):
        thrust=0.0
        #a = 8.00518438e-05
        #b = -2.07019915e-02
        #c = 1.13461592e+00 - force
        a=2.27874648e-05
        b=-7.26737492e-04
        c=.65633657e-01- force

        d = b ** 2 - 4 * a * c  # discriminant
        x1=0.0
        x2=0.0
        if d < 0:
            dummy=0
        elif d == 0:
            x1 = -b / (2 * a)
            #print 'The sole solution is', x1 / 500
        else:  # if d > 0
            x1 = (-b + math.sqrt(d)) / (2 * a)
            x2 = (-b - math.sqrt(d)) / (2 * a)
            #print 'Solutions are', x1 / 500, 'and', x2 / 500
        if x1 >0 and x1>x2:
            thrust= x1/500
        if x2 >0 and x2>x1:
            thrust = x2/500

        return thrust
#----------------------------------------------------------------------------------------------------------------------------------------------
def main():
    rospy.init_node('newbodyrate_control')
    print("Starting Body Rate Controller ... ")
    rate = rospy.Rate(30)
    control = LQRThrustMixingController()

    while not rospy.is_shutdown():
        #control.body_rate_control()
        #control.publishControlInputs()
        #control.publishControlInputs2()
        rate.sleep()

if __name__ == '__main__':
    main()