#! /usr/bin/env python3

import rospy
import time
import os
import _thread
from math import *
from flight_teleop.msg import FlightParameters, FlightTarget, FlightState

param_msg    = FlightParameters()
target_msg   = FlightTarget()
qc_state_msg = FlightState()

target_msg.header.seq = 0
target_input_open     = False

def qc_state_cb(state):
    global qc_state_msg
    qc_state_msg = state

def quat2eul(q0, q1, q2, q3):
    phi    = atan2(2*(q0*q1 + q2*q3), (1 - 2*(q1**2 + q2**2)))
    theta  = asin(2*(q0*q2 - q3*q1))
    psi    = atan2(2*(q0*q3 + q1*q2), (1 - 2*(q2**2 + q3**2)))
    return psi, theta, phi

def target_input():
    global target_msg, target_input_open
    print('\nEnter the target coordinates below: -\n')

    time.sleep(0.75)

    var_x             = float(input("Enter the target x-coordinate : "))
    var_y             = float(input("Enter the target y-coordinate : "))
    var_z             = float(input("Enter the target z-coordinate : "))
    # var_yaw           = float(input("Enter the target yaw angle    : "))

    target_msg.x      = var_x
    target_msg.y      = var_y
    target_msg.z      = var_z
    # target_msg.yaw    = var_yaw

    print('\nTarget sent')
    
    time.sleep(0.5)

    os.system('clear')

    target_input_open = False

if __name__ == "__main__":
    rospy.init_node("teleop_GUI")

    qc_state_sub = rospy.Subscriber("picopter/state", FlightState, callback = qc_state_cb)

    param_pub  = rospy.Publisher("picopter/parameters", FlightParameters, queue_size = 10)
    target_pub = rospy.Publisher("picopter/target", FlightTarget, queue_size = 10)
    
    rate = rospy.Rate(20)

    time.sleep(0.75)

    param_msg.m   = 0.75
    param_msg.g   = 9.81

    param_msg.Kpt = 0.1
    param_msg.Kdt = 0.1
    param_msg.Kit = 0.2
    param_msg.Kwt = 0.085*0.9

    param_msg.Kpx = 0.1
    param_msg.Kdx = 0.2

    param_msg.Kpy = param_msg.Kpx
    param_msg.Kdy = param_msg.Kdx

    param_msg.takeoff_clearance = 0.65

    param_pub.publish(param_msg)

    # target_msg.x               = float(input("Enter the target x-coordinate : "))
    # target_msg.y               = float(input("Enter the target y-coordinate : "))
    # target_msg.z               = float(input("Enter the target z-coordinate : "))
    # target_msg.yaw             = float(input("Enter the target yaw angle    : "))
    # target_msg.header.frame_id = "Target-Pose to Quadcopter"

    target_msg.x               = 0.0
    target_msg.y               = 0.0
    target_msg.z               = 0.75
    target_msg.yaw             = 0.0
    target_msg.header.frame_id = "Target-Pose to Quadcopter"

    target_pub.publish(target_msg)

    while not rospy.is_shutdown():

        psi, theta, phi = quat2eul(qc_state_msg.body_pose.orientation.w, qc_state_msg.body_pose.orientation.x, qc_state_msg.body_pose.orientation.y, qc_state_msg.body_pose.orientation.z)

        if target_input_open == False:
            target_input_open = True
            _thread.start_new_thread(target_input, ())

        target_msg.header.seq      = target_msg.header.seq + 1
        target_msg.header.stamp    = rospy.Time.now()

        param_pub.publish(param_msg)
        target_pub.publish(target_msg)

        # print(f'''\n\n\n\n\n\n\n
        # -------------------------------------------

        # Quadcopter State: -

        # X       : {round(qc_state_msg.body_pose.position.x, 3)}
        # Y       : {round(qc_state_msg.body_pose.position.y, 3)}
        # Z       : {round(qc_state_msg.body_pose.position.z, 3)}
        # dX      : {round(  qc_state_msg.body_rate.linear.x, 3)}
        # dY      : {round(  qc_state_msg.body_rate.linear.y, 3)}
        # dZ      : {round(  qc_state_msg.body_rate.linear.z, 3)}
        # Psi     : {round(                              psi, 3)}
        # Theta   : {round(                            theta, 3)}
        # Phi     : {round(                              phi, 3)}
        # dPsi    : {round( qc_state_msg.body_rate.angular.z, 3)}
        # dTheta  : {round( qc_state_msg.body_rate.angular.y, 3)}
        # dPhi    : {round( qc_state_msg.body_rate.angular.x, 3)}
        # Thrust  : {round(              qc_state_msg.thrust, 3)}
        # Mode    : {qc_state_msg.offboard_mode}
        # Flytime : {round(     qc_state_msg.flight_duration, 3)}

        # -------------------------------------------
        # ''')

        rate.sleep()
