#! /usr/bin/env python3

import rospy
import sympy as sp
import time
import os
import _thread
from math import *
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg   import State, AttitudeTarget, BatteryStatus
from mavros_msgs.srv   import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from flight_teleop.msg import FlightParameters, FlightTarget, FlightState

state_msg    = State()
battery_msg  = BatteryStatus()
param_msg    = FlightParameters()
target_msg   = FlightTarget()
attitude_msg = AttitudeTarget()
qc_state_msg = FlightState()

# param_msg.m   = 0.75
# param_msg.g   = 9.81

# param_msg.Kpt = 0.1
# param_msg.Kit = 0.2
# param_msg.Kdt = 0.1
# param_msg.Kwt = 0.085*0.9

# param_msg.Kpx = 0.1
# param_msg.Kix = 0.01
# param_msg.Kdx = 0.2
# param_msg.Kpy = param_msg.Kpx
# param_msg.Kiy = param_msg.Kix
# param_msg.Kdy = param_msg.Kdx

# param_msg.takeoff_clearance = 0.65

# target_msg.x = 0.0
# target_msg.y = 0.0
# target_msg.z = 0.5

Kb  = 1.0

Ix  = 0.0
Iy  = 0.0
Iz  = 0.0

takeoff_done            = False
start                   = False
qc_state_msg.header.seq = 0
got_battery             = False
angle_limit_indeg       = 5.0

t       = 0.0
t_1     = 0.0
x       = 0.0
x_1     = 0.0
y       = 0.0
y_1     = 0.0
z       = 0.0
z_1     = 0.0
Px      = 0.0
Px_1    = 0.0
Py      = 0.0
Py_1    = 0.0
Pz      = 0.0
Pz_1    = 0.0
Pgx     = 0.0
Pgx_1   = 0.0
Pgy     = 0.0
Pgy_1   = 0.0
Pgz     = 0.0
Pgz_1   = 0.0
psi     = 0.0
psi_1   = 0.0
theta   = 0.0
theta_1 = 0.0
phi     = 0.0
phi_1   = 0.0

dx      = 0.0
dy      = 0.0
dz      = 0.0
dPx     = 0.0
dPy     = 0.0
dPz     = 0.0
dPgx    = 0.0
dPgy    = 0.0
dPgz    = 0.0

dpsi    = 0.0
dtheta  = 0.0
dphi    = 0.0

def state_cb(state):
    global state_msg
    state_msg = state

def battery_cb(battery_voltage):
    global bat_voltage, Kb, got_battery, battery_matrix
    battery_msg = battery_voltage

    if got_battery == False:
        battery_matrix = battery_msg.voltage*sp.ones(30, 1)
        got_battery = True
    else:
        for i in range(29):
            battery_matrix[i] = battery_matrix[i + 1]
        battery_matrix[29] = battery_msg.voltage
    
    bat_voltage = sp.ones(1, 30)*battery_matrix*(1/30)
    Kb          = 2.84518 - 0.1631*bat_voltage[0]

def vicon_cb(vicon):
    global t, x, y, z, q0, q1, q2, q3
    t  = time.perf_counter()
    x  = vicon.transform.translation.x
    y  = vicon.transform.translation.y
    z  = vicon.transform.translation.z
    q0 = vicon.transform.rotation.w
    q1 = vicon.transform.rotation.x
    q2 = vicon.transform.rotation.y
    q3 = vicon.transform.rotation.z

def parameters_cb(parameters):
    global param_msg
    param_msg = parameters

def target_cb(target):
    global target_msg
    target_msg = target

def eul2quat(phi, theta, psi):
    roll, pitch, yaw = phi, theta, psi

    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
    
    return q0, q1, q2, q3

def fixed_to_body_frame(x, y, z, q0, q1, q2, q3, target_x, target_y, target_z):
    Tf = sp.Matrix([
        [(pow(q0, 2) + pow(q1, 2) - pow(q2, 2) - pow(q3, 2)), 2*(q1*q2 - q0*q3), 2*(q0*q2 + q1*q3), x],
        [2*(q1*q2 + q0*q3), (pow(q0, 2) - pow(q1, 2) + pow(q2, 2) - pow(q3, 2)), 2*(q2*q3 - q0*q1), y],
        [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), (pow(q0, 2) - pow(q1, 2) - pow(q2, 2) + pow(q3, 2)), z],
        [                     0,                     0,                     0,                      1]
    ])

    Point = Tf**-1*sp.Matrix([[target_x], [target_y], [target_z], [1]])

    phi   = atan2(2*(q0*q1 + q2*q3), (1 - 2*(q1**2 + q2**2)))
    theta = asin(2*(q0*q2 - q3*q1))
    psi   = atan2(2*(q0*q3 + q1*q2), (1 - 2*(q2**2 + q3**2)))

    return Point[0], Point[1], Point[2], psi, theta, phi

def frame_update():
    global Px, Py, Pz, Pgx, Pgy, Pgz, psi, theta, phi, dx, dy, dz, dPx, dPy, dPz, dPgx, dPgy, dPgz, dpsi, dtheta, dphi

    while not rospy.is_shutdown():
        Px, Py, Pz, psi, theta, phi = fixed_to_body_frame(x, y, z, q0, q1, q2, q3, target_msg.x, target_msg.y, target_msg.z)
        Pgx, Pgy, Pgz               = target_msg.x, target_msg.y, target_msg.z


        try:           
            dx   = (    (x - x_1)/(t - t_1))
            dy   = (    (y - y_1)/(t - t_1))
            dz   = (    (z - z_1)/(t - t_1))
            dPx  = (  (Px - Px_1)/(t - t_1))
            dPy  = (  (Py - Py_1)/(t - t_1))
            dPz  = (  (Pz - Pz_1)/(t - t_1))
            dPgx = ((Pgx - Pgx_1)/(t - t_1))
            dPgy = ((Pgy - Pgy_1)/(t - t_1))
            dPgz = ((Pgz - Pgz_1)/(t - t_1))
            
            dpsi   = (    (psi - psi_1)/(t - t_1))
            dtheta = ((theta - theta_1)/(t - t_1))
            dphi   = (    (phi - phi_1)/(t - t_1))

        except:
            dx   = dx
            dy   = dy
            dz   = dz
            dPx  = dPx
            dPy  = dPy
            dPz  = dPz
            dPgx = dPgx
            dPgy = dPgy
            dPgz = dPgz

            dpsi   = dpsi
            dtheta = dtheta
            dphi   = dphi

        t_1     = t
        x_1     = x
        y_1     = y
        z_1     = z
        Px_1    = Px
        Py_1    = Py
        Pz_1    = Pz
        Pgx_1   = Pgx
        Pgy_1   = Pgy
        Pgz_1   = Pgz
        psi_1   = psi
        theta_1 = theta
        phi_1   = phi
        
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("picopter")

    while True:
        try:
            node_up = rospy.wait_for_message("picopter/parameters", FlightParameters, timeout = 10)
            print('communication established to the ground station')
            break
        except:
            print('waiting for the response from ground station...')

    state_sub    = rospy.Subscriber("mavros/state", State, callback = state_cb)
    battery_sub  = rospy.Subscriber("mavros/battery", BatteryStatus, callback = battery_cb)
    vicon_sub    = rospy.Subscriber("vicon/picopter/picopter", TransformStamped, callback = vicon_cb)
    param_sub    = rospy.Subscriber("picopter/parameters", FlightParameters, callback = parameters_cb)
    target_sub   = rospy.Subscriber("picopter/target", FlightTarget, callback = target_cb)

    attitude_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 10)
    qc_state_pub = rospy.Publisher("picopter/state", FlightState, queue_size = 10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client   = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    rate = rospy.Rate(50)

    time.sleep(1.75)

    # frame_update()
    _thread.start_new_thread(frame_update, ())

    while(not rospy.is_shutdown() and not state_msg.connected):
        rate.sleep()

    attitude_msg.thrust      = 0.3
    attitude_msg.body_rate.x = 0.0
    attitude_msg.body_rate.y = 0.0
    attitude_msg.body_rate.z = 0.0

    for i in range(75):   
        if(rospy.is_shutdown()):
            break

        attitude_pub.publish(attitude_msg)
        rate.sleep()

    target_msg.x = 0.0
    target_msg.y = 0.0
    target_msg.z = 0.65  # param_msg.takeoff_clearance
    
    if start == False:
        start_t   = t
        start_x   = x
        start_y   = y
        start_z   = z
        start_psi = psi
        start     = True

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    last_t = time.perf_counter()

    while not rospy.is_shutdown():

        freq   = 1/(time.perf_counter() - last_t)           # comment these 3 lines
        last_t = time.perf_counter()
        # print(freq)

        if(state_msg.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()

        else:
            if(not state_msg.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        if takeoff_done == False:
            px, py, pz, psi, theta, phi = fixed_to_body_frame(x, y, z, q0, q1, q2, q3, start_x, start_y, start_z)

            Ix = Ix + param_msg.Kix*(px)*(1.0/freq)
            Iy = Iy + param_msg.Kiy*(py)*(1.0/freq)
            Iz = Iz + param_msg.Kit*(param_msg.takeoff_clearance - z)*(1.0/freq)

            if   Iz >  param_msg.Kwt*param_msg.m*param_msg.g*0.25:
                Iz =  param_msg.Kwt*param_msg.m*param_msg.g*0.25
            elif Iz < -param_msg.Kwt*param_msg.m*param_msg.g*0.25:
                Iz = -param_msg.Kwt*param_msg.m*param_msg.g*0.25
            else:
                Iz = Iz
            
            thrust = Kb*(param_msg.Kpt*(param_msg.takeoff_clearance - z) + param_msg.Kdt*(0 - dz) + Iz + param_msg.Kwt*param_msg.m*param_msg.g)
            yaw    = psi
            pitch  = param_msg.Kpx*px   + param_msg.Kdx*(dPx) + Ix
            roll   = -(param_msg.Kpy*py + param_msg.Kdy*(dPy) + Iy)

            q0_t, q1_t, q2_t, q3_t = eul2quat(roll, pitch, yaw)

            if   thrust > 1:
                thrust = 1
            if   pitch  < -angle_limit_indeg*(pi/180):
                pitch  = -angle_limit_indeg*(pi/180)
            elif pitch  >  angle_limit_indeg*(pi/180):
                pitch  =  angle_limit_indeg*(pi/180)
            else:
                pitch  = pitch
            if   roll   < -angle_limit_indeg*(pi/180):
                roll   = -angle_limit_indeg*(pi/180)
            elif roll   >  angle_limit_indeg*(pi/180):
                roll   =  angle_limit_indeg*(pi/180)
            else:
                roll   = roll

            attitude_msg.type_mask     = 7      #128
            attitude_msg.thrust        = thrust
            attitude_msg.orientation.w = q0_t
            attitude_msg.orientation.x = q1_t
            attitude_msg.orientation.y = q2_t
            attitude_msg.orientation.z = q3_t

            attitude_pub.publish(attitude_msg)

            qc_state_msg.quadcopter_id           = "Quadcopter 1"
            qc_state_msg.header.seq              = qc_state_msg.header.seq + 1
            qc_state_msg.header.stamp            = rospy.Time.now()
            qc_state_msg.header.frame_id         = "Quadcopter State in Local Coordinate System"
            qc_state_msg.flight_duration         = t - start_t
            qc_state_msg.offboard_mode           = "Takeoff"
            qc_state_msg.thrust                  = thrust
            qc_state_msg.body_pose.position.x    = x
            qc_state_msg.body_pose.position.y    = y
            qc_state_msg.body_pose.position.z    = z
            qc_state_msg.body_pose.orientation.w = q0
            qc_state_msg.body_pose.orientation.x = q1
            qc_state_msg.body_pose.orientation.y = q2
            qc_state_msg.body_pose.orientation.z = q3
            qc_state_msg.body_rate.linear.x      = dx
            qc_state_msg.body_rate.linear.y      = dy
            qc_state_msg.body_rate.linear.z      = dz
            qc_state_msg.body_rate.angular.x     = dphi
            qc_state_msg.body_rate.angular.y     = dtheta
            qc_state_msg.body_rate.angular.z     = dpsi

            qc_state_pub.publish(qc_state_msg)

            if (z >= (param_msg.takeoff_clearance - 0.015)) & (z <= (param_msg.takeoff_clearance + 0.015)) & (dz <= 0.1):
                takeoff_done = True

                target_msg.x = start_x
                target_msg.y = start_y
                target_msg.z = param_msg.takeoff_clearance

        else:
            Ix = Ix + param_msg.Kix*(Px)*(1.0/freq)
            Iy = Iy + param_msg.Kiy*(Py)*(1.0/freq)
            Iz = Iz + param_msg.Kit*(target_msg.z - z)*(1.0/freq)

            if   Iz >  param_msg.Kwt*param_msg.m*param_msg.g*0.25:
                Iz =  param_msg.Kwt*param_msg.m*param_msg.g*0.25
            elif Iz < -param_msg.Kwt*param_msg.m*param_msg.g*0.25:
                Iz = -param_msg.Kwt*param_msg.m*param_msg.g*0.25
            else:
                Iz = Iz
            
            thrust = Kb*(param_msg.Kpt*(target_msg.z - z) + param_msg.Kdt*(0 - dz) + Iz + param_msg.Kwt*param_msg.m*param_msg.g)
            yaw    = target_msg.yaw
            pitch  = param_msg.Kpx*Px   + param_msg.Kdx*(dPx) + Ix
            roll   = -(param_msg.Kpy*Py + param_msg.Kdy*(dPy) + Iy)

            q0_t, q1_t, q2_t, q3_t = eul2quat(roll, pitch, yaw)

            if   thrust > 1:
                thrust = 1
            if   pitch  < -angle_limit_indeg*(pi/180):
                pitch  = -angle_limit_indeg*(pi/180)
            elif pitch  >  angle_limit_indeg*(pi/180):
                pitch  =  angle_limit_indeg*(pi/180)
            else:
                pitch  = pitch
            if   roll   < -angle_limit_indeg*(pi/180):
                roll   = -angle_limit_indeg*(pi/180)
            elif roll   >  angle_limit_indeg*(pi/180):
                roll   =  angle_limit_indeg*(pi/180)
            else:
                roll   = roll

            attitude_msg.type_mask     = 7
            attitude_msg.thrust        = thrust
            attitude_msg.orientation.w = q0_t
            attitude_msg.orientation.x = q1_t
            attitude_msg.orientation.y = q2_t
            attitude_msg.orientation.z = q3_t

            attitude_pub.publish(attitude_msg)

            qc_state_msg.quadcopter_id           = "Quadcopter 1"
            qc_state_msg.header.seq              = qc_state_msg.header.seq + 1
            qc_state_msg.header.stamp            = rospy.Time.now()
            qc_state_msg.header.frame_id         = "Quadcopter State in Local Coordinate System"
            qc_state_msg.flight_duration         = t - start_t
            qc_state_msg.offboard_mode           = "Position Track"
            qc_state_msg.thrust                  = thrust
            qc_state_msg.body_pose.position.x    = x
            qc_state_msg.body_pose.position.y    = y
            qc_state_msg.body_pose.position.z    = z
            qc_state_msg.body_pose.orientation.w = q0
            qc_state_msg.body_pose.orientation.x = q1
            qc_state_msg.body_pose.orientation.y = q2
            qc_state_msg.body_pose.orientation.z = q3
            qc_state_msg.body_rate.linear.x      = dx
            qc_state_msg.body_rate.linear.y      = dy
            qc_state_msg.body_rate.linear.z      = dz
            qc_state_msg.body_rate.angular.x     = dphi
            qc_state_msg.body_rate.angular.y     = dtheta
            qc_state_msg.body_rate.angular.z     = dpsi

            qc_state_pub.publish(qc_state_msg)

        try:
            Iz_percent = Iz*100.0/thrust
        except:
            Iz_percent = 0

        print(f'''\n\n\n\n\n\n\n
        --------  Target Locked : {takeoff_done}  --------

        Quadcopter State: -

        X       : {round(           x, 3)}
        Y       : {round(           y, 3)}
        Z       : {round(           z, 3)}
        X_vel   : {round(          dx, 3)}
        Y_vel   : {round(          dy, 3)}
        Z_vel   : {round(          dz, 3)}
        Psi     : {round(         psi, 3)}
        Theta   : {round(       theta, 3)}
        Phi     : {round(         phi, 3)}
        Uptime  : {round( t - start_t, 3)}


        Target Coordinates: -

        X_g     : {round(target_msg.x, 3)}
        Y_g     : {round(target_msg.y, 3)}
        Z_g     : {round(target_msg.z, 3)}

        X_b     : {round(          Px, 3)}
        Y_b     : {round(          Py, 3)}
        Z_b     : {round(          Pz, 3)}


        Calculated Inputs: -

        Thrust  : {round(      thrust, 3)}
        Roll    : {round(        roll, 3)}
        Pitch   : {round(       pitch, 3)}
        Yaw     : {round(         yaw, 3)}

        Test Data: -

        Mass    : {round( param_msg.m, 3)}
        Iz      : {round(  Iz_percent, 3)}
        Kb      : {round(          Kb, 3)}

        -----------------------------------------
        ''')

        rate.sleep()