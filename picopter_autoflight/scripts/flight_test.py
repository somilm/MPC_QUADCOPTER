#! /usr/bin/env python3

import rospy
import sympy as sp
import time
from math import *
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

def quat2eul(q0, q1, q2, q3):
    phi_   = atan2(2*(q0*q1 + q2*q3), (1 - 2*(q1**2 + q2**2)))
    theta_ = asin(2*(q0*q2 - q3*q1))
    psi_   = atan2(2*(q0*q3 + q1*q2), (1 - 2*(q2**2 + q3**2)))
    return phi_, theta_, psi_

def eul2quat(phi_, th_, psi_):
    roll, pitch, yaw = phi_, th_, psi_

    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);

    q0_ = cr * cp * cy + sr * sp * sy;
    q1_ = sr * cp * cy - cr * sp * sy;
    q2_ = cr * sp * cy + sr * cp * sy;
    q3_ = cr * cp * sy - sr * sp * cy;
    
    return q0_, q1_, q2_, q3_

current_state = State()

m   = 0.75
g   = 9.81

Iz  = 0.0

Kpt = 0.1
Kdt = 0.1
Kit = 0.2
Kwt = 0.085*0.9

# Kpz = 0.30
# Kdz = 0.212
Kpx = 0.1
Kdx = 0.2
Kpy = Kpx
Kdy = Kdx

target_x = 0.0
target_y = 0.0
target_z = 0.5

takeoff_done      = False
takeoff_clearance = 0.65
start             = False

var_list = ['t', 'x', 'y', 'z', 'Px', 'Py', 'Pz', 'psi', 'theta', 'phi']

for i in var_list:
    for j in range(5):
        globals()[i + f'_{j + 1}'] = 0.0

dx  = 0.0
dy  = 0.0
dz  = 0.0
dPx = 0.0
dPy = 0.0
dPz = 0.0

dpsi   = 0.0
dtheta = 0.0
dphi   = 0.0

def state_cb(state_msg):
    global current_state
    current_state = state_msg

def vicon_cb(vicon_msg):
    global t, x, y, z, q0, q1, q2, q3
    t  = time.perf_counter()
    x  = vicon_msg.transform.translation.x
    y  = vicon_msg.transform.translation.y
    z  = vicon_msg.transform.translation.z
    q0 = vicon_msg.transform.rotation.w
    q1 = vicon_msg.transform.rotation.x
    q2 = vicon_msg.transform.rotation.y
    q3 = vicon_msg.transform.rotation.z

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
    global Px, Py, Pz, psi, theta, phi, dx, dy, dz, dPx, dPy, dPz, dpsi, dtheta, dphi

    Px, Py, Pz, psi, theta, phi = fixed_to_body_frame(x, y, z, q0, q1, q2, q3, target_x, target_y, target_z)

    try:
        dx  = (5*(  (x - x_1)/(t - t_1)) + 4*(  (x_1 - x_2)/(t_1 - t_2)) + 3*(  (x_2 - x_3)/(t_2 - t_3)) + 2*(  (x_3 - x_4)/(t_3 - t_4)) + 1*(  (x_4 - x_5)/(t_4 - t_5)))/15
        dy  = (5*(  (y - y_1)/(t - t_1)) + 4*(  (y_1 - y_2)/(t_1 - t_2)) + 3*(  (y_2 - y_3)/(t_2 - t_3)) + 2*(  (y_3 - y_4)/(t_3 - t_4)) + 1*(  (y_4 - y_5)/(t_4 - t_5)))/15
        dz  = (5*(  (z - z_1)/(t - t_1)) + 4*(  (z_1 - z_2)/(t_1 - t_2)) + 3*(  (z_2 - z_3)/(t_2 - t_3)) + 2*(  (z_3 - z_4)/(t_3 - t_4)) + 1*(  (z_4 - z_5)/(t_4 - t_5)))/15
        dPx = (5*((Px - Px_1)/(t - t_1)) + 4*((Px_1 - Px_2)/(t_1 - t_2)) + 3*((Px_2 - Px_3)/(t_2 - t_3)) + 2*((Px_3 - Px_4)/(t_3 - t_4)) + 1*((Px_4 - Px_5)/(t_4 - t_5)))/15
        dPy = (5*((Py - Py_1)/(t - t_1)) + 4*((Py_1 - Py_2)/(t_1 - t_2)) + 3*((Py_2 - Py_3)/(t_2 - t_3)) + 2*((Py_3 - Py_4)/(t_3 - t_4)) + 1*((Py_4 - Py_5)/(t_4 - t_5)))/15
        dPz = (5*((Pz - Pz_1)/(t - t_1)) + 4*((Pz_1 - Pz_2)/(t_1 - t_2)) + 3*((Pz_2 - Pz_3)/(t_2 - t_3)) + 2*((Pz_3 - Pz_4)/(t_3 - t_4)) + 1*((Pz_4 - Pz_5)/(t_4 - t_5)))/15
        
        dpsi   = (5*(    (psi - psi_1)/(t - t_1)) + 4*(    (psi_1 - psi_2)/(t_1 - t_2)) + 3*(    (psi_2 - psi_3)/(t_2 - t_3)) + 2*(    (psi_3 - psi_4)/(t_3 - t_4)) + 1*(    (psi_4 - psi_5)/(t_4 - t_5)))/15
        dtheta = (5*((theta - theta_1)/(t - t_1)) + 4*((theta_1 - theta_2)/(t_1 - t_2)) + 3*((theta_2 - theta_3)/(t_2 - t_3)) + 2*((theta_3 - theta_4)/(t_3 - t_4)) + 1*((theta_4 - theta_5)/(t_4 - t_5)))/15
        dphi   = (5*(    (phi - phi_1)/(t - t_1)) + 4*(    (phi_1 - phi_2)/(t_1 - t_2)) + 3*(    (phi_2 - phi_3)/(t_2 - t_3)) + 2*(    (phi_3 - phi_4)/(t_3 - t_4)) + 1*(    (phi_4 - phi_5)/(t_4 - t_5)))/15

    except:
        dx  = dx
        dy  = dy
        dz  = dz
        dPx = dPx
        dPy = dPy
        dPz = dPz

        dpsi   = dpsi
        dtheta = dtheta
        dphi   = dphi

    for i in var_list:
        globals()[i + '_5'] = globals()[i + '_4']
        globals()[i + '_4'] = globals()[i + '_3']
        globals()[i + '_3'] = globals()[i + '_2']
        globals()[i + '_2'] = globals()[i + '_1']
        globals()[i + '_1'] = globals()[i]


if __name__ == "__main__":
    rospy.init_node("flight_test_py")

    state_sub    = rospy.Subscriber("mavros/state", State, callback = state_cb)
    vicon_sub    = rospy.Subscriber("vicon/picopter/picopter", TransformStamped, callback = vicon_cb)

    attitude_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size = 10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client   = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    rate = rospy.Rate(20)

    time.sleep(1.75)

    frame_update()

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    attitude_msg = AttitudeTarget()

    attitude_msg.thrust      = 0.3
    attitude_msg.body_rate.x = 0.0
    attitude_msg.body_rate.y = 0.0
    attitude_msg.body_rate.z = 0.0

    for i in range(75):   
        if(rospy.is_shutdown()):
            break

        attitude_pub.publish(attitude_msg)
        rate.sleep()

    # target_x = float(input("Enter the target x-coordinate : "))
    # target_y = float(input("Enter the target y-coordinate : "))
    # target_z = float(input("Enter the target z-coordinate : "))

    target_x = 0.0
    target_y = 0.0
    target_z = 0.65
    
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

    while(not rospy.is_shutdown()):
        frame_update()

        # if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        #     if(set_mode_client.call(offb_set_mode).mode_sent == True):
        #         rospy.loginfo("OFFBOARD enabled")
            
        #     last_req = rospy.Time.now()

        # else:
        #     if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
        #         if(arming_client.call(arm_cmd).success == True):
        #             rospy.loginfo("Vehicle armed")
            
        #         last_req = rospy.Time.now()

        if takeoff_done == False:
            px, py, pz, psi, theta, phi = fixed_to_body_frame(x, y, z, q0, q1, q2, q3, start_x, start_y, start_z)
            
            Iz = Iz + Kit*(0.5 - z)*(1.0/20.0)

            if   Iz >  Kwt*m*g*0.25:
                Iz =  Kwt*m*g*0.25
            elif Iz < -Kwt*m*g*0.25:
                Iz = -Kwt*m*g*0.25
            else:
                Iz = Iz
            
            thrust = Kpt*(takeoff_clearance - z) + Kdt*(0 - dz) + Iz + Kwt*m*g
            yaw    = psi
            pitch  = Kpx*px        + Kdx*(0 + dPx)
            roll   = -(Kpy*py      + Kdy*(0 + dPy))

            q0_t, q1_t, q2_t, q3_t = eul2quat(roll, pitch, yaw)

            if   thrust > 1:
                thrust = 1
            if   pitch  < -15*(pi/180):
                pitch  = -15*(pi/180)
            elif pitch  >  15*(pi/180):
                pitch  =  15*(pi/180)
            else:
                pitch  = pitch
            if   roll   < -15*(pi/180):
                roll   = -15*(pi/180)
            elif roll   >  15*(pi/180):
                roll   =  15*(pi/180)
            else:
                roll   = roll

            attitude_msg.type_mask     = 7 #128
            attitude_msg.thrust        = thrust

            attitude_msg.orientation.w = q0_t
            attitude_msg.orientation.x = q1_t
            attitude_msg.orientation.y = q2_t
            attitude_msg.orientation.z = q3_t

            attitude_pub.publish(attitude_msg)

            if (z >= (takeoff_clearance - 0.015)) & (z <= (takeoff_clearance + 0.015)) & (dz <= 0.1):
                takeoff_done = True

        else:
            Iz = Iz + Kit*(target_z - z)*(1.0/20.0)

            if   Iz >  Kwt*m*g*0.25:
                Iz =  Kwt*m*g*0.25
            elif Iz < -Kwt*m*g*0.25:
                Iz = -Kwt*m*g*0.25
            else:
                Iz = Iz
            
            thrust = Kpt*(target_z - z) + Kdt*(0 - dz) + Iz + Kwt*m*g
            yaw    = 0
            pitch  = Kpx*Px             + Kdx*(0 + dPx)
            roll   = -(Kpy*Py           + Kdy*(0 + dPy))

            q0_t, q1_t, q2_t, q3_t = eul2quat(roll, pitch, yaw)

            if   thrust > 1:
                thrust = 1
            if   pitch  < -15*(pi/180):
                pitch  = -15*(pi/180)
            elif pitch  >  15*(pi/180):
                pitch  =  15*(pi/180)
            else:
                pitch  = pitch
            if   roll   < -15*(pi/180):
                roll   = -15*(pi/180)
            elif roll   >  15*(pi/180):
                roll   =  15*(pi/180)
            else:
                roll   = roll

            attitude_msg.type_mask   = 7 #128
            attitude_msg.thrust      = thrust

            attitude_msg.orientation.w = q0_t
            attitude_msg.orientation.x = q1_t
            attitude_msg.orientation.y = q2_t
            attitude_msg.orientation.z = q3_t

            attitude_pub.publish(attitude_msg)

        print(f'''\n\n\n\n\n\n\n
        --------  Target Locked : {takeoff_done}  --------

        Quadcopter State: -

        X       : {round(x, 4)}
        Y       : {round(y, 4)}
        Z       : {round(z, 4)}
        X_vel   : {round(dx, 4)}
        Y_vel   : {round(dy, 4)}
        Z_vel   : {round(dz, 4)}
        Psi     : {round(psi, 4)}
        Theta   : {round(theta, 4)}
        Phi     : {round(phi, 4)}
        Uptime  : {round(t - start_t, 4)}


        Target Coordinates: -

        X_g     : {round(target_x, 4)}
        Y_g     : {round(target_y, 4)}
        Z_g     : {round(target_z, 4)}

        X_b     : {round(Px, 4)}
        Y_b     : {round(Py, 4)}
        Z_b     : {round(Pz, 4)}


        Calculated Inputs: -

        Thrust  : {round(thrust, 5)}
        Iz      : {round(Iz*100.0/thrust,2)}
        Roll    : {round(roll, 5)}
        Pitch   : {round(pitch, 5)}
        Yaw     : {round(yaw, 5)}

        -----------------------------------------
        ''')

        rate.sleep()
