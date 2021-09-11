#!/usr/bin/env python
##TODO: Fill in the code. You will have to add your node in the launch file as well. The launch file is in cw3_launch.
import rospy
import rosbag
import rospkg
import sys
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from cw3q2.iiwa14Kine import iiwa14_kinematic
from kdl_kine.kdl_kine_solver_iiwa14 import iiwa14KDL

iiwa = iiwa14_kinematic()
kdl_iiwa = iiwa14KDL(2)   ## prefix == 2

## Same method as q5a to change a matrix function
def kdl_change_matrix_function(mat):
    M = np.array(np.zeros((mat.rows(), mat.columns())))
    for i in range(mat.rows()):
        for j in range(mat.columns()):
            M[i,j] = mat[i,j]

    return M
## Same method as q5a to change a array function
def kdl_change_array_function(array):
    arr = []
    for i in array:
        arr.append(i)

    return np.array(arr)

def callback_function(msg):
    t = rospy.get_time()  # time
    position = msg.position
    tau = msg.effort
    G = kdl_change_array_function(kdl_iiwa.getG(position))  # G from dynamic equation
    external_torque = tau - G
    if rospy.get_time() > 21 and rospy.get_time() < 22:
        A = np.append((external_torque), t)  # computes the results between those times
        keep_result_A.append(A)
        renew_position = np.append(position, t)
        keep_renew_result_A.append(renew_position)

    if rospy.get_time() > 41 and rospy.get_time() < 42:
        A = np.append((external_torque), t)
        keep_result_A.append(A)
        renew_position = np.append(position, t)
        keep_renew_result_A.append(renew_position)

    if rospy.get_time() > 61 and rospy.get_time() < 62:
        A = np.append((external_torque), t)
        keep_result_A.append(A)
        renew_position = np.append(position, t)
        keep_renew_result_A.append(renew_position)

    if rospy.get_time() > 64.5: # time until movement starts
        rospy.signal_shutdown('Stop movement')

def calculation_three_configuration(val, pos):
    configurations = np.array([[45, -35, 55, -30, -25, 65, -10],  ## q_a
                               [5, 40, 80, 10, 8, -50, -10],      ## q_b  
                               [-50, 60, 0, -15, 60, -25, -50]])  ## q_c
    ## Initialise
    g = -iiwa.g
    mass = np.zeros(3)
    translation = np.zeros([3, 3])
    for i in range(3):   ## Average mass and centre of mass
        config_a = []   ## Save the values in array
        config_b = []
        config_c = []
        # joint number
        setting_on_joint = np.zeros(7)
        q = 1
        w = 3
        e = 5                
        for position in pos[:]:
            if position[-1] < (25 + i * 20) and position[-1] > (15 + i * 20):
                setting_on_joint = np.vstack((setting_on_joint, position[0:7]))

        for value in val[:]:
            if value[-1] < (25 + i * 20) and value[-1] > (15 + i * 20):
                config_a.append(value[q])
                config_b.append(value[w])
                config_c.append(value[e])

        avg_config_a = np.mean(config_a)
        avg_config_b = np.mean(config_b)
        avg_config_c = np.mean(config_c) 
        joints = np.mean(setting_on_joint[1:, :], axis = 0)

        A = avg_config_a
        B = avg_config_b
        C = avg_config_c

        ## Jacobian function from iiwa14Kine.py. I used my iiwa code because kdl result and my results are same.
        ## Also, I need the results of the Z_s and O_s, respectively, not as full Jacobian
        T = np.identity(4)
        T[2, 3] = 0.1575
        Z_s = [T[0:3, 2]]
        O_s = [T[0:3, 3]]
        for j in range(7):
            T = T.dot(iiwa.T_rotationZ(joints[j]))
            T = T.dot(iiwa.T_translation(iiwa.translation_matrix[j, :]))
            T = T.dot(iiwa.T_rotationX(iiwa.rot_alpha_x[j]))
            T = T.dot(iiwa.T_rotationY(iiwa.rot_alpha_y[j]))

            Z_s.append(T[0:3, 2])
            O_s.append(T[0:3, 3])
        ## In prismatic and revolute joint p_i_j matrix and o_i_j matrix
        ## Compute O_s from Jacobian
        o_1_1 = O_s[q][0]
        o_1_2 = O_s[w][0]
        o_1_3 = O_s[e][0]
        o_2_1 = O_s[q][1]
        o_2_2 = O_s[w][1]
        o_2_3 = O_s[e][1]

        z_1_1 = Z_s[q][0]
        z_1_2 = Z_s[w][0]
        z_1_3 = Z_s[e][0]
        z_2_1 = Z_s[q][1]
        z_2_2 = Z_s[w][1]
        z_2_3 = Z_s[e][1]

        M = -(avg_config_a*z_1_2*z_2_3 - avg_config_a*z_1_3*z_2_2 - avg_config_b*z_1_1*z_2_3 + avg_config_b*z_1_3*z_2_1 + avg_config_c*z_1_1*z_2_2 - avg_config_c*z_1_2*z_2_1) / (g * (o_1_1*z_1_2*z_2_1*z_2_3 - o_1_1*z_1_3*z_2_1*z_2_2 - o_2_1*z_1_1*z_1_2*z_2_3 + o_2_1*z_1_1*z_1_3*z_2_2 - o_1_2*z_1_1*z_2_2*z_2_3 + o_1_2*z_1_3*z_2_1*z_2_2 + o_2_2*z_1_1*z_1_2*z_2_3 - o_2_2*z_1_2*z_1_3*z_2_1 + o_1_3*z_1_1*z_2_2*z_2_3 - o_1_3*z_1_2*z_2_1*z_2_3 - o_2_3*z_1_1*z_1_3*z_2_2 + o_2_3*z_1_2*z_1_3*z_2_1))

        x_axis_pos = -(avg_config_a*o_1_2*z_1_3*z_2_2 - avg_config_a*o_2_2*z_1_2*z_1_3 - avg_config_a*o_1_3*z_1_2*z_2_3 + avg_config_a*o_2_3*z_1_2*z_1_3 - avg_config_b*o_1_1*z_1_3*z_2_1 + avg_config_b*o_2_1*z_1_1*z_1_3 + avg_config_b*o_1_3*z_1_1*z_2_3 - avg_config_b*o_2_3*z_1_1*z_1_3 + avg_config_c*o_1_1*z_1_2*z_2_1 - avg_config_c*o_2_1*z_1_1*z_1_2 - avg_config_c*o_1_2*z_1_1*z_2_2 + avg_config_c*o_2_2*z_1_1*z_1_2) / (avg_config_a*z_1_2*z_2_3 - avg_config_a*z_1_3*z_2_2 - avg_config_b*z_1_1*z_2_3 + avg_config_b*z_1_3*z_2_1 + avg_config_c*z_1_1*z_2_2 - avg_config_c*z_1_2*z_2_1)

        y_axis_pos = -(avg_config_a*o_1_2*z_2_2*z_2_3 - avg_config_a*o_2_2*z_1_2*z_2_3 - avg_config_a*o_1_3*z_2_2*z_2_3 + avg_config_a*o_2_3*z_1_3*z_2_2 - avg_config_b*o_1_1*z_2_1*z_2_3 + avg_config_b*o_2_1*z_1_1*z_2_3 + avg_config_b*o_1_3*z_2_1*z_2_3 - avg_config_b*o_2_3*z_1_3*z_2_1 + avg_config_c*o_1_1*z_2_1*z_2_2 - avg_config_c*o_2_1*z_1_1*z_2_2 - avg_config_c*o_1_2*z_2_1*z_2_2 + avg_config_c*o_2_2*z_1_2*z_2_1) / (avg_config_a*z_1_2*z_2_3 - avg_config_a*z_1_3*z_2_2 - avg_config_b*z_1_1*z_2_3 + avg_config_b*z_1_3*z_2_1 + avg_config_c*z_1_1*z_2_2 - avg_config_c*z_1_2*z_2_1)

        ## Compute the vector from ee
        v_pos = (iiwa.forward_kine(joints, 7) - iiwa.forward_kine(joints, 6))[0:3, -1]
        v_pos_unit = np.copy(v_pos / np.linalg.norm(v_pos))
        pos_ee = iiwa.forward_kine(joints, 7)[0:3, -1]
        pos_x = np.copy((x_axis_pos - pos_ee[0]) / v_pos_unit[0])
        pos_y = np.copy((y_axis_pos - pos_ee[1]) / v_pos_unit[1])
        cv = np.mean(np.array([pos_x, pos_y]))
        z_axis_pos = pos_ee[2] + (v_pos_unit[2] * cv) 

        p_centre_of_mass = np.array([x_axis_pos, y_axis_pos, z_axis_pos])
        A = np.atleast_2d(p_centre_of_mass - pos_ee)
        d_ee = np.transpose(A)
        final_rotation_m = iiwa.forward_kine(joints, 7)[0:3, 0:3]
        B = np.linalg.inv(final_rotation_m)
        C = d_ee
        D = np.matmul(B, C)

        transl = np.transpose(D)
        translation[i, :] = transl[:]
        mass[i] = M

    ### Average over the configurations.
    final_m = mass
    final_avg_m = np.mean(mass)
    final_avg_t = np.mean(transl, axis = 0)

    return final_m, final_avg_m, final_avg_t

rospy.init_node('cw3q5b_receiver')

start_time = rospy.get_time()
start_publish = 4
keep_result_A = []
keep_renew_result_A = []

sub = rospy.Subscriber('/object_iiwa/joint_states', JointState, callback_function)
rospy.spin()

l = len(keep_result_A)  # length
val = np.zeros([l, 8])    # value
pos = np.zeros([l, 8])  # position
for i in range(l):
    val[i, :] = keep_result_A[i]
    pos[i, :] = keep_renew_result_A[i]

mass, avg_mass, avg_cm_transl = calculation_three_configuration(val, pos)
## save in loginfo and print the all results
rospy.loginfo('\n Each mass at each configuration q_a, q_b and q_c is: {mass} kg \n Average of the masses for each configuration is: {avg_mass} kg \n Average distance x-axis from end-effector frame to com is: {pos_x} m \n Average distance y-axis from end-effector frame to com is: {pos_y} m \n Average distance z-axis from end-effector frame to com is: {pos_z} m \n Total average distance between end-effector and com is: {d} m \n'.format(mass = mass, avg_mass = avg_mass, pos_x = avg_cm_transl[0], pos_y = avg_cm_transl[1], pos_z = avg_cm_transl[2], d = np.round(np.linalg.norm(avg_cm_transl), 3)))

## External Torque
joints_final = np.array([0, -90, -90, 90, -90, 90, -30]) * np.pi / 180
## compute jacobian
T = np.identity(4)
T[2, 3] = 0.1575
Z_s = [T[0:3, 2]]
O_s = [T[0:3, 3]]
for i in range(7):
    T = T.dot(iiwa.T_rotationZ(joints_final[i]))
    T = T.dot(iiwa.T_translation(iiwa.translation_matrix[i, :]))
    T = T.dot(iiwa.T_rotationX(iiwa.rot_alpha_x[i]))
    T = T.dot(iiwa.T_rotationY(iiwa.rot_alpha_y[i]))

    Z_s.append(T[0:3, 2])
    O_s.append(T[0:3, 3])

external_torque = np.zeros(7)
J_p = np.zeros([3, 7])

T_ee = iiwa.forward_kine(joints_final, 7)
T_cm = T_ee.dot(iiwa.T_translation(avg_cm_transl))
p_centre_of_mass = np.atleast_2d(T_cm[0:3, -1])

for k in range(7):
    Z = np.atleast_2d(Z_s[k])
    O = np.atleast_2d(O_s[k])

    J_p[:, k] = np.cross(Z, (p_centre_of_mass - O))

ext_torq = (avg_mass * np.matmul(np.array([0, 0, -1 * iiwa.g]), J_p))

rospy.loginfo('\n The external torque at the configuration q_d: {tau} \n'.format(tau = ext_torq))















