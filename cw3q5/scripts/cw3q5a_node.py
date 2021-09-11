#!/usr/bin/env python
##TODO: Fill in the code. You will have to add your node in the launch file as well. The launch file is in cw3_launch.
from __future__ import division
import rospy
import rospkg
import rosbag
import sys
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from cw3q2.iiwa14Kine import iiwa14_kinematic
from kdl_kine.kdl_kine_solver_iiwa14 import iiwa14KDL

iiwa = iiwa14_kinematic()
kdl_iiwa = iiwa14KDL()

def kdl_change_matrix_function(mat):
    M = np.array(np.zeros((mat.rows(), mat.columns())))
    for i in range(mat.rows()):
        for j in range(mat.columns()):
            M[i,j] = mat[i,j]

    return M

def kdl_change_array_function(array):
    arr = []
    for i in array:
        arr.append(i)

    return arr

def callback_function(msg):
    tau = msg.effort
    position = msg.position
    velocity = msg.velocity

    kdl_B = kdl_change_matrix_function(kdl_iiwa.getB(position))
    kdl_C = kdl_change_array_function(kdl_iiwa.getC(position, velocity))
    kdl_G = kdl_change_array_function(kdl_iiwa.getG(position))

    tau = np.atleast_2d(tau)
    kdl_C = np.atleast_2d(kdl_C)
    kdl_G = np.atleast_2d(kdl_G)

    X = np.transpose(tau) - np.transpose(kdl_G) - np.transpose(kdl_C)
    kdl_acceleration.append(np.matmul(np.linalg.inv(kdl_B), X))  ## Dynamic equation

    time.append(rospy.get_time() - publish_s - time_s)

    if rospy.get_time() > time_s + publish_s + 31: # set time for spinning
        rospy.signal_shutdown('Stop spinning')

def ground_truth_function():
    rospack = rospkg.RosPack()
    bag_path = rospack.get_path('cw3_launch')
    bag = rosbag.Bag(bag_path + '/bags/cw3bag1.bag')

    pos = np.zeros(len(iiwa.current_joint_position))
    vel = np.zeros(len(pos))
    accel = np.zeros(len(pos))
    for topic, msg, t in bag.read_messages(topics = ['/iiwa/EffortJointInterface_trajectory_controller/command']):
        points = msg.points

    bag.close()
    for i in range(len(points)):
        pos = np.vstack((pos, points[i].positions))
        vel = np.vstack((vel, points[i].velocities))
        accel = np.vstack((accel, points[i].accelerations))

    dt = 1
    num_of_pos = pos.shape[0]
    diff_time_pos = 10
    
    joint_position = np.zeros([int(diff_time_pos / dt * (num_of_pos - 1) + 1), 7])
    joint_velocity = np.zeros([int(diff_time_pos / dt * (num_of_pos - 1) + 1), 7])
    joint_acceleration = np.zeros([int(diff_time_pos / dt * (num_of_pos - 1) + 1), 7])

    joint_position[0,:] = pos[0]
    joint_velocity[0,:] = vel[0]
    joint_acceleration[0,:] = accel[0]


    for n in range(num_of_pos - 1):

        start_time = n * diff_time_pos
        final_time = start_time + diff_time_pos

        poly_function_1 = np.array([[1, start_time, start_time ** 2, start_time ** 3, start_time ** 4, start_time ** 5],
                                    [0, 1, 2 * start_time, 3 * start_time ** 2, 4 * start_time ** 3, 5 * start_time ** 4],
                                    [0, 0, 2, 6 * start_time ** 1, 12 * start_time ** 2, 20 * start_time ** 3],
                                    [1, final_time, final_time ** 2, final_time ** 3, final_time ** 4, final_time ** 5],
                                    [0, 1, 2 * final_time, 3 * final_time ** 2, 4 * final_time ** 3, 5 * final_time ** 4],
                                    [0, 0, 2, 6 * final_time ** 1, 12 * final_time ** 2, 20 * final_time ** 3]])
        poly_function_2 = np.array([pos[n], vel[n], accel[n], pos[n + 1], vel[n + 1],  accel[n + 1]])
        X = np.linalg.solve(poly_function_1, poly_function_2) 

        x0 = X[0, :]
        x1 = X[1, :]
        x2 = X[2, :]
        x3 = X[3, :]
        x4 = X[4, :]
        x5 = X[5, :]
        A = start_time + dt
        B = final_time
        C = int(diff_time_pos / dt)
        for t in np.linspace(A, B, C):
            i = int(t)
            joint_position[i] = x0 + x1*t + x2*t**2 + x3*t**3 + x4*t**4 + x5*t**5
            joint_velocity[i] = x1 + 2*x2*t + 3*x3*t**2 + 4*x4*t**3 + 5*x5*t**4
            joint_acceleration[i] = 2*x2 + 6*x3*t**1 + 12*x4*t**2 + 20*x5*t**3

    return joint_position, joint_velocity, joint_acceleration

def make_plot_function(ground_truth_accel, kdl_acceleration, time):
    start_time = np.arange(0, 31)
    y_axis_accel_1 = ground_truth_accel[:, 4]   # [:, 0] = joint 1, [:, 1] = joint 2 ...... [:, 6] = joint 7
    A = np.atleast_2d(kdl_acceleration)
    kdl_acceleration = np.squeeze(A)
    y_axis_accel_2 = kdl_acceleration[:, 4]     # [:, 0] = joint 1, [:, 1] = joint 2 ...... [:, 6] = joint 7

    plt.plot(start_time, y_axis_accel_1, label = 'ground truth')
    plt.plot(time, y_axis_accel_2, label = 'kdl')
    plt.xlim(0, 30)
    plt.title('Joint 5')       # you need to change the joint number when you change the joint
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')       # you can select joint 1 ~ 7
    plt.legend()
    plt.show()

rospy.init_node('cw3q5a_receiver')

ground_truth_pos, ground_truth_vel, ground_truth_accel = ground_truth_function()

time_s = rospy.get_time()
publish_s = 5
kdl_acceleration = []
accel = []
time = []

sub = rospy.Subscriber('/iiwa/joint_states', JointState, callback_function)
rospy.spin()

kdl_accel_array = np.squeeze(np.array(kdl_acceleration))
time_array = np.squeeze(np.array(time))

rospack = rospkg.RosPack()

path = rospack.get_path('cw3_launch')
bag = rosbag.Bag(path + '/bags/cw3q5abag.bag', 'w')

time_data = Float64MultiArray()
time_data.data = time_array[:]
bag.write('Time', time_data)

accel_data = Float64MultiArray()
accel_data.data = kdl_accel_array[:, 0]
bag.write('Joint_1', accel_data)

accel_data.data = kdl_accel_array[:, 1]
bag.write('Joint_2', accel_data)

accel_data.data = kdl_accel_array[:, 2]
bag.write('Joint_3', accel_data)

accel_data.data = kdl_accel_array[:, 3]
bag.write('Joint_4', accel_data)

accel_data.data = kdl_accel_array[:, 4]
bag.write('Joint_5', accel_data)

accel_data.data = kdl_accel_array[:, 5]
bag.write('Joint_6', accel_data)

accel_data.data = kdl_accel_array[:, 6]
bag.write('Joint_7', accel_data)
bag.close()

make_plot_function(ground_truth_accel, kdl_acceleration, time) 










