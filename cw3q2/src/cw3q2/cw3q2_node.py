#!/usr/bin/env python
import rospy
import random
from sensor_msgs.msg import JointState
from cw3q2.iiwa14Kine import iiwa14_kinematic
from kdl_kine.kdl_kine_solver_iiwa14 import  iiwa14KDL
from kdl_kine.urdf import *
import numpy as np


def KDL2Vec(kdlV):
    V = np.zeros([7,1])
    for i in range(7):
        V[i] = kdlV[i]
    return V

def show_jacobian():

    rospy.init_node('kuka_dynamics_node')

    rate = rospy.Rate(10)
    # Initiate the base link(where the kinematic chain starts) and the end-effector link(where it ends).
    # Please look at the urdf file of the respective robot arm for the names.
    iiwa = iiwa14_kinematic()
    kdl_iiwa = iiwa14KDL(1)

    q = PyKDL.JntArray(7)
    qdot = PyKDL.JntArray(7)

    while not rospy.is_shutdown():

        
        #Because joint_state_publisher does not publish velocity, we manually generate random joint positions and velocities as input to B, C, g matrices.
        for i in range(7):
            q[i] = random.uniform(-1, 1)
            qdot[i] = random.uniform(-0.5, 0.5)

        print('The current joint position: ')
        print(q)
        print('The current joint velocity: ')
        print(qdot)

        print('The current robot pose: ')
        print(kdl_iiwa.forward_kinematics(q))
        print('Current pose (my):')
        print(iiwa.forward_kine(q, 7))

        # =====================
        #  Jacobian
        # =====================
        print('Jacobian matrix (kdl): ')
        print(kdl_iiwa.get_jacobian(q))

        print('Jacobian matrix (my): ')
        print(iiwa.get_jacobian(q))


        print('B(q) (kdl): ')
        #PyKDL has a unique representation for the matrix B (only in Python, doesn't apply to C++), so we have to convert the representation to a matrix manually.
        # kdl_B = kdl_iiwa.getB(q)
        kdl_B = kdl_iiwa.getB(iiwa.current_joint_position)
        Bmat = np.zeros([7, 7])
        for i in range(7):
            for j in range(7):
                Bmat[i, j] = kdl_B[i, j]
        print(Bmat)
        print('B(q) (my): ')
        print(iiwa.getB(iiwa.current_joint_position))


        print('C(q, qdot) (kdl): ')
        kdl_C = kdl_iiwa.getC(q, qdot)
        C1 = KDL2Vec(kdl_C)
        C = np.matmul(C1, np.transpose(C1))
        print(C)
        # print('C(q, qdot) (my): ')
        # frame = [0, 1, 2, 3, 4, 5, 6]
        # frame_dot = iiwa.differential(iiwa.current_joint_position, frame)
        # # my_C = iiwa.getC(frame, frame_dot)
        # # my_C1 = KDL2Vec(my_C)
        # # my_C = np.matmul(my_C1, np.transpose(my_C1))
        # my_C = iiwa.getC(iiwa.current_joint_position, qdot)
        # print(my_C)


        print('g(q) (kdl): ')
        # print(kdl_iiwa.getG(q))
        print(kdl_iiwa.getG(iiwa.current_joint_position))
        print('g(q) (my): ')
        print(iiwa.getG(iiwa.current_joint_position))

        rate.sleep()



if __name__ == '__main__':
    show_jacobian()

