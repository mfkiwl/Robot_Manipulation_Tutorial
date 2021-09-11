#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion

class iiwa14_kinematic(object):

    def __init__(self):
        ##TODO: Fill in the DH parameters based on the xacro file (cw3/iiwa_description/urdf/iiwa14.xacro)
        self.DH_params = np.array([[0.0, -pi /2, 0.2025, 0.0],
                                   [0.0, pi /2, 0.0, 0.0],
                                   [0.0, pi /2, 0.42, 0.0],
                                   [0.0, -pi /2, 0.0, 0.0],
                                   [0.0, -pi /2, 0.4, 0.0],
                                   [0.0, pi /2, 0.0, 0.0],
                                   [0.0, 0.0, 0.126, 0.0]])

        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_limit_min = [-170 * pi / 180, -120 * pi / 180, -170 * pi / 180, -120 * pi / 180, -170 * pi / 180,
                                -120 * pi / 180, -175 * pi / 180]
        self.joint_limit_max = [170 * pi / 180, 120 * pi / 180, 170 * pi / 180, 120 * pi / 180, 170 * pi / 180,
                                120 * pi / 180, 175 * pi / 180]
        self.rot_alpha_x = [pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, 0.0]
        self.rot_alpha_y = [pi, pi, 0, pi, 0, pi, 0]        
        self.rot_alpha_z = [pi, 0, pi, 0, 0, 0, 0]
        ##The mass of each link.
        self.mass = [4, 4, 3, 2.7, 1.7, 1.8, 0.3]

        ##Moment on inertia of each link, defined at the centre of mass.
        ##Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
        self.Ixyz = np.array([[0.1, 0.09, 0.02],
                              [0.05, 0.018, 0.044],
                              [0.08, 0.075, 0.01],
                              [0.03, 0.01, 0.029],
                              [0.02, 0.018, 0.005],
                              [0.005, 0.0036, 0.0047],
                              [0.001, 0.001, 0.001]])

        ##Computing the Forward Kinematic by using the translation for each joint.
        self.translation_matrix = np.array([[0, 0, 0.2025],
                                            [0, 0.2045, 0],
                                            [0, 0, 0.2155],
                                            [0, 0.1845, 0],
                                            [0, 0, 0.2155],
                                            [0, 0.081, 0],
                                            [0, 0, 0.045]])
        
        ##Centre of the mass for each link
        self.centre_of_mass = np.array([[0, -0.03, 0.12],
                                        [0.0003, 0.059, 0.042],
                                        [0, 0.03, 0.13],
                                        [0, 0.067, 0.034],
                                        [0.0001, 0.021, 0.076],
                                        [0, 0.0006, 0.0004],
                                        [0, 0, 0.02]])

        ##gravity
        self.g = 9.8

        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                queue_size=5)

        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def joint_state_callback(self, msg):
        for i in range(0, 7):
            self.current_joint_position[i] = msg.position[i]

        current_pose = self.forward_kine(self.current_joint_position, 7)
        self.broadcast_pose(current_pose)

    def dh_matrix_standard(self, a, alpha, d, theta):
        A = np.zeros((4, 4))

        A[0, 0] = np.cos(theta)
        A[0, 1] = -np.sin(theta) * np.cos(alpha)
        A[0, 2] = np.sin(theta) * np.sin(alpha)
        A[0, 3] = a * np.cos(theta)

        A[1, 0] = np.sin(theta)
        A[1, 1] = np.cos(theta) * np.cos(alpha)
        A[1, 2] = -np.cos(theta) * np.sin(alpha)
        A[1, 3] = a * np.sin(theta)

        A[2, 1] = np.sin(alpha)
        A[2, 2] = np.cos(alpha)
        A[2, 3] = d

        A[3, 3] = 1.0

        return A

    def broadcast_pose(self, pose):

        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'iiwa_ee'

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    ##Useful Transformation function
    def T_translation(self, t):
        T = np.identity(4)
        for i in range(0, 3):
            T[i, 3] = t[i]
        return T

    ##Useful Transformation function
    def T_rotationZ(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        return T

    ##Useful Transformation function
    def T_rotationX(self, theta):
        T = np.identity(4)
        T[1, 1] = np.cos(theta)
        T[1, 2] = -np.sin(theta)
        T[2, 1] = np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T

    ##Useful Transformation function
    def T_rotationY(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 2] = np.sin(theta)
        T[2, 0] = -np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T

    def rotmat2q(self, T):
        q = Quaternion()

        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        xr = T[2, 1] - T[1, 2]
        yr = T[0, 2] - T[2, 0]
        zr = T[1, 0] - T[0, 1]

        if (xr == 0) and (yr == 0) and (zr == 0):
            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
        else:

            x = xr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            y = yr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            z = zr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            q.w = np.cos(angle / 2)
            q.x = x * np.sin(angle / 2)
            q.y = y * np.sin(angle / 2)
            q.z = z * np.sin(angle / 2)
        return q

    def rotmat_rodrigues(self, T):
        x = np.zeros([6])
        x[0:3] = T[0:3,-1]

        angle = np.arccos((T[0,0] + T[1,1] + T[2,2] - 1)/2)

        if theta == 0:
            x[3:6] = 0
        else:
            x[3] = (1 / (2 * np.sin(angle))) * (T[2,1] - T[1,2])
            x[4] = (1 / (2 * np.sin(angle))) * (T[0,2] - T[2,0])
            x[5] = (1 / (2 * np.sin(angle))) * (T[1,0] - T[0,1])

        return x

    def differential(self, joint, first_q):
        offset = 1e-6

        first_joint = np.copy(joint)
        first_joint[first_q] = first_joint[first_q] + offset
        results = (self.getB(first_joint) - self.getB(joint)) / offset

        return results

    def forward_kine(self, joint, frame):
        T = np.identity(4)
        ##Add offset from the iiwa platform.
        T[2, 3] = 0.1575
        ##TODO: Fill in this function to complete Q2.
        for i in range(0, frame):
            T = T.dot(self.T_rotationZ(joint[i]))
            T = T.dot(self.T_translation(self.translation_matrix[i, :]))
            T = T.dot(self.T_rotationX(self.rot_alpha_x[i]))
            T = T.dot(self.T_rotationY(self.rot_alpha_y[i]))

        return T

    def forward_kine_cm(self, joint, frame):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to the centre of mass of the specified link.
        frame = frame - 1 # offsets
        T = self.forward_kine(joint, frame)

        T = T.dot(self.T_rotationZ(joint[frame]))
        T = T.dot(self.T_translation(self.centre_of_mass[frame, :]))
        T = T.dot(self.T_rotationX(self.rot_alpha_x[frame]))
        T = T.dot(self.T_rotationY(self.rot_alpha_y[frame]))

        return T

    def get_jacobian(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at each frame.
        l = len(self.current_joint_position)
        T = np.identity(4)
        T[2, 3] = 0.1575

        Z_s = [T[0:3, 2]]
        O_s = [T[0:3, 3]]
        for i in range(l):
            T = T.dot(self.T_rotationZ(joint[i]))
            T = T.dot(self.T_translation(self.translation_matrix[i, :]))
            T = T.dot(self.T_rotationX(self.rot_alpha_x[i]))
            T = T.dot(self.T_rotationY(self.rot_alpha_y[i]))

            Z_s.append(T[0:3, 2])
            O_s.append(T[0:3, 3])

        J = np.zeros([6, l])
        P_ee = O_s[-1]
        for i in range(l):
            Z_i = Z_s[i]
            O_i = O_s[i]

            J_p = np.cross(Z_i, (P_ee - O_i))
            J_o = Z_i
            for n in range(3):
                J[n, i] = J_p[n]
                J[n + 3, i] = J_o[n]

        return J

    def get_jacobian_cm(self, joint, frame):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the centre of mass of the specified link.
        l = len(self.current_joint_position)
        T = np.identity(4)
        T[2, 3] = 0.1575
        Z_s = [T[0:3, 2]]
        O_s = [T[0:3, 3]]

        J = np.zeros([6, l])
        P_centre_mass = self.forward_kine_cm(joint, frame)[0:3, 3]
        for i in range(l):
            T = T.dot(self.T_rotationZ(joint[i]))
            T = T.dot(self.T_translation(self.translation_matrix[i, :]))
            T = T.dot(self.T_rotationX(self.rot_alpha_x[i]))
            T = T.dot(self.T_rotationY(self.rot_alpha_y[i]))

            Z_s.append(T[0:3, 2])
            O_s.append(T[0:3, 3])

        for i in range(l):
            Z_i = Z_s[i]
            O_i = O_s[i]
            J_p = np.cross(Z_i, (P_centre_mass - O_i))
            J_o = Z_i
            for k in range(3):
                J[k, i] = J_p[k]
                J[k + 3, i] = J_o[k]

        return J

    def inverse_kine_ite(self, desired_pose, current_joint):
        ##TODO: Fill in this function to complete Q2.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## "current_joint" is an array of double consisting of the joint value (works as a starting point for the optimisation).
        ## The output is numpy vector containing an optimised joint configuration for the desired pose.
        range_max = np.array(self.joint_limit_max)
        range_min = np.array(self.joint_limit_min)
        k = 0

        alpha = 0.001  
        margin_error = 0.5 * np.pi / 180  
        gradient_descent = 0.01  
        q_k = current_joint
        pose_end = self.rotmat2pr(desired_pose)
        pose_q_k = self.rotmat2pr(self.forward_kine(q_k, len(q_k)))
        while any((abs(diff) > alpha) for diff in (pose_end - pose_q_k)[0 : 3]) or any((abs(diff) > margin_error) for diff in (pose_end - pose_q_k)[3 : 6]):
            T_q = self.forward_kine(q_k, len(q_k))
            J_q = self.get_jacobian(q_k)
            pose_q_k = self.rotmat2pr(T_q)
            renew_q_k = q_k

            x = np.transpose(J_q)
            A = np.matmul(J_q, A) + gradient_descent**2 * np.eye(J_q.shape[0])
            y = np.linalg.inv(A)
            z = (pose_end - pose_q_k)
            B = np.matmul(x, A)
            q_k = q_k + np.matmul(B, z)
            k += 1

            if all(abs(renew_q_k - q_k)[0:3]) < alpha and all(abs(renew_q_k - q_k)[3 : 6]) < margin_error:
                ##local minia
                break
            if k >= 1500:
                # After 1500 cycles, then stop.
                break

        for i in range(q_k.shape[0]):
            while q_k[i] < range_min[i]:
                q_k[i] = q_k[i] + 2 * np.pi

            while q_k[i] > range_max[i]:
                q_k[i] = q_k[i] - 2 * np.pi

        return q_k

    def inverse_kine_closed_form(self, desired_pose):
        ##TODO: Fill in this function to complete Q2.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## The output is a numpy matrix consisting of the joint value for the desired pose.
        ## You may need to re-structure the input of this function.
        l_w = 0.126 ##0.045 + 0.081 = wrist length
        d_w = np.transpose(np.array([0, 0, -l_w, 1]))  ## different length of wrist
        pose_w = np.matmul(desired_pose, d_w)[0:3]  ## pose wrist

        pose = self.forward_kine(np.zeros(7),1)[0:3,-1]

        p_shoulder_wrist = pose_w - pose

        length_bs = self.forward_kine(np.zeros(7), 1)[2, -1]
        length_es = self.forward_kine(np.zeros(7), 3)[2, -1] - length_bs
        length_we = self.forward_kine(np.zeros(7), 5)[2, -1] - (length_es + length_bs)
        length_sw = np.linalg.norm(p_shoulder_wrist)

        third_angle = 0
        first_angle = np.arctan2(p_shoulder_wrist[1], p_shoulder_wrist[0])
        X = (length_es**2 + length_sw**2 - length_we**2)/(2 * length_sw * length_es)
        if abs(X) > 1:
            X = X / abs(X)

        ext_the = np.arccos(X)
        Y = (length_sw**2 - length_es**2 - length_we**2)/(2*length_es*length_we)
        if abs(Y) > 1:
            Y = Y / abs(Y)

        for s in itertools.product([-1, 1], repeat = 2):
            fourth_angle = s[0] * np.arccos(Y) * s[1]
            second_angle = np.arctan2(np.sqrt(p_shoulder_wrist[0]**2 + p_shoulder_wrist[1]**2), p_shoulder_wrist[2]) + ext_the * s[0] * s[1]
            if np.allclose(self.forward_kine(np.array([first_angle, second_angle, third_angle, fourth_angle, 0, 0, 0]), 5)[0:3, -1], pose_w):
                break

        trans_matrix_e = self.forward_kine(np.array([first_angle, second_angle, third_angle, fourth_angle, 0, 0, 0]), 3)
        pose_e = trans_matrix_e[0:3,-1]
        pose_s_e = pose_e - pose
        pose_centre = np.dot(pose_s_e, p_shoulder_wrist)
        final_pose_centre = (pose_centre / length_sw) * (p_shoulder_wrist / length_sw)
        radius_centre = pose_s_e - final_pose_centre
        R = np.linalg.norm(radius_centre)
        init_theta = 0 
        
        value_shoulder_wrist = p_shoulder_wrist / np.linalg.norm(p_shoulder_wrist)
        matrix_shoulder_wrist = np.zeros([3,3])

        matrix_shoulder_wrist[0,1] = -value_shoulder_wrist[2]
        matrix_shoulder_wrist[0,2] = value_shoulder_wrist[1]
        matrix_shoulder_wrist[1,0] = value_shoulder_wrist[2]
        matrix_shoulder_wrist[1,2] = -value_shoulder_wrist[0]
        matrix_shoulder_wrist[2,0] = -value_shoulder_wrist[1]
        matrix_shoulder_wrist[2,1] = value_shoulder_wrist[0]

        init_third_R = trans_matrix_e[0:3,0:3]
        theta_R = np.eye(3) + np.sin(init_theta) * matrix_shoulder_wrist + (1 - np.cos(init_theta)) * np.matmul(matrix_shoulder_wrist, matrix_shoulder_wrist)
        third_R = np.matmul(init_third_R, third_R)
        value = np.empty([1,8])
        for s in itertools.product([-1, 1], repeat = 5):
            angle_1 = np.arctan2(s[0] * third_R[1,1], s[0] * third_R[0,1])
            angle_2 = s[0] * np.arccos(third_R[2,1]) * s[3]
            angle_3 = np.arctan2(s[0] * -third_R[2, 2], s[0] * -third_R[2, 0])
            angle_4 = s[1] * np.arccos((length_sw**2 - length_es**2 - length_we**2)/(2 * length_es * length_we))

            trans_matrix_4 = self.forward_kine(np.array([angle_1, angle_2, angle_3, angle_4, 0, 0, 0]), 4)
            trans_matrix_4_7 = np.matmul(np.linalg.inv(trans_matrix_4), desired_pose)
            rot_mat_4_7 = trans_matrix_4_7[0:3,0:3]

            angle_5 = np.arctan2(s[2] * rot_mat_4_7[1,2], s[2] * rot_mat_4_7[0,2])
            angle_6 = s[2] * np.arccos(rot_mat_4_7[2,2]) * s[4]
            angle_7 = np.arctan2(s[2] * rot_mat_4_7[2, 1], s[2] * -rot_mat_4_7[2,0])

            n_joints = np.array([angle_1, angle_2, angle_3, angle_4, angle_5, angle_6, angle_7, 1]) # one more angle array
            distance = desired_pose[0:3,-1] - self.forward_kine(n_joints,7)[0:3,-1]
            if np.allclose(distance, np.zeros(3)) == True:
                if np.all((self.joint_limit_max - n_joints[0 : -1]) > 0) and np.all((n_joints[0:-1] - self.joint_limit_min) > 0):
                    if self.check_singularity(n_joints[0 : -1]) == 1:
                        n_joints[-1] = 0
                    value = np.vstack((value, n_joints))
                    # break
                else:
                    n_joints[-1] = 0
                    value = np.vstack((value, n_joints))
            else:
                n_joints[-1] = -1
                value = np.vstack((value, n_joints))

        return value[1:, :]

    def check_singularity(self, current_joint):
        J = self.get_jacobian(current_joint)
        if J.shape[0] == J.shape[1]:
            final_J = J
        else:
            J_T = np.transpose(J)
            final_J = np.matmul(J_T, J)

        det = np.linalg.det(final_J)
        if det == 0:

            return True
        else:

            return False

    def getB(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 7*7 matrix.
        num_of_frames = len(joint)
        k_s = np.zeros([num_of_frames, 7, 7])  ## make 7 by 7 matrix.
        for i in range(num_of_frames):
            jacobian = self.get_jacobian_cm(joint, i)

            J_p = jacobian[0:3, :]
            J_o = jacobian[3:6, :]
            x = self.mass[i] * np.matmul(np.transpose(J_p), J_p)
            
            I = np.eye(3)
            I = I * self.Ixyz[i] * 4
            rot = self.forward_kine(joint, i)[0:3, 0:3]
            J = np.matmul(rot, np.matmul(I, np.transpose(rot)))
            B = np.matmul(J, J_o)
            y = np.matmul(np.transpose(J_o), B)

            k_s[i] = x + y

        B = np.sum(k_s, axis = 0)

        return B

    def getC(self, joint, vel):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "vel" is a numpy array of double consisting of the joint velocity.
        ## The output is a numpy 7*7 matrix.
        num_of_frames = len(joint)
        C = np.zeros([num_of_frames, num_of_frames])
        vel_matrix = np.zeros([num_of_frames, num_of_frames, num_of_frames])
        for i in range(num_of_frames):
            vel_matrix[i, :, :] = self.differential(joint, i)

        for i in range(num_of_frames):
            for j in range(num_of_frames):
                H_s = np.zeros(7)
                for k in range(num_of_frames):
                    H_s[k] = vel_matrix[k, i, j] - 0.5 * vel_matrix[i, j, k]
                    H_s[k] = H_s[k] * vel[k]
                C[i, j] = np.sum(H_s)

        return C

    def getG(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy array 7*1.
        num_of_frames = len(joint)
        G = np.zeros(num_of_frames)
        gravity_mat = np.array([0.0, 0.0, -self.g])
        for i in range(num_of_frames):
            M = self.mass[i]
            J = self.get_jacobian_cm(joint, i)[0:3, :]
            G -= (M * np.matmul(gravity_mat, J))

        G = np.atleast_2d(G)
        G = np.transpose(G)

        return G
