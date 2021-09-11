#!/usr/bin/env python

from __future__ import division
import rospy
import rospkg
import rosbag
import sys
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('cw3q5b_sender')
pub = rospy.Publisher('/object_iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=10)
rate = rospy.Rate(10) # 10hz
joint_traj = JointTrajectory()
rospy.sleep(5) 

joint_traj.header.stamp = rospy.Time.now()
joint_traj.joint_names.append('object_iiwa_joint_1')
joint_traj.joint_names.append('object_iiwa_joint_2')
joint_traj.joint_names.append('object_iiwa_joint_3')
joint_traj.joint_names.append('object_iiwa_joint_4')
joint_traj.joint_names.append('object_iiwa_joint_5')
joint_traj.joint_names.append('object_iiwa_joint_6')
joint_traj.joint_names.append('object_iiwa_joint_7')

rospack = rospkg.RosPack()

bag_path = rospack.get_path('cw3_launch')
bag = rosbag.Bag(bag_path + '/bags/cw3bag2.bag')
t_start = 9
for topic, msg, t in bag.read_messages(topics=['/iiwa/EffortJointInterface_trajectory_controller/command']):
    for i in range(3):
        time = t_start + (i * 20)
        traj_point = JointTrajectoryPoint()
        traj_point.positions = msg.points[i].positions
        traj_point.velocities = msg.points[i].velocities
        traj_point.accelerations = msg.points[i].accelerations
        traj_point.time_from_start = rospy.Duration(time)
        joint_traj.points.append(traj_point)

        time = time + 9
        traj_point = JointTrajectoryPoint()
        traj_point.positions = msg.points[i].positions
        traj_point.velocities = msg.points[i].velocities
        traj_point.accelerations = msg.points[i].accelerations
        traj_point.time_from_start = rospy.Duration(time)
        joint_traj.points.append(traj_point)

bag.close()

pub.publish(joint_traj)






