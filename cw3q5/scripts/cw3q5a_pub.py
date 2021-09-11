#!/usr/bin/env python
from __future__ import division
import rospy
import rospkg
import rosbag
import sys
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from cw3q2.iiwa14Kine import iiwa14_kinematic

my_iiwa = iiwa14_kinematic()

pub = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size = 10)
rospy.init_node('cw3q5a_sender')
rate = rospy.Rate(10) # 10hz

rospack = rospkg.RosPack()

bag_path = rospack.get_path('cw3_launch')
bag = rosbag.Bag(bag_path + '/bags/cw3bag1.bag')

for topic, msg, t in bag.read_messages(topics = ['/iiwa/EffortJointInterface_trajectory_controller/command']):
    rospy.sleep(5)
    pub.publish(msg)

bag.close()




