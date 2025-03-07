#! /usr/bin/python3

import rospy

rospy.init_node('our_set_private_param_node')
rospy.set_param('~our_private_param', 'private started from rospy')