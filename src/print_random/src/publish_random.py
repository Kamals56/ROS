#! /usr/bin/python3

import rospy
import random
from std_msgs.msg import Int32

rospy.init_node("talker")
pub = rospy.Publisher("publish_random", Int32, queue_size = 10)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    number =  random.randint(100, 2000)
    pub.publish(number)
    rate.sleep()