#! /usr/bin/python3

import rospy
from std_msgs.msg import String

def callback(msg):
    print(msg)
    print("-------")

rospy.init_node("simple_listener")
rospy.Subscriber("simple_pub",String,callback)
rospy.spin()