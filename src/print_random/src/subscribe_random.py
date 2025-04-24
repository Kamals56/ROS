#! /usr/bin/python3

import rospy
from std_msgs.msg import Int32


def printing_continuous(msg):
    print(msg)
    print("........")


rospy.init_node("listener")
rospy.Subscriber("publish_random",Int32, printing_continuous)
rospy.spin()