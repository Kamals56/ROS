#! /usr/bin/python3
import rospy

rospy.init_node("our_get_global_param")
our_global_param = rospy.get_param('/this_is_global_starts_with_slash')

print(our_global_param)