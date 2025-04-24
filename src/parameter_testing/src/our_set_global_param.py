#!  /usr/bin/python3

import rospy

rospy.init_node('our_set_global_param_node')

rospy.set_param('/this_is_global_starts_with_slash', 'started from rospy')