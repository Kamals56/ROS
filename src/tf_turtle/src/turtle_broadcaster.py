#! /usr/bin/python3

import rospy
import tf2_ros
import tf_conversions
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

def cb(msg, turtle_name):
    broadcaster = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = turtle_name
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0

    quaternion = tf_conversions.transformations.quaternion_from_euler(0,0,msg.theta)
    t.transform.rotation.x = quaternion[0]
    t.transform.rotation.y = quaternion[1]
    t.transform.rotation.z = quaternion[2]
    t.transform.rotation.w = quaternion[3]
    broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node(f'broadcaster')
    turtle_name = rospy.get_param('~turtle')
    rospy.Subscriber(f'/{turtle_name}/pose',
                     Pose,
                     cb,
                     turtle_name)
    rospy.spin()


