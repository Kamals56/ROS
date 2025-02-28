#! /usr/bin/python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
import math


WALL_THRESHOLD = 1.5  
TURN_SENSITIVITY = 2.5  

def callback(msg):
    vel_msg = Twist()

  
    x = msg.x
    y = msg.y
    theta = msg.theta
    
 
    if x < WALL_THRESHOLD:  # Close to left wall
        vel_msg.linear.x = 1.0  # Move forward with some speed
        vel_msg.angular.z = random.uniform(1.0, 2.0)  # Turn to the right to avoid the wall
    elif x > 11 - WALL_THRESHOLD:  # Close to right wall
        vel_msg.linear.x = 1.0  # Move forward with some speed
        vel_msg.angular.z = random.uniform(-2.0, -1.0)  # Turn to the left to avoid the wall
    elif y < WALL_THRESHOLD:  # Close to bottom wall
        vel_msg.linear.x = 1.0  # Move forward with some speed
        if abs(theta) < 1.0:  # Heading straight downwards
            vel_msg.angular.z = random.uniform(1.0, 2.5)  # Turn up (clockwise) to avoid bottom wall
        else:
            vel_msg.angular.z = random.uniform(0.5, 1.0)  # Small adjustments if heading elsewhere
    elif y > 11 - WALL_THRESHOLD:  # Close to top wall
        vel_msg.linear.x = 1.0  # Move forward with some speed
        vel_msg.angular.z = random.uniform(-1.0, -2.0)  # Turn downwards to avoid the wall
    else:
        # If not near any walls, move randomly in a safe space
        vel_msg.linear.x = random.uniform(1.0, 2.0)  # Move forward with random speed
        vel_msg.angular.z = random.uniform(-2.0, 2.0)  # Rotate randomly to avoid getting stuck

    # Gradually slow down or adjust direction if near walls
    if x < WALL_THRESHOLD + 1 or x > 11 - WALL_THRESHOLD - 1:
        vel_msg.linear.x = max(vel_msg.linear.x, 0.5)  # Don't go too fast near walls
    
    velocity_publisher.publish(vel_msg)

rospy.init_node("smooth_random_walker")


velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)


rospy.Subscriber('/turtle1/pose', Pose, callback)

while not rospy.is_shutdown():
    rospy.spin()
