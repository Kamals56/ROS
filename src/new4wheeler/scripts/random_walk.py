#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RandomWalk:
    def __init__(self):
        rospy.init_node('random_walk')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.rate = rospy.Rate(10)
        self.safe_distance = 0.5  # 50cm safety margin
        self.obstacle_detected = False
        
    def laser_callback(self, msg):
        # Check front 90 degree cone (simplified)
        front_scan = msg.ranges[:45] + msg.ranges[-45:]
        self.obstacle_detected = any(d < self.safe_distance for d in front_scan if d > msg.range_min)
        
    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.obstacle_detected:
                # Avoidance behavior
                twist.angular.z = random.uniform(0.5, 1.0) * random.choice([-1, 1])
                twist.linear.x = -0.1  # slight reverse
                rospy.loginfo("Obstacle detected! Turning away")
            else:
                # Random exploration
                twist.linear.x = random.uniform(0.1, 0.3)
                twist.angular.z = random.uniform(-0.3, 0.3)
                rospy.loginfo("Exploring randomly")
                
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        walker = RandomWalk()
        walker.run()
    except rospy.ROSInterruptException:
        pass