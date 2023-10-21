#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
twist_msg = Twist()

def scan_callback(msg):
    ranges = msg.ranges

    # Check if there is any obstacle within 30 cm
    for i in range(len(ranges)):
        if ranges[i] < 0.3:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            cmd_vel_pub.publish(twist_msg())
            return

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.spin()

