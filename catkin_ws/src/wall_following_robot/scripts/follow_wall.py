#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

g_pub = None
g_sub = None

def update_command_vel(linear_vel, angular_vel):
    msg = Twist()
    msg.linear.x = linear_vel
    msg.angular.z = angular_vel
    g_pub.publish(msg)

def callback(obj):
    print(obj)

if __name__ == '__main__':
    rospy.init_node('wall_following_robot')

    g_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    g_sub = rospy.Subscriber('/scan', LaserScan, callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        update_command_vel(0.2, 0)
        rate.sleep()