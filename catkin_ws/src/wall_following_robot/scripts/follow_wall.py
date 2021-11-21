#!/usr/bin/env python

import math
import random
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

distance_wall = 0.2
wall_lead = 0.4

g_pub = None
g_sub = None

g_side = 0

g_alpha = 0

g_linear_speed = 0

g_state = 0

g_turn_start_time = 0

g_wall_direction = None


def update_command_vel(linear_vel, angular_vel):
    msg = Twist()
    msg.linear.x = linear_vel
    msg.angular.z = angular_vel
    g_pub.publish(msg)


def scan_callback(msg):
    scan_max_value = msg.range_max

    regions = {
        'front':  min(msg.ranges[0], scan_max_value),
        'fright':  min(msg.ranges[315], scan_max_value),
        'right':  min(msg.ranges[270], scan_max_value),
        'fleft':  min(msg.ranges[45], scan_max_value),
        'left':  min(msg.ranges[90], scan_max_value),
    }

    global g_side, g_alpha, g_linear_speed, g_state, g_turn_start_time, g_wall_direction

    if g_state == 0:  # wander
        g_linear_speed = 0.1

        for r, v in regions.items():
            if v < scan_max_value:
                g_state = 1
                g_wall_direction = r
                g_turn_start_time = time.time()
                return

        delta_time = time.time() - g_turn_start_time

        if delta_time > 6:
            rand = random.randrange(0, 5)

            g_alpha = math.pi / 2 - rand * math.pi / 4

            g_turn_start_time = time.time()
        elif delta_time > 1:
            g_alpha = 0
    elif g_state == 1:  # drive towards wall
        g_linear_speed = 0.1

        if regions['front'] < 0.4:
            g_state = 2
            if g_side == 0:
                left = (regions['left'] + regions['fleft']) / 2
                right = (regions['right'] + regions['fright']) / 2

                if left < scan_max_value or right < scan_max_value:
                    g_side = -1 if right < left else 1
                else:
                    g_side = random.randrange(-1, 2, 2)
            return

        delta_time = time.time() - g_turn_start_time

        if delta_time <= 1:
            if g_wall_direction == 'left':
                g_alpha = math.pi / 2
            elif g_wall_direction == 'fleft':
                g_alpha = math.pi / 4
            elif g_wall_direction == 'front':
                g_alpha = 0
            elif g_wall_direction == 'fright':
                g_alpha = -math.pi / 4
            elif g_wall_direction == 'right':
                g_alpha = -math.pi / 2
        else:
            g_alpha = 0
    elif g_state == 2:  # follow wall
        if g_side == -1:
            y0 = regions['right']
            x1 = regions['fright'] * math.sin(math.pi / 4)
            y1 = regions['fright'] * math.cos(math.pi / 4)
        else:
            y0 = regions['left']
            x1 = regions['fleft'] * math.sin(math.pi / 4)
            y1 = regions['fleft'] * math.cos(math.pi / 4)

        turn_fix = (0 if regions['front'] > 0.5 else 0.7 - regions['front'])

        g_alpha = g_side * (math.atan2(y1 - distance_wall,
                            x1 + wall_lead - y0) + turn_fix)

        print(g_alpha)


if __name__ == '__main__':
    rospy.init_node('wall_following_robot')

    g_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    g_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        update_command_vel(g_linear_speed, g_alpha)
        rate.sleep()
