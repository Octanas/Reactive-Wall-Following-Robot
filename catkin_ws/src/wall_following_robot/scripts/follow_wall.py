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

    # Each region scans 9 degrees
    regions = {
        'N':  min(min(msg.ranges[len(msg.ranges) - 4 : len(msg.ranges) - 1] + msg.ranges[0:5]), scan_max_value),
        'NNW':  min(min(msg.ranges[11:20]), scan_max_value),
        'NW':  min(min(msg.ranges[41:50]), scan_max_value),
        'WNW':  min(min(msg.ranges[64:73]), scan_max_value),
        'W':  min(min(msg.ranges[86:95]), scan_max_value),
        'E':  min(min(msg.ranges[266:275]), scan_max_value),
        'ENE':  min(min(msg.ranges[289:298]), scan_max_value),
        'NE':  min(min(msg.ranges[311:320]), scan_max_value),
        'NNE':  min(min(msg.ranges[341:350]), scan_max_value),
    }

    global g_side, g_alpha, g_linear_speed, g_state, g_turn_start_time, g_wall_direction

    g_linear_speed = 0.1

    if g_state == 0:  # wander
        for r, v in regions.items():
            if r in ["N", "W", "E", "NW", "NE"] and v < scan_max_value:
                print('Will change to state 1')
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
        min_distance = distance_wall + 0.3

        if regions['N'] < min_distance or regions['NW'] < min_distance or regions['NE'] < min_distance:
            g_state = 2
            print('Will change to state 2')
            if g_side == 0:
                left = (regions['W'] + regions['NW']) / 2
                right = (regions['E'] + regions['NE']) / 2

                if left < scan_max_value or right < scan_max_value:
                    g_side = -1 if right < left else 1
                else:
                    g_side = random.randrange(-1, 2, 2)

                print('Chosen side: ', g_side)
            return

        delta_time = time.time() - g_turn_start_time

        if delta_time <= 1:
            if g_wall_direction == 'W':
                g_alpha = math.pi / 2
            elif g_wall_direction == 'NW':
                g_alpha = math.pi / 4
            elif g_wall_direction == 'N':
                g_alpha = 0
            elif g_wall_direction == 'NE':
                g_alpha = -math.pi / 4
            elif g_wall_direction == 'E':
                g_alpha = -math.pi / 2
        else:
            g_alpha = 0
    elif g_state == 2:  # follow wall
        if g_side == -1:
            y0 = regions['E']
            x1 = regions['ENE'] * math.sin(math.radians(23))
            y1 = regions['ENE'] * math.cos(math.radians(23))
        else:
            y0 = regions['W']
            x1 = regions['WNW'] * math.sin(math.radians(23))
            y1 = regions['WNW'] * math.cos(math.radians(23))

        print('y0: ', y0)

        front_scan = min([regions['N'], regions['NNW'] + (scan_max_value - regions['WNW']), regions['NNE'] + (scan_max_value - regions['ENE'])])

        print('front_scan: ', front_scan)

        if y0 >= distance_wall * 2 and regions['N'] < scan_max_value:
            print('NOT USING ALGORITHM')
            g_alpha = -math.pi / 4 * g_side
        else:
            print('USING ALGORITHM')
            turn_fix = (0 if front_scan >= 0.5 else 1 - front_scan)

            print('Turn fix: ', turn_fix)

            abs_alpha = math.atan2(y1 - distance_wall,
                                x1 + wall_lead - y0) - turn_fix * 1.5
            
            print('Abs alpha: ', abs_alpha)

            g_alpha = g_side * abs_alpha

        print('Alpha: ', g_alpha)
        print('')


if __name__ == '__main__':
    rospy.init_node('wall_following_robot')

    g_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    g_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        update_command_vel(g_linear_speed, g_alpha)
        rate.sleep()
