#!/usr/bin/env python

import rospy
import os
from astar import AStarAlgorithm
import math
import numpy as np
import heapq
from PIL import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class PathFollowing:
    def __init__(self, start_pos, goal_pos):
        self.max_speed = rospy.get_param("~max_speed", 1)
        self.max_steering = rospy.get_param("~max_steering", 0.37)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        # self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.start_pos = start_pos
        self.goal_pos = goal_pos

        astar_instance = AStarAlgorithm(
            "/home/chris/catkin_ws/src/RacecarS5/racecar_behaviors/scripts/brushfire.bmp"
        )
        path, cost = astar_instance.astar(
            start_pos,
            goal_pos,
            astar_instance.m_cost,
            astar_instance.m_n,
            astar_instance.m_h,
        )

        print(
            "Le plus court chemin entre %s et %s est : %s (%d)"
            % (start_pos, goal_pos, path, cost)
        )


def main():
    rospy.init_node("path_following")
    pathFollowing = PathFollowing((315, 167), (59, 135))
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
