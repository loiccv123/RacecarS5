#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

def move_base_send_coordinates(x, y, theta):
    try:
        # Initialize ROS node
        rospy.init_node("move_base_sender")

        # Create SimpleActionClient for move_base
        client = SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # Create MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'racecar/map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        quat = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        #goal.target_pose.pose.orientation.w = 1.0  # Assume no rotation for simplicity
 
        # Send the goal to move_base
        client.send_goal(goal)
        client.wait_for_result()

        # Check the result
        if client.get_state() == 3:  # SUCCEEDED
            rospy.loginfo("Goal reached successfully!")
            x_origin = 0
            y_origin = 0
            move_base_send_coordinates(x_origin, y_origin, 3.14159283)
            return True
        else:
            rospy.loginfo("Failed to reach the goal.")
            return False

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
        return False

if __name__ == "__main__":
    x_coord = 13.5
    y_coord = 2.1
    move_base_send_coordinates(x_coord, y_coord, 0)
