#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
class PosPoll:
    def __init__(self):
        # Add your subscribers to the class instance here, ex. :
        # self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.sub_odo = rospy.Subscriber("/odometry/filtered", Odometry, self.print_odo)
        print("pos_poll node started.")

        # Creates a ROS Timer that will call the timer_cb method every 1.0 sec:
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)

    # Timer callback:
    def timer_cb(self, event):
        print("Timer event.")

    # Subscriber callback:
    # def scan_cb(self, msg):
    #   print("Got msg from /scan")
    
    def print_odo(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print(f"yaw : {yaw}")
        print(f"x : {x}")
        print(f"y : {y}")

if __name__ == "__main__":
    rospy.init_node("pos_poll")

    node = PosPoll()

    rospy.spin()
