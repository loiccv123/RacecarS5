#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import message_filters
import tf
import tf2_ros
from racecar_behaviors.cfg import BlobDetectorConfig
from dynamic_reconfigure.server import Server
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from libbehaviors import *
from geometry_msgs.msg import Twist

class BlobDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.object_frame_id = rospy.get_param('~object_frame_id', 'object')
        self.color_hue = rospy.get_param('~color_hue', 10) # 160=purple, 100=blue, 10=Orange
        self.color_range = rospy.get_param('~color_range', 15) 
        self.color_saturation = rospy.get_param('~color_saturation', 50) 
        self.color_value = rospy.get_param('~color_value', 50) 
        self.border = rospy.get_param('~border', 10) 
        self.config_srv = Server(BlobDetectorConfig, self.config_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle_detected = False
        self.target_distance = 0.75
        self.linear_speed = 2  
        self.angular_speed = 0.8 
        self.goal_reached_tolerance = 0.1 
        self.angle_adjust = 0
        self.distance_adjust = 0
        
        params = cv2.SimpleBlobDetector_Params()
        # see https://www.geeksforgeeks.org/find-circles-and-ellipses-in-an-image-using-opencv-python/
        #     https://docs.opencv.org/3.4/d0/d7a/classcv_1_1SimpleBlobDetector.html
        
        params.thresholdStep = 10;
        params.minThreshold = 50;
        params.maxThreshold = 220;
        params.minRepeatability = 2;
        params.minDistBetweenBlobs = 10;
        
        # Set Color filtering parameters 
        params.filterByColor = False
        params.blobColor = 255
        
        # Set Area filtering parameters 
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 5000000000
          
        # Set Circularity filtering parameters 
        params.filterByCircularity = True 
        params.minCircularity = 0.3
          
        # Set Convexity filtering parameters 
        params.filterByConvexity = False
        params.minConvexity = 0.95
              
        # Set inertia filtering parameters 
        params.filterByInertia = False
        params.minInertiaRatio = 0.1

        self.detector = cv2.SimpleBlobDetector_create(params)
        
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        self.image_pub = rospy.Publisher('image_detections', Image, queue_size=1)
        self.object_pub = rospy.Publisher('object_detected', String, queue_size=1)
        
        self.image_sub = message_filters.Subscriber('image', Image)
        self.depth_sub = message_filters.Subscriber('depth', Image)
        self.info_sub = message_filters.Subscriber('camera_info', CameraInfo)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub, self.info_sub], 10)
        self.ts.registerCallback(self.image_callback)
        
    def config_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {color_hue}, {color_saturation}, {color_value}, {color_range}, {border}""".format(**config))
        self.color_hue = config.color_hue
        self.color_range = config.color_range
        self.color_saturation = config.color_saturation
        self.color_value = config.color_value
        self.border = config.border
        return config
  
    def image_callback(self, image, depth, info):
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            print(e)
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, np.array([self.color_hue-self.color_range,self.color_saturation,self.color_value]), np.array([self.color_hue+self.color_range,255,255]))
        keypoints = self.detector.detect(mask) 
        
        closestObject = [0,0,0] # Object pose (x,y,z) in camera frame (x->right, y->down, z->forward)
        if len(keypoints) > 0:
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            for i in range(0, len(keypoints)):
                if info.K[0] > 0 and keypoints[i].pt[0] >= self.border and keypoints[i].pt[0] < cv_image.shape[1]-self.border:
                    pts_uv = np.array([[[keypoints[i].pt[0], keypoints[i].pt[1]]]], dtype=np.float32)
                    info_K = np.array(info.K).reshape([3, 3])
                    info_D = np.array(info.D)
                    info_P = np.array(info.P).reshape([3, 4])
                    pts_uv = cv2.undistortPoints(pts_uv, info_K, info_D, info_P)
                    angle = np.arcsin(-pts_uv[0][0][0]) # negative to get angle from forward x axis
                    x = pts_uv[0][0][0]
                    y = pts_uv[0][0][1]
                    #rospy.loginfo("(%d/%d) %f %f -> %f %f angle=%f deg", i+1, len(keypoints), keypoints[i].pt[0], keypoints[i].pt[1], x, y, angle*180/np.pi)
                    
                    # Get depth.
                    u = int(x * info.P[0] + info.P[2])
                    v = int(y * info.P[5] + info.P[6])
                    depth = -1
                    if u >= 0 and u < cv_depth.shape[1]:
                        for j in range(0, cv_depth.shape[0]):
                            if cv_depth[j, u] > 0:
                                depth = cv_depth[j, u]
                                break
                                # is the depth contained in the blob?
                                if abs(j-v) < keypoints[i].size/2:
                                    depth = cv_depth[j, u]
                                    break
                                
                    if depth > 0 and (closestObject[2]==0 or depth<closestObject[2]):
                        closestObject[0] = x*depth
                        closestObject[1] = 0 # to be same height than camera
                        closestObject[2] = depth

        # We process only the closest object detected
        if closestObject[2] > 0:
            # assuming the object is circular, use center of the object as position
            transObj = (closestObject[0], closestObject[1], closestObject[2])
            rotObj = tf.transformations.quaternion_from_euler(0, np.pi/2, -np.pi/2)
            self.br.sendTransform(transObj, rotObj,
                    image.header.stamp,
                    self.object_frame_id,
                    image.header.frame_id)  
            msg = String()
            msg.data = self.object_frame_id
            self.object_pub.publish(msg) # signal that an object has been detected
            self.obstacle_detected = True
            
            # Compute object pose in map frame
            try:
                self.listener.waitForTransform(self.map_frame_id, image.header.frame_id, image.header.stamp, rospy.Duration(0.5))
                (transMap,rotMap) = self.listener.lookupTransform(self.map_frame_id, image.header.frame_id, image.header.stamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                print(e)
                return
            (transMap, rotMap) = multiply_transforms(transMap, rotMap, transObj, rotObj)
            
            # Compute object pose in base frame
            try:
                self.listener.waitForTransform(self.frame_id, image.header.frame_id, image.header.stamp, rospy.Duration(0.5))
                (transBase,rotBase) = self.listener.lookupTransform(self.frame_id, image.header.frame_id, image.header.stamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                print(e)
                return
            (transBase, rotBase) = multiply_transforms(transBase, rotBase, transObj, rotObj)
            
            distance = np.linalg.norm(transBase[0:2])
            angle = np.arcsin(transBase[1]/transBase[0])
            self.angle_adjust = angle*180.0/np.pi
            self.distance_adjust = distance
            
            rospy.loginfo("Object detected at [%f,%f] in %s frame! Distance and direction from robot: %fm %fdeg.", transMap[0], transMap[1], self.map_frame_id, distance, angle*180.0/np.pi)

        # debugging topic
        if self.image_pub.get_num_connections()>0:
            cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

    def stabilize_obstacle(self):
        rate = rospy.Rate(10)  # Control the rate of obstacle detection
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                while self.obstacle_detected and not rospy.is_shutdown():
                    twist_cmd = Twist()
                    if abs(self.angle_adjust) > 0.5:  # Adjust the threshold for turning direction change
                        # Change steering direction
                        twist_cmd.angular.z = -self.angular_speed if self.angle_adjust > 0 else self.angular_speed
                        rospy.loginfo("angle_adjust=%f",twist_cmd.angular.z)
                        self.cmd_vel_pub.publish(twist_cmd);
                    else:
                        # Move forward if angle is approximately 0
                        twist_cmd.linear.x = self.linear_speed
                        twist_cmd.angular.z = 0
                        self.cmd_vel_pub.publish(twist_cmd);
                        # Check if distance is close to the desired distance
                        if self.distance_adjust > self.target_distance + self.goal_reached_tolerance:
                            # Move towards the obstacle
                            twist_cmd.linear.x = self.linear_speed
                            self.cmd_vel_pub.publish(twist_cmd);
                        elif self.distance_adjust < self.target_distance - self.goal_reached_tolerance:
                            # Move away from the obstacle
                            twist_cmd.linear.x = -self.linear_speed
                            self.cmd_vel_pub.publish(twist_cmd);
                        else:
                            twist_cmd.linear.x = 0.0
                            self.obstacle_detected = False
                            self.cmd_vel_pub.publish(twist_cmd);

                    rate.sleep()
      
def main():
    rospy.init_node('blob_detector')
    blobDetector = BlobDetector()
    blobDetector.stabilize_obstacle()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

