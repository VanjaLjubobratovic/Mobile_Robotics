from typing import Set

from numpy.core.defchararray import upper
import rclpy
import math
from rclpy.node import Node
from rclpy.utilities import get_available_rmw_implementations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel, GetWorldProperties
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
import sys
import numpy as np

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from zad32.blob_detector import *
import time

K_LAT_DIST_TO_STEER     = 2.0

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class Robot_Spinner(Node):
    def __init__(self, thr_min, thr_max, blur = 15, blob_params=None, detection_window=None):
        super().__init__('robotSpinner')
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        self._t0 = time.time()
        self.blob_point = Point()

        self.declare_parameter('r_min', 0)
        self.declare_parameter('g_min', 0)
        self.declare_parameter('b_min', 50)

        self.declare_parameter('r_max', 255)
        self.declare_parameter('g_max', 255)
        self.declare_parameter('b_max', 255)

        self.declare_parameter('input_topic', '/robot/camera/image')
        self.declare_parameter('output_topic', '/robot/camera/blob_location')

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Input Topic: {self.get_parameter("input_topic").value}')
        self.get_logger().info(f'Output Topic: {self.get_parameter("output_topic").value}')

        self.twistPublisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

        self.subscribeCamera = self.create_subscription(
            Image,
            self.get_parameter('input_topic').value,
            self.read_camera,
            10
        )

    def parameter_callback(self, parameter_list):
        for p in parameter_list:
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]
    
    def set_blur(self, blur):
        self._blur = blur
    
    def set_blob_params(self, blob_params):
        self._blob_params = blob_params
    
    # def get_blob_relative_position(self, image, keyPoint):
    #     cols = float(image.shape[0])
    #     rows = float(image.shape[1])

    #     center_x = 0.5*cols
    #     center_y = 0.5*rows

    #     x = (keyPoint.pt[0] - center_x) / (center_x)
    #     y = (keyPoint.pt[1] - center_y) / (center_y)
    #     return (x,y)

    def read_camera(self, msg):
        #--- image is 320x240
        frame_img = CvBridge().imgmsg_to_cv2(msg)
        r_min = int(self.get_parameter('r_min').value)
        g_min = int(self.get_parameter('g_min').value)
        b_min = int(self.get_parameter('b_min').value)

        r_max = int(self.get_parameter('r_max').value)
        g_max = int(self.get_parameter('g_max').value)
        b_max = int(self.get_parameter('b_max').value)

        frame = cv2.inRange(frame_img, (b_min, g_min, r_min), (b_max, g_max, r_max))
        #cv2.imshow("Threshold", frame)

        (rows, cols, channels) = frame_img.shape
        #self.get_logger().info(f'Rows, cols: {rows, cols}')
        if cols > 60 and rows > 60:
            keypoints, frame = blob_detect(frame, self._threshold[0], self._threshold[1], 0,
                                        blob_params = self._blob_params, search_window=self.detection_window,
                                         imshow=False)
            #frame = blur_outside(frame, 10, self.detection_window)
            frame = draw_window(frame, self.detection_window)
            #frame = draw_frame(frame)
            frame = draw_keypoints(frame, keypoints)
            #self.get_logger().info(f'Keypoints: {keypoints}')
            cv2.imshow("FRAME", frame)
            cv2.waitKey(25)
            
            for i, keyPoint in enumerate(keypoints):
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size

                x,y = get_blob_relative_position(frame, keyPoint)
    
                
                self.blob_point.x = x
                self.blob_point.y = y
                #publish tocke na temu
                self.get_logger().info(f'Set x,y to: {self.blob_point.x, self.blob_point.y}')
                #self.publisher.publish(self.blob_point)
                break
        fps = 1.0/(time.time() - self._t0)
        self._t0 = time.time()

        steer_action = K_LAT_DIST_TO_STEER * self.blob_point.x * (-1)
        steer_action = saturate(steer_action, -0.5, 0.5)

        message = Twist()
        message.linear.x = 0.0
        message.angular.z = steer_action
        #self.get_logger().info(f'Set steer to: {steer_action}')
        self.twistPublisher.publish(message)

        #self.get_logger().info("Hello")

        # r_min = int(self.get_parameter('r_min').value)
        # g_min = int(self.get_parameter('g_min').value)
        # b_min = int(self.get_parameter('b_min').value)

        # r_max = int(self.get_parameter('r_max').value)
        # g_max = int(self.get_parameter('g_max').value)
        # b_max = int(self.get_parameter('b_max').value)


        # frame_threshold = cv2.inRange(frame, (b_min, g_min, r_min), (b_max, g_max, r_max))
        # cv2.imwrite('camera_image.jpeg', frame)
        # cv2.imwrite('camera_image_threshold.jpeg', frame_threshold)
        #self.get_logger().info("Image written")

        #image_msg = CvBridge().cv2_to_imgmsg(frame_threshold, encoding="mono8")
        #image_msg.header = msg.header
        #self.publisher.publish(image_msg)


def main(args = None):
        blue_min = (50, 0, 0)
        blue_max = (255, 255, 255)

        blur = 5
        min_size = 10
        max_size = 40

        x_min = 0.0
        x_max = 1.0
        y_min = 0.0
        y_max = 1.0

        detection_window = [x_min, y_min, x_max, y_max]
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 100;
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 20
        params.maxArea = 20000
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.2
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.7 

        rclpy.init(args = args)
        robotSpinner = Robot_Spinner(blue_min, blue_max, blur, params, detection_window)
        rclpy.spin(robotSpinner)
        rclpy.shutdown()

if __name__ == "__main__":
    main()