#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from tf2_ros import TransformStamped
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Quaternion
import pyrealsense2 as rs
import sys
import os 
from rclpy.qos import qos_profile_sensor_data
import math
from scipy.spatial.transform import Rotation as R





class ColorDetectNode(Node):
    def __init__(self):
        super().__init__('color_detect_node')
        self.get_logger().info('Starting Color Detect Node')

        # Create subscribers
        self.color_image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_image_callback, 10)
        self.depth_image_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # Create a publisher for object coordinates
        self.object_coords_pub = self.create_publisher(Pose, '/color_detect_node/object_coords', 10)

        # Create a CvBridge object
        self.bridge = CvBridge()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize lists to store object coordinates and center coordinates
        self.object_coordinates = []
        self.center_coordinates = []
        self.color_image = None
        self.depth_image = None
        self.camera_info = None

    def color_image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def camera_info_callback(self, msg):
        self.camera_info = msg

        # Process frames when all data is available
        if self.color_image is not None and self.depth_image is not None and self.camera_info is not None:
            self.detect_object(self.color_image, self.depth_image, self.camera_info)

    def detect_object(self, color_image, depth_image, camera_info):
        # Get the intrinsic parameter
        intrinsics = camera_info.k

        intrinsics = rs.intrinsics()
        intrinsics.width = int(camera_info.k[2]) * 2
        intrinsics.height = int(camera_info.k[5]) * 2
        intrinsics.fx = camera_info.k[0]
        intrinsics.fy = camera_info.k[4]
        intrinsics.ppx = camera_info.k[2]
        intrinsics.ppy = camera_info.k[5]
        intrinsics.model = rs.distortion.brown_conrady
        intrinsics.coeffs = [0, 0, 0, 0, 0]

        # Calculate the pixel coordinates of the camera's zero-zero value
        zero_zero_x = int(intrinsics.ppx)
        zero_zero_y = int(intrinsics.ppy)

        # Convert color image from BGR to HSV
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        lower_limit = np.array([58, 183, 67])
        upper_limit = np.array([92, 255, 255])

        mask = cv2.inRange(hsv_image, lower_limit, upper_limit)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected objects
        self.center_coordinates = []
        for contour in contours:
            # x, y, w, h = cv2.boundingRect(contour)
            # area = w*h
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            (center, (width, height), angle) = rect
            area = width * height


            if 0 < area <= 1500:    # refine boundaries
                # # Get the rotated rectangle
                # rect = cv2.minAreaRect(contour)
                # # Draw the rotated rectangle
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)
                cv2.drawContours(color_image, [box], 0, (255, 0, 0), 2)

                # cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate center coordinates
                # center_x = x + w / 2 
                # center_y =  y + h / 2

                center_x = rect[0][0]
                center_y = rect[0][1]

                # Bounds checking
                center_x = max(0, min(center_x, depth_image.shape[1] - 1))
                center_y = max(0, min(center_y, depth_image.shape[0] - 1))

                # Get the depth value at the center coordinates
                depth_value = self.depth_image[int(center_y), int(center_x)]

                if depth_value != 0:

                    point = rs.rs2_deproject_pixel_to_point(intrinsics, [center_x, center_y], depth_value)

                    point_3d = [point[0] / 1000.0, point[1] / 1000.0, point[2] / 1000.0]

                    # Calculate orientation of the object
                    # angle_radians = math.radians(angle)
                    # qx, qy, qz, qw = quaternion.from_rotation_vector([0, 0, angle_radians])
                    # pose_orientation = quaternion.quaternion(qw, qx, qy, qz)
                    
                    # Annotate center coordinates and distance on the image
                    cv2.putText(color_image, f"({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})", (int(center_x), int(center_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                    # Publish each coordinate separately
                    msg = Pose()
                    msg.position.x = point_3d[0]
                    msg.position.y = point_3d[1]
                    msg.position.z = point_3d[2]
                    # msg.orientation.x = float(orientation[0])
                    # msg.orientation.y = float(orientation[1])
                    # msg.orientation.z = float(orientation[2])
                    # msg.orientation.w = float(orientation[3])
                    self.object_coords_pub.publish(msg)
                     # Broadcast the center fo the object
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_color_optical_frame'
                    t.child_frame_id = 'center'
                    t.transform.translation.x = point_3d[0]
                    t.transform.translation.y = point_3d[1]
                    t.transform.translation.z = point_3d[2]
                    self.tf_broadcaster.sendTransform(t)

        # Draw a cross at the camera's zero-zero value
        cv2.line(color_image, (zero_zero_x - 10, zero_zero_y), (zero_zero_x + 10, zero_zero_y), (255, 0, 0), 2) 
        cv2.line(color_image, (zero_zero_x, zero_zero_y - 10), (zero_zero_x, zero_zero_y + 10), (255, 0, 0), 2)

        # Display the color image with bounding boxes
        cv2.imshow('Color Image', color_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()