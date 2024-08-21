#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from dsr_msgs2.msg import RobotStop, RobotState
from ament_index_python.packages import get_package_share_directory
import DR_init

###########################################
################ N O D E S ##################

# Global variables
ROBOT_ID = "dsr01"
ROBOT_MODEL = ''
spring = None # spring location
spring_detected= False

def msgRobotState_cb(node, msg):
    node.get_logger().info(f"Robot State: {msg.robot_state}, {msg.robot_state_str}")

# Start camera node
rclpy.init()
cam_node = Node('realsense_camera_node')
cam_node.get_logger().info('Starting RealSense Camera Node')

# Start RealSense camera
cam_node.pipeline = rs.pipeline()
cam_node.config = rs.config()
cam_node.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cam_node.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cam_node.pipeline.start(cam_node.config)

# Create a timer to capture frames
cam_node.timer = cam_node.create_timer(0.033, cam_node.capture_frames)

# Initialize lists to store bounding box coordinates and center coordinates
cam_node.bounding_box_coordinates = []
cam_node.center_coordinates = []

# Start robot node
robo_node = Node('robot_node')
robo_node.pub_stop = robo_node.create_publisher(RobotStop, f'/{ROBOT_ID}{ROBOT_MODEL}/stop', 10)
robo_node.create_subscription(RobotState, f'/{ROBOT_ID}{ROBOT_MODEL}/joint_states', msgRobotState_cb, 10)
DR_init.__dsr__robo_node = robo_node
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT2 import * 

#################################################
################ M E T H O D S ##################

def capture_frames(cam_node):
# Wait for a coherent pair of frames: depth and color
    frames = cam_node.pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        return

    # Convert color frame to numpy array
    color_image = np.asanyarray(color_frame.get_data())

    # Convert color image from BGR to HSV
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Lower and upper limits for the color
    lower_limit = np.array([40, 40, 40])  
    upper_limit = np.array([80, 255, 255])  

    mask = cv2.inRange(hsv_image, lower_limit, upper_limit)
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding boxes around detected objects
    current_bounding_boxes = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w*h
        if 50 <= area <= 100:    # refine boundaries
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            current_bounding_boxes.append((x, y, x + w, y + h))

            # Calculate center coordinates
            center_x = x + w / 2
            center_y = y + h / 2

            # Get depth value at center coordinates
            distance = depth_frame.get_distance(int(center_x), int(center_y))

            if distance != 0:
                # Print distance
                cam_node.get_logger().info(f"Distance: {distance:.2f}")

                # Store center coordinates
                spring_location = cam_node.center_coordinates.append((center_x, center_y, distance))

                # Store bounding boxes
                cam_node.bounding_box_coordinates.append((x, y, w, h))

                # Annotate center coordinates and distance on the image
                cv2.putText(color_image, f"({center_x:.2f}, {center_y:.2f}, {distance:.2f}m)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)

    # Display the color image with bounding boxes
    cv2.imshow('Color Image', color_image)
    cv2.waitKey(1)
    # If spring detected
    if spring_location:
        spring_detected = True
        set_spring(spring_location)
    else:
        spring_detected = False
        spring_location = None
    return spring_detected

def set_spring(spring_coord): # Set spring location
    # MAP FUNCTIONS HERE
    spring = [spring_coord, 0, 0, 0]
    return spring

# Get grip state
def get_grip_state():
    return get_digital_input(1) == 0

# Print grip state
def print_grip_state():
    print("gripper is open" if get_grip_state() else "gripper is closed")

def open_grip():
    set_digital_output(1, 1) # (port number, ON)

def close_grip():
    set_digital_output(1, 0) # (port number, OFF)


###########################################
############# M A I N #####################

def main():

    # Robot postition and co-ordinate setup
    set_ref_coord(101) # user co-ordinates
    P0 = [0, 0, 0, 0, 0, 0] # rest position
    bin = [90, 90, 90, 0, 0, 0] # location of bin
    # acceleration and velocity values
    a = 30
    v = 40
    # Move to rest position
    movel(P0, time = 10)

    # Opens the gripper if closed
    if not get_grip_state():
        open_grip()

    # call camera function
    capture_frames(cam_node)

    if spring_detected == True:
        # Move robot to spring location
        movel(spring, v, a) 
        close_grip()
        # Move robot to bin location
        movel(bin, v, a)
        open_grip()
        spring = None
        spring_detected == False
    else:
        movel(P0, v, a)
        print("No springs detected, shutting down.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()


