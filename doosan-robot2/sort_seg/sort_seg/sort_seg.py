#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###################### I N I T I A L I S E ################################

from dsr_msgs2.msg import RobotStop, RobotState
from ament_index_python.packages import get_package_share_directory
import time
import sys
import os
import rclpy
sys.path.append(os.path.abspath("/home/jacobs/ros2_ws/src/doosan-robot2/common2/imp"))
import DR_init
sys.path.append(os.path.abspath("/home/jacobs/ros2_ws/src/doosan-robot2/sort_seg/sort_seg"))
from robot_controller import RobotController

node = RobotController()

DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = ''
DR_init.__dsr__node = node
from DSR_ROBOT2 import *


######################### F U N C T I O N S #############################

# get grip state
def get_grip_state():
    return get_digital_input(1) == 0

# print grip state
def print_grip_state():
   print("gripper is open" if get_grip_state() else "gripper is closed")

# open gripper
def open_grip():
    set_digital_output(1, 1) # (port number, ON)

# close gripper
def close_grip():
    set_digital_output(1, 0) # (port number, OFF)

def get_coords():
    x = node.x *1000.0
    y = node.y *1000.0

    # node.get_logger().info(f"Spring_height: {node.z}")
    # z = 362 + (node.z *1000.0) # height of  gripper and table
    z = 380 

    node.get_logger().info(f"Received coordinates: x={x}, y={y}, z={z}")
    return x, y, z

def move_to_midpoint():
    # Midpoint
    mid = posx(-85, 533, 450, 35, 180, 100)

    # move to midpoint
    movel(mid, vel = 80, acc = 50)
    time.sleep(1)

def move_to_bin():
    # Bin Position
    bin= posx(-596, -69, 652, 25, 180, 50)

    # move to bin
    movel(bin, vel = 80, acc = 50)
    open_grip()
    time.sleep(2)

def move_to_spring(x, y, z):
    # Spring Locations
    above_spring = posx(x, y, z+100, 35, 180, 100)  # 10cm above
    to_spring = posx(x, y, z, 35, 180, 100)

    # move to spring
    movel(above_spring, vel = 80, acc = 50)
    time.sleep(3)
    movel(to_spring, vel = 80, acc = 50)

def move(x, y, z):
    # Checks if it hits the table
    if (z > 362) and (y > 0): 
        # node.get_logger().info(f"moving to {x, y, z}")
        # move to midpoint
        move_to_midpoint()

        # move to spring
        move_to_spring(x, y, z)

        # pick up the spring
        close_grip()
        time.sleep(2)

        # move to midpoint
        move_to_midpoint()

        # move to bin
        move_to_bin()
    else:
        node.get_logger().info('HITS TABLE!')

def check_msg_time():
    message_time = node.last_message_time
    current_time = node.get_clock().now()

    if (current_time - message_time).nanoseconds > 10e9:  # 10 seconds
        node.get_logger().info('Timeout exceeded, stopping node')
        node.destroy_node()



######################### M A I N ###############################

def main():
    rclpy.spin_once(node)

    while rclpy.ok():
        # Check if any springs are being detected 
        check_msg_time()

        # Move to bin first
        move_to_bin()

        # Check if the gripper is open
        if get_grip_state() == False:
            open_grip()

        # Check if the arm is in motion
        if check_motion() == 0:
            # Get spring co-ordinates
            x, y, z = get_coords()

            # Move to co-ordinates
            move(x, y, z)


    # turn off node
    node.destroy_node()
    print("all done")

if __name__ == "__main__":
    main()
