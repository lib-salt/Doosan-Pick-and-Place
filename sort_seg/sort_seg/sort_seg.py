#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###################### I N I T I A L I S E ################################

from dsr_msgs2.msg import RobotStop, RobotState
from ament_index_python.packages import get_package_share_directory
import time
import sys
import os
import rclpy
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
sys.path.append(os.path.abspath("/home/jacobs/ros2_ws/src/Sort-and-Segregate/common2/imp"))
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),"../../common2/imp")))
import DR_init
sys.path.append(os.path.abspath("/home/jacobs/ros2_ws/src/Sort-and-Segregate/sort_seg/sort_seg"))
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
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
    if node.z < 0.02:
        z = 380
    else:
        z = 369 + (node.z *1000.0) # height of  gripper and table
    node.get_logger().info(f"Received coordinates: x={x}, y={y}, z={z}")

    qx = node.orientation.x
    qy = node.orientation.y
    qz = node.orientation.z
    qw = node.orientation.w

    return x, y, z, qx, qy, qz, qw

# def move_to_midpoint():
#     # Midpoint
#     mid = posx(-85, 533, 450, 35, 180, 100)

#     # move to midpoint
#     movel(mid, vel = 80, acc = 50)
#     time.sleep(1)

# def move_to_bin():
#     # Bin Position
#     bin= posx(-596, -69, 652, 25, 180, 50)

#     # move to bin
#     movel(bin, vel = 80, acc = 50)
#     open_grip()
#     time.sleep(2)

# def move_to_spring(x, y, z):
#     # Spring Locations
#     above_spring = posx(x, y, z+100, 35, 180, 100)  # 10cm above
#     to_spring = posx(x, y, z, 35, 180, 100)

#     # move to spring
#     movel(above_spring, vel = 80, acc = 50)
#     time.sleep(3)
#     movel(to_spring, vel = 80, acc = 50)


def quaternion_to_euler(qx, qy, qz, qw):
    # Extract quaternion orientation values
    
    rotation = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

def move(x, y, z, qx, qy, qz, qw):
    # Checks if it hits the table
    if (z > 362) and (y > 0): 
        # node.get_logger().info(f"moving to {x, y, z}")
        # move to midpoint

        # Convert quaternion to Euler angles (roll, pitch, yaw) if needed
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch) + 180  # Add 180 degrees to pitch to account for gripper being upside down
        yaw_deg = math.degrees(yaw) 

        # node.get_logger().info(f"Received angles: r={roll_deg}, p={pitch_deg}, y={yaw_deg}")

        bin = posx(-596, -69, 680, roll_deg, 180, yaw_deg + 45)
        mid = posx(-85, 533, 680,roll_deg, 180, yaw_deg + 45)
        above_spring = posx(x, y, z+120, roll_deg, 180, yaw_deg)
        spring = posx(x, y, z, roll_deg, 180, yaw_deg)
        to_spring_x_pos = [bin, mid, above_spring, spring]
        to_spring_x_neg = [bin, above_spring, spring]
        to_bin_x_pos = [above_spring, mid, bin]
        to_bin_x_neg = [above_spring, bin]

        if x >= 0:
            for i in to_spring_x_pos:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            close_grip()
            time.sleep(2)

            for i in to_bin_x_pos:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            open_grip()
            time.sleep(2)
        else:
            for i in to_spring_x_neg:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            close_grip()
            time.sleep(2)

            for i in to_bin_x_neg:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            open_grip()
            time.sleep(2)

        # if x >= 0:
        #     movesx(to_spring_x_pos, vel = 100, acc = 200, mod = DR_MV_MOD_ABS, vel_opt = DR_MVS_VEL_NONE)
        #     close_grip()
        #     time.sleep(2)
        #     movesx(to_bin_x_pos, vel = 100, acc = 200, mod = DR_MV_MOD_ABS, vel_opt = DR_MVS_VEL_NONE)
        # else:
        #     movesx(to_spring_x_neg, vel = 100, acc = 200, mod = DR_MV_MOD_ABS, vel_opt = DR_MVS_VEL_NONE)
        #     close_grip()
        #     time.sleep(2)
        #     movesx(to_bin_x_neg, vel = 100, acc = 200, mod = DR_MV_MOD_ABS, vel_opt = DR_MVS_VEL_NONE)

        # open_grip()
        # time.sleep(2)

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
        # move_to_bin()

        # Check if the gripper is open
        if get_grip_state() == False:
            open_grip()

        # Check if the arm is in motion
        if check_motion() == 0:
            # Get spring co-ordinates
            x, y, z , qx, qy, qz, qw= get_coords()

            # Move to co-ordinates
            move(x, y, z, qx, qy, qz, qw)


    # turn off node
    node.destroy_node()
    print("all done")

if __name__ == "__main__":
    main()
