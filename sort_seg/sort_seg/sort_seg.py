#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###################### I N I T I A L I S E ################################

from dsr_msgs2.msg import RobotStop, RobotState
import time
import sys
import os
import rclpy
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
from pynput import keyboard
imp_path=os.path.abspath(os.path.join(os.path.abspath(__file__),"../../../../../../../src/Sort-and-Segregate/common2/imp"))
sys.path.append((imp_path))
import DR_init
controller_path=os.path.abspath(os.path.join(os.path.abspath(__file__), '../'))
sys.path.append(controller_path)
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
    time.sleep(2)

# close gripper
def close_grip():
    set_digital_output(1, 0) # (port number, OFF)
    time.sleep(2)

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

def quaternion_to_euler(qx, qy, qz, qw):
    # Extract quaternion orientation values
    
    rotation = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

def move(x, y, z, qx, qy, qz, qw):
    # Checks if it hits the table
    if (z > 362) and (y > 0): 
        node.get_logger().info(f"moving to {x, y, z}")

        # Convert quaternion to Euler angles (roll, pitch, yaw) if needed
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch) + 180  # Add 180 degrees to pitch to account for gripper being upside down
        yaw_deg = math.degrees(yaw) 

        node.get_logger().info(f"Received angles: r={roll_deg}, p={pitch_deg}, y={yaw_deg}")

        bin = posx(-645, -68, 675, roll_deg, 180, yaw_deg + 45)
        mid = posx(-85, 533, 680,roll_deg, 180, yaw_deg + 45)
        above_spring = posx(x, y, z+120, roll_deg, 180, -yaw_deg)
        spring = posx(x, y, z, roll_deg, 180, -yaw_deg)
        to_spring_x_pos = [bin, mid, above_spring, spring]
        to_spring_x_neg = [bin, above_spring, spring]
        to_bin_x_pos = [above_spring, mid, bin]
        to_bin_x_neg = [above_spring, bin]

        if x >= 0:
            for i in to_spring_x_pos:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            close_grip()

            for i in to_bin_x_pos:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            open_grip()

        else:
            for i in to_spring_x_neg:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            close_grip()

            for i in to_bin_x_neg:
                movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            open_grip()

    else:
        node.get_logger().info('HITS TABLE OR OUT OF BOUNDS!')

def check_msg_time():
    message_time = node.last_message_time
    current_time = node.get_clock().now()

    if (current_time - message_time).nanoseconds > 10e9:  # 10 seconds
        # Bin and tipping location
        mid = posx(-85, 533, 680,38, 180, 40)
        bin_above = posx(-635, -165, 530, 25, 180, -65)
        bin = posx(-635, -165, 475, 25, 180, -65)
        tip_location = posx(456, -2, 720, 36, 180, 120) 
        tip_bin =  posx(456, 10, 720, 95, 90, -180) 
        tip_shake_1 = posx(456, 10, 720, 95, 90, -135)
        tip_shake_2 = posx(456, 10, 720, 95, 93, 135)
        bin_grab = [bin_above, bin]
        tip = [bin_above, mid, tip_location, tip_bin, tip_shake_1, tip_shake_2, tip_bin, tip_location, mid, bin_above, bin]
        # Redistribute Springs
        node.get_logger().info('All springs colleccted, Time to make a mess!')
        # Grab bin
        for i in bin_grab:
            movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
            time.sleep(0.5)
        close_grip()
        # Tip and return bin
        for i in tip:
            movel(i, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
            time.sleep(0.5)
        open_grip()
        movel(bin_above, vel = 100, acc = 65, ra = DR_MV_RA_DUPLICATE)
        time.sleep(2)

######################### M A I N ###############################

def main():
    rclpy.spin_once(node)

    while rclpy.ok():
        # Check if any springs are being detected 
        check_msg_time()

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
