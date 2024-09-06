#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###################### I N I T I A L I S E ################################

from dsr_msgs2.msg import RobotStop, RobotState
import time
import sys
import os
import rclpy
from scipy.spatial.transform import Rotation as R
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

# Get grip state
def get_grip_state():
    return get_digital_input(1) == 0

# Print grip state
def print_grip_state():
   print("gripper is open" if get_grip_state() else "gripper is closed")

# Open gripper
def open_grip():
    set_digital_output(1, 1) # (port number, ON)
    time.sleep(2)

# Close gripper
def close_grip():
    set_digital_output(1, 0) # (port number, OFF)
    time.sleep(2)

def get_coords():
    x = node.x *1000.0
    y = node.y *1000.0
    # node.get_logger().info(f"Spring_height: {node.z}")
    if node.z < 0.02:
        z = 285 # old value: 380
    else:
        if x >=0: # Acounts on slight variation bettween two sides of the table
            z = 268 + (node.z *1000.0)
        else:
            z = 278 + (node.z *1000.0) # height of  gripper # old value: 369
    # node.get_logger().info(f"Received coordinates: x={x}, y={y}, z={z}")

    qx = node.orientation.x
    qy = node.orientation.y
    qz = node.orientation.z
    qw = node.orientation.w

    return x, y, z, qx, qy, qz, qw

def quaternion_to_euler(qx, qy, qz, qw):
    # Convert quaternion values to euler
    rotation = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)
    # Ensure the angle is within the range of -90 to 90 degrees 
    if yaw > 90:
            yaw -= 180
    elif yaw < -90:
        yaw += 180
    yaw = yaw * -1
    node.get_logger().info(f"angle: {yaw}")
    return roll, pitch, yaw

def tip_bin():
    # Bin and tipping location
    mid = posx(-85, 533, 680,38, 180, 40)
    bin_above = posx(-635, -165, 430, 25, 180, -65) # old value: z=540
    bin = posx(-635, -165, 354, 25, 180, -65) # old value: z=475
    tip_location = posx(456, 200, 720, 36, 180, 120)
    tip_bin =  posx(456, 240, 720, 95, 90, -180)
    tip_shake_1 = posx(456, 240, 720, 95, 90, -135)
    tip_shake_2 = posx(456, 240, 720, 95, 93, 135)
    bin_grab = [bin_above, bin]
    tip = [bin_above, mid, tip_location, tip_bin, tip_shake_1, tip_shake_2, tip_bin, tip_location, mid, bin_above, bin]

    # Redistribute Springs
    node.get_logger().info('All springs colleccted, Time to make a mess!')
    # Grab bin
    for i in bin_grab:
        movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
    close_grip()

    # Tip and return bin
    for i in tip:
        movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
    open_grip()
    movel(bin_above, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
    time.sleep(2)

def shutdown():
    node.get_logger().info('Springs collected, shutiing down')
    node.destroy_node()

def move(x, y, z, qx, qy, qz, qw):
    # Checks if it hits the table
    if (z > 280) and (y > -185): # 362
        node.get_logger().info(f"moving to {x, y, z}")

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        # node.get_logger().info(f"Received angles: r={roll}, p={pitch}, y={yaw}")

        bin = posx(-645, -68, 675, roll, 180, yaw + 45)
        mid = posx(-85, 533, 680,roll, 180, yaw + 45)
        above_spring = posx(x, y, z+120, roll, 180, yaw)
        spring = posx(x, y, z, roll, 180, yaw)
        to_spring_x_pos = [mid, above_spring, spring]
        to_spring_x_neg = [bin, above_spring, spring]
        to_bin_x_pos = [above_spring, mid, bin]
        to_bin_x_neg = [above_spring, bin]

        if x >= 0:
            for i in to_spring_x_pos:
                movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            close_grip()

            for i in to_bin_x_pos:
                movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            open_grip()

        else:
            for i in to_spring_x_neg:
                movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            close_grip()

            for i in to_bin_x_neg:
                movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
                time.sleep(0.5)
            open_grip()

    else:
        node.get_logger().info('HITS TABLE OR OUT OF BOUNDS!')

def check_msg_time():
    # Checks if the last spring location received was more than 10 seconds ago 
    message_time = node.last_message_time
    current_time = node.get_clock().now()
    if (current_time - message_time).nanoseconds > 10e9:  # 10 seconds
        node.loop = node.get_parameter('loop').get_parameter_value().bool_value

        if node.loop:
            tip_bin()
        else:
            shutdown()

######################### M A I N ###############################

def main():
    rclpy.spin_once(node)
    node.declare_parameter('loop', True)  

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
            quaternion_to_euler(qx, qy, qz, qw)

            # Move to co-ordinates
            move(x, y, z, qx, qy, qz, qw)


    # turn off node
    node.destroy_node()
    print("all done")

if __name__ == "__main__":
    main()
