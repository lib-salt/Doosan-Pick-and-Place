#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###################### I N I T I A L I S E ################################

from dsr_msgs2.msg import RobotStop, RobotState
import time
import sys
import os
import rclpy
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

def tip_bin():
    # Bin and tipping location
    mid = posx(-85, 533, 680, 38, 180, 40)
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

def move(spring):

    bin = posx(-645, -68, 675, 25, 180, -65)
    above_spring = spring[:] # copy of spring
    above_spring[2] += 120 # add to z value
    to_spring = [above_spring, spring]
    to_bin = [above_spring, bin]

    for i in to_spring:
        movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
    close_grip()

    for i in to_bin:
        movel(i, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
    open_grip()


######################### M A I N ###############################

def main():
    rclpy.spin_once(node)
    spring_1 = posx()
    spring_2 = posx()
    while rclpy.ok():

        # Check if the gripper is open
        if get_grip_state() == False:
            open_grip()

        # Check if the arm is in motion
        if check_motion() == 0:
            # Move to co-ordinates
            move(spring_1)
            move(spring_2)
            shutdown()


if __name__ == "__main__":
    main()