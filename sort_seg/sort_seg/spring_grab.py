#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###################### I N I T I A L I S E ################################

from dsr_msgs2.msg import RobotStop, RobotState
import time
import sys
import os
import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import RobotStop, RobotState
imp_path=os.path.abspath(os.path.join(os.path.abspath(__file__),"../../../../../../../src/Sort-and-Segregate/common2/imp"))
sys.path.append((imp_path))
import DR_init

# Global variables
ROBOT_ID = "dsr01"
ROBOT_MODEL = ''

def msgRobotState_cb(node, msg):
    node.get_logger().info(f"Robot State: {msg.robot_state}, {msg.robot_state_str}")
   
rclpy.init()
node = Node('dsr_simple_test_py')
node.pub_stop = node.create_publisher(RobotStop, f'/{ROBOT_ID}{ROBOT_MODEL}/stop', 10)
node.create_subscription(RobotState, f'/{ROBOT_ID}{ROBOT_MODEL}/joint_states', msgRobotState_cb, 10)
DR_init.__dsr__node = node
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = ''
DR_init.__dsr__node = node
from DSR_ROBOT2 import *

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


def shutdown():
    node.get_logger().info('Springs collected, shutiing down')
    node.destroy_node()

def move(spring):

    bin = posx(-450, 320, 430, 86, 180, -90)
    above_bin = posx(-450, 320, 500, 86, 180, -90)
    above_spring = spring[:] # copy of spring
    above_spring[2] += 120 # add to z value
    to_spring = [above_bin, above_spring, spring]
    to_bin = [above_spring, above_bin, bin]

    for i in to_spring:
        movel(i, vel = 80, acc = 60, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
    close_grip()

    for i in to_bin:
        movel(i, vel = 80, acc = 60, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
    open_grip()


######################### M A I N ###############################

def main():
    node.get_logger().info('Main method')
    # rclpy.spin_once(node) 
    # rclpy.spin(node)
    # node.get_logger().info('Node has been spun')
    # while rclpy.ok():
    node.get_logger().info('Main loop started')
    spring_1 = posx(-226, 451, 327, 44, -180, -36)
    spring_2 = posx(-208, 355, 332, 108, 180, -71)
    spring_3 = posx(-265, 348, 306, 103, 180, -45)

    # Check if the gripper is open
    if get_grip_state() == False:
        open_grip()

    # Check if the arm is in motion
    if check_motion() == 0:
        # Move to co-ordinates
        move(spring_1)
        move(spring_2)
        move(spring_3)
        shutdown()


if __name__ == "__main__":
    main()