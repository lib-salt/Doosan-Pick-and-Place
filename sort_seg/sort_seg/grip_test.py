#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import RobotStop, RobotState
import time
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

from DSR_ROBOT2 import * 


# get grip state
def get_grip_state():
    return get_digital_input(1) == 0

# print grip state
def print_grip_state():
    print("gripper is open" if get_grip_state() else "gripper is closed")

# open and close gripper
def open_grip():
    set_digital_output(1, 1) # (port number, ON)

def close_grip():
    set_digital_output(1, 0) # (port number, OFF)

def main():
    
    print("Try 1")
    close_grip()
    time.sleep(4) # wait 4 seconds for gripper to close
    print_grip_state()
    open_grip()
    time.sleep(4) # wait 4 seconds for gripper to open
    print_grip_state()

     # turn off node
    node.destroy_node()
    print("all done")
      
if __name__ == "__main__":
    main()