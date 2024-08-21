#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import RobotStop, RobotState
from ament_index_python.packages import get_package_share_directory
import time
import DR_init

# Global variables
ROBOT_ID = "dsr01"
ROBOT_MODEL = ''

# Communication with robot
def msgRobotState_cb(node, msg):
    node.get_logger().info(f"Robot State: {msg.robot_state}, {msg.robot_state_str}")
   
# The node to communicate with the robot
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
    
    # save posititions
    point_2 = posx(310, 0, 0, 0, 0, 0) 
    point_3 = posx(310, 640, 0, 0, 0, 0)

    set_ref_coord(101)

    if set_ref_coord:
      # if user co-ordinates was successful
      print("attempting to move gripper to point 2")
      # moves to the point_2 position with a velocity of 30(mm/sec) and acceleration of 50(mm/sec2)
      movel(point_2, vel = 30, acc = 40) 
      close_grip()
      time.sleep(2)
      print("moving to point_3")
      movel(point_3, vel = 30, acc = 40)
      open_grip()
      print("all done")
      node.destroy_node()

    else:
        # programme will stop if unable to get user co-ordinates
        print("failed to get user co-ordinates, shutting down")
        node.destroy_node()
      
if __name__ == "__main__":
    main()