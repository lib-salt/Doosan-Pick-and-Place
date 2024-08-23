#!/usr/bin/env python3
# -*- coding: utf-8 -*-


################### D E M O ####################


from dsr_msgs2.msg import RobotStop, RobotState
from ament_index_python.packages import get_package_share_directory
import time
import sys
import os
import rclpy
imp_path=os.path.join(os.path.abspath(__file__),"../../common2/imp")
sys.path.append(os.path.abspath(imp_path))
controller_path=os.path.abspath(os.path.join(os.path.abspath(__file__), '../'))
sys.path.append(controller_path)
from robot_controller import RobotController

node = RobotController()

import DR_init

DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = ''
DR_init.__dsr__node = node
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

def grip_collision():
   gripper_collision = get_digital_input(3)
   return gripper_collision
   
def main():

   rclpy.spin_once(node)
   # rclpy.spin()
   
   while rclpy.ok():

      # Check if the gripper is opens
      if get_grip_state() == True:
         pass
      else:
         open_grip()

      # Check if the arm is in motion
      if check_motion() == 0:
      # Get spring co-ordinates
         x = node.x *1000.0
         y = node.y *1000.0
         
         node.get_logger().info(f"Spring_height: {node.z}")
         # z = 362 + (node.z *1000.0) # height of  gripper and table
         z = 380 # 420
         node.get_logger().info(f"Received coordinates: x={x}, y={y}, z={z}")

         bin= posx(-596, -69, 652, 25, 180, 50)      #BIN LOCATION
         point_3 = posx(x, y, z+100, 35, 180, 100)
         ## change the x, y, z poisitions to the transformed values in the robots co-ordinates
         point_1 = posx(x, y, z, 35, 180, 100) 
         mid = posx(-85, 533, 450, 35, 180, 100)

         # Checks if it hits the table
         if (z > 362) and (y > 0): # at 380 to close gripper
            node.get_logger().info(f"moving to {point_1}")
            movel(mid, vel = 80, acc = 50)
            time.sleep(3)
            movel(point_3, vel = 80, acc = 50)
            time.sleep(3)
            movel(point_1, vel = 60, acc = 50)
            # time.sleep(2)
            close_grip()
            # if grip_collision() == 1:
            #    open_grip()
            #    node.get_logger.info("Gripper in collision!")
            #    break
            # else:
            time.sleep(2)
            movel(mid, vel = 80, acc = 50)
            time.sleep(2)
            movel(bin, vel = 80, acc = 50)
            open_grip()
            time.sleep(2)
         else:
            node.get_logger().info('HITS TABLE!')
      else:
         pass

   # turn off node
   node.destroy_node()
   print("all done")
 
if __name__ == "__main__":
   main()
