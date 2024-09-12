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

object_count = 0

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
    node.get_logger().info(f"Spring_height: {node.z}")
    if node.z < 0.0205:
        z = 284 
    else:
        if x >=0: # Acounts on slight variation bettween two sides of the table
            z = 265 + (node.z *1000.0)
        else:
            z = 282 + (node.z *1000.0) # height of  gripper 

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


def shutdown():
    node.get_logger().info('Springs collected, shutiing down')
    node.destroy_node()

def move(x, y, z, qx, qy, qz, qw):
    # Checks if it hits the table
    if (600> z > 280) and (y > -185): # 362
        node.get_logger().info(f"moving to {x, y, z}")

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        bin = posx(-645, -68, 675, roll, 180, yaw + 45)
        mid = posx(-85, 533, 680,roll, 180, yaw + 45)
        above_spring = posx(x, y, z+120, roll, 180, yaw)
        spring = posx(x, y, z, roll, 180, yaw)
        to_spring_x_pos = [mid, above_spring, spring]
        to_spring_x_neg = [above_spring, spring]
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

def move_objects(): # move some of the rubish objects
    global object_count
    # Locations of the objects
    long_screw = posx(-308, 469, 290, 73, 180, -59)
    sponge = posx(425, 165, 385, 6, 180, -125)
    short_bolt = posx(89, 387, 290, 78, 180, -124)
    hook = posx(172, 358, 283, 90, -180, -136)
    paper = posx(452, 520, 310, 50, 180, 180)
    old_gripper = posx(145, 532, 288, 73, 180, -107)
    short_screw = posx(396, 446, 286, 124, 180, -51)
    object_list = [long_screw, sponge, short_bolt, hook, paper, old_gripper, short_screw]
    mid = posx(-85, 533, 680, 290, 180, 90)
    node.get_logger().info(f"object index: {object_count}, object: {object_list[object_count]}")

    # Move the object
    if object_count == 3 or object_count == 4: # move to object pick it up and put it back down
        above = object_list[object_count][:]
        above[2] += 120
        movel(mid, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(above, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(object_list[object_count], vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        close_grip()
        movel(above, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(2)
        movel(object_list[object_count], vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        open_grip()
        movel(mid, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        object_count = object_count + 1
        return object_count
    
    elif object_count == 1:
        above = object_list[object_count][:]
        above[2] += 120
        place = object_list[object_count][:]
        place[0] += 150
        node.get_logger().info(f"above: {above}, place: {place}")
        movel(mid, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(above, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(object_list[object_count], vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        close_grip()
        movel(above, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(place, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        open_grip()
        movel(mid, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        object_count = object_count + 1
        return object_count

    else: # Will move object then place 3cm to the right
        above = object_list[object_count][:]
        above[2] += 120
        place = object_list[object_count][:]
        place[0] -= 50
        node.get_logger().info(f"above: {above}, place: {place}")
        movel(mid, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(above, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(object_list[object_count], vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        close_grip()
        movel(above, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        movel(place, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        time.sleep(0.5)
        open_grip()
        movel(mid, vel = 140, acc = 85, ra = DR_MV_RA_DUPLICATE)
        object_count = object_count + 1
        return object_count


def check_msg_time():
    # Checks if the last spring location received was more than 10 seconds ago 
    message_time = node.last_message_time
    current_time = node.get_clock().now()
    if (current_time - message_time).nanoseconds > 10e9:  # 10 seconds
        if object_count > 6:
            shutdown()
        else:
            move_objects()      

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
            quaternion_to_euler(qx, qy, qz, qw)

            # Move to co-ordinates
            move(x, y, z, qx, qy, qz, qw)
 


if __name__ == "__main__":
    main()
