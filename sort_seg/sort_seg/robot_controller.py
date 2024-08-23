import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from dsr_msgs2.msg import RobotStop, RobotState
from rclpy.qos import qos_profile_sensor_data
import sys
import os
sys.path.append(os.path.abspath("/home/jacobs/ros2_ws/src/doosan-robot2/common2/imp"))
 
class RobotController(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('robot_controller')
        self.pub_stop = self.create_publisher(RobotStop, f'/dsr01/stop', 10)
        self.create_subscription(RobotState, f'/dsr01/joint_states', msgRobotState_cb, 10)
        self.create_subscription(Pose, '/transformed_object_pose', self.coords_cb, qos_profile_sensor_data)

        self.x = None
        self.y = None
        self.z = None
        self.orientation = None
        self.last_message_time = None

    def coords_cb(self, msg):
        self.last_message_time = self.get_clock().now()

        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
            
        # Extract orientation (quaternion) from the Pose message
        self.orientation = msg.orientation
        print(f"Received coordinates: x={self.x}, y={self.y}, z={self.z}")  
    

def msgRobotState_cb(node, msg): node.get_logger().info(f"Robot State: {msg.robot_state}, {msg.robot_state_str}")
