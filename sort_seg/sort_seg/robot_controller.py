import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from dsr_msgs2.msg import RobotStop, RobotState
import sys
import os
imp_path=os.path.join(os.path.abspath(__file__),"../../common2/imp")
sys.path.append(os.path.abspath(imp_path))
 
class RobotController(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('robot_controller')
        self.pub_stop = self.create_publisher(RobotStop, f'/dsr01/stop', 10)
        self.create_subscription(RobotState, f'/dsr01/joint_states', msgRobotState_cb, 10)
        self.create_subscription(Pose, '/transformed_object_pose', self.coords_cb, 10)

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
            
        # # Extract orientation (quaternion) from the Pose message
        self.orientation = msg.orientation
        
def msgRobotState_cb(node, msg): node.get_logger().info(f"Robot State: {msg.robot_state}, {msg.robot_state_str}")
