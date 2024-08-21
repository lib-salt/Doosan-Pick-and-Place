import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformStamped
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.qos import qos_profile_sensor_data


class ObjectTransformer(Node):
    def __init__(self):
        super().__init__('object_transformer')

        # Initailize buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to spring co-ordinates
        self.object_sub = self.create_subscription(Float64MultiArray, '/color_detect_node/object_coords', self.listener_callback, qos_profile_sensor_data)

        # Publisher of transformed object pose
        self.publisher = self.create_publisher(Pose, 'transformed_object_pose', 10)

        # Visualize spring location
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    # Convert array into a list
    def listener_callback(self, msg):
        object_coordinates = list(msg.data)

        # Apply transformation to co-ordinates
        transformed_pose = self.apply_transformation(object_coordinates)
        
        if transformed_pose is not None:
            self.publisher.publish(transformed_pose.pose)


    def get_transform(self):
        try:
            # Get transformation from camera to marker
            transformation = self.tf_buffer.lookup_transform(
                'base_link',  # Robot frame
                'camera_color_optical_frame',  # Camera frame
                rclpy.time.Time(),rclpy.duration.Duration(seconds=1))  
            return transformation
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Could not find transform: {e}")
            return None

    def apply_transformation(self, object_coordinates):
        transformation = self.get_transform()

        if transformation is None:
            self.get_logger().error("Transformation not found.")
            return None

         # Create a PoseStamped message for the object pose
        pose_in_camera_frame = PoseStamped()
        pose_in_camera_frame.header.frame_id = "camera_color_optical_frame"
        pose_in_camera_frame.pose.position.x = object_coordinates[0]
        pose_in_camera_frame.pose.position.y = object_coordinates[1]
        pose_in_camera_frame.pose.position.z = object_coordinates[2]
        
        # Set a default orientation (e.g., no rotation)
        pose_in_camera_frame.pose.orientation.x = 0.0
        pose_in_camera_frame.pose.orientation.y = 0.0
        pose_in_camera_frame.pose.orientation.z = 0.0
        pose_in_camera_frame.pose.orientation.w = 1.0

        # Transform the pose to the marker (base_link) frame
        pose_in_robot_frame = do_transform_pose_stamped(pose_in_camera_frame, transformation)
        
        # Broadcast the transformation
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'spring'
        t.transform.translation.x = pose_in_robot_frame.pose.position.x
        t.transform.translation.y = pose_in_robot_frame.pose.position.y
        t.transform.translation.z = pose_in_robot_frame.pose.position.z
        self.tf_broadcaster.sendTransform(t)

        return pose_in_robot_frame


def main(args=None):
    rclpy.init(args=args)
    object_transformer = ObjectTransformer()
    rclpy.spin(object_transformer)
    object_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()