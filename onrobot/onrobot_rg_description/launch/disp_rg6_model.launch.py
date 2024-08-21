import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    onrobot_rg_description_dir = get_package_share_directory('onrobot_rg_description')
    urdf_file = os.path.join(onrobot_rg_description_dir, 'urdf', 'onrobot_rg6_model.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    
    return launch.LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
