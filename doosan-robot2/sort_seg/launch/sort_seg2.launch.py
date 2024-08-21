import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    ARGUMENTS =[ 
      DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'    ),
      DeclareLaunchArgument('host',  default_value = '192.168.1.100', description = 'ROBOT_IP'  ),
      DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'    ),
      DeclareLaunchArgument('mode',  default_value = 'real',   description = 'OPERATION MODE'   ),
      DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'   ),
    ]

    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')

    # path to sort_seg python executables
    prefix = get_package_prefix('sort_seg')
    script_dir = os.path.join(prefix, 'lib', 'python3.10', 'site-packages', 'sort_seg')

    robot_description_content = Command(
      [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", PathJoinSubstitution([
        FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration('model'),
        ]),
        ".urdf.xacro",
      ])

    # robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
      FindPackageShare("dsr_controller2"),
      "config",
      "dsr_controller2.yaml",
      ])
  
    # Connection parameters
    doosan_connection_node = Node(
        package='sort_seg',
        executable='sort_seg.connection',
        namespace=LaunchConfiguration('name'),
        parameters=[
                {'name': LaunchConfiguration('name')},
                {'rate': 100},
                {'standby': 5000},
                {'command': True},
                {'host': LaunchConfiguration('host')},
                {'port': LaunchConfiguration('port')},
                {'mode': LaunchConfiguration('mode')},
                {'model': LaunchConfiguration('model')},
            ],
        output='screen',
    )

    # Establishes connection to the robot
    control_node = Node(
        package='controller_manager',
        executable='controller_manager.ros2_control_node',
        namespace=LaunchConfiguration('name'),
        parameters=[
                {'robot_description': robot_description_content},
                {'robot_controllers': robot_controllers},
            ],
    )

    # Camera node
    camera_node = IncludeLaunchDescription(
        PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
        ])
    )

    # Detect aruco markers
    aruco_quadruple = IncludeLaunchDescription(
        PathJoinSubstitution([
                FindPackageShare('aruco_ros'),
                'launch',
                'quadruple.launch.py'
        ])
    )

    # Spring detection
    color_detect = Node(
        package="sort_seg",
        executable="color_detect",

    )

    # View aruco detection
    image_viewer =  Node(
        package="image_view",
        executable="image_view",
        name= "image_viewer",
        remappings=[("image","aruco_quadruple/result")]
    )

    # # TF from camera to aruco
    # frame_listener_aruco = Node(
    #     package='sort_seg', 
    #     executable='frame_listener_aruco.py',
    # )

    # # Transformed co-ordinates
    # camera_to_aruco = Node(
    #     package="sort_seg",
    #     executable="camera_to_aruco.py",
    # )

    # # Static transforamtion between robot and aruco marker
    # static_transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="screen",
    #     arguments=[
    #         "0.13923", 
    #         "-0.42024",
    #         "-0.00751", 
    #         "0", 
    #         "0", 
    #         "0", 
    #         "marker2_object_frame", 
    #         "base_link"
    #     ]
    # )

    # # Robot frame listener
    # frame_listener_robot = Node(
    #     package="sort_seg",
    #     executable="frame_listener_robot.py",
    # )
    
    # # TF from aruco to robot
    # aruco_to_robot = Node(
    #     package="sort_seg",
    #     executable="aruco_to_robot.py"
    # )

    # # Communication with robot
    # connect_test = Node(
    #     package="sort_seg",
    #     executable="connect_test.py"
    # )

    # # Communication with robot
    # sort_seg = Node(
    #     package="sort_seg",
    #     executable="sort_seg.py"
    # )

    # Nodes after camera node
    # execute_after_camera_node = [
    #     aruco_quadruple,
    #     color_detect,
    # ]

    # Nodes after aruco quad
    # execute_after_aruco_quadruple = [
    #     image_viewer, 
        # frame_listener_aruco,
        #camera_to_aruco,
    # ]

    # execute_after_static_transform =[
    #     frame_listener_robot,
    #     aruco_to_robot,

    # ]

    on_camera_node_start= RegisterEventHandler(event_handler=OnProcessStart(
        target_action = camera_node,
        on_start = [aruco_quadruple]
    ))

    on_aruco_quadruple_start = RegisterEventHandler(event_handler=OnProcessStart(
        target_action = aruco_quadruple, 
        on_start = [image_viewer]
    ))

    # on_camera_to_aruco_start = RegisterEventHandler(event_handler=OnProcessStart(
    #     target_action = camera_to_aruco, 
    #     on_start = [static_transform_publisher]
    # ))

    # on_static_transform_publisher_start = RegisterEventHandler(OnProcessStart(
    #     target_action= static_transform_publisher,
    #     on_start=[execute_after_static_transform]
    # ))

    # on_aruco_to_robot_start = OnProcessStart(
    #     target_action=on_static_transform_publisher_start,
    #     on_start=[connect_test] # [sort_seg]
    # )

    nodes = [
        doosan_connection_node,
        control_node,
        camera_node,
    ]
    
    actions = [
        on_camera_node_start,
        on_aruco_quadruple_start,
        # on_camera_to_aruco_start,
        # on_static_transform_publisher_start,
        # on_aruco_to_robot_start,

    ]

    return LaunchDescription(ARGUMENTS + nodes + actions)