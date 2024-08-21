import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
      DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'   ),
    ]

    xacro_path = os.path.join( get_package_share_directory('dsr_description2'), 'xacro')

    robot_description_content = Command(
      [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", PathJoinSubstitution([
        FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration('model'),
        ]),
        ".urdf.xacro",
      ])

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
      FindPackageShare("dsr_controller2"),
      "config",
      "dsr_controller2.yaml",
      ])
  
    # Connection parameters
    doosan_connection_node = Node(
        package='sort_seg',
        executable='connection',
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
                {"gripper": "none"      },
                {"mobile":  "none"      },
            ],
        output='screen',
    )

    # Establishes connection to the robot
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=LaunchConfiguration('name'),
        parameters=[robot_description, robot_controllers],
    )

    #robot publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{
        'robot_description': Command(['xacro', ' ', xacro_path, '/', LaunchConfiguration('model'), '.urdf.xacro color:=', LaunchConfiguration('color')])
    }])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    # dsr_position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=LaunchConfiguration('name'),
    #     arguments=["dsr_position_controller", "--controller-manager", "controller_manager"],
    # )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
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
        PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('aruco_ros'),
                'launch',
                'quadruple.launch.py'
        ]))
    )

    # Spring detection
    color_detect = Node(
        package="sort_seg",
        executable="color_detect",

    )
    # View aruco detection
    image_view_node =  Node(
        package="image_view",
        executable="image_view",
        # name= "image_view_node",
        remappings=[("image","aruco_quadruple/result")]
    )

    # TF from camera to aruco
    frame_listener_aruco = Node(
        package='sort_seg', 
        executable='frame_listener_aruco',
    )

    # Transformed co-ordinates
    camera_to_aruco = Node(
        package="sort_seg",
        executable="camera_to_aruco",
    )

    # Static transforamtion between robot and aruco marker
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[
            "0.13923", 
            "-0.42024",
            "-0.00751", 
            "0", 
            "0", 
            "0", 
            "marker2_object_frame", 
            "base_link"
        ]
    )

    # TF from aruco to robot
    aruco_to_robot = Node(
        package="sort_seg",
        executable="aruco_to_robot"
    )

    # Communication with robot
    connect_test = Node(
        package="sort_seg",
        executable="connect_test"
    )

    # # Communication with robot
    # sort_seg = Node(
    #     package="sort_seg",
    #     executable="sort_seg.py"
    # )


    on_image_viewer_start = RegisterEventHandler(event_handler=OnProcessStart(
        target_action= image_view_node, 
        on_start=[color_detect, 
                  frame_listener_aruco]
    ))

    on_frame_listener_aruco_start = RegisterEventHandler(event_handler=OnProcessStart(
        target_action = frame_listener_aruco, 
        on_start = [camera_to_aruco]
    ))

    on_camera_to_aruco_start = RegisterEventHandler(event_handler=OnProcessStart(
        target_action = camera_to_aruco, 
        on_start=[
            TimerAction(
                period=2.0,  # wait for 5 seconds
                actions=[static_transform_publisher]
            )]
    ))

    on_static_transform_publisher_start = RegisterEventHandler(event_handler=OnProcessStart(
        target_action= static_transform_publisher,
        on_start=[aruco_to_robot]
    ))

    on_boradcaster_exit = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[robot_controller_spawner] # [dsr_position_controller_spawner, robot_controller_spawner]
    ))

    on_spawner_exit = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=robot_controller_spawner,
        on_exit=[connect_test] # [sort_seg]
    ))

    nodes = [
        doosan_connection_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        camera_node,
        aruco_quadruple,
        image_view_node,
        on_image_viewer_start,
        on_frame_listener_aruco_start,
        on_camera_to_aruco_start,
        on_static_transform_publisher_start,
        on_boradcaster_exit,
        on_spawner_exit,
    ]
    
    return LaunchDescription(ARGUMENTS + nodes)