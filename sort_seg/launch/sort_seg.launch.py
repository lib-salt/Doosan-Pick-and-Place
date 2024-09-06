import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, Shutdown
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ARGUMENTS =[ 
      DeclareLaunchArgument('name',  default_value = 'dsr01',     description = 'NAME_SPACE'    ),
      DeclareLaunchArgument('host',  default_value = '192.168.1.100', description = 'ROBOT_IP'  ),
      DeclareLaunchArgument('port',  default_value = '12345',     description = 'ROBOT_PORT'    ),
      DeclareLaunchArgument('mode',  default_value = 'real',   description = 'OPERATION MODE'   ),
      DeclareLaunchArgument('model', default_value = 'm1013',     description = 'ROBOT_MODEL'   ),
      DeclareLaunchArgument('color', default_value = 'white',     description = 'ROBOT_COLOR'   ),
      DeclareLaunchArgument('loop',  default_value = 'true',     description = 'LOOP_SETTING'   ),
      DeclareLaunchArgument('rviz',  default_value = 'true',     description = 'RVIZ'           ),
      DeclareLaunchArgument('depth_cloud', default_value = 'false', description = 'RVIZ_CONFIGURATION'),
    ]

    depth_cloud = LaunchConfiguration('depth_cloud')
    rviz = LaunchConfiguration('rviz')
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
    
    # RVIZ config files
    default_config = PathJoinSubstitution(
            [FindPackageShare("sort_seg"), "rviz", "default.rviz"]
        ),
    depth_config = PathJoinSubstitution(
            [FindPackageShare("sort_seg"), "rviz", "depth_cloud.rviz"]
        ),
  
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

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )


    # Camera node
    camera_node = IncludeLaunchDescription(
        PathJoinSubstitution([
                FindPackageShare('sort_seg'),
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
        remappings=[("image","aruco_quadruple/result")]
    )

    # Transformation 
    cam_map = Node(
        package='sort_seg',
        executable='cam_map',
    )

    # Static transforamtion between robot and aruco marker
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[
            "0.05969", # 0.06069
            "-0.51299", # -0.51399
            "0", 
            "0", 
            "0", 
            "0", 
            "marker2_object_frame", 
            "base_link"
        ]
    )

    # Start RVIZ
    rviz_default = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", default_config],
        condition=IfCondition(PythonExpression(["'", rviz, "' and not '", depth_cloud, "'"]))
    )

    rviz_depth_cloud = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", depth_config],
        condition=IfCondition(PythonExpression(["'", rviz, "' and '", depth_cloud, "'"]))
    )

    # # Communication with robot
    # connect_test = Node(
    #     package="sort_seg",
    #     executable="connect_test"
    # )

    # Communication with robot
    sort_seg = Node(
        package="sort_seg",
        executable="sort_seg",
        parameters=[{'loop': LaunchConfiguration('loop')}]
    )


    on_image_viewer_start = RegisterEventHandler(event_handler=OnProcessStart(
        target_action= image_view_node, 
        on_start=[color_detect]
    ))

    on_cam_map_start = RegisterEventHandler(event_handler=OnProcessStart(
        target_action = cam_map, 
        on_start=[
            TimerAction(
                period=2.0,  # wait for 2 seconds
                actions=[static_transform_publisher]
            )]
    ))

    on_broadcaster_exit = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[robot_controller_spawner, cam_map] 
    ))

    on_spawner_exit = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=robot_controller_spawner,
        on_exit=[sort_seg] 
    ))

    # rviz_node = RegisterEventHandler(event_handler=OnProcessExit(
    #     target_action=robot_controller_spawner,
    #     on_exit=[rviz_default, rviz_depth_cloud]
    # ))

    on_sort_seg_exit = RegisterEventHandler(event_handler=OnProcessExit(
        target_action=sort_seg,
        on_exit=[Shutdown()] 
    ))

    nodes = [
        doosan_connection_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        camera_node,
        aruco_quadruple,
        image_view_node,
        # rviz_node,
        rviz_default,
        rviz_depth_cloud,
        on_image_viewer_start,
        on_cam_map_start,
        on_broadcaster_exit,
        on_spawner_exit,
        on_sort_seg_exit,
    ]
    
    return LaunchDescription(ARGUMENTS + nodes)
