from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    aruco_quadruple_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id1': LaunchConfiguration('marker_id1'),
        'marker_id2': LaunchConfiguration('marker_id2'),
        'marker_id3': LaunchConfiguration('marker_id3'),
        'marker_id4': LaunchConfiguration('marker_id4'),
        'normalizeImage': LaunchConfiguration('dct_normalization'),
        'dct_components_to_remove': LaunchConfiguration('dct_filter_size'),
        'parent_name': 'camera_color_optical_frame',
        'child_name1': LaunchConfiguration('marker1_frame'),
        'child_name2': LaunchConfiguration('marker2_frame'),
        'child_name3': LaunchConfiguration('marker3_frame'),
        'child_name4': LaunchConfiguration('marker4_frame'),
        'frame_id': LaunchConfiguration('frame_id'), 
    }

    aruco_quadruple = Node(
        package='aruco_ros',
        executable='quadruple',
        parameters=[aruco_quadruple_params],
        remappings=[('/camera_info', '/camera/camera/color/camera_info'),
                    ('/image', '/camera/camera/color/image_raw')],
    )

    return [aruco_quadruple]


def generate_launch_description():

    marker_id1_arg = DeclareLaunchArgument(
        'marker_id1', default_value='25',
        description='Marker ID 1. '
    )

    marker_id2_arg = DeclareLaunchArgument(
        'marker_id2', default_value='30',
        description='Marker ID 2. '
    )

    marker_id3_arg = DeclareLaunchArgument(
        'marker_id3', default_value='35',
        description='Marker ID 3. '
    )

    marker_id4_arg = DeclareLaunchArgument(
        'marker_id4', default_value='45',
        description='Marker ID 4. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.08',
        description='Marker size in m. '
    )

    dct_normalization_arg = DeclareLaunchArgument(
        'dct_normalization', default_value='false',
        description='dct normalization. ',
        choices=['true', 'false'],
    )

    dct_filter_size_arg = DeclareLaunchArgument(
        'dct_filter_size', default_value='2',
        description='dct filter size. ',
    )

    marker1_frame_arg = DeclareLaunchArgument(
        'marker1_frame', default_value='marker1_object_frame',
        description='Frame in which the marker1 pose will be refered. '
    )

    marker2_frame_arg = DeclareLaunchArgument(
        'marker2_frame', default_value='marker2_object_frame',
        description='Frame in which the marker2 pose will be refered. '
    )

    marker3_frame_arg = DeclareLaunchArgument(
        'marker3_frame', default_value='marker3_object_frame',
        description='Frame in which the marker3 pose will be refered. '
    )

    marker4_frame_arg = DeclareLaunchArgument(
        'marker4_frame', default_value='marker4_object_frame',
        description='Frame in which the marker4 pose will be refered. '
    )

    frame_id_arg = DeclareLaunchArgument(
    'frame_id', default_value='d455_color_optical_frame',
    description='Frame ID. '
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_id1_arg)
    ld.add_action(marker_id2_arg)
    ld.add_action(marker_id3_arg)
    ld.add_action(marker_id4_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(dct_normalization_arg)
    ld.add_action(dct_filter_size_arg)
    ld.add_action(marker1_frame_arg)
    ld.add_action(marker2_frame_arg)
    ld.add_action(marker3_frame_arg)
    ld.add_action(marker4_frame_arg)
    ld.add_action(frame_id_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld