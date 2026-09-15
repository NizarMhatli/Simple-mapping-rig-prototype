import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ── Robot description ──────────────────────────────────────────
    pkg = get_package_share_directory('jig_description')
    xacro_file = os.path.join(pkg, 'urdf', 'Main_jig.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ── RViz ──────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'rviz', 'jig.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ── Topic relays ───────────────────────────────────────────────
    relay_lidar = Node(
        package='topic_tools',
        executable='relay',
        name='relay_lidar',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/scan/points',
        }],
        output='screen'
    )

    relay_imu = Node(
        package='topic_tools',
        executable='relay',
        name='relay_imu',
        parameters=[{
            'input_topic': '/livox/imu',
            'output_topic': '/imu/data_raw',
        }],
        output='screen'
    )

    relay_rgb = Node(
        package='topic_tools',
        executable='relay',
        name='relay_rgb',
        parameters=[{
            'input_topic': '/camera/camera/color/image_raw',
            'output_topic': '/camera/color/image_raw',
        }],
        output='screen'
    )

    relay_depth = Node(
        package='topic_tools',
        executable='relay',
        name='relay_depth',
        parameters=[{
            'input_topic': '/camera/camera/depth/image_rect_raw',
            'output_topic': '/camera/depth/image_rect_raw',
        }],
        output='screen'
    )

    relay_depth_points = Node(
        package='topic_tools',
        executable='relay',
        name='relay_depth_points',
        parameters=[{
            'input_topic': '/camera/camera/depth/color/points',
            'output_topic': '/camera/depth/points',
        }],
        output='screen'
    )

    # ── Static TF: base_link → livox_frame ────────────────────────
    tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_livox',
        arguments=[
            '0.025', '-0.005', '0.035',
            '0', '0.7071', '0', '0.7071',
            'base_link', 'livox_frame'
        ]
    )

    # ── Static TF: base_link → camera_link ────────────────────────
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=[
            '0.050', '-0.001', '-0.01',
            '0', '0', '-0.7071', '0.7071',
            'base_link', 'camera_link'
        ]
    )

    # ── Static TF: camera_link → camera_depth_optical_frame ───────
    tf_camera_to_depth_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_to_depth_optical',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',
            'camera_link', 'camera_depth_optical_frame'
        ]
    )

    # ── Static TF: camera_link → camera_color_optical_frame ───────
    tf_camera_to_color_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_to_color_optical',
        arguments=[
            '0', '0.015', '0',
            '-0.5', '0.5', '-0.5', '0.5',
            'camera_link', 'camera_color_optical_frame'
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
        relay_lidar,
        relay_imu,
        relay_rgb,
        relay_depth,
        relay_depth_points,
        tf_base_to_livox,
        tf_base_to_camera,
        tf_camera_to_depth_optical,
        tf_camera_to_color_optical,
    ])