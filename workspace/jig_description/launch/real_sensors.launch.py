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
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ── RViz ──────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'rviz', 'jig.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ── Static TF: base_link → livox_frame ────────────────────────
    # Connects real LiDAR frame to jig base
    # Adjust xyz/rpy to match physical mounting position on jig
    tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_livox',
        arguments=[
            '0.021', '-0.12', '0.00',  # lidar_xyz from urdf
            '0', '0', '0', '1',
            'base_link', 'livox_frame'
        ]
    )

    # ── Static TF: base_link → lidar_link (for URDF mesh display) ─
    tf_base_to_lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_lidar_link',
        arguments=[
            '0.021', '-0.12', '0.00',
            '0', '0', '0', '1',
            'base_link', 'lidar_link'
        ]
    )

    # ── Static TF: base_link → camera_link ────────────────────────
    # Connects jig URDF camera_link to real RealSense TF tree
    tf_base_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera_link',
        arguments=[
            '0.050', '-0.001', '-0.01',  # camera_xyz from urdf
            '0', '0', '-1.5707', '1',    # yaw -90° to face +X
            'base_link', 'camera_link'
        ]
    )

    # ── Static TF: camera_link → camera_depth_optical_frame ───────
    # Bridges jig URDF to real RealSense depth frame
    tf_camera_to_depth_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_to_depth_optical',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',  # optical frame rotation
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
        tf_base_to_livox,
        tf_base_to_lidar_link,
        tf_base_to_camera_link,
        tf_camera_to_depth_optical,
        tf_camera_to_color_optical,
    ])