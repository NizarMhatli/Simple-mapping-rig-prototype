import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro 

def generate_launch_description():
    pkg = get_package_share_directory('jig_description')
    xacro_file = os.path.join(pkg, 'urdf', 'Main_jig.urdf.xacro')
    bridge_config = os.path.join(pkg, 'config', 'gz_bridge.yaml')

    # with open(urdf_file, 'r') as f:
    #     # robot_description = f.read()
    robot_description = xacro.process_file(xacro_file).toxml()
    return LaunchDescription([

        # Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r',
                os.path.join(pkg, 'worlds', 'jig_world.sdf')],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'jig',
                '-topic', 'robot_description',
            ],
            output='screen'
        ),

        # Bridge Gazebo <-> ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p',
                       f'config_file:={bridge_config}'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '-0.005', '0.035', '0', '0', '0',
                    'base_link', 'jig/base_link/lidar_sensor'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.075', '-0.005', '0.040', '0', '0', '0',
                    'base_link', 'jig/base_link/imu_sensor'],
            output='screen'
        ),
        # In display.launch.py, add these alongside existing static_transform_publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.05', '-0.001', '-0.01',  # camera_xyz
                    '0', '0', '0', '1',
                    'base_link', 'camera_bottom_screw_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0.0175', '0.0125',
                    '0', '0', '0', '1',
                    'camera_bottom_screw_frame', 'camera_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0',
                    '0', '0', '0', '1',
                    'camera_link', 'camera_depth_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0.015', '0',
                    '0', '0', '0', '1',
                    'camera_link', 'camera_color_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0',
                    '0', '0', '0', '1',
                    'base_link', 'jig/base_link/depth_camera'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0',
                    '0', '0', '0', '1',
                    'base_link', 'jig/base_link/rgb_camera'],
        ),
                # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'jig.rviz')],
            output='screen'
        ),
    ])