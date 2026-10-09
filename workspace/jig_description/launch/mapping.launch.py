import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg = get_package_share_directory('jig_description')
    xacro_file    = os.path.join(pkg, 'urdf', 'Main_jig.urdf.xacro')
    bridge_config = os.path.join(pkg, 'config', 'gz_bridge.yaml')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── Launch arguments ───────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world', default_value='jig_world',
        description='Gazebo world name (without .sdf)',
    )
    speed_arg = DeclareLaunchArgument(
        'speed', default_value='0.3',
        description='Jig linear speed in m/s',
    )
    world = LaunchConfiguration('world')
    speed = LaunchConfiguration('speed')

    # ── Gazebo ─────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r',
             os.path.join(pkg, 'worlds', 'jig_world.sdf')],
        output='screen'
    )

    # ── Robot State Publisher ──────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen'
    )

    # ── Spawn jig in Gazebo ────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'jig', '-topic', 'robot_description'],
        output='screen'
    )

    # ── Gazebo ↔ ROS 2 bridge ──────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen'
    )

    # ── Static TF publishers ───────────────────────────────────
    tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', '1',
                   'odom', 'base_link'],
    )
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '-0.005', '0.035', '0', '0', '0',
                   'base_link', 'jig/base_link/lidar_sensor'],
    )
    tf_lidar_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.1', '0.005', '-0.035', '0', '0', '0', '1',
                   'jig/base_link/lidar_sensor', 'base_link'],
    )
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.075', '-0.005', '0.040', '0', '0', '0',
                   'base_link', 'jig/base_link/imu_sensor'],
    )
    tf_camera_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.05', '-0.001', '-0.01', '0', '0', '0', '1',
                   'base_link', 'camera_bottom_screw_frame'],
    )
    tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.0175', '0.0125', '0', '0', '0', '1',
                   'camera_bottom_screw_frame', 'camera_link'],
    )
    tf_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'camera_link', 'camera_depth_frame'],
    )
    tf_color = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.015', '0', '0', '0', '0', '1',
                   'camera_link', 'camera_color_frame'],
    )
    tf_gz_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'base_link', 'jig/base_link/depth_camera'],
    )
    tf_gz_rgb = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'base_link', 'jig/base_link/rgb_camera'],
    )

    # ── RTAB-Map ───────────────────────────────────────────────
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':           True,
            'frame_id':               'jig/base_link/lidar_sensor',
            'odom_frame_id':          'odom',
            'subscribe_depth':        False,
            'subscribe_rgb':          False,
            'subscribe_scan_cloud':   True,
            'approx_sync':            True,
            'queue_size':             10,
            'Mem/IncrementalMemory':  'true',
            'Mem/InitWMWithAllNodes': 'true',
        }],
        remappings=[
            ('scan_cloud', '/scan/points'),
        ],
    )

    # ── RViz ───────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'jig.rviz')],
        output='screen'
    )

    # ── Path motion (delayed 5s) ───────────────────────────────
    path_motion = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='jig_description',
                executable='jig_path_motion',
                name='jig_path_motion',
                output='screen',
                parameters=[{
                    'world': world,
                    'speed': speed,
                }],
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        speed_arg,
        gz_sim,
        rsp,
        spawn,
        bridge,
        tf_odom_base,
        tf_lidar,
        tf_lidar_to_base,
        tf_imu,
        tf_camera_base,
        tf_camera_link,
        tf_depth,
        tf_color,
        tf_gz_depth,
        tf_gz_rgb,
        rtabmap,
        rviz,
        path_motion,
    ])