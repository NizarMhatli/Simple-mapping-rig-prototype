"""
Jig mapping in Gazebo with RTAB-Map (lidar + IMU only).
The RealSense is simulated and bridged for other tasks but is not used for mapping.

  ros2 launch jig_description mapping.launch.py                  # ICP+IMU odometry
  ros2 launch jig_description mapping.launch.py odom_source:=gt       # ground-truth odom (debug mapping only)
  ros2 launch jig_description mapping.launch.py odom_source:=fastlio  # FAST-LIO2 odometry + RTAB-Map loop closure

Requires: ros-jazzy-rtabmap-ros, ros-jazzy-imu-filter-madgwick,
          fast_lio (hku-mars/FAST_LIO, ROS2 branch) for odom_source:=fastlio
All sensor frames come from the URDF via <gz_frame_id>.

TF tree with odom_source:=fastlio
  map -> odom                (rtabmap)
  odom -> camera_init        (static: IMU pose in base_link at start)
  camera_init -> body        (FAST-LIO, body = IMU frame)
  body -> base_link          (static: inverse of base_link -> imu_link)
  base_link -> lidar_link -> imu_link  (robot_state_publisher)
FAST-LIO's world frame (camera_init) is the IMU's *initial* frame, which is
pitched 90° with the lidar; the odom -> camera_init static TF levels it.
"""
import os
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
import xacro


# ── Helpers: base_link -> imu_link from the URDF (single source of truth) ──
def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp,     cp * sr,                cp * cr]])


def _R_to_quat(R):
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


def _joint_T(root, child):
    for j in root.findall('joint'):
        if j.find('child').get('link') == child:
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0')).split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0')).split()]
            T = np.eye(4)
            T[:3, :3] = _rpy_to_R(*rpy)
            T[:3, 3] = xyz
            return j.find('parent').get('link'), T
    raise RuntimeError(f'no joint with child {child}')


def _T_base_to(robot_description, link):
    root = ET.fromstring(robot_description)
    T = np.eye(4)
    while link != 'base_link':
        link, Tj = _joint_T(root, link)
        T = Tj @ T
    return T


def _static_tf(name, T, parent, child, condition):
    x, y, z = T[:3, 3]
    qx, qy, qz, qw = _R_to_quat(T[:3, :3])
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=['--x', f'{x:.6f}', '--y', f'{y:.6f}', '--z', f'{z:.6f}',
                   '--qx', f'{qx:.6f}', '--qy', f'{qy:.6f}',
                   '--qz', f'{qz:.6f}', '--qw', f'{qw:.6f}',
                   '--frame-id', parent, '--child-frame-id', child],
        parameters=[{'use_sim_time': True}],
        condition=condition,
    )


def generate_launch_description():

    pkg = get_package_share_directory('jig_description')
    xacro_file = os.path.join(pkg, 'urdf', 'Main_jig.urdf.xacro')
    bridge_config = os.path.join(pkg, 'config', 'gz_bridge.yaml')
    fastlio_config = os.path.join(pkg, 'config', 'fast_lio_sim.yaml')
    robot_description = xacro.process_file(xacro_file).toxml()

    world = LaunchConfiguration('world')
    speed = LaunchConfiguration('speed')
    odom_source = LaunchConfiguration('odom_source')

    args = [
        DeclareLaunchArgument('world', default_value='jig_world',
                              description='Gazebo world name (without .sdf)'),
        DeclareLaunchArgument('speed', default_value='0.3',
                              description='Jig linear speed in m/s'),
        DeclareLaunchArgument('odom_source', default_value='icp',
                              description='icp = rtabmap ICP+IMU, fastlio = FAST-LIO2, gt = Gazebo ground truth'),
    ]

    # ── Shared ICP params for a 3D lidar (based on rtabmap lidar3d example) ──
    icp_params = {
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/VoxelSize': '0.1',
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '2',
        'Icp/MaxCorrespondenceDistance': '1.0',
        'Icp/Strategy': '1',          # libpointmatcher; use '0' if not available
        'Icp/OutlierRatio': '0.7',
    }

    # ── Gazebo ────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r',
             [os.path.join(pkg, 'worlds', ''), world, '.sdf']],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'jig', '-topic', 'robot_description', '-z', '0.15'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # Ground-truth odom -> base_link TF, only in gt mode
    gt_tf_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gt_tf_bridge',
        arguments=['/model/jig/odom_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/jig/odom_tf', '/tf')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(odom_source, 'gt')),
    )

    # ── Robot State Publisher: the ONLY source of sensor TFs ──
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # ── IMU orientation filter (mimics real pipeline) ─────────
    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        parameters=[{'use_sim_time': True, 'use_mag': False,
                     'publish_tf': False, 'world_frame': 'enu'}],
        remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')],
    )

    # ── Lidar(+IMU) odometry ──────────────────────────────────
    icp_odom = Node(
        package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
        output='screen',
        condition=IfCondition(EqualsSubstitution(odom_source, 'icp')),
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'wait_for_transform': 0.2,
            'expected_update_rate': 15.0,
            'deskewing': False,          # sim clouds have no per-point time
            'wait_imu_to_init': True,
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            'OdomF2M/BundleAdjustment': 'false',
            **icp_params,
        }],
        remappings=[('scan_cloud', '/scan/points'), ('imu', '/imu/data')],
    )

    # ── FAST-LIO2 odometry (odom_source:=fastlio) ─────────────
    is_fastlio = IfCondition(EqualsSubstitution(odom_source, 'fastlio'))
    T_base_imu = _T_base_to(robot_description, 'imu_link')

    lidar_to_fastlio = Node(
        package='jig_description', executable='lidar_to_fastlio',
        name='lidar_to_fastlio', condition=is_fastlio,
        parameters=[{'use_sim_time': True, 'min_range': 0.1}],
        remappings=[('points_in', '/scan/points'), ('points_out', '/fastlio/points')],
    )

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
        output='screen', condition=is_fastlio,
        parameters=[fastlio_config, {'use_sim_time': True}],
    )

    # Connect FAST-LIO's frames to our tree (values computed from the URDF)
    tf_odom_camera_init = _static_tf('tf_odom_camera_init', T_base_imu,
                                     'odom', 'camera_init', is_fastlio)
    tf_body_base = _static_tf('tf_body_base', np.linalg.inv(T_base_imu),
                              'body', 'base_link', is_fastlio)

    # ── RTAB-Map (graph SLAM + loop closure) ──────────────────
    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        arguments=['-d'],                 # delete previous db on start; remove to keep maps
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',      # odom from TF (icp_odometry, FAST-LIO chain or gt bridge)
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,         # cloud + odom share the same stamp
            'wait_for_transform': 0.2,
            'wait_imu_to_init': True,
            # Graph / memory
            'Mem/IncrementalMemory': 'true',
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Mem/LaserScanNormalK': '20',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            # Registration: ICP on lidar, full 6DoF
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'false',
            'Icp/CorrespondenceRatio': '0.2',
            'Optimizer/GravitySigma': '0.3',   # IMU gravity constraint against z drift
            # ── Maps: one SLAM graph → both outputs ──
            #   3D: /cloud_map (assembled lidar cloud), /cloud_obstacles, /cloud_ground
            #   2D: /map (nav_msgs/OccupancyGrid, obstacles projected to the floor)
            'Grid/Sensor': '0',               # build grids from the lidar
            'Grid/3D': 'true',                # keep 3D occupancy (also projected to 2D /map)
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '20',
            'Grid/RayTracing': 'true',        # clear free space in the 2D map
            'Grid/NormalsSegmentation': 'true',
            'Grid/MaxGroundAngle': '30',      # deg; flatter surfaces = ground
            'Grid/MaxObstacleHeight': '2.0',  # m above base_link; ignore ceiling in 2D map
            'Grid/NoiseFilteringRadius': '0.1',
            'Grid/NoiseFilteringMinNeighbors': '5',
            **icp_params,
        }],
        remappings=[('scan_cloud', '/scan/points'), ('imu', '/imu/data')],
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'jig.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Waits for /ground_truth/odom itself; the delay keeps the jig still while
    # ICP odom / FAST-LIO initialise the IMU (FAST-LIO needs it stationary)
    path_motion = TimerAction(
        period=8.0,
        actions=[Node(
            package='jig_description', executable='jig_path_motion',
            name='jig_path_motion', output='screen',
            parameters=[{'world': world, 'speed': speed, 'use_sim_time': True}],
        )],
    )

    return LaunchDescription(args + [
        gz_sim, rsp, spawn, bridge, gt_tf_bridge,
        imu_filter, icp_odom,
        lidar_to_fastlio, fast_lio, tf_odom_camera_init, tf_body_base,
        rtabmap,
        rviz, path_motion,
    ])