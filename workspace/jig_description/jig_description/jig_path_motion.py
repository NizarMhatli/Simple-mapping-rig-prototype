#!/usr/bin/env python3
"""Move the jig via Gazebo's set_pose service using gz.msgs.Pose."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import subprocess


# WAYPOINTS = [
#     ( 0.0,  0.0,   0),
#     ( 1.5,  0.0,   0),
#     ( 1.5,  1.5,  90),
#     ( 0.0,  1.5, 180),
#     (-1.5,  1.5, 180),
#     (-1.5,  0.0, 270),
#     (-1.5, -1.5, 270),
#     ( 0.0, -1.5,   0),
#     ( 1.5, -1.5,   0),
#     ( 1.5,  0.0,  90),
#     ( 0.0,  0.0, 180),
# ]
# WAYPOINTS = [
#     (0.0, 0.0,   0),
#     (0.0, 0.0,  45),
#     (0.0, 0.0,  90),
#     (0.0, 0.0, 135),
#     (0.0, 0.0, 180),
#     (0.0, 0.0, 225),
#     (0.0, 0.0, 270),
#     (0.0, 0.0, 315),
#     (0.0, 0.0, 360),
# ]
WAYPOINTS = [
    # Start — rotate to scan surroundings
    ( 0.0,  0.0,   0),
    ( 0.0,  0.0,  90),
    ( 0.0,  0.0, 180),
    ( 0.0,  0.0, 270),
    # Move right and scan
    ( 1.5,  0.0,   0),
    ( 1.5,  0.0,  90),
    ( 1.5,  1.5,  90),
    ( 1.5,  1.5, 180),
    # Move left and scan
    ( 0.0,  1.5, 180),
    (-1.5,  1.5, 180),
    (-1.5,  1.5, 270),
    (-1.5,  0.0, 270),
    # Move south and scan
    (-1.5, -1.5, 270),
    (-1.5, -1.5,   0),
    ( 0.0, -1.5,   0),
    ( 1.5, -1.5,   0),
    # Return to start
    ( 1.5,  0.0,  90),
    ( 0.0,  0.0, 180),
    ( 0.0,  0.0, 360),
]
Z = 0.05  # fixed height above ground


class JigPathMotion(Node):

    def __init__(self):
        super().__init__('jig_path_motion')

        self.declare_parameter('world', 'jig_world')
        self.declare_parameter('speed', 0.3)
        self.world = self.get_parameter('world').value
        self.speed = self.get_parameter('speed').value
        self.yaw_sp = 1.5

        self.tf_broadcaster = TransformBroadcaster(self)

        self.x   = WAYPOINTS[0][0]
        self.y   = WAYPOINTS[0][1]
        self.yaw = math.radians(WAYPOINTS[0][2])
        self.wp  = 0

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info(
            f'Jig path motion started — {len(WAYPOINTS)} waypoints, '
            f'speed={self.speed} m/s, world={self.world}'
        )

    def update(self):
        if self.wp >= len(WAYPOINTS):
            self.wp = 0
            self.get_logger().info('Path complete — looping')
            return

        tx, ty, tyaw_deg = WAYPOINTS[self.wp]
        tyaw = math.radians(tyaw_deg)
        dt   = 0.05

        dx   = tx - self.x
        dy   = ty - self.y
        dist = math.sqrt(dx**2 + dy**2)

        dyaw = tyaw - self.yaw
        while dyaw >  math.pi: dyaw -= 2 * math.pi
        while dyaw < -math.pi: dyaw += 2 * math.pi

        if dist > 0.05:
            step = min(self.speed * dt, dist)
            self.x += step * dx / dist
            self.y += step * dy / dist
        elif abs(dyaw) > 0.02:
            step = min(self.yaw_sp * dt, abs(dyaw))
            self.yaw += step * (1 if dyaw > 0 else -1)
        else:
            self.get_logger().info(
                f'Waypoint {self.wp+1}/{len(WAYPOINTS)} '
                f'({tx:.2f}, {ty:.2f}, {tyaw_deg}°)'
            )
            self.wp += 1
            return

        self._set_gz_pose()
        self._publish_tf()

    def _set_gz_pose(self):
        """Call gz service via subprocess — same as the working manual command."""
        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)
        req = (
            f'name: "jig" '
            f'position: {{x: {self.x:.4f}, y: {self.y:.4f}, z: {Z}}} '
            f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}'
        )
        subprocess.Popen(
            ['gz', 'service',
             '-s', f'/world/{self.world}/set_pose',
             '--reqtype', 'gz.msgs.Pose',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '100',
             '--req', req],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = Z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = JigPathMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()