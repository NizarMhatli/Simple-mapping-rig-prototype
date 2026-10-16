#!/usr/bin/env python3
"""
Drive the jig through waypoints with physically consistent motion
and reactive obstacle avoidance.

Motion
  * reads the jig's actual pose from /ground_truth/odom (Gazebo OdometryPublisher)
  * sends body-frame velocity commands to /model/jig/cmd_vel (VelocityControl)
  * ramps velocity with acceleration limits so the IMU sees smooth motion
  * publishes NO TF — odom->base_link comes from the odometry pipeline

Obstacle avoidance (holonomic, reactive)
  * fuses the lidar cloud and the RealSense cloud into a short-term 2D
    obstacle memory (world frame, voxelised, forgets after `memory_time`)
      - the lidar is pitched 90°: it sees the sides/top well but has a ~38°
        blind cone straight ahead; the camera covers that cone
      - the memory keeps obstacles the sensors saw a moment ago but can't
        see now (e.g. behind or beside the jig while it slides past)
  * only points between `obstacle_min_z` and `obstacle_max_z` (base_link
    frame) count, so the floor and ceiling are ignored
  * velocity = goal attraction + repulsion from obstacles AHEAD within
    `influence_radius`, plus a tangential "slide" term when an obstacle sits
    between the jig and the goal (escapes local minima)
  * hard safety: any velocity component toward an obstacle closer than
    robot_radius + safety_margin is removed
  * a waypoint inside an obstacle, or one with no progress for
    `stuck_timeout`, is skipped with a warning

Waypoints: (x, y, yaw_deg) or (x, y, yaw_deg, z). Without z, the start height is held.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


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
    # Return to start (revisiting the start triggers loop closure)
    ( 1.5,  0.0,  90),
    ( 0.0,  0.0, 180),
    ( 0.0,  0.0, 360),
]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]])


def ramp(current, target, max_step):
    return current + max(-max_step, min(max_step, target - current))


class JigPathMotion(Node):

    def __init__(self):
        super().__init__('jig_path_motion')

        # ── motion ──
        self.declare_parameter('world', 'jig_world')   # kept for launch compatibility
        self.declare_parameter('speed', 0.3)           # m/s
        self.declare_parameter('yaw_speed', 0.5)       # rad/s
        self.declare_parameter('max_lin_acc', 0.3)     # m/s^2
        self.declare_parameter('max_ang_acc', 1.0)     # rad/s^2
        self.declare_parameter('pos_tol', 0.03)        # m
        self.declare_parameter('yaw_tol', 0.02)        # rad
        self.declare_parameter('loop', False)
        self.declare_parameter('odom_topic', '/ground_truth/odom')
        self.declare_parameter('cmd_topic', '/model/jig/cmd_vel')
        # ── obstacle avoidance ──
        self.declare_parameter('avoid_enable', True)
        self.declare_parameter('cloud_topics', ['/scan/points',
                                                '/camera/camera/depth/color/points'])
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'odom')  # frame of the debug cloud only
        self.declare_parameter('robot_radius', 0.15)   # m, jig footprint (circle)
        self.declare_parameter('safety_margin', 0.10)  # m, hard no-approach zone
        self.declare_parameter('influence_radius', 0.8)  # m of clearance where repulsion acts
        self.declare_parameter('repulse_gain', 1.5)    # x speed at zero clearance
        self.declare_parameter('obstacle_min_z', -0.08)  # m in base_link (floor is ~ -0.15)
        self.declare_parameter('obstacle_max_z', 1.0)
        self.declare_parameter('self_filter_radius', 0.2)  # m, ignore the jig's own body
        self.declare_parameter('voxel', 0.05)
        self.declare_parameter('memory_time', 8.0)     # s
        self.declare_parameter('max_points_per_cloud', 15000)
        self.declare_parameter('stuck_timeout', 15.0)  # s without 5 cm progress -> skip

        gp = lambda n: self.get_parameter(n).value
        self.speed, self.yaw_speed = gp('speed'), gp('yaw_speed')
        self.max_lin_acc, self.max_ang_acc = gp('max_lin_acc'), gp('max_ang_acc')
        self.pos_tol, self.yaw_tol = gp('pos_tol'), gp('yaw_tol')
        self.loop = gp('loop')
        self.avoid = gp('avoid_enable')
        self.base_frame, self.world_frame = gp('base_frame'), gp('world_frame')
        self.r_robot, self.margin = gp('robot_radius'), gp('safety_margin')
        self.R_inf, self.k_rep = gp('influence_radius'), gp('repulse_gain')
        self.zmin, self.zmax = gp('obstacle_min_z'), gp('obstacle_max_z')
        self.r_self, self.voxel = gp('self_filter_radius'), gp('voxel')
        self.memory_time = gp('memory_time')
        self.max_pts = int(gp('max_points_per_cloud'))
        self.stuck_timeout = gp('stuck_timeout')

        self.cmd_pub = self.create_publisher(Twist, gp('cmd_topic'), 10)
        self.create_subscription(Odometry, gp('odom_topic'), self._odom_cb, 10)

        # obstacle memory: voxel key -> last-seen time
        self.voxels = {}
        self.obstacles = np.zeros((0, 2))
        self.tf_cache = {}
        if self.avoid:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            for t in gp('cloud_topics'):
                self.create_subscription(PointCloud2, t, self._cloud_cb, qos_profile_sensor_data)
            self.obs_pub = self.create_publisher(PointCloud2, '~/obstacles', 1)
            self.create_timer(0.1, self._rebuild_obstacles)
            self.create_timer(0.5, self._publish_obstacles)

        self.pose = None          # (x, y, z, yaw)
        self.z_hold = None
        self.wp = 0
        self.done = False
        self.v = [0.0, 0.0, 0.0]  # commanded world-frame vx, vy, vz
        self.wz = 0.0
        self._reset_progress()

        self.dt = 0.02            # 50 Hz
        self.create_timer(self.dt, self._update)
        self.get_logger().info(
            f'Jig path motion: {len(WAYPOINTS)} waypoints, speed={self.speed} m/s, '
            f'avoidance={"on" if self.avoid else "off"}, waiting for {gp("odom_topic")}...')

    # ───────────────────────────── inputs ─────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z, yaw_from_quat(q))
        if self.z_hold is None:
            self.z_hold = p.z
            self.get_logger().info(f'Odom received, holding z={p.z:.3f}')

    def _tf_to_base(self, frame):
        """Sensor -> base_link (static, from robot_state_publisher). Cached."""
        if frame in self.tf_cache:
            return self.tf_cache[frame]
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except Exception:
            return None
        r, tr = t.transform.rotation, t.transform.translation
        T = (quat_to_R(r.x, r.y, r.z, r.w), np.array([tr.x, tr.y, tr.z]))
        self.tf_cache[frame] = T
        self.get_logger().info(f'Obstacle source: {frame}')
        return T

    def _cloud_cb(self, msg):
        if self.pose is None:
            return
        T = self._tf_to_base(msg.header.frame_id)
        if T is None:
            return
        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        if len(pts) > self.max_pts:
            pts = pts[:: int(math.ceil(len(pts) / self.max_pts))]
        pts = pts[np.isfinite(pts).all(axis=1)].astype(np.float64)

        R, t = T
        pb = pts @ R.T + t
        keep = ((pb[:, 2] > self.zmin) & (pb[:, 2] < self.zmax) &
                (np.hypot(pb[:, 0], pb[:, 1]) > self.r_self))
        pb = pb[keep, :2]
        if len(pb) == 0:
            return

        x, y, _, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        pw = np.column_stack((x + c * pb[:, 0] - s * pb[:, 1],
                              y + s * pb[:, 0] + c * pb[:, 1]))
        keys = np.unique(np.floor(pw / self.voxel).astype(np.int64), axis=0)
        now = self._now()
        for kx, ky in keys:
            self.voxels[(int(kx), int(ky))] = now

    def _rebuild_obstacles(self):
        cutoff = self._now() - self.memory_time
        self.voxels = {k: t for k, t in self.voxels.items() if t >= cutoff}
        if self.voxels:
            k = np.array(list(self.voxels.keys()), dtype=float)
            self.obstacles = (k + 0.5) * self.voxel
        else:
            self.obstacles = np.zeros((0, 2))

    def _publish_obstacles(self):
        if self.obs_pub.get_subscription_count() == 0 or len(self.obstacles) == 0:
            return
        h = Header(frame_id=self.world_frame, stamp=self.get_clock().now().to_msg())
        z = np.full((len(self.obstacles), 1), self.z_hold or 0.0)
        self.obs_pub.publish(pc2.create_cloud_xyz32(h, np.hstack([self.obstacles, z]).tolist()))

    # ──────────────────────────── control ─────────────────────────────
    def _reset_progress(self):
        self.best_dist = float('inf')
        self.best_time = None
        self.side = 0.0

    def _next_waypoint(self, reason=None):
        wp = WAYPOINTS[self.wp]
        if reason:
            self.get_logger().warn(f'Skipping waypoint {self.wp + 1} ({wp[0]:.2f}, {wp[1]:.2f}): {reason}')
        else:
            self.get_logger().info(
                f'Waypoint {self.wp + 1}/{len(WAYPOINTS)} reached ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]}°)')
        self.wp += 1
        self._reset_progress()
        if self.wp >= len(WAYPOINTS):
            if self.loop:
                self.wp = 0
                self.get_logger().info('Path complete — looping')
            else:
                self.done = True
                self.get_logger().info('Path complete — stopping. Save your maps now.')

    def _avoid(self, p, goal, v_att):
        """Return an obstacle-aware world-frame velocity (2,)."""
        obs = self.obstacles
        if not self.avoid or len(obs) == 0:
            return v_att
        d_vec = p - obs
        dist = np.linalg.norm(d_vec, axis=1)
        clear = dist - self.r_robot
        near = clear < self.R_inf
        if not near.any():
            return v_att

        u = d_vec[near] / np.maximum(dist[near], 1e-6)[:, None]   # unit, away from obstacle
        cl = np.maximum(clear[near], 0.0)
        w = ((self.R_inf - cl) / self.R_inf) ** 2
        # Only obstacles AHEAD (in the direction of travel) push back; walls beside or
        # behind don't, so the jig can drive along corridors and settle on goals near walls.
        na = np.linalg.norm(v_att)
        if na > 1e-6:
            w = w * np.clip(-(u @ (v_att / na)), 0.0, 1.0)
        rep_dir = (w[:, None] * u).sum(0)
        n = np.linalg.norm(rep_dir)
        v = v_att.copy()
        if n > 1e-6:
            rep_dir /= n
            strength = w.max()
            # Scaled by the attraction speed, so repulsion fades with it near the goal
            # (no equilibrium offset); the hard safety step below still prevents contact.
            v = v + self.k_rep * na * strength * rep_dir

            # Obstacle between jig and goal -> slide along it (escape local minimum)
            if na > 1e-6 and np.dot(v_att / na, rep_dir) < -0.6:
                if self.side == 0.0:
                    to_goal = goal - p
                    cross = rep_dir[0] * to_goal[1] - rep_dir[1] * to_goal[0]
                    self.side = 1.0 if cross >= 0 else -1.0   # keep the side for this waypoint
                tangent = self.side * np.array([-rep_dir[1], rep_dir[0]])
                v = v + na * strength * tangent

        # Hard safety: no motion toward anything inside robot_radius + margin
        close = np.where(clear < self.margin)[0]
        for i in close[np.argsort(clear[close])][:30]:
            ui = d_vec[i] / max(dist[i], 1e-6)
            toward = -np.dot(v, ui)
            if toward > 0:
                v = v + toward * ui

        sp = np.linalg.norm(v)
        if sp > self.speed:
            v *= self.speed / sp
        return v

    def _update(self):
        if self.pose is None:
            return
        if self.done:
            self._send(0.0, 0.0, 0.0, 0.0)
            return

        x, y, z, yaw = self.pose
        wp = WAYPOINTS[self.wp]
        tx, ty, tyaw = wp[0], wp[1], math.radians(wp[2])
        tz = wp[3] if len(wp) > 3 else self.z_hold
        p, goal = np.array([x, y]), np.array([tx, ty])

        ex, ey, ez = tx - x, ty - y, tz - z
        dist = math.hypot(ex, ey)
        dyaw = wrap(tyaw - yaw)
        now = self._now()

        # Waypoint sits inside an obstacle -> skip it
        if self.avoid and dist > self.pos_tol and len(self.obstacles):
            if np.min(np.linalg.norm(self.obstacles - goal, axis=1)) < self.r_robot:
                self._next_waypoint('inside an obstacle')
                return

        vx_d = vy_d = wz_d = 0.0
        if dist > self.pos_tol:
            # progress watchdog
            if self.best_time is None or dist < self.best_dist - 0.05:
                self.best_dist, self.best_time = dist, now
            elif now - self.best_time > self.stuck_timeout:
                self._next_waypoint(f'no progress for {self.stuck_timeout:.0f}s (blocked?)')
                return
            v = min(self.speed, 1.0 * dist)                 # slow down near the target
            v_att = np.array([v * ex / dist, v * ey / dist])
            vx_d, vy_d = self._avoid(p, goal, v_att)
        elif abs(dyaw) > self.yaw_tol:
            wz_d = math.copysign(min(self.yaw_speed, 1.5 * abs(dyaw)), dyaw)
        else:
            self._next_waypoint()
            return
        vz_d = max(-0.2, min(0.2, 1.0 * ez))

        # Acceleration limits -> smooth IMU signal
        dv, dw = self.max_lin_acc * self.dt, self.max_ang_acc * self.dt
        self.v = [ramp(self.v[0], vx_d, dv), ramp(self.v[1], vy_d, dv), ramp(self.v[2], vz_d, dv)]
        self.wz = ramp(self.wz, wz_d, dw)

        # World -> body frame (VelocityControl expects a body-frame twist)
        c, s = math.cos(yaw), math.sin(yaw)
        vbx = c * self.v[0] + s * self.v[1]
        vby = -s * self.v[0] + c * self.v[1]
        self._send(vbx, vby, self.v[2], self.wz)

    def _send(self, vx, vy, vz, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = float(vx), float(vy), float(vz)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = JigPathMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send(0.0, 0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()