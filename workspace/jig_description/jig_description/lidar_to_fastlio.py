#!/usr/bin/env python3
"""
Convert the Gazebo gpu_lidar cloud into an Ouster-style cloud for FAST-LIO2.

Why this is needed (sim only — the real MID360 driver doesn't need it):
  * Gazebo clouds contain inf/NaN for rays with no return. FAST-LIO's
    preprocess only checks a minimum range, so those points would pass
    straight into the map.
  * Gazebo captures the whole scan at one instant, so there is no motion
    distortion. FAST-LIO (lidar_type 3 = Ouster) reads per-point time from
    field `t`; setting t = 0 tells it "no deskewing needed". Velodyne mode
    would instead invent times from the azimuth and "undistort" a scan that
    isn't distorted.

Output fields match FAST-LIO's ouster_ros::Point:
  x y z intensity (float32), t (uint32), reflectivity (uint16),
  ring (uint8), ambient (uint16), range (uint32, mm)
"""
import array
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

# PointField datatype -> numpy type
_NP = {PointField.INT8: np.int8, PointField.UINT8: np.uint8,
       PointField.INT16: np.int16, PointField.UINT16: np.uint16,
       PointField.INT32: np.int32, PointField.UINT32: np.uint32,
       PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}

_OUT_DTYPE = np.dtype({
    'names':   ['x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range'],
    'formats': [np.float32, np.float32, np.float32, np.float32,
                np.uint32, np.uint16, np.uint8, np.uint16, np.uint32],
    'offsets': [0, 4, 8, 12, 16, 20, 22, 24, 28],
    'itemsize': 32,
})

_OUT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='t', offset=16, datatype=PointField.UINT32, count=1),
    PointField(name='reflectivity', offset=20, datatype=PointField.UINT16, count=1),
    PointField(name='ring', offset=22, datatype=PointField.UINT8, count=1),
    PointField(name='ambient', offset=24, datatype=PointField.UINT16, count=1),
    PointField(name='range', offset=28, datatype=PointField.UINT32, count=1),
]


class LidarToFastLio(Node):
    def __init__(self):
        super().__init__('lidar_to_fastlio')
        self.declare_parameter('min_range', 0.1)
        self.min_range = float(self.get_parameter('min_range').value)
        self.in_dtype = None
        self.pub = self.create_publisher(PointCloud2, 'points_out', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'points_in', self.cb, qos_profile_sensor_data)

    def _make_in_dtype(self, msg):
        names, formats, offsets = [], [], []
        for f in msg.fields:
            if f.name in ('x', 'y', 'z', 'intensity', 'ring') and f.count == 1:
                names.append(f.name)
                formats.append(_NP[f.datatype])
                offsets.append(f.offset)
        if not {'x', 'y', 'z'} <= set(names):
            raise RuntimeError('input cloud has no x/y/z')
        return np.dtype({'names': names, 'formats': formats,
                         'offsets': offsets, 'itemsize': msg.point_step})

    def cb(self, msg):
        if self.in_dtype is None or self.in_dtype.itemsize != msg.point_step:
            self.in_dtype = self._make_in_dtype(msg)
            self.get_logger().info(f'input fields: {list(self.in_dtype.names)}')

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.row_step != msg.width * msg.point_step:
            raw = raw.reshape(msg.height, msg.row_step)[:, : msg.width * msg.point_step].reshape(-1)
        pts = np.ascontiguousarray(raw).view(self.in_dtype)

        x, y, z = pts['x'], pts['y'], pts['z']
        r = np.sqrt(x * x + y * y + z * z)
        keep = np.isfinite(r) & (r >= self.min_range)
        n = int(keep.sum())

        out = np.zeros(n, dtype=_OUT_DTYPE)
        out['x'], out['y'], out['z'] = x[keep], y[keep], z[keep]
        if 'intensity' in pts.dtype.names:
            out['intensity'] = np.nan_to_num(pts['intensity'][keep])
        if 'ring' in pts.dtype.names:
            out['ring'] = pts['ring'][keep].astype(np.uint8)
        out['range'] = (r[keep] * 1000.0).astype(np.uint32)
        # out['t'] stays 0: the simulated scan is instantaneous

        o = PointCloud2()
        o.header = msg.header
        o.height, o.width = 1, n
        o.fields = _OUT_FIELDS
        o.is_bigendian = False
        o.point_step = _OUT_DTYPE.itemsize
        o.row_step = o.point_step * n
        o.is_dense = True
        o.data = array.array('B', out.tobytes())
        self.pub.publish(o)


def main():
    rclpy.init()
    node = LidarToFastLio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()