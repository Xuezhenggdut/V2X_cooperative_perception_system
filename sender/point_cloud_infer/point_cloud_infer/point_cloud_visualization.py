import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
import open3d


class PointCloudSubscriber(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window(window_name='Point cloud')

        self.crt = self.vis.get_view_control()
        # self.crt.set_lookat(np.array([0, 0, 55]))

        render_opt = self.vis.get_render_option()
        render_opt.point_size = 1
        render_opt.background_color = np.asarray([0, 0, 0])
        self.open3d_pcd = open3d.geometry.PointCloud()

        self.pcd_subscriber = self.create_subscription(
            PointCloud2,  # Msg type
            'point_cloud',  # topic
            self.listener_callback,  # Function to call
            100  # QoS
        )

    def listener_callback(self, msg):
        # frame_id = msg.header.frame_id
        # if frame_id == 'LiDAR0':
        pcd_as_numpy_array = np.array(list(read_points(msg)))
        # self.vis.remove_geometry(self.open3d_pcd)
        self.open3d_pcd = open3d.geometry.PointCloud(
            open3d.utility.Vector3dVector(pcd_as_numpy_array[:, 0:3]))  # pcd_as_numpy_array第4列包含强度

        self.vis.clear_geometries()
        self.vis.add_geometry(self.open3d_pcd)

        self.crt.set_up((1, 0, 1))  # 设置垂直指向屏幕上方的向量
        self.crt.set_front((-1, 0, 1))  # 设置垂直指向屏幕外的向量
        self.crt.set_zoom(0.2)  # 设置视角放大比例

        self.vis.poll_events()
        self.vis.update_renderer()


import sys
# from collections import namedtuple
# import ctypes
import math
import struct
# from sensor_msgs.msg import PointCloud2, PointField


_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber('point_cloud_visualization')
    rclpy.spin(point_cloud_subscriber)

    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
