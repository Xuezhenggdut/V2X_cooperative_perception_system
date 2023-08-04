import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3DArray
# from numpy import quaternion

import numpy as np
import open3d


class Detection3DArraySubscriber(Node):

    pcd_buff = []
    buff_read_ind = 0
    buff_write_ind = 0
    BUFF_LEN = 10

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window(window_name='Bounding box')

        self.crt = self.vis.get_view_control()

        render_opt = self.vis.get_render_option()
        render_opt.point_size = 1
        render_opt.background_color = np.asarray([0, 0, 0])
        self.open3d_pcd = open3d.geometry.PointCloud()  # 创建空点云

        self.pcd_subscriber = self.create_subscription(
            PointCloud2,  # Msg type
            'point_cloud',  # topic
            self.point_cloud_callback,  # Function to call
            100  # QoS
        )

        self.pcd_subscriber = self.create_subscription(
            Detection3DArray,  # Msg type
            'bbox',  # topic
            self.bbox_callback,  # Function to call
            100  # QoS
        )

    def bbox_callback(self, msg):
        objs = msg.detections  # Detection3D[]->list
        obj_num = len(objs)
        start_time = self.get_clock().now().to_msg()

        self.vis.clear_geometries()
        if len(self.pcd_buff) == 0:
            self.get_logger().warn('No point cloud data to visualize')
        else:
            self.vis.add_geometry(self.pcd_buff[self.buff_read_ind])

            if self.buff_read_ind < self.BUFF_LEN-1:
                self.buff_read_ind += 1
            else:
                self.buff_read_ind = 0

        # objs[i]->Detection3D objs[i].bbox->BoundingBox3D
        for i in range(obj_num):
            score = objs[i].results[0].score
            if score < 0.5:  # 置信度小于阈值则跳过
                continue

            size = objs[i].bbox.size  # l,w,h
            center = objs[i].bbox.center.position  # x,y,z
            quaternion = objs[i].bbox.center.orientation  # x,y,z,w

            v = np.array([0, 1, 0, 0])  # 指向ego车辆前方的向量：w, x, y, z。实部为0
            q = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])  # 四元数
            # print('quaternion: ')
            # print(q)
            q_conj = np.array([q[0], -1 * q[1], -1 * q[2], -1 * q[3]])  # 四元数的共轭
            # q * v * q_conj 旋转后的向量：w, x, y, z
            v_new = quaternion_inner_product(quaternion_inner_product(q, v), q_conj)
            v_obj = v_new[1:]

            l_x = size.x / 2
            w_y = size.y / 2
            h_z = size.z / 2
            # 中心位于坐标原点的框的顶点
            a_points = np.array([[-l_x, -w_y, -h_z], [-l_x, -w_y, h_z], [-l_x, w_y, h_z], [-l_x, w_y, -h_z],
                                 [l_x, -w_y, -h_z], [l_x, -w_y, h_z], [l_x, w_y, h_z], [l_x, w_y, -h_z]])
            center_point = np.array([center.x, center.y, center.z])

            b_points = np.zeros((8, 3))  # 旋转后的框
            for j in range(8):
                a_points_j = np.zeros(4)
                a_points_j[1:] = a_points[j, :]  # w x y z
                # q * b_points_j * q_conj  旋转后的向量：w, x, y, z
                a_points_j_new = quaternion_inner_product(quaternion_inner_product(q, a_points_j), q_conj)
                b_points[j, :] = a_points_j_new[1:]
            # print('points_obj: ')
            # print(points_obj)
            points_obj = b_points + center_point  # 平移整个框

            # 框顶点之间的连接线
            box_lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                                  [4, 5], [5, 6], [6, 7], [7, 4],
                                  [0, 4], [1, 5], [2, 6], [3, 7]])
            # 线的颜色
            colors = np.array([[0, 1, 0] for k in range(12)])
            line_set = open3d.geometry.LineSet()
            line_set.lines = open3d.utility.Vector2iVector(box_lines)
            line_set.colors = open3d.utility.Vector3dVector(colors)
            line_set.points = open3d.utility.Vector3dVector(points_obj)

            self.vis.add_geometry(line_set)

        self.crt.set_up((1, 0, 1))  # 设置垂直指向屏幕上方的向量
        self.crt.set_front((-1, 0, 1))  # 设置垂直指向屏幕外的向量
        # self.crt.set_up((0, -1, 0))  # 设置垂直指向屏幕上方的向量
        # self.crt.set_front((0, 0, -1))  # 设置垂直指向屏幕外的向量
        self.crt.set_zoom(0.2)  # 设置视角放大比例

        self.vis.poll_events()
        self.vis.update_renderer()
        stop_time = self.get_clock().now().to_msg()
        print("bounding box visualization delay: ", stop_time.sec + stop_time.nanosec / 1e9 - (start_time.sec + start_time.nanosec / 1e9))
        # frame_info是包含当前帧信息的字典，包括帧id，sec，nanosec
        frame_info = {"start_sec": start_time.sec + start_time.nanosec / 1e9,
                      "stop_sec": stop_time.sec + stop_time.nanosec / 1e9}

        print("bounding box visualization delay: ", stop_time.sec + stop_time.nanosec / 1e9 - (start_time.sec + start_time.nanosec / 1e9))
        def save_frame_info(frame_info):
            # 打开文件，并以“a”模式追加写入
            with open("frame_info_bbox_visualize.txt", 'a') as file:
                start_sec = "{:.9f}".format(frame_info["start_sec"])
                stop_sec = "{:.9f}".format(frame_info["stop_sec"])

                # 将帧信息写入文件，并在每个帧信息后面添加换行符
                file.write(
                    f"start_sec: {start_sec}, stop_sec: {stop_sec}\n")

        save_frame_info(frame_info)

    def point_cloud_callback(self, msg):
        pcd_as_numpy_array = np.array(list(read_points(msg)))
        # self.vis.remove_geometry(self.open3d_pcd)
        open3d_pcd = open3d.geometry.PointCloud(
            open3d.utility.Vector3dVector(pcd_as_numpy_array[:, 0:3]))  # pcd_as_numpy_array第4列包含强度
        if len(self.pcd_buff) < self.BUFF_LEN:
            self.pcd_buff.append(open3d_pcd)
        else:
            self.pcd_buff[self.buff_write_ind] = open3d_pcd
            if self.buff_write_ind < self.BUFF_LEN-1:
                self.buff_write_ind += 1
            else:
                self.buff_write_ind = 0


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

def quaternion_inner_product(q1: np.ndarray, q2: np.ndarray):
    """
    四元数内积
    :param q1: w x y z
    :param q2: w x y z
    :return: w x y z
    """
    r1 = q1[0]
    r2 = q2[0]
    v1 = np.array([q1[1], q1[2], q1[3]])
    v2 = np.array([q2[1], q2[2], q2[3]])
    r = r1 * r2 - np.dot(v1, v2)
    v = r1 * v2 + r2 * v1 + np.cross(v1, v2)
    q = np.array([r, v[0], v[1], v[2]])
    return q


def main(args=None):
    rclpy.init(args=args)
    bounding_box_3d_visualization = Detection3DArraySubscriber('bounding_box_3d_visualization')
    rclpy.spin(bounding_box_3d_visualization)

    bounding_box_3d_visualization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
