import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import TransformStamped

# from numpy import quaternion
# from tlvmessage import *

import numpy as np
import yaml, math
import open3d
import struct
import socket
import time
from scipy.spatial.transform import Rotation


class Detection3DArraySubscriber(Node):

    pcd_buff = []
    buff_read_ind = 0
    buff_write_ind = 0
    BUFF_LEN = 100
    box_msgs_buff = []
    box_msgs_buff_read_ind = 0
    box_msgs_buff_write_ind = 0
    box_msgs_BUFF_LEN = 100
    ego_lidar_pose_msgs_buff = []
    ego_lidar_pose_msgs_buff_read_ind = 0
    ego_lidar_pose_msgs_buff_write_ind = 0
    ego_lidar_pose_msgs_BUFF_LEN = 100

    lidar_pose_x = 0
    lidar_pose_y = 0
    lidar_pose_z = 0
    lidar_pose_roll = 0
    lidar_pose_yaw = 0
    lidar_pose_pitch = 0

    # ego_lidar_pose_timestamp = 0
    # ego_lidar_pose_x = 0
    # ego_lidar_pose_y = 0
    # ego_lidar_pose_z = 0
    # ego_lidar_pose_roll = 0
    # ego_lidar_pose_yaw = 0
    # ego_lidar_pose_pitch = 0

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window(window_name='Cooperative perception')

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

        self.lidar_pose_subscriber = self.create_subscription(
            TransformStamped,  # Msg type
            'receive_lidar_pose',  # topic
            self.lidar_pose_callback,  # Function to call
            100  # QoS

        )

        self.ego_lidar_pose_subscriber = self.create_subscription(
            TransformStamped,  # Msg type
            'ego_lidar_pose',  # topic
            self.ego_lidar_pose_callback,  # Function to call
            100  # QoS

        )

        self.pcd_subscriber = self.create_subscription(
            Detection3DArray,  # Msg type
            'cav_bbox',  # topic
            self.box_data_callback,  # Function to call
            100  # QoS

        )

    def bbox_callback(self, msg):
        objs = msg.detections  # Detection3D[]->list
        obj_num = len(objs)

        if len(self.box_msgs_buff) < self.box_msgs_BUFF_LEN:
            self.box_msgs_buff.append(msg)
        else:
            self.box_msgs_buff[self.box_msgs_buff_write_ind] = msg
            if self.box_msgs_buff_write_ind < self.box_msgs_BUFF_LEN - 1:
                self.box_msgs_buff_write_ind += 1
            else:
                self.box_msgs_buff_write_ind = 0

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

    def lidar_pose_callback(self, msg):
        self.lidar_pose_x = msg.transform.translation.x
        self.lidar_pose_y = msg.transform.translation.y
        self.lidar_pose_z = msg.transform.translation.z
        self.lidar_pose_roll = msg.transform.rotation.x
        self.lidar_pose_yaw = msg.transform.rotation.y
        self.lidar_pose_pitch = msg.transform.rotation.z

    def ego_lidar_pose_callback(self, msg):
        if len(self.ego_lidar_pose_msgs_buff) < self.ego_lidar_pose_msgs_BUFF_LEN:
            self.ego_lidar_pose_msgs_buff.append(msg)
        else:
            self.ego_lidar_pose_msgs_buff[self.ego_lidar_pose_msgs_buff_write_ind] = msg
            if self.ego_lidar_pose_msgs_buff_write_ind < self.ego_lidar_pose_msgs_BUFF_LEN - 1:
                self.ego_lidar_pose_msgs_buff_write_ind += 1
            else:
                self.ego_lidar_pose_msgs_buff_write_ind = 0

        # self.ego_lidar_pose_timestamp = msg.header.stamp
        # self.ego_lidar_pose_x = msg.transform.translation.x
        # self.ego_lidar_pose_y = msg.transform.translation.y
        # self.ego_lidar_pose_z = msg.transform.translation.z
        # self.ego_lidar_pose_roll = msg.transform.rotation.x
        # self.ego_lidar_pose_yaw = msg.transform.rotation.y
        # self.ego_lidar_pose_pitch = msg.transform.rotation.z


    def box_data_callback(self, msg):

        # objs = msg.detections  # Detection3D[]->list
        # obj_num = len(objs)
        print(11111111111111111111111111111111111111111111111111111111111111111111111111)
        if len(self.pcd_buff) == 0 or len(self.box_msgs_buff) == 0 or len(self.ego_lidar_pose_msgs_buff) == 0:
            time.sleep(0.2)

        # 从点云缓存中读取点云并显示
        self.vis.clear_geometries()
        if len(self.pcd_buff) == 0:
            self.get_logger().warn('No point cloud data to visualize')
        else:
            self.vis.add_geometry(self.pcd_buff[self.buff_read_ind])

            if self.buff_read_ind < self.BUFF_LEN - 1:
                self.buff_read_ind += 1
            else:
                self.buff_read_ind = 0

        # 从本车检测框缓存中读取框并显示
        if len(self.box_msgs_buff) == 0:
            self.get_logger().warn('No listener box data to visualize')
        else:
            # 检测框读取显示
            box_msg = self.box_msgs_buff[self.box_msgs_buff_read_ind]
            ego_time_stamp = box_msg.header.stamp
            ego_objs = box_msg.detections  # Detection3D[]->list
            ego_obj_num = len(ego_objs)
            for i in range(ego_obj_num):
                score = ego_objs[i].results[0].score
                if score < 0.5:  # 置信度小于阈值则跳过
                    continue

                ego_size = ego_objs[i].bbox.size  # l,w,h
                ego_center = ego_objs[i].bbox.center.position  # x,y,z
                ego_quaternion = ego_objs[i].bbox.center.orientation  # x,y,z,w

                # self.get_logger().info('l:%f w:%f' % (size.x, size.y))
                # print(quaternion)

                ego_v = np.array([0, 1, 0, 0])  # 指向ego车辆前方的向量：w, x, y, z。实部为0
                ego_q = np.array([ego_quaternion.w, ego_quaternion.x, ego_quaternion.y, ego_quaternion.z])  # 四元数
                # print('quaternion: ')
                # print(q)
                ego_q_conj = np.array([ego_q[0], -1 * ego_q[1], -1 * ego_q[2], -1 * ego_q[3]])  # 四元数的共轭
                # q * v * q_conj 旋转后的向量：w, x, y, z
                v_new = quaternion_inner_product(quaternion_inner_product(ego_q, ego_v), ego_q_conj)
                v_obj = v_new[1:]

                ego_l_x = ego_size.x / 2
                ego_w_y = ego_size.y / 2
                ego_h_z = ego_size.z / 2
                # 中心位于坐标原点的框的顶点
                a_points = np.array([[-ego_l_x, -ego_w_y, -ego_h_z], [-ego_l_x, -ego_w_y, ego_h_z], [-ego_l_x, ego_w_y, ego_h_z], [-ego_l_x, ego_w_y, -ego_h_z],
                                     [ego_l_x, -ego_w_y, -ego_h_z], [ego_l_x, -ego_w_y, ego_h_z], [ego_l_x, ego_w_y, ego_h_z], [ego_l_x, ego_w_y, -ego_h_z]])
                ego_center_point = np.array([ego_center.x, ego_center.y, ego_center.z])

                ego_b_points = np.zeros((8, 3))  # 旋转后的框
                for j in range(8):
                    ego_a_points_j = np.zeros(4)
                    ego_a_points_j[1:] = a_points[j, :]  # w x y z
                    # q * b_points_j * q_conj  旋转后的向量：w, x, y, z
                    a_points_j_new = quaternion_inner_product(quaternion_inner_product(ego_q, ego_a_points_j), ego_q_conj)
                    ego_b_points[j, :] = a_points_j_new[1:]
                # print('points_obj: ')
                # print(points_obj)
                ego_points_obj = ego_b_points + ego_center_point  # 平移整个框

                # 框顶点之间的连接线
                ego_box_lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                                      [4, 5], [5, 6], [6, 7], [7, 4],
                                      [0, 4], [1, 5], [2, 6], [3, 7]])
                # 线的颜色 红色
                colors = np.array([[1, 0, 0] for k in range(12)])
                line_set1 = open3d.geometry.LineSet()
                line_set1.lines = open3d.utility.Vector2iVector(ego_box_lines)
                line_set1.colors = open3d.utility.Vector3dVector(colors)
                line_set1.points = open3d.utility.Vector3dVector(ego_points_obj)

                self.vis.add_geometry(line_set1)

            if self.box_msgs_buff_read_ind < self.box_msgs_BUFF_LEN - 1:
                self.box_msgs_buff_read_ind += 1
            else:
                self.box_msgs_buff_read_ind = 0

        # 接收协同车辆的感知检测框数据及lidar pose
        # print('cooperation       ', self.lidar_pose_x)
        # print('cooperation      ego ', self.ego_lidar_pose_x)

        # 计算接收的lidar pose信息转换为平移向量和旋转矩阵
        # lidar_rotation = np.array([-self.lidar_pose_roll, self.lidar_pose_pitch, -self.lidar_pose_yaw])  # roll, pitch, yaw
        # lidar_rotation = (lidar_rotation / 180) * np.pi
        # lidar_rotate_matrix = euler_angle_2_rotation_matrix(-lidar_rotation)  # 点云旋转矩阵
        # quaternion1 = Rotation.from_matrix(lidar_rotate_matrix).as_quat()
        # translation_vector1 = np.array([self.lidar_pose_x, self.lidar_pose_y, self.lidar_pose_z])

        # 计算ego lidar pose信息转换为平移向量和旋转矩阵
        # ego_lidar_rotation = np.array([self.ego_lidar_pose_roll, -self.ego_lidar_pose_pitch, self.ego_lidar_pose_yaw])  # roll, pitch, yaw
        # ego_lidar_rotation = (ego_lidar_rotation / 180) * np.pi
        # ego_lidar_rotate_matrix = euler_angle_2_rotation_matrix(ego_lidar_rotation)  # 点云旋转矩阵

        if len(self.ego_lidar_pose_msgs_buff) == 0:
            self.get_logger().warn('No ego lidar pose data to visualize')
        else:
            ego_lidar_pose_msg = self.ego_lidar_pose_msgs_buff[self.ego_lidar_pose_msgs_buff_read_ind]
            ego_lidar_pose_timestamp = ego_lidar_pose_msg.header.stamp
            ego_lidar_pose_x = ego_lidar_pose_msg.transform.translation.x
            ego_lidar_pose_y = ego_lidar_pose_msg.transform.translation.y
            ego_lidar_pose_z = ego_lidar_pose_msg.transform.translation.z
            ego_lidar_pose_roll = ego_lidar_pose_msg.transform.rotation.x
            ego_lidar_pose_yaw = ego_lidar_pose_msg.transform.rotation.y
            ego_lidar_pose_pitch = ego_lidar_pose_msg.transform.rotation.z
            if self.ego_lidar_pose_msgs_buff_read_ind < self.ego_lidar_pose_msgs_BUFF_LEN - 1:
                self.ego_lidar_pose_msgs_buff_read_ind += 1
            else:
                self.ego_lidar_pose_msgs_buff_read_ind = 0

        ###########################################
        pos_ego = np.array([ego_lidar_pose_x, ego_lidar_pose_y, ego_lidar_pose_z,
                            ego_lidar_pose_roll, ego_lidar_pose_yaw, ego_lidar_pose_pitch])
        pos_cav = np.array([self.lidar_pose_x, self.lidar_pose_y, self.lidar_pose_z,
                            self.lidar_pose_roll, self.lidar_pose_yaw, self.lidar_pose_pitch])

        x1_to_world = x_to_world(pos_cav)
        x2_to_world = x_to_world(pos_ego)
        world_to_x2 = np.linalg.inv(x2_to_world)
        transformation_matrix = np.dot(world_to_x2, x1_to_world)
        lidar_rotate_matrix = transformation_matrix[:-1,:-1]
        Sy = np.eye(3)
        Sy[-1,-1] = -1
        lidar_rotate_matrix = Sy @ lidar_rotate_matrix @ Sy
        lidar_rotate_matrix = lidar_rotate_matrix.T

        translation_vector = transformation_matrix[:-1,-1]
        translation_vector[1] = -translation_vector[1]

        ############################################

        objs = msg.detections  # Detection3D[]->list
        cav_time_stamp = msg.header.stamp
        print('self.ego_lidar_pose_timestamp   ', ego_lidar_pose_timestamp)
        print('ego_time_stamp           ', ego_time_stamp)
        print('cav_time_stamp           ', cav_time_stamp)
        receiver_time = self.get_clock().now().to_msg()
        print("det + com + fusion delay (s) ", receiver_time.sec + receiver_time.nanosec * 1e-9 - (cav_time_stamp.sec + cav_time_stamp.nanosec * 1e-9))
        # print(type(ego_time_stamp))
        # print(type(cav_time_stamp))
        # print(abs(ego_time_stamp - cav_time_stamp))
        #if ego_time_stamp == cav_time_stamp:
        if 1 == 1:
            obj_num = len(objs)
            # objs[i]->Detection3D objs[i].bbox->BoundingBox3D
            for i in range(obj_num):
                # score = objs[i].results[0].score
                # if score < 0.5:  # 置信度小于阈值则跳过
                #     continue

                size = objs[i].bbox.size  # l,w,h
                center = objs[i].bbox.center.position  # x,y,z
                quaternion = objs[i].bbox.center.orientation  # x,y,z,w

                # self.get_logger().info('l:%f w:%f' % (size.x, size.y))
                # print(quaternion)

                v = np.array([0, 1, 0, 0])  # 指向ego车辆前方的向量：w, x, y, z。实部为0
                q = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])  # 四元数

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
                points_obj = b_points + center_point  # 平移整个框 8x3

                # 将中心点坐标从lidar pose坐标系转换到全局坐标系
                center_point_global = np.dot(lidar_rotate_matrix, points_obj.T).T + translation_vector.reshape(1,-1)

                # 框顶点之间的连接线
                box_lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                                      [4, 5], [5, 6], [6, 7], [7, 4],
                                      [0, 4], [1, 5], [2, 6], [3, 7]])
                # 线的颜色 绿色
                colors = np.array([[0, 1, 0] for k in range(12)])
                line_set = open3d.geometry.LineSet()
                line_set.lines = open3d.utility.Vector2iVector(box_lines)
                line_set.colors = open3d.utility.Vector3dVector(colors)
                line_set.points = open3d.utility.Vector3dVector(center_point_global)

                self.vis.add_geometry(line_set)

            self.crt.set_up((1, 0, 1))  # 设置垂直指向屏幕上方的向量
            self.crt.set_front((-1, 0, 1))  # 设置垂直指向屏幕外的向量
            # self.crt.set_up((0, -1, 0))  # 设置垂直指向屏幕上方的向量
            # self.crt.set_front((0, 0, -1))  # 设置垂直指向屏幕外的向量


            self.crt.set_zoom(0.2)  # 设置视角放大比例

            self.vis.poll_events()
            self.vis.update_renderer()

def x_to_world(pose):
    """
    The transformation matrix from x-coordinate system to carla world system

    Parameters
    ----------
    pose : list
        [x, y, z, roll, yaw, pitch]

    Returns
    -------
    matrix : np.ndarray
        The transformation matrix.
    """
    x, y, z, roll, yaw, pitch = pose[:]

    # used for rotation matrix
    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))

    matrix = np.identity(4)
    # translation matrix
    matrix[0, 3] = x
    matrix[1, 3] = y
    matrix[2, 3] = z

    # rotation matrix
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r

    return matrix

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

def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])

def quaternion_inverse(q):
    x, y, z, w = q
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    inverse = np.array([x, y, z, w]) / norm
    return inverse

def euler_angle_2_rotation_matrix(theta):
    """
    欧拉角转旋转矩阵
    :param theta: roll, pitch, yaw 弧度单位
    :return:
    """
    r_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]])
    r_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])
    r_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]])
    return np.dot(r_z, np.dot(r_y, r_x))


SEND = 0
RECEIVE = 1

class TLVMessage:
    def __init__(self, msg: bytes, message_type, new_msg=False, config: tuple = (b'\x00\x00\x00\x11',  # aid
                                                                                 b'\x00\x00\x00\x01',  # traffic_period
                                                                                 b'\x00\x00\x00\x00',  # priority
                                                                                 b'\x00\x00\xff\xff')):  # traffic_id
        """
        Initialization function.

        :param msg: Bytes TLV message or bytes payload.
        :param message_type: SEND or RECEIVE TLV message type.
        :param new_msg: Whether to generate a new TLV message.
        """
        self.message_type = message_type  # 消息类型，发送或接收
        self.unresolved = False
        if new_msg:
            self.send_tlv_message = {'aid': config[0], 'traffic_period': config[1],
                                     'network_protocol_type': b'\x00\x00\x00\x04', 'priority': config[2],
                                     'app_layer_id_changed': b'\x00\x00\x00\x00', 'traffic_id': config[3],
                                     'source_address': b'\x00\x00\x00\x00\x00\x00\x00\x00'
                                                       b'\x00\x00\x00\x00\x00\x00\x00\x00',
                                     'destination_address': b'\x00\x00\x00\x00\x00\x00\x00\x00'
                                                            b'\x00\x00\x00\x00\x00\x00\x00\x00',
                                     'payload': msg, 'payload_len': len(msg)}
            self.tlv_raw_message = b'\x00\x04\x00\x04' + self.send_tlv_message['aid']
            self.tlv_raw_message += b'\x00\x05\x00\x04' + self.send_tlv_message['traffic_period']
            self.tlv_raw_message += b'\x00\x06\x00\x04' + self.send_tlv_message['network_protocol_type']
            self.tlv_raw_message += b'\x00\x07\x00\x04' + self.send_tlv_message['priority']
            self.tlv_raw_message += b'\x00\x08\x00\x04' + self.send_tlv_message['app_layer_id_changed']
            self.tlv_raw_message += b'\x00\x09\x00\x04' + self.send_tlv_message['traffic_id']
            self.tlv_raw_message += b'\x00\x0a\x00\x10' + self.send_tlv_message['source_address']
            self.tlv_raw_message += b'\x00\x0b\x00\x10' + self.send_tlv_message['destination_address']
            s = hex(self.send_tlv_message['payload_len'])
            s = s[2:]
            if len(s) < 4:
                s = (4 - len(s)) * '0' + s

            self.tlv_raw_message += b'\x00\x01' + bytes.fromhex(s) + self.send_tlv_message['payload']
        else:
            self.tlv_raw_message = msg
            self.message_len = len(msg)
            if message_type is SEND:
                self.send_tlv_message = {'aid': b'', 'traffic_period': b'', 'network_protocol_type': b'',
                                         'priority': b'', 'app_layer_id_changed': b'', 'traffic_id': b'',
                                         'source_address': b'', 'destination_address': b'', 'payload': b'',
                                         'payload_len': -1}
                index = 0
                while index < self.message_len - 1:
                    tag = self.tlv_raw_message[index:index + 2]
                    if tag == b'\x00\x04':  # AID
                        self.send_tlv_message['aid'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x05':  # Traffic Period
                        self.send_tlv_message['traffic_period'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x06':  # Network Protocol Type
                        self.send_tlv_message['network_protocol_type'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x07':  # Priority
                        self.send_tlv_message['priority'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x08':  # Application Layer ID Changed
                        self.send_tlv_message['app_layer_id_changed'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x09':  # Traffic ID
                        self.send_tlv_message['traffic_id'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0a':  # Source Address
                        self.send_tlv_message['source_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x0b':  # Destination Address
                        self.send_tlv_message['destination_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x01':  # Payload
                        self.send_tlv_message['payload_len'] = int.from_bytes(self.tlv_raw_message[index + 2:index + 4],
                                                                              byteorder='big', signed=False)
                        self.send_tlv_message['payload'] = \
                            self.tlv_raw_message[index + 4:index + 4 + self.send_tlv_message['payload_len']]
                        index += 4 + self.send_tlv_message['payload_len']
                    else:
                        self.unresolved = True
                        break
            elif message_type is RECEIVE:
                self.receive_tlv_message = {'aid': b'', 'network_protocol_type': b'', 'priority': b'',
                                            'source_address': b'', 'destination_address': b'',
                                            'payload': b'', 'payload_len': -1, 'rsrp': b'', 'sinr': b'',
                                            'rx_total_power': b'', 'res_pool1_crb': b'', 'res_pool2_crb': b'',
                                            'reserved1': b''}
                index = 0
                while index < self.message_len - 1:
                    tag = self.tlv_raw_message[index:index + 2]
                    # print(tag.hex())
                    if tag == b'\x00\x05':  # AID
                        self.receive_tlv_message['aid'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x06':  # Source Address
                        self.receive_tlv_message['source_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x07':  # Destination Address
                        self.receive_tlv_message['destination_address'] = self.tlv_raw_message[index + 4:index + 20]
                        index += 20
                    elif tag == b'\x00\x08':  # Network Protocol Type
                        self.receive_tlv_message['network_protocol_type'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x09':  # Priority
                        self.receive_tlv_message['priority'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0a':  # RSRP dBm
                        self.receive_tlv_message['rsrp'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0b':  # SINR dB
                        self.receive_tlv_message['sinr'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0c':  # RX Total Power
                        self.receive_tlv_message['rx_total_power'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0d':  # Resource Pool 1 CBR
                        self.receive_tlv_message['res_pool1_crb'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x0e':  # Resource Pool 2 CBR
                        self.receive_tlv_message['res_pool2_crb'] = self.tlv_raw_message[index + 4:index + 8]
                        index += 8
                    elif tag == b'\x00\x01':  # Payload
                        self.receive_tlv_message['payload_len'] = \
                            int.from_bytes(self.tlv_raw_message[index + 2:index + 4], byteorder='big', signed=False)
                        self.receive_tlv_message['payload'] = \
                            self.tlv_raw_message[index + 4:index + 4 + self.receive_tlv_message['payload_len']]
                        index += 4 + self.receive_tlv_message['payload_len']
                    elif tag == b'\x00\x02':  # Reserved1
                        length = int.from_bytes(self.tlv_raw_message[index + 2:index + 4],
                                                byteorder='big', signed=False)
                        self.receive_tlv_message['reserved1'] = self.tlv_raw_message[index + 4:index + 4 + length]
                        index += 4 + length
                    else:
                        self.unresolved = True
                        break
            else:
                print('Unknown message type!')
                raise ValueError

    def get_tlv_raw_message(self) -> bytes:
        return self.tlv_raw_message

    def get_payload(self) -> bytes:
        if self.unresolved:
            print('Can not resolve this TLV message!')
            raise ValueError
        else:
            if self.message_type is SEND:
                return self.send_tlv_message['payload']
            else:
                return self.receive_tlv_message['payload']

    def __str__(self):
        if self.unresolved:
            return 'Unresolved message: 0x' + self.tlv_raw_message.hex()
        else:
            if self.message_type is SEND:
                return 'AID: %d, ' % int.from_bytes(self.send_tlv_message['aid'], byteorder='big', signed=False) + \
                       'Traffic Period: %d, ' % \
                       int.from_bytes(self.send_tlv_message['traffic_period'], byteorder='big', signed=False) + \
                       'Network Protocol Type: %d, ' % \
                       int.from_bytes(self.send_tlv_message['network_protocol_type'], byteorder='big', signed=False) + \
                       'Priority: %d, ' % \
                       int.from_bytes(self.send_tlv_message['priority'], byteorder='big', signed=False) + \
                       'Application Layer ID Changed: %d, ' % \
                       int.from_bytes(self.send_tlv_message['app_layer_id_changed'], byteorder='big', signed=False) + \
                       'Traffic ID: %d\n' % \
                       int.from_bytes(self.send_tlv_message['traffic_id'], byteorder='big', signed=False) + \
                       'Source Address: %d, ' % \
                       int.from_bytes(self.send_tlv_message['source_address'], byteorder='big', signed=False) + \
                       'Destination Address: %d, ' \
                       % int.from_bytes(self.send_tlv_message['destination_address'], byteorder='big', signed=False) + \
                       'Payload length: %d\n' % self.send_tlv_message['payload_len'] + \
                       'Payload: 0x' + self.send_tlv_message['payload'].hex()
            else:
                return 'AID: %d, ' % int.from_bytes(self.receive_tlv_message['aid'], byteorder='big', signed=False) + \
                       'Source Address: ' + self.receive_tlv_message['source_address'].hex() + ', ' + \
                       'Destination Address: ' + self.receive_tlv_message['destination_address'].hex() + '\n' + \
                       'Network Protocol Type: %d, ' % \
                       int.from_bytes(self.receive_tlv_message['network_protocol_type'],
                                      byteorder='big', signed=False) + \
                       'Priority: %d, ' % \
                       int.from_bytes(self.receive_tlv_message['priority'], byteorder='big', signed=False) + \
                       'RSRP dBm: %ddBm, ' % \
                       int.from_bytes(self.receive_tlv_message['rsrp'], byteorder='big', signed=True) + \
                       'SINR dB: %ddB, ' % \
                       int.from_bytes(self.receive_tlv_message['sinr'], byteorder='big', signed=True) + \
                       'RX Total Power: %ddBm, ' % \
                       int.from_bytes(self.receive_tlv_message['rx_total_power'], byteorder='big', signed=True) + \
                       'Resource Pool 1 CBR: %d%%, ' % \
                       int.from_bytes(self.receive_tlv_message['res_pool1_crb'], byteorder='big', signed=False) + \
                       'Resource Pool 2 CBR: %d%%, ' % \
                       int.from_bytes(self.receive_tlv_message['res_pool2_crb'], byteorder='big', signed=False) + \
                       'Payload length: %d\n' % self.receive_tlv_message['payload_len'] + \
                       'Payload: 0x' + self.receive_tlv_message['payload'].hex()



def main(args=None):
    rclpy.init(args=args)
    bounding_box_server = Detection3DArraySubscriber('bounding_box_server')
    rclpy.spin(bounding_box_server)

    bounding_box_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


