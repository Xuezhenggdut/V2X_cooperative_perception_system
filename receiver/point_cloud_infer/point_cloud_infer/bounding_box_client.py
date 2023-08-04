import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import TransformStamped
# from numpy import quaternion
# from tlvmessage import *

import numpy as np
import open3d
import struct
import socket
import time


class Detection3DArraySubscriber(Node):

    pcd_buff = []
    buff_read_ind = 0
    buff_write_ind = 0
    BUFF_LEN = 10

    lidar_pose_buff = []
    lidar_pose_buff_read_ind = 0
    lidar_pose_buff_write_ind = 0
    lidar_pose_BUFF_LEN = 10

    def __init__(self, node_name: str):
        super().__init__(node_name)
        # self.vis = open3d.visualization.Visualizer()
        # self.vis.create_window(window_name='Bounding box')
        #
        # self.crt = self.vis.get_view_control()
        #
        # render_opt = self.vis.get_render_option()
        # render_opt.point_size = 1
        # render_opt.background_color = np.asarray([0, 0, 0])
        # self.open3d_pcd = open3d.geometry.PointCloud()  # 创建空点云

        # self.pcd_subscriber = self.create_subscription(
        #     PointCloud2,  # Msg type
        #     'point_cloud',  # topic
        #     self.point_cloud_callback,  # Function to call
        #     100  # QoS
        # )
        self.lidar_pose_subscriber = self.create_subscription(
            TransformStamped,  # Msg type
            'ego_lidar_pose1',  # topic
            self.lidar_pose_callback,  # Function to call
            100  # QoS
        )

        self.pcd_subscriber = self.create_subscription(
            Detection3DArray,  # Msg type
            'bbox1',  # topic
            self.bbox_callback,  # Function to call
            100  # QoS
        )

    def bbox_callback(self, msg):
        objs_stamp_sec = msg.header.stamp.sec
        objs_stamp_nsec = msg.header.stamp.nanosec
        print(self.get_clock().now().to_msg())
        print("objs_stamp_sec", objs_stamp_sec)
        print("objs_stamp_nsec",objs_stamp_nsec)
        objs = msg.detections  # Detection3D[]->list
        # print(objs)
        obj_num = len(objs)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建UDP套接字
        remote_address = ('192.168.62.199', 30299)  # OBU的地址和端口
        # remote_address = ('192.168.62.223', 30299)  # 网线测试

        PACKET_MAX_LENGTH = 1450  # byte 最大TLV消息长度

        # 发送周期（单位：秒）
        SEND_PERIOD = 0.02

        # self.vis.clear_geometries()
        # if len(self.pcd_buff) == 0:
        #     self.get_logger().warn('No point cloud data to visualize')
        # else:
        #     self.vis.add_geometry(self.pcd_buff[self.buff_read_ind])
        #
        #     if self.buff_read_ind < self.BUFF_LEN-1:
        #         self.buff_read_ind += 1
        #     else:
        #         self.buff_read_ind = 0


        # 获取当前时间戳
        timestamp = time.time()
        print("timestamp        ", timestamp)


        # 上次发送的时间戳
        last_send_timestamp = timestamp

        # 构建TLV消息
        tlv_msg = b''  # 初始化tlv消息数据

        # 添加时间戳字段到TLV消息中
        tlv_msg += struct.pack('!d', objs_stamp_sec)
        tlv_msg += struct.pack('!d', objs_stamp_nsec)
        # print(len(tlv_msg))

        # 从lidar_pose缓存中读取数据并转换为字节流准备发送
        if len(self.lidar_pose_buff) == 0:
            self.get_logger().warn('No lidar pose data to visualize')
        else:
            lidar_pose_msg = self.lidar_pose_buff[self.lidar_pose_buff_read_ind]
            # print('lidar_pose_msg            ', lidar_pose_msg)
            lidar_pose_x = lidar_pose_msg.transform.translation.x
            lidar_pose_y = lidar_pose_msg.transform.translation.y
            lidar_pose_z = lidar_pose_msg.transform.translation.z
            lidar_pose_roll = lidar_pose_msg.transform.rotation.x
            lidar_pose_yaw = lidar_pose_msg.transform.rotation.y
            lidar_pose_pitch = lidar_pose_msg.transform.rotation.z
            frame_id = lidar_pose_msg.transform.rotation.w
            print("frame_id:  ",frame_id)
            tlv_msg += struct.pack('fffffff',frame_id, lidar_pose_x, lidar_pose_y, lidar_pose_z, lidar_pose_roll, lidar_pose_yaw, lidar_pose_pitch)
        if self.lidar_pose_buff_read_ind < self.lidar_pose_BUFF_LEN - 1:
            self.lidar_pose_buff_read_ind += 1
        else:
            self.lidar_pose_buff_read_ind = 0


        # objs[i]->Detection3D objs[i].bbox->BoundingBox3D
        for i in range(obj_num):
            score = objs[i].results[0].score
            if score < 0.5:  # 置信度小于阈值则跳过
                continue

            size = objs[i].bbox.size  # l,w,h
            center = objs[i].bbox.center.position  # x,y,z
            quaternion = objs[i].bbox.center.orientation  # x,y,z,w

            # 提取框的大小
            size_x = size.x
            size_y = size.y
            size_z = size.z

            # 提取框的中心位置
            center_x = center.x
            center_y = center.y
            center_z = center.z

            # 提取框的姿态
            quaternion_x = quaternion.x
            quaternion_y = quaternion.y
            quaternion_z = quaternion.z
            quaternion_w = quaternion.w

            # 将框的数据打包为字节流
            box_bytes = struct.pack('ffffffffff', size_x, size_y, size_z, center_x, center_y, center_z, quaternion_x, quaternion_y, quaternion_z, quaternion_w)
            # print(size_x, size_y, size_z, center_x, center_y, center_z, quaternion_x, quaternion_y, quaternion_z, quaternion_w)
            # print(len(box_bytes))
            # 判断TLV消息长度是否超过最大限制
            if len(tlv_msg) + len(box_bytes) > PACKET_MAX_LENGTH:
                # 达到最大尺寸限制，发送当前的TLV消息
                tlv_message = TLVMessage(tlv_msg, SEND,
                                         new_msg=True,
                                         config=(b'\x00\x00\x00\x70',   # aid
                                                 b'\x00\x00\x00\x0b',   # traffic_period
                                                 b'\x00\x00\x00\x7f',   # priority
                                                 b'\x00\x00\xff\xff'))  # traffic_id
                current_timestamp = time.time()


                # 计算距离上一次发送的时间间隔
                elapsed_time = current_timestamp - last_send_timestamp

                if elapsed_time < SEND_PERIOD:
                    # 未超过发送周期，等待剩余时间
                    time.sleep(SEND_PERIOD - elapsed_time)

                send_len = sock.sendto(tlv_message.get_tlv_raw_message(), remote_address)
                tlv_msg = b''  # 重置TLV消息数据
                # 重新添加时间戳字段到新的TLV消息中
                tlv_msg += struct.pack('!d', timestamp)
                # print(timestamp)
                # 更新上一次发送的时间戳
                last_send_timestamp = time.time()

            # 将框数据追加到TLV消息中
            tlv_msg += box_bytes


            # re_data = struct.unpack('ffffffffff', frame_data)

            # print(size)

            # self.get_logger().info('l:%f w:%f' % (size.x, size.y))
            # print(quaternion)

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
            # colors = np.array([[0, 1, 0] for k in range(12)])
            # line_set = open3d.geometry.LineSet()
            # line_set.lines = open3d.utility.Vector2iVector(box_lines)
            # line_set.colors = open3d.utility.Vector3dVector(colors)
            # line_set.points = open3d.utility.Vector3dVector(points_obj)

        #     self.vis.add_geometry(line_set)
        #
        # self.crt.set_up((1, 0, 1))  # 设置垂直指向屏幕上方的向量
        # self.crt.set_front((-1, 0, 1))  # 设置垂直指向屏幕外的向量
        # self.crt.set_zoom(0.2)  # 设置视角放大比例
        #
        # self.vis.poll_events()
        # self.vis.update_renderer()

            # 发送最后一帧TLV消息
        if len(tlv_msg) > 0:
            print(len(tlv_msg))
            tlv_message = TLVMessage(tlv_msg, SEND,
                                     new_msg=True,
                                     config=(b'\x00\x00\x00\x70',  # aid
                                             b'\x00\x00\x00\x0b',  # traffic_period
                                             b'\x00\x00\x00\x7f',  # priority
                                             b'\x00\x00\xff\xff'))  # traffic_id

            current_timestamp = time.time()

            # 计算距离上一次发送的时间间隔
            elapsed_time = current_timestamp - last_send_timestamp
            print(elapsed_time*1000)
            print("1 tlv")
            timetest = self.get_clock().now().to_msg()
            # frame_info是包含当前帧信息的字典，包括帧id，sec，nanosec
            frame_info = {"id": frame_id, "sec": timetest.sec, "nanosec": timetest.nanosec}

            def save_frame_info(frame_info):
                # 打开文件，并以“a”模式追加写入
                with open("frame_info_bbox.txt", 'a') as file:
                    frame_id = frame_info["id"]
                    sec = frame_info["sec"]
                    nanosec = frame_info["nanosec"]

                    # 将帧信息写入文件，并在每个帧信息后面添加换行符
                    file.write(f"Frame ID: {frame_id}, sec: {sec}, nanosec:{nanosec}\n")

            save_frame_info(frame_info)

            if elapsed_time < SEND_PERIOD:
                # 未超过发送周期，等待剩余时间
                time.sleep(SEND_PERIOD - elapsed_time)
                print("sleep")

            send_len = sock.sendto(tlv_message.get_tlv_raw_message(), remote_address) # OBU测试
            # send_len = sock.sendto(tlv_msg, remote_address) # 网线测试




        # 关闭套接字
        sock.close()



    def lidar_pose_callback(self, msg):
        if len(self.lidar_pose_buff) < self.lidar_pose_BUFF_LEN:
            self.lidar_pose_buff.append(msg)
        else:
            self.lidar_pose_buff[self.lidar_pose_buff_write_ind] = msg
            if self.lidar_pose_buff_write_ind < self.lidar_pose_BUFF_LEN-1:
                self.lidar_pose_buff_write_ind += 1
            else:
                self.lidar_pose_buff_write_ind = 0

    # def point_cloud_callback(self, msg):
    #     point_cloud_stamp_sec = msg.header.stamp.sec
    #     point_cloud_stamp_nsec = msg.header.stamp.nanosec
    #     print("point_cloud_stamp_sec", point_cloud_stamp_sec)
    #     print(point_cloud_stamp_sec)
    #     pcd_as_numpy_array = np.array(list(read_points(msg)))
    #     # self.vis.remove_geometry(self.open3d_pcd)
    #     open3d_pcd = open3d.geometry.PointCloud(
    #         open3d.utility.Vector3dVector(pcd_as_numpy_array[:, 0:3]))  # pcd_as_numpy_array第4列包含强度
    #     if len(self.pcd_buff) < self.BUFF_LEN:
    #         self.pcd_buff.append(open3d_pcd)
    #     else:
    #         self.pcd_buff[self.buff_write_ind] = open3d_pcd
    #         if self.buff_write_ind < self.BUFF_LEN-1:
    #             self.buff_write_ind += 1
    #         else:
    #             self.buff_write_ind = 0


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
    bounding_box_client = Detection3DArraySubscriber('bounding_box_client')
    rclpy.spin(bounding_box_client)

    bounding_box_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()