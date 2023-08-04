# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# from vision_msgs.msg import Detection3DArray
# from numpy import quaternion
# from tlvmessage import *
import select
import sys
#
import numpy as np
import open3d
import struct
import socket
import threading
import queue
import time
# import _thread


# SEND = 0
# RECEIVE = 1
#
#
# class TLVMessage:
#     def __init__(self, msg: bytes, message_type, new_msg=False, config: tuple = (b'\x00\x00\x00\x11',  # aid
#                                                                                  b'\x00\x00\x00\x01',  # traffic_period
#                                                                                  b'\x00\x00\x00\x00',  # priority
#                                                                                  b'\x00\x00\xff\xff')):  # traffic_id
#         """
#         Initialization function.
#
#         :param msg: Bytes TLV message or bytes payload.
#         :param message_type: SEND or RECEIVE TLV message type.
#         :param new_msg: Whether to generate a new TLV message.
#         """
#         self.message_type = message_type  # 消息类型，发送或接收
#         self.unresolved = False
#         if new_msg:
#             self.send_tlv_message = {'aid': config[0], 'traffic_period': config[1],
#                                      'network_protocol_type': b'\x00\x00\x00\x04', 'priority': config[2],
#                                      'app_layer_id_changed': b'\x00\x00\x00\x00', 'traffic_id': config[3],
#                                      'source_address': b'\x00\x00\x00\x00\x00\x00\x00\x00'
#                                                        b'\x00\x00\x00\x00\x00\x00\x00\x00',
#                                      'destination_address': b'\x00\x00\x00\x00\x00\x00\x00\x00'
#                                                             b'\x00\x00\x00\x00\x00\x00\x00\x00',
#                                      'payload': msg, 'payload_len': len(msg)}
#             self.tlv_raw_message = b'\x00\x04\x00\x04' + self.send_tlv_message['aid']
#             self.tlv_raw_message += b'\x00\x05\x00\x04' + self.send_tlv_message['traffic_period']
#             self.tlv_raw_message += b'\x00\x06\x00\x04' + self.send_tlv_message['network_protocol_type']
#             self.tlv_raw_message += b'\x00\x07\x00\x04' + self.send_tlv_message['priority']
#             self.tlv_raw_message += b'\x00\x08\x00\x04' + self.send_tlv_message['app_layer_id_changed']
#             self.tlv_raw_message += b'\x00\x09\x00\x04' + self.send_tlv_message['traffic_id']
#             self.tlv_raw_message += b'\x00\x0a\x00\x10' + self.send_tlv_message['source_address']
#             self.tlv_raw_message += b'\x00\x0b\x00\x10' + self.send_tlv_message['destination_address']
#             s = hex(self.send_tlv_message['payload_len'])
#             s = s[2:]
#             if len(s) < 4:
#                 s = (4 - len(s)) * '0' + s
#
#             self.tlv_raw_message += b'\x00\x01' + bytes.fromhex(s) + self.send_tlv_message['payload']
#         else:
#             self.tlv_raw_message = msg
#             self.message_len = len(msg)
#             if message_type is SEND:
#                 self.send_tlv_message = {'aid': b'', 'traffic_period': b'', 'network_protocol_type': b'',
#                                          'priority': b'', 'app_layer_id_changed': b'', 'traffic_id': b'',
#                                          'source_address': b'', 'destination_address': b'', 'payload': b'',
#                                          'payload_len': -1}
#                 index = 0
#                 while index < self.message_len - 1:
#                     tag = self.tlv_raw_message[index:index + 2]
#                     if tag == b'\x00\x04':  # AID
#                         self.send_tlv_message['aid'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x05':  # Traffic Period
#                         self.send_tlv_message['traffic_period'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x06':  # Network Protocol Type
#                         self.send_tlv_message['network_protocol_type'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x07':  # Priority
#                         self.send_tlv_message['priority'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x08':  # Application Layer ID Changed
#                         self.send_tlv_message['app_layer_id_changed'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x09':  # Traffic ID
#                         self.send_tlv_message['traffic_id'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x0a':  # Source Address
#                         self.send_tlv_message['source_address'] = self.tlv_raw_message[index + 4:index + 20]
#                         index += 20
#                     elif tag == b'\x00\x0b':  # Destination Address
#                         self.send_tlv_message['destination_address'] = self.tlv_raw_message[index + 4:index + 20]
#                         index += 20
#                     elif tag == b'\x00\x01':  # Payload
#                         self.send_tlv_message['payload_len'] = int.from_bytes(self.tlv_raw_message[index + 2:index + 4],
#                                                                               byteorder='big', signed=False)
#                         self.send_tlv_message['payload'] = \
#                             self.tlv_raw_message[index + 4:index + 4 + self.send_tlv_message['payload_len']]
#                         index += 4 + self.send_tlv_message['payload_len']
#                     else:
#                         self.unresolved = True
#                         break
#             elif message_type is RECEIVE:
#                 self.receive_tlv_message = {'aid': b'', 'network_protocol_type': b'', 'priority': b'',
#                                             'source_address': b'', 'destination_address': b'',
#                                             'payload': b'', 'payload_len': -1, 'rsrp': b'', 'sinr': b'',
#                                             'rx_total_power': b'', 'res_pool1_crb': b'', 'res_pool2_crb': b'',
#                                             'reserved1': b''}
#                 index = 0
#                 while index < self.message_len - 1:
#                     tag = self.tlv_raw_message[index:index + 2]
#                     # print(tag.hex())
#                     if tag == b'\x00\x05':  # AID
#                         self.receive_tlv_message['aid'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x06':  # Source Address
#                         self.receive_tlv_message['source_address'] = self.tlv_raw_message[index + 4:index + 20]
#                         index += 20
#                     elif tag == b'\x00\x07':  # Destination Address
#                         self.receive_tlv_message['destination_address'] = self.tlv_raw_message[index + 4:index + 20]
#                         index += 20
#                     elif tag == b'\x00\x08':  # Network Protocol Type
#                         self.receive_tlv_message['network_protocol_type'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x09':  # Priority
#                         self.receive_tlv_message['priority'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x0a':  # RSRP dBm
#                         self.receive_tlv_message['rsrp'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x0b':  # SINR dB
#                         self.receive_tlv_message['sinr'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x0c':  # RX Total Power
#                         self.receive_tlv_message['rx_total_power'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x0d':  # Resource Pool 1 CBR
#                         self.receive_tlv_message['res_pool1_crb'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x0e':  # Resource Pool 2 CBR
#                         self.receive_tlv_message['res_pool2_crb'] = self.tlv_raw_message[index + 4:index + 8]
#                         index += 8
#                     elif tag == b'\x00\x01':  # Payload
#                         self.receive_tlv_message['payload_len'] = \
#                             int.from_bytes(self.tlv_raw_message[index + 2:index + 4], byteorder='big', signed=False)
#                         self.receive_tlv_message['payload'] = \
#                             self.tlv_raw_message[index + 4:index + 4 + self.receive_tlv_message['payload_len']]
#                         index += 4 + self.receive_tlv_message['payload_len']
#                     elif tag == b'\x00\x02':  # Reserved1
#                         length = int.from_bytes(self.tlv_raw_message[index + 2:index + 4],
#                                                 byteorder='big', signed=False)
#                         self.receive_tlv_message['reserved1'] = self.tlv_raw_message[index + 4:index + 4 + length]
#                         index += 4 + length
#                     else:
#                         self.unresolved = True
#                         break
#             else:
#                 print('Unknown message type!')
#                 raise ValueError
#
#     def get_tlv_raw_message(self) -> bytes:
#         return self.tlv_raw_message
#
#     def get_payload(self) -> bytes:
#         # self.unresolved = False
#         if self.unresolved:
#             print('Can not resolve this TLV message!')
#             raise ValueError
#         else:
#             if self.message_type is SEND:
#                 return self.send_tlv_message['payload']
#             else:
#                 return self.receive_tlv_message['payload']
#
#
#     def __str__(self):
#         if self.unresolved:
#             return 'Unresolved message: 0x' + self.tlv_raw_message.hex()
#         else:
#             if self.message_type is SEND:
#                 return 'AID: %d, ' % int.from_bytes(self.send_tlv_message['aid'], byteorder='big', signed=False) + \
#                        'Traffic Period: %d, ' % \
#                        int.from_bytes(self.send_tlv_message['traffic_period'], byteorder='big', signed=False) + \
#                        'Network Protocol Type: %d, ' % \
#                        int.from_bytes(self.send_tlv_message['network_protocol_type'], byteorder='big', signed=False) + \
#                        'Priority: %d, ' % \
#                        int.from_bytes(self.send_tlv_message['priority'], byteorder='big', signed=False) + \
#                        'Application Layer ID Changed: %d, ' % \
#                        int.from_bytes(self.send_tlv_message['app_layer_id_changed'], byteorder='big', signed=False) + \
#                        'Traffic ID: %d\n' % \
#                        int.from_bytes(self.send_tlv_message['traffic_id'], byteorder='big', signed=False) + \
#                        'Source Address: %d, ' % \
#                        int.from_bytes(self.send_tlv_message['source_address'], byteorder='big', signed=False) + \
#                        'Destination Address: %d, ' \
#                        % int.from_bytes(self.send_tlv_message['destination_address'], byteorder='big', signed=False) + \
#                        'Payload length: %d\n' % self.send_tlv_message['payload_len'] + \
#                        'Payload: 0x' + self.send_tlv_message['payload'].hex()
#             else:
#                 return 'AID: %d, ' % int.from_bytes(self.receive_tlv_message['aid'], byteorder='big', signed=False) + \
#                        'Source Address: ' + self.receive_tlv_message['source_address'].hex() + ', ' + \
#                        'Destination Address: ' + self.receive_tlv_message['destination_address'].hex() + '\n' + \
#                        'Network Protocol Type: %d, ' % \
#                        int.from_bytes(self.receive_tlv_message['network_protocol_type'],
#                                       byteorder='big', signed=False) + \
#                        'Priority: %d, ' % \
#                        int.from_bytes(self.receive_tlv_message['priority'], byteorder='big', signed=False) + \
#                        'RSRP dBm: %ddBm, ' % \
#                        int.from_bytes(self.receive_tlv_message['rsrp'], byteorder='big', signed=True) + \
#                        'SINR dB: %ddB, ' % \
#                        int.from_bytes(self.receive_tlv_message['sinr'], byteorder='big', signed=True) + \
#                        'RX Total Power: %ddBm, ' % \
#                        int.from_bytes(self.receive_tlv_message['rx_total_power'], byteorder='big', signed=True) + \
#                        'Resource Pool 1 CBR: %d%%, ' % \
#                        int.from_bytes(self.receive_tlv_message['res_pool1_crb'], byteorder='big', signed=False) + \
#                        'Resource Pool 2 CBR: %d%%, ' % \
#                        int.from_bytes(self.receive_tlv_message['res_pool2_crb'], byteorder='big', signed=False) + \
#                        'Payload length: %d\n' % self.receive_tlv_message['payload_len'] + \
#                        'Payload: 0x' + self.receive_tlv_message['payload'].hex()

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


def test():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # sock.bind(('192.168.62.223', 30301))
    sock.bind(('192.168.62.223', 30299))  # 网线测试


    vis = open3d.visualization.Visualizer()
    vis.create_window(window_name='Bounding box')

    crt = vis.get_view_control()

    render_opt = vis.get_render_option()
    render_opt.point_size = 1
    render_opt.background_color = np.asarray([0, 0, 0])
    open3d_pcd = open3d.geometry.PointCloud()  # 创建空点云

    while True:
    #for _ in range(10):
        rec, client = sock.recvfrom(2048)
        print("来自客户端%s,发送的%s\n" % (client, rec))  # 打印接收的内容
        # tlv_rec = TLVMessage(rec, RECEIVE)
        # message = tlv_rec.get_payload()
        message = rec
        print(len(message))
        # # 解码时间戳
        timestamp_sec, = struct.unpack('!d', message[:8])
        timestamp_nsec, = struct.unpack('!d', message[8:16])
        print("Timestamp_sec:", timestamp_sec)
        print("Timestamp_nsec:", timestamp_nsec)
        # #
        vis.clear_geometries()
        # # 解码检测框
        offset = 16  # 起始偏移量
        while offset < len(message):
            # 检测框的解码
            box_data = message[offset:offset+40]
            box_values = struct.unpack('ffffffffff', box_data)

            # 打印检测框的值
            print("Box:")
            print(" size_x:", box_values[0])
            print(" size_y:", box_values[1])
            print(" size_z:", box_values[2])
            print(" center_x:", box_values[3])
            print(" center_y:", box_values[4])
            print(" center_z:", box_values[5])
            print(" quaternion_x:", box_values[6])
            print(" quaternion_y:", box_values[7])
            print(" quaternion_z:", box_values[8])
            print(" quaternion_w:", box_values[9])

            offset += 40

            size_x = box_values[0]
            size_y = box_values[1]
            size_z = box_values[2]
            center_x = box_values[3]
            center_y = box_values[4]
            center_z = box_values[5]
            quaternion_x = box_values[6]
            quaternion_y = box_values[7]
            quaternion_z = box_values[8]
            quaternion_w = box_values[9]


            v = np.array([0, 1, 0, 0])  # 指向ego车辆前方的向量：w, x, y, z。实部为0
            q = np.array([quaternion_w, quaternion_x, quaternion_y, quaternion_z])  # 四元数
            q_conj = np.array([q[0], -1 * q[1], -1 * q[2], -1 * q[3]])  # 四元数的共轭
            # q * v * q_conj 旋转后的向量：w, x, y, z
            v_new = quaternion_inner_product(quaternion_inner_product(q, v), q_conj)
            v_obj = v_new[1:]
            l_x = size_x / 2
            w_y = size_y / 2
            h_z = size_z / 2
            # 中心位于坐标原点的框的顶点
            a_points = np.array([[-l_x, -w_y, -h_z], [-l_x, -w_y, h_z], [-l_x, w_y, h_z], [-l_x, w_y, -h_z],
                                 [l_x, -w_y, -h_z], [l_x, -w_y, h_z], [l_x, w_y, h_z], [l_x, w_y, -h_z]])
            center_point = np.array([center_x, center_y, center_z])
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

            vis.add_geometry(line_set)

        crt.set_up((1, 0, 1))  # 设置垂直指向屏幕上方的向量
        crt.set_front((-1, 0, 1))  # 设置垂直指向屏幕外的向量
        crt.set_zoom(0.2)  # 设置视角放大比例

        vis.poll_events()
        vis.update_renderer()

    sock.close()






if __name__ == '__main__':
    test()
    # receive_and_print()





