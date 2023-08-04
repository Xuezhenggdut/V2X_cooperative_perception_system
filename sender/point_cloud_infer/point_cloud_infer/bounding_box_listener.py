import os
import rclpy
from rclpy.node import Node
import struct
import socket
from vision_msgs.msg import Detection3DArray, Detection3D
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class BoxPublisher(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Detection3DArray, 'cav_bbox', 100)
        self.lidar_pose_publisher = self.create_publisher(TransformStamped, 'receive_lidar_pose', 100)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 接收到的检测框，需要转换到本车的坐标系下
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('192.168.62.224', 30301)) # OBU测试
        # sock.bind(('192.168.62.224', 30299))  # 网线测试

        while True:
            rec, client = sock.recvfrom(2048)
            # print("来自客户端%s,发送的%s\n" % (client, rec))  # 打印接收的内容
            tlv_rec = TLVMessage(rec, RECEIVE)
            message = tlv_rec.get_payload()
            # message = rec
            print(len(message))
            # # 解码时间戳
            timestamp_sec, = struct.unpack('!d', message[:8])
            timestamp_nsec, = struct.unpack('!d', message[8:16])
            print("Listener:Timestamp_sec:", timestamp_sec)
            print("Listener:Timestamp_nsec:", timestamp_nsec)
            receiver_time = self.get_clock().now().to_msg()
            print("det + com delay (s) ", receiver_time.sec + receiver_time.nanosec * 1e-9 - (
                        timestamp_sec + timestamp_nsec * 1e-9))
            # print(self.get_clock().now().to_msg())
            timestamp = Time()
            timestamp.sec = int(timestamp_sec)
            timestamp.nanosec = int(timestamp_nsec)

            # # 解码lidar pose
            lidar_pose_msg = struct.unpack('fffffff', message[16:44])
            frame_id = lidar_pose_msg[0]
            lidar_pose_x = lidar_pose_msg[1]
            lidar_pose_y = lidar_pose_msg[2]
            lidar_pose_z = lidar_pose_msg[3]
            lidar_pose_roll = lidar_pose_msg[4]
            lidar_pose_yaw = lidar_pose_msg[5]
            lidar_pose_pitch = lidar_pose_msg[6]
            # 创建 TransformStamped 对象添加到列表
            lidar_pose_msg = TransformStamped()
            lidar_pose_msg.header.frame_id = 'Lidar'
            lidar_pose_msg.transform.translation.x = lidar_pose_x
            lidar_pose_msg.transform.translation.y = lidar_pose_y
            lidar_pose_msg.transform.translation.z = lidar_pose_z
            lidar_pose_msg.transform.rotation.x = lidar_pose_roll
            lidar_pose_msg.transform.rotation.y = lidar_pose_yaw
            lidar_pose_msg.transform.rotation.z = lidar_pose_pitch
            lidar_pose_msg.transform.rotation.w = frame_id
            self.lidar_pose_publisher.publish(lidar_pose_msg)



            # # 解码检测框
            offset = 44  # 起始偏移量
            detections = []
            while offset < len(message):
                # 检测框的解码
                box_data = message[offset:offset + 40]
                box_values = struct.unpack('ffffffffff', box_data)

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

                # 创建 Detection3D 对象并添加到列表中
                detection = Detection3D()
                detection.bbox.size.x = size_x
                detection.bbox.size.y = size_y
                detection.bbox.size.z = size_z
                detection.bbox.center.position.x = center_x
                detection.bbox.center.position.y = center_y
                detection.bbox.center.position.z = center_z
                detection.bbox.center.orientation.x = quaternion_x
                detection.bbox.center.orientation.y = quaternion_y
                detection.bbox.center.orientation.z = quaternion_z
                detection.bbox.center.orientation.w = quaternion_w
                detections.append(detection)



            # 构建 Detection3DArray()
            box_msg = Detection3DArray()
            box_msg.header.stamp = timestamp
            box_msg.header.frame_id = 'bounding_box_frame'
            box_msg.detections = detections

            # 发布消息
            self.publisher.publish(box_msg)

        sock.close()

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
    bounding_box_listener_publisher = BoxPublisher('bounding_box_listener_publisher')
    rclpy.spin(bounding_box_listener_publisher)

    bounding_box_listener_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()