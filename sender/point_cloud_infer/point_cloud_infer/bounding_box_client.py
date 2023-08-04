import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import TransformStamped
# from numpy import quaternion
# from tlvmessage import *

import os
import sys
current_folder = os.path.dirname(__file__)
tlvmessage_path = os.path.join(current_folder, "tlvmessage.py")
sys.path.append(os.path.dirname(tlvmessage_path))
from tlvmessage import TLVMessage

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

        self.lidar_pose_subscriber = self.create_subscription(
            TransformStamped,  # Msg type
            'ego_lidar_pose',  # topic
            self.lidar_pose_callback,  # Function to call
            100  # QoS
        )

        self.pcd_subscriber = self.create_subscription(
            Detection3DArray,  # Msg type
            'bbox',  # topic
            self.bbox_callback,  # Function to call
            100  # QoS
        )

    def bbox_callback(self, msg):
        objs_stamp_sec = msg.header.stamp.sec
        objs_stamp_nsec = msg.header.stamp.nanosec

        start_time = self.get_clock().now().to_msg()

        objs = msg.detections  # Detection3D[]->list
        # print(objs)
        obj_num = len(objs)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建UDP套接字
        remote_address = ('192.168.62.199', 30299)  # OBU的地址和端口
        # remote_address = ('192.168.62.223', 30299)  # 网线测试

        PACKET_MAX_LENGTH = 1450  # byte 最大TLV消息长度

        # 发送周期（单位：秒）
        SEND_PERIOD = 0.02

        # 获取当前时间戳
        timestamp = time.time()

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
            tlv_msg += struct.pack('fffffff', frame_id,lidar_pose_x, lidar_pose_y, lidar_pose_z, lidar_pose_roll, lidar_pose_yaw, lidar_pose_pitch)
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

            # 将框数据追加到TLV消息中
            tlv_msg += box_bytes

            # 发送一帧TLV消息
        if len(tlv_msg) > 0:
            print(len(tlv_msg))
            SEND = 0
            RECEIVE = 1
            tlv_message = TLVMessage(tlv_msg, SEND,
                                     new_msg=True,
                                     config=(b'\x00\x00\x00\x70',  # aid
                                             b'\x00\x00\x00\x0b',  # traffic_period
                                             b'\x00\x00\x00\x7f',  # priority
                                             b'\x00\x00\xff\xff'))  # traffic_id

            current_timestamp = time.time()

            # 计算距离上一次发送的时间间隔
            elapsed_time = current_timestamp - last_send_timestamp
            # print(elapsed_time*1000)
            print("1 tlv")

            if elapsed_time < SEND_PERIOD:
                # 未超过发送周期，等待剩余时间
                time.sleep(SEND_PERIOD - elapsed_time)
                print("sleep")

            bbox_size_bytes = len(tlv_msg)
            stop_time = self.get_clock().now().to_msg()
            # frame_info是包含当前帧信息的字典，包括帧id，sec，nanosec
            frame_info = {"id": frame_id, "start_sec": start_time.sec + start_time.nanosec / 1e9, "stop_sec": stop_time.sec + stop_time.nanosec / 1e9, "bbox_size_bytes": bbox_size_bytes}

            print("client delay: ", stop_time.sec + stop_time.nanosec / 1e9 - (start_time.sec + start_time.nanosec / 1e9))

            def save_frame_info(frame_info):
                # 打开文件，并以“a”模式追加写入
                with open("frame_info_sender_bbox.txt", 'a') as file:
                    frame_id = frame_info["id"]
                    start_sec = "{:.9f}".format(frame_info["start_sec"])
                    stop_sec = "{:.9f}".format(frame_info["stop_sec"])
                    bbox_size_bytes =  frame_info["bbox_size_bytes"]

                    # 将帧信息写入文件，并在每个帧信息后面添加换行符
                    file.write(f"Frame ID: {frame_id}, start_sec: {start_sec}, stop_sec: {stop_sec}, bbox_size_bytes: {bbox_size_bytes}\n")

            save_frame_info(frame_info)

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


def main(args=None):
    rclpy.init(args=args)
    bounding_box_client = Detection3DArraySubscriber('bounding_box_client')
    rclpy.spin(bounding_box_client)

    bounding_box_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
