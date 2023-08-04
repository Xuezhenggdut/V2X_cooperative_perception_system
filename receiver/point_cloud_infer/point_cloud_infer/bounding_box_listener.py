import os
import rclpy
from rclpy.node import Node
import struct
import socket
from vision_msgs.msg import Detection3DArray, Detection3D
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

import os
import sys
current_folder = os.path.dirname(__file__)
tlvmessage_path = os.path.join(current_folder, "tlvmessage.py")
sys.path.append(os.path.dirname(tlvmessage_path))
from tlvmessage import TLVMessage

class BoxPublisher(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Detection3DArray, 'cav_bbox1', 100)
        self.lidar_pose_publisher = self.create_publisher(TransformStamped, 'receive_lidar_pose1', 100)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # start_time = self.get_clock().now().to_msg()
        # 接收到的检测框，需要转换到本车的坐标系下
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('192.168.62.223', 30301)) # OBU测试
        # sock.bind(('192.168.62.223', 30299))  # 网线测试

        SEND = 0
        RECEIVE = 1

        while True:
            rec, client = sock.recvfrom(2048)
            start_time = self.get_clock().now().to_msg()
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

            # time_listener = self.get_clock().now().to_msg()
            # # frame_info是包含当前帧信息的字典，包括帧id，sec，nanosec
            # frame_info = {"id": frame_id, "sec": time_listener.sec, "nanosec": time_listener.nanosec, "bbox_receiver_size": len(message)}
            #
            # def save_frame_info(frame_info):
            #     # 打开文件，并以“a”模式追加写入
            #     with open("frame_info_listener.txt", 'a') as file:
            #         frame_id = frame_info["id"]
            #         sec = frame_info["sec"]
            #         nanosec = frame_info["nanosec"]
            #         bbox_receiver_size = frame_info["bbox_receiver_size"]
            #
            #         # 将帧信息写入文件，并在每个帧信息后面添加换行符
            #         file.write(f"Frame ID: {frame_id}, sec: {sec}, nanosec:{nanosec}, bbox_receiver_size:{bbox_receiver_size}\n")
            #
            # save_frame_info(frame_info)

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

            bbox_size_bytes = len(message)
            stop_time = self.get_clock().now().to_msg()
            # frame_info是包含当前帧信息的字典，包括帧id，sec，nanosec
            frame_info = {"id": frame_id, "start_sec": start_time.sec + start_time.nanosec / 1e9,
                          "stop_sec": stop_time.sec + stop_time.nanosec / 1e9, "bbox_size_bytes": bbox_size_bytes}

            def save_frame_info(frame_info):
                # 打开文件，并以“a”模式追加写入
                with open("frame_info_listener.txt", 'a') as file:
                    frame_id = frame_info["id"]
                    start_sec = "{:.9f}".format(frame_info["start_sec"])
                    stop_sec = "{:.9f}".format(frame_info["stop_sec"])
                    bbox_size_bytes = frame_info["bbox_size_bytes"]

                    # 将帧信息写入文件，并在每个帧信息后面添加换行符
                    file.write(f"Frame ID: {frame_id}, start_sec: {start_sec}, stop_sec: {stop_sec}, bbox_size_bytes: {bbox_size_bytes}\n")

            save_frame_info(frame_info)

            # 发布消息
            self.publisher.publish(box_msg)

        sock.close()


def main(args=None):
    rclpy.init(args=args)
    bounding_box_listener_publisher = BoxPublisher('bounding_box_listener_publisher')
    rclpy.spin(bounding_box_listener_publisher)

    bounding_box_listener_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()