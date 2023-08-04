# import sys
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
# from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import numpy as np
import open3d
import yaml, math
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message


class PCDPublisher(Node):

    rate = 10  # frame/seconds
    FRAME_START_NUM = 69
    FRAME_END_NUM = 221
    frame_num = FRAME_START_NUM

    header = Header()
    header.frame_id = 'LiDAR0'

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('pcd_path', '/home/thu/Downloads/2021_08_23_21_47_19/225')
        self.declare_parameter('rate', '5')

        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 100)
        self.lidar_pose_publisher = self.create_publisher(TransformStamped, 'ego_lidar_pose', 100)
        rate = self.get_parameter('rate').get_parameter_value().integer_value
        # rate = int(rate_str)
        timer_period = 1/rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        pcd_path = self.get_parameter('pcd_path').get_parameter_value().string_value
        assert os.path.exists(pcd_path), "路径不存在：" + pcd_path

        num_str = str(self.frame_num).zfill(6)
        pcd_file = pcd_path + '/' + num_str + '.pcd'
        # print(num_str)
        yaml_file = pcd_path + '/' + num_str + '.yaml'
        assert os.path.isfile(pcd_file), 'PCD文件不存在：' + pcd_file
        assert os.path.isfile(yaml_file), 'YAML配置文件不存在：' + yaml_file
        pcd = open3d.io.read_point_cloud(pcd_file)  # 返回open3d.geometry.PointCloud
        right_hand = np.array([[1, 0, 0],
                               [0, -1, 0],
                               [0, 0, 1]])
        pcd.rotate(right_hand, center=[0, 0, 0])  # 旋转点云到右手坐标系，y轴取反

        with open(yaml_file, 'r', encoding='utf-8') as f:
            result = yaml.load(f.read(), Loader=yaml.FullLoader)
            f.close()
        lidar_pose = result['lidar_pose']  # x y z roll yaw pitch，雷达（即ego车辆）在地图坐标系中的位置和姿态
        # 发布lidar pose
        lidar_pose_msg = TransformStamped()
        lidar_pose_msg.header.frame_id = 'Lidar'
        lidar_pose_msg.transform.translation.x = lidar_pose[0]
        lidar_pose_msg.transform.translation.y = lidar_pose[1]
        lidar_pose_msg.transform.translation.z = lidar_pose[2]
        lidar_pose_msg.transform.rotation.x = lidar_pose[3]
        lidar_pose_msg.transform.rotation.y = lidar_pose[4]
        lidar_pose_msg.transform.rotation.z = lidar_pose[5]
        lidar_pose_msg.transform.rotation.w = float(num_str)
        # print('lidar_pose:      ', lidar_pose)
        self.lidar_pose_publisher.publish(lidar_pose_msg)

        lidar_rotation = np.array([-lidar_pose[3], lidar_pose[5], -lidar_pose[4]])  # roll, pitch, yaw
        lidar_rotation = (lidar_rotation / 180) * np.pi
        lidar_rotate_matrix = euler_angle_2_rotation_matrix(-lidar_rotation)  # 点云旋转矩阵

        # 裁剪点云数据
        vol = open3d.visualization.SelectionPolygonVolume()
        vol.orthogonal_axis = 'z'
        vol.axis_max = 1
        vol.axis_min = -3
        bounding_ploy = np.array([[140.8, 140.8, 0],  # x-axis和y-axis区域
                                  [140.8, -140.8, 0],
                                  [-140.8, -140.8, 0],
                                  [-140.8, 140.8, 0]])
        bounding_ploy_pcd = open3d.geometry.PointCloud()
        bounding_ploy_pcd.points = open3d.utility.Vector3dVector(bounding_ploy)
        bounding_ploy_pcd.rotate(lidar_rotate_matrix, center=[0, 0, 0])
        vol.bounding_polygon = bounding_ploy_pcd.points
        pcd = vol.crop_point_cloud(pcd)

        points = np.asarray(pcd.points)  # 返回numpy.ndarray
        point_intensity = np.asarray(pcd.colors)[:, 0:1]

        # self.get_logger().info('PCD file: ' + pcd_file)

        self.header.stamp = self.get_clock().now().to_msg()

        # 时间戳为数据集帧的id----------------------------------------------------
        # timestamp = float(num_str)
        # time_msg = Time()
        # time_msg.sec = int(timestamp)
        # self.header.stamp = time_msg
        # ---------------------------------------------------------------------

        print(self.header.stamp)

        pc = create_cloud(self.header, points, point_intensity)  # 返回PointCloud2

        # 序列化PointCloud2消息为字节流
        serialize_msg = serialize_message(pc)
        # 计算字节流的长度
        size_bytes = len(serialize_msg)
        timetest = self.get_clock().now().to_msg()

        print("pc delay :", (timetest.sec+timetest.nanosec*1e-9) - (self.header.stamp.sec + self.header.stamp.nanosec *1e-9))
        # frame_info是包含当前帧信息的字典，包括帧id，sec，nanosec
        frame_info = {"id": float(num_str), "sec": timetest.sec, "nanosec": timetest.nanosec, "pc_size_bytes": size_bytes}

        def save_frame_info(frame_info):
            # 打开文件，并以“a”模式追加写入
            with open("frame_info_sender_pcd.txt", 'a') as file:
                frame_id = frame_info["id"]
                sec = frame_info["sec"]
                nanosec = frame_info["nanosec"]
                pc_size_bytes = frame_info["pc_size_bytes"]

                # 将帧信息写入文件，并在每个帧信息后面添加换行符
                file.write(f"Frame ID: {frame_id}, sec: {sec}, nanosec: {nanosec}, pc_size_bytes: {pc_size_bytes}\n")

        save_frame_info(frame_info)

        self.publisher.publish(pc)

        if self.frame_num+2 > self.FRAME_END_NUM:
            self.frame_num = self.FRAME_START_NUM
        else:
            self.frame_num += 2
        self.count += 1


def create_cloud(header, points: np.ndarray, point_intensity: np.ndarray = None):
    """ Creates a point cloud message.
    Args:
        header: PointCloud2 header
        points: Nx3 array of xyz positions.
        point_intensity: point intensity 只有一列的矩阵
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    References:
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
        http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    if point_intensity is not None:
        data = np.concatenate((points, point_intensity), axis=1)
        data = data.astype(dtype).tobytes()
        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate(['x', 'y', 'z', 'intensity'])]
        enum = 4
    else:
        data = points.astype(dtype).tobytes()
        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        enum = 3

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * enum),  # Every point consists of three float32s.
        row_step=(itemsize * enum * points.shape[0]),
        data=data
    )


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


def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PCDPublisher('point_cloud_publisher')
    rclpy.spin(point_cloud_publisher)

    point_cloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
