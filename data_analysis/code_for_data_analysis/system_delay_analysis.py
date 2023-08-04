import os
import matplotlib.pyplot as plt
import numpy as np


# 获取当前文件所在的文件夹路径
current_folder = os.path.dirname(__file__)

# 获取上一级目录的路径
parent_folder = os.path.abspath(os.path.join(current_folder, os.pardir))

# 构建文件路径
folder_name = '20230801_test11_10_50'
file_path1 = os.path.join(parent_folder, folder_name, 'sender', 'frame_info_sender_pcd.txt')
file_path2 = os.path.join(parent_folder, folder_name, 'sender', 'frame_info_detection.txt')
file_path3 = os.path.join(parent_folder, folder_name, 'sender', 'frame_info_sender_bbox.txt')
file_path4 = os.path.join(parent_folder, folder_name, 'receiver', 'frame_info_listener.txt')
file_path5 = os.path.join(parent_folder, folder_name, 'receiver', 'frame_info_fusion.txt')
file_path6 = os.path.join(parent_folder, folder_name, 'sender', 'frame_info_bbox_visualize.txt')
file_path7 = os.path.join(parent_folder, folder_name, 'receiver', 'frame_info_detection.txt')
file_path8 = os.path.join(parent_folder, folder_name, 'receiver', 'frame_info_bbox_visualize.txt')

# 定义一个列表用于存储所有的帧信息
frame_info_list_pcd = []
frame_info_list_detection = []
frame_info_list_bbox = []
frame_info_list_listener = []
frame_info_list_fusion = []
frame_info_list_sender_bbox_visualize = []
frame_info_list_receiver_detection = []
frame_info_list_receiver_bbox_visualize = []

# 打开文件并逐行读取数据
with open(file_path1, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_pcd.append(frame_info)

# 打开文件并逐行读取数据
with open(file_path2, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_detection.append(frame_info)


# 打开文件并逐行读取数据
with open(file_path3, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_bbox.append(frame_info)

# 打开文件并逐行读取数据
with open(file_path4, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_listener.append(frame_info)


# 打开文件并逐行读取数据
with open(file_path5, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_fusion.append(frame_info)


# 打开文件并逐行读取数据
with open(file_path6, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_sender_bbox_visualize.append(frame_info)


# 打开文件并逐行读取数据
with open(file_path7, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_receiver_detection.append(frame_info)

# 打开文件并逐行读取数据
with open(file_path8, 'r') as file:
    for line in file:
        # 分割每行数据，并存储到字典中
        parts = line.strip().split(', ')
        frame_info = {}
        for part in parts:
            if ': ' in part:
                key, value = part.split(': ')
            else:
                key, value = part.split(':')
            frame_info[key.strip()] = value.strip()

        # 将每个字典添加到列表中
        frame_info_list_receiver_bbox_visualize.append(frame_info)


#-------------------数据包大小----------------------------------
# 将字典 frame_info_list_pcd 转换成两个列表，分别存储帧ID和数据包大小
frame_ids_pcd = [float(info['Frame ID']) for info in frame_info_list_pcd]
packet_sizes_pcd = [int(info['pc_size_bytes'])* 1e-6 for info in frame_info_list_pcd]

# 将字典 frame_info_list_bbox 转换成两个列表，分别存储帧ID和数据包大小
frame_ids_bbox = [float(info['Frame ID']) for info in frame_info_list_bbox]
packet_sizes_bbox = [int(info['bbox_size_bytes']) for info in frame_info_list_bbox]

# 计算平均数据包大小
average_packet_size_pcd = sum(packet_sizes_pcd) / len(packet_sizes_pcd)
average_packet_size_bbox = sum(packet_sizes_bbox) / len(packet_sizes_bbox)

# 绘制散点图
bbox_values = list(range(1, len(packet_sizes_bbox) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, packet_sizes_bbox, marker='x', label='bbox_packet_size')
plt.axhline(average_packet_size_bbox, color='green', linestyle='dashed', label='average_bbox_size')
plt.xlabel('Frame ID')
plt.ylabel('Packet size (Bytes)')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')
# plt.minorticks_on()
plt.legend()  # 显示图例

pcd_values = list(range(1, len(packet_sizes_pcd) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(pcd_values, packet_sizes_pcd, marker='*', label='pcd_data_size')
plt.axhline(average_packet_size_pcd, color='blue', linestyle='dashed', label='average_pcd_size')
plt.xlabel('Frame ID')
plt.ylabel('Packet size (MB)')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')
# plt.minorticks_on()
plt.legend()  # 显示图例

 

#-------------------发送端点云到推理模块传输、检测、可视化时间-------------------------
# 创建一个字典，用于存储每个帧的点云到检测框的检测时间
pointcloud_to_bbox_detection_time = {}
ros2_pcd_time_sec = {}
bbox_vis_time_sec = {}
ros2_bbox_client_time_sec = {}
ros2_sender_bbox_vis_time_sec = {}

# 遍历 frame_info_list_pcd 中的每个帧信息
for index, pcd_info in enumerate(frame_info_list_pcd):
    frame_id = float(pcd_info['Frame ID'])
    pcd_time_sec = int(pcd_info['sec'])
    pcd_time_nanosec = int(pcd_info['nanosec'])

    # 获取 frame_info_list_bbox 对应位置的元素
    bbox_info = frame_info_list_detection[index]
    bbox_info_vis = frame_info_list_sender_bbox_visualize[index]

    bbox_time_strat_sec = float(bbox_info['detection_start_time'])
    bbox_time_stop_sec = float(bbox_info['detection_stop_time'])

    bbox_tran_info = frame_info_list_bbox[index]
    timestart_sender_bbox = float(bbox_tran_info['start_sec']) 

    bbox_vis_time_strat_sec = float(bbox_info_vis['start_sec'])
    bbox_vis_time_stop_sec = float(bbox_info_vis['stop_sec'])

    # 计算点云到推理模块传输时间（单位：秒）
    ros2_pcd_time_sec[index] = bbox_time_strat_sec - ( pcd_time_sec + pcd_time_nanosec * 1e-9)

    # 计算检测框到Client传输时间
    ros2_bbox_client_time_sec[index] = timestart_sender_bbox - bbox_time_stop_sec

    # 计算检测框到可视化模块通信时间
    ros2_sender_bbox_vis_time_sec[index] = bbox_vis_time_strat_sec - bbox_time_stop_sec

    # 计算推理时间（单位：秒）
    detection_time_sec = bbox_time_stop_sec - bbox_time_strat_sec
    pointcloud_to_bbox_detection_time[index] = detection_time_sec
    bbox_vis_time_sec[index] = bbox_vis_time_stop_sec - bbox_vis_time_strat_sec

# 输出每个帧的点云到检测框的检测时间
for frame_id, detection_time_sec in pointcloud_to_bbox_detection_time.items():
    print(f'帧 {frame_id} 的点云发布到推理模块通信时间为：{ros2_pcd_time_sec[frame_id]} 秒')
    print(f'帧 {frame_id} 的点云到检测框的检测时间为：{detection_time_sec} 秒')
    print(f'帧 {frame_id} 的点云和检测框可视化时间为：{bbox_vis_time_stop_sec - bbox_vis_time_strat_sec} 秒')


# 将字典 pointcloud_to_bbox_detection_time 转换成两个列表，分别存储帧ID和检测时间
frame_ids = list(pointcloud_to_bbox_detection_time.keys())
detection_times_ms = [time * 1000 for time in pointcloud_to_bbox_detection_time.values()]
ros2_pcd_times_ms = [time * 1000 for time in ros2_pcd_time_sec.values()]
ros2_bbox_client_time_ms = [time * 1000 for time in ros2_bbox_client_time_sec.values()]
bbox_vis_times_ms = [time * 1000 for time in bbox_vis_time_sec.values()]
ros2_sender_bbox_vis_time_ms = [time * 1000 for time in ros2_sender_bbox_vis_time_sec.values()]

# 计算平均检测时延
average_detection_time_ms = sum(detection_times_ms) / len(detection_times_ms)

average_ros2_pcd_times_ms = sum(ros2_pcd_times_ms) / len(ros2_pcd_times_ms)

average_ros2_bbox_client_time_ms = sum(ros2_bbox_client_time_ms) / len(ros2_bbox_client_time_ms)

average_bbox_vis_times_ms = sum(bbox_vis_times_ms) / len(bbox_vis_times_ms)

average_ros2_sender_bbox_vis_time_ms = sum(ros2_sender_bbox_vis_time_ms) / len(ros2_sender_bbox_vis_time_ms)

# 绘制图表
bbox_values = list(range(1, len(detection_times_ms) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, detection_times_ms, marker='*')
plt.axhline(y=average_detection_time_ms, color='r', linestyle='--', label='average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Detection delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')
# plt.minorticks_on()

# 绘制图表
bbox_values = list(range(1, len(ros2_pcd_times_ms) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, ros2_pcd_times_ms, marker='*')
plt.axhline(y=average_ros2_pcd_times_ms, color='r', linestyle='--', label='average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('RoS2 pcd trans delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')

# 绘制图表
bbox_values = list(range(1, len(bbox_vis_times_ms) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, bbox_vis_times_ms, marker='*')
plt.axhline(y=average_bbox_vis_times_ms, color='r', linestyle='--', label='average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Sender bbox visualize delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')


#-------------------接收端推理模块检测时间-------------------------
# 创建一个字典，用于存储每个帧的点云到检测框的检测时间
receiver_detection_time = {}

# 遍历 frame_info_list_receiver_detection 中的每个帧信息
for index, bbox_info in enumerate(frame_info_list_receiver_detection):
    bbox_time_strat_sec = float(bbox_info['detection_start_time_nsec_'])
    bbox_time_stop_sec = float(bbox_info['detection_stop_time'])

    # 计算推理时间（单位：秒）
    detection_time_sec = bbox_time_stop_sec - bbox_time_strat_sec
    receiver_detection_time[index] = detection_time_sec


# 检测时间
receiver_detection_times_ms = [time * 1000 for time in receiver_detection_time.values()]


# 计算平均检测时延
receiver_average_detection_time_ms = sum(receiver_detection_times_ms) / len(receiver_detection_times_ms)


# 绘制图表
bbox_values = list(range(1, len(receiver_detection_times_ms) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, receiver_detection_times_ms, marker='*')
plt.axhline(y=receiver_average_detection_time_ms, color='r', linestyle='--', label='average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Receiver Detection delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')
# plt.minorticks_on()

#-----------------------------------传输时延统计-------------------------------------
# 定义一个函数来计算时间戳
def calculate_timestamp(sec, nanosec):
    return sec + nanosec * 1e-9

# 获取接收端每一帧数据的时间戳和帧ID
start_time_listener = [float(info['start_sec']) for info in frame_info_list_listener]
stop_time_listener = [float(info['stop_sec']) for info in frame_info_list_listener]
frame_ids_listener = [float(info['Frame ID']) for info in frame_info_list_listener]

# 获取发送端数据的时间戳和帧ID
timestamps_bbox = [float(info['stop_sec']) for info in frame_info_list_bbox]
timestart_sender_bbox = [float(info['start_sec']) for info in frame_info_list_bbox]
frame_ids_bbox = [float(info['Frame ID']) for info in frame_info_list_bbox]

delay_trans = []
delay_client = [(timestamps_bbox[i] - timestart_sender_bbox[i])*1000 for i in range(len(timestamps_bbox))]
delay_listener = [(stop_time_listener[i] - start_time_listener[i])*1000 for i in range(len(stop_time_listener))]

for i in range(len(timestamps_bbox)):
    if i < len(stop_time_listener):
        if frame_ids_bbox[i] == frame_ids_listener[i]:
            j = i
            delay_ms = (stop_time_listener[j] - timestamps_bbox[i]) * 1000
        elif j == len(stop_time_listener):
            break
        elif frame_ids_bbox[i] == frame_ids_listener[j]:
            delay_ms = (stop_time_listener[j] - timestamps_bbox[i]) * 1000
            j += 1
        else:
            j += 1
            while True:
                i += 1
                if j < len(stop_time_listener):
                    if frame_ids_bbox[i] == frame_ids_listener[j]:
                        delay_ms = (stop_time_listener[j] - timestamps_bbox[i]) * 1000
                        # j += 1
                        break
                else:
                    break
        delay_trans.append(delay_ms)
    else:
        break

# 计算平均检测时延
average_trans_time_ms = sum(delay_trans) / len(delay_trans)
av_delay_client_ms = sum(delay_client) / len(delay_client)
av_delay_listener_ms = sum(delay_listener) / len(delay_listener)

# 绘制图表
lis_bbox_values = list(range(1, len(delay_trans) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(lis_bbox_values, delay_trans, marker='*')
plt.axhline(y=average_trans_time_ms, color='r', linestyle='--', label='average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Transmission delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')
# plt.minorticks_on()

# 绘制图表
lis_bbox_values = list(range(1, len(delay_client) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(lis_bbox_values, delay_client, marker='*')
plt.axhline(y=av_delay_client_ms, color='r', linestyle='--', label='average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Client processing delay (ms)')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')


# 绘制图表
lis_bbox_values = list(range(1, len(delay_listener) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(lis_bbox_values, delay_listener, marker='*')
plt.axhline(y=av_delay_listener_ms, color='r', linestyle='--', label='average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Listener processing delay (ms)')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')



#--------------------接收到融合时延统计-------------------------------

# 创建一个字典，用于存储每个帧的点云到检测框的检测时间
ros_listener_to_fusion_time = {}
fusion_times = {}
receiver_bbox_vis_times = {}
receiver_ros2_bbox_vis_time = {}

# 遍历 frame_info_list_listener 中的每个帧信息
for index, fusion_info in enumerate(frame_info_list_fusion):
    frame_id = float(fusion_info['Frame ID'])
    fusion_start_time_sec = float(fusion_info['start_sec'])
    fusion_stop_time_sec = float(fusion_info['stop_sec'])

    # 获取 frame_info_list_fusion 对应位置的元素
    rec_bbox_info = frame_info_list_listener[index]

    rec_bbox_stop_time_sec = float(rec_bbox_info['stop_sec'])

    # 计算RoS接收到融合通信时间（单位：秒）
    ros_listener_to_fusion_time[index] = fusion_start_time_sec - rec_bbox_stop_time_sec

    # 计算融合显示时间
    fusion_times[index] = fusion_stop_time_sec - fusion_start_time_sec

for index, rec_vis_info in enumerate(frame_info_list_receiver_bbox_visualize): 
    receiver_bbox_vis_start_time_sec = float(rec_vis_info['start_sec'])
    receiver_bbox_vis_stop_time_sec = float(rec_vis_info['stop_sec'])
    receiver_bbox_vis_times[index] = receiver_bbox_vis_stop_time_sec - receiver_bbox_vis_start_time_sec

    rec_bbox_info = frame_info_list_receiver_detection[index]
    rec_bbox_stop_time_sec = float(rec_bbox_info['detection_stop_time'])

    # 计算检测到可视化ROS2通信时间
    receiver_ros2_bbox_vis_time[index] = receiver_bbox_vis_start_time_sec - rec_bbox_stop_time_sec


# 输出每个帧的点云到检测框的检测时间
for frame_id, fusion_time in fusion_times.items():
    print(f'帧 {frame_id} 的接收到融合节点之间的传输时间为：{ros_listener_to_fusion_time[frame_id]} 秒')
    print(f'帧 {frame_id} 的融合时间为：{fusion_time} 秒')


# 将字典 listener_to_fusion_time 转换成两个列表，分别存储帧ID和检测时间
frame_ids = list(fusion_times.keys())
fusion_times_ms = [time * 1000 for time in fusion_times.values()]

ros_listener_to_fusion_ms = [time * 1000 for time in ros_listener_to_fusion_time.values()]

receiver_bbox_vis_ms = [time * 1000 for time in receiver_bbox_vis_times.values()]

receiver_ros2_bbox_vis_ms = [time * 1000 for time in receiver_ros2_bbox_vis_time.values()]

# 计算平均检测时延
average_fusion_time_ms = sum(fusion_times_ms) / len(fusion_times_ms)
average_ros_listener_to_fusion_time = sum(ros_listener_to_fusion_ms) / len(ros_listener_to_fusion_ms)
average_receiver_bbox_vis_ms = sum(receiver_bbox_vis_ms) / len(receiver_bbox_vis_ms)
average_receiver_ros2_bbox_vis_ms = sum(receiver_ros2_bbox_vis_ms) / len(receiver_ros2_bbox_vis_ms)

# 绘制图表
bbox_values = list(range(1, len(fusion_times_ms) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, fusion_times_ms, marker='*')
plt.axhline(y=average_fusion_time_ms, color='r', linestyle='--', label='Average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Fusion delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')
# plt.minorticks_on()

# 绘制图表
bbox_values = list(range(1, len(ros_listener_to_fusion_ms) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, ros_listener_to_fusion_ms, marker='*')
plt.axhline(y=average_ros_listener_to_fusion_time, color='r', linestyle='--', label='Average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('ros_listener_to_fusion delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')

# 绘制图表
bbox_values = list(range(1, len(receiver_bbox_vis_ms) + 1))
plt.figure()  # 可以调整图表大小
plt.scatter(bbox_values, receiver_bbox_vis_ms, marker='*')
plt.axhline(y=average_receiver_bbox_vis_ms, color='r', linestyle='--', label='Average')  # 添加平均时延的水平线
plt.xlabel('Frame ID')
plt.ylabel('Receiver bbox visualize delay (ms)')
# plt.title('每个包的检测时间')
plt.grid(axis='both', which='major', linestyle='-')
plt.grid(axis='y', which='minor', linestyle=':')
# plt.show()

drop_ratio = (len(frame_info_list_bbox) - len(frame_info_list_listener) )/ len(frame_info_list_bbox)
print('-----------------------------------------------------------')
print('总发送包数量:', len(frame_info_list_bbox))
print('丢包率:', drop_ratio)
print('点云数据平均大小(MBytes):', average_packet_size_pcd)
print('检测框数据平均大小(Bytes):', average_packet_size_bbox)
print('ROS2:点云发布到推理通信时间:', average_ros2_pcd_times_ms)
print('推理模块检测时间:', average_detection_time_ms)
print('ROS2:检测框发布到Client通信时间:', average_ros2_bbox_client_time_ms)
print('Client 处理时延:',av_delay_client_ms)
print('平均传输时延(上位机到OBU到OBU到上位机,ms):', average_trans_time_ms)
print('Listener处理时延', av_delay_listener_ms)
print('ROS2:侦听到融合通信时间:',average_ros_listener_to_fusion_time)
print('平均融合时延(ms):', average_fusion_time_ms)
print('系统总平均时延:', average_ros2_pcd_times_ms+average_detection_time_ms+average_ros2_bbox_client_time_ms+av_delay_client_ms+average_trans_time_ms+av_delay_listener_ms+average_ros_listener_to_fusion_time+average_fusion_time_ms)

print('接收端检测时延:',receiver_average_detection_time_ms)
print('ROS2:接收端检测到可视化通信时间:', average_receiver_ros2_bbox_vis_ms)
print('接收端可视化时延:', average_receiver_bbox_vis_ms)
print('发送端可视化时间:', average_bbox_vis_times_ms)
print('ROS2:发送端检测到可视化通信时间:', average_ros2_sender_bbox_vis_time_ms)

# 输出所有帧信息
# for frame_info in frame_info_list_pcd:
#     print(frame_info)