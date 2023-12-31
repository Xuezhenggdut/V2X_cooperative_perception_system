# V2X_cooperative_perception_system

This is a V2X Cooperative Perception Demo Platform developed based on NVIDIA Jetson AGX Orin and the Datang In-Vehicle On-Board Unit (OBU) platform.

## Network topology of connected devices
The device IP and port configurations are as shown in the figure below. The left side represents the sender, and the right side represents the receiver. For information on how to run the devices, please refer to [[how_to_run]](./document/how_to_run.md).
![network](./document/images/network_topology.png)

## ROS2 graph 
- sender

![sender_rosgraph](./document/images/sender_rosgraph.png)

- receiver

![receiver_rosgraph](./document/images/receiver_rosgraph.png)


## Demo
https://github.com/Xuezhenggdut/V2X_cooperative_perception_system/assets/46556712/ef8c0cf9-fe07-4639-98ef-5732f4cd2ee6

https://github.com/Xuezhenggdut/V2X_cooperative_perception_system/assets/46556712/c9f54fb6-881a-4892-b0d8-0fa109310214

## Reference
- [ROS2 node for TAO-PointPillars](https://github.com/NVIDIA-AI-IOT/ros2_tao_pointpillars)
- [ROS 2 Documentation: Foxy](https://docs.ros.org/en/foxy/index.html)
- [NVIDIA TAO](https://docs.nvidia.com/tao/tao-toolkit/text/point_cloud/pointpillars.html#creating-an-experiment-spec-file)


<!-- See the [video](./video/CP_output_file.mp4).
<!-- mp4格式 -->
<video id="video" controls="" preload="none" poster="封面">
      <source id="mp4" src="./video/CP_output_file.mp4" type="video/mp4">
</videos> -->
