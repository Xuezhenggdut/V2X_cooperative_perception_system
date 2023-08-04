/*
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#define BOOST_BIND_NO_PLACEHOLDERS

#include <chrono>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <map>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <unistd.h>
#include <string>
#include <fstream>

#include "cuda_runtime.h"
#include "../include/pp_infer/pointpillar.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    this->declare_parameter("class_names");
    this->declare_parameter<float>("nms_iou_thresh", 0.01);
    this->declare_parameter<int>("pre_nms_top_n", 4096);
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::string>("engine_path", "");
    this->declare_parameter<std::string>("data_type", "fp16");
    this->declare_parameter<float>("intensity_scale", 1.0);
    

    rclcpp::Parameter class_names_param = this->get_parameter("class_names");
    class_names = class_names_param.as_string_array();
    nms_iou_thresh = this->get_parameter("nms_iou_thresh").as_double();
    pre_nms_top_n = this->get_parameter("pre_nms_top_n").as_int();
    model_path = this->get_parameter("model_path").as_string();
    engine_path = this->get_parameter("engine_path").as_string();
    data_type = this->get_parameter("data_type").as_string();
    intensity_scale = this->get_parameter("intensity_scale").as_double();
    
    cudaStream_t stream = NULL;
    pointpillar = new PointPillar(model_path, engine_path, stream, data_type);

    publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("bbox1", 700);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/point_cloud1", 700, std::bind(&MinimalPublisher::topic_callback, this, _1));

  }

private:
  std::vector<std::string> class_names;
  float nms_iou_thresh;
  int pre_nms_top_n;
  bool do_profile{false};
  std::string model_path;
  std::string engine_path;
  std::string data_type;
  float intensity_scale;
  tf2::Quaternion myQuaternion;
  cudaStream_t stream = NULL;
  PointPillar* pointpillar;  


  void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  { 
    assert(data_type == "fp32" || data_type == "fp16");
    cudaEvent_t start, stop;
    float elapsedTime = 0.0f;
    cudaStream_t stream = NULL;

    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));

    std::vector<Bndbox> nms_pred;
    nms_pred.reserve(100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

      unsigned int num_point_values = pcl_cloud->size();

      unsigned int points_size = pcl_cloud->points.size();

      std::vector<float> pcl_data;

      for (const auto& point : pcl_cloud->points) {
        pcl_data.push_back(point.x);
        pcl_data.push_back(point.y);
        pcl_data.push_back(point.z);
        pcl_data.push_back(point.intensity/intensity_scale);
      }

      float* points = static_cast<float *>(pcl_data.data());
      
      
      //Use 4 because PCL has padding (4th value now has intensity information)
      unsigned int points_data_size = points_size * sizeof(float) * 4;


      float *points_data = nullptr;
      unsigned int *points_num = nullptr;
      //unsigned int points_data_size = points_size * num_point_values * sizeof(float);
      checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
      checkCudaErrors(cudaMallocManaged((void **)&points_num, sizeof(unsigned int)));
      checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
      checkCudaErrors(cudaMemcpy(points_num, &points_size, sizeof(unsigned int), cudaMemcpyDefault));
      checkCudaErrors(cudaDeviceSynchronize());

      cudaEventRecord(start, stream);

      //获取实际时间，作为CUDA时间的开始时间
      auto ros_start_time = rclcpp::Clock().now();
      
      pointpillar->doinfer(
        points_data, points_num, nms_pred,
        nms_iou_thresh,
        pre_nms_top_n,
        class_names,
        do_profile
      );

      auto pc_detection_arr = std::make_shared<vision_msgs::msg::Detection3DArray>();
      std::vector<vision_msgs::msg::Detection3D> detections;
      for(int i=0; i<nms_pred.size(); i++) {
        vision_msgs::msg::Detection3D detection;
        detection.results.resize(1); 
        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        vision_msgs::msg::BoundingBox3D bbox;
        geometry_msgs::msg::Pose center;
        geometry_msgs::msg::Vector3 size;
        geometry_msgs::msg::Point position;	
	      geometry_msgs::msg::Quaternion orientation;
        
        detection.bbox.center.position.x = nms_pred[i].x;
        detection.bbox.center.position.y = nms_pred[i].y;
        detection.bbox.center.position.z = nms_pred[i].z;
        detection.bbox.size.x = nms_pred[i].l;
        detection.bbox.size.y = nms_pred[i].w;
        detection.bbox.size.z = nms_pred[i].h;

        myQuaternion.setRPY(0, 0, nms_pred[i].rt);
        orientation = tf2::toMsg(myQuaternion);

        detection.bbox.center.orientation = orientation;

        hyp.id = std::to_string(nms_pred[i].id);
        hyp.score = nms_pred[i].score;
        
        detection.header = msg->header;
        
        detection.results[0] = hyp;
        detections.push_back(detection);
      }

      pc_detection_arr->header = msg->header;
      pc_detection_arr->detections = detections;
      publisher_->publish(*pc_detection_arr);

      cudaEventRecord(stop, stream);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);

      // 获取实际时间，作为CUDA时间的结束时间
      auto ros_stop_time = rclcpp::Clock().now();

      // 将ROS时间转换为秒数和纳秒数
//      auto ros_start_time_sec = ros_start_time.seconds();
      auto ros_start_time_nsec = ros_start_time.nanoseconds();
//      auto ros_stop_time_sec = ros_stop_time.seconds();
      auto ros_stop_time_nsec = ros_stop_time.nanoseconds();

      // 换算为秒
      double ros_start_time_nsec_ = ros_start_time_nsec / 1e9;
      double ros_stop_time_nsec_ = ros_stop_time_nsec / 1e9;

      // 保存到文件中
      std::ofstream file;
      file.open("frame_info_detection.txt", std::ios_base::app);
      file << std::fixed << std::setprecision(9) << "detection_start_time_nsec_:" << ros_start_time_nsec_ << ", detection_stop_time:" << ros_stop_time_nsec_ << std::endl;
      file.close();

      auto message = std_msgs::msg::String();
      message.data = "TIME: " + std::to_string(elapsedTime) + " ms, Objects detected: " + std::to_string(nms_pred.size());
      RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());

      checkCudaErrors(cudaFree(points_data));
      checkCudaErrors(cudaFree(points_num));
      nms_pred.clear();
    

  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));

  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
