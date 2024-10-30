#pragma once
#include <eigen3/Eigen/Core>
#include "data_type.h"

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using ros_nh = rclcpp::Node::SharedPtr;
using ros_pointcloud2 = sensor_msgs::msg::PointCloud2;
#else
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
using ros_nh = ros::NodeHandle;
using ros_pointcloud2 = sensor_msgs::PointCloud2;
#endif

// 为了减少头文件包含的依赖，.cpp文件里面定义了算法对象，在这个.h文件里面直接封装成函数接口的形式，其实也是调用了里面的对象指针来实现算法的各项工作
void loop_algo_init(const ros_nh nh, const std::string &cfg_path);
void loop_algo_receive_topic_init(); // 初始化loop的话题接收，在使用话题接收的方式传入图像和imu数据的时候使用
void loop_algo_start();
void loop_algo_stop();
void loop_algo_reset();

void loop_add_left_image(const image_type &img_left); // 外部输入左目图像数据接口
void loop_add_feature_point(point_type &point, ros_pointcloud2 &pointcloud2_msg); // 外部输入特征点数据接口
void loop_add_pose(pose_type &pose);            // 外部输入位姿数据接口

void loop_set_extrinsic(const Eigen::Matrix3d ric, const Eigen::Vector3d tic); // 外部输入body到左目的外参的函数（如果body系为左目，则为单位阵）
void loop_set_intrinsic(const intrinsic_type intrinsic);                       // 外部输入相机内参
std::string loop_version();                                                    // 获取loop算法版本
