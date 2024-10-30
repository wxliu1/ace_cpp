#pragma once
#include <cstring>
#include <cstdint>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef struct
{
    bool image_flag; //是不是新的未处理图像数据
    double timestamp; // 图像时间戳
    cv::Mat image;
} image_type;

typedef struct
{
    double timestamp;                          // 图像时间戳
    std::vector<cv::Point2f> points_2d_uv;     // 像素坐标
    std::vector<cv::Point3f> points_3d;        // 3D坐标
} point_type;

typedef struct
{
    double timestamp;               // 图像时间戳
    Eigen::Vector3d position;       // 位置
    Eigen::Quaterniond orientation; // 姿态
} pose_type;

typedef struct
{
    std::string cam_model;           // 相机类型
    double projection_parameters[4]; // fx fy cx cy
    double distortion_parameters[5]; // 根据相机模型自定义，预留5个参数：(radial tangential distortion： p1 p2 k1 k2 k3) /  (ds: xi aplpha)
} intrinsic_type;

struct pose_pack
{
    float px, py, pz, qx, qy, qz, qw;
    // 重载赋值运算符
    pose_pack &operator=(const pose_pack &other) noexcept
    {
        if (this != &other)
        {
            px = other.px;
            py = other.py;
            pz = other.pz;
            qx = other.qx;
            qy = other.qy;
            qz = other.qz;
            qw = other.qw;
        }
        return *this;
    }
};

struct speed_pack
{
    float lx, ly, lz, ax, ay, az;
    // 重载赋值运算符的声明
    speed_pack &operator=(const speed_pack &other) noexcept
    {
        if (this != &other)
        {
            lx = other.lx;
            ly = other.ly;
            lz = other.lz;
            ax = other.ax;
            ay = other.ay;
            az = other.az;
        }
        return *this;
    }
};
