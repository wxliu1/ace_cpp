#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "data_type.h"
#include <cv_bridge/cv_bridge.h>

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "loop_action/action/key_frame_handle.hpp"

using ros_nh = rclcpp::Node::SharedPtr;
using ros_pointcloud2 = sensor_msgs::msg::PointCloud2;
using ros_pointcloud = sensor_msgs::msg::PointCloud;
using ros_PointFields = std::vector<sensor_msgs::msg::PointField>;
using ros_pointfield = sensor_msgs::msg::PointField;
using ros_odom = nav_msgs::msg::Odometry;
using ros_path = nav_msgs::msg::Path;
using ros_image = sensor_msgs::msg::Image;
using ros_pose = geometry_msgs::msg::PoseStamped;

#else
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include "loop_action/KeyFrameHandleAction.h"

using ros_nh = ros::NodeHandle;
using ros_pointcloud2 = sensor_msgs::PointCloud2;
using ros_pointcloud = sensor_msgs::PointCloud;
using ros_PointFields = std::vector<sensor_msgs::PointField>;
using ros_pointfield = sensor_msgs::PointField;
using ros_odom = nav_msgs::Odometry;
using ros_path = nav_msgs::Path;
using ros_image = sensor_msgs::Image;
using ros_pose = geometry_msgs::PoseStamped;

#endif

class ROS_IO
{
public:
    ROS_IO(const ros_nh nh, std::string cfg_path) : nh_(nh), cfg_path_(cfg_path)
    {
#ifdef ROS2
        pub_point_cloud = nh_->create_publisher<ros_pointcloud2>("loop/pointcloud", 10);
        pub_odometry = nh_->create_publisher<ros_odom>("loop/odometry", 10);
        pub_pg_path = nh_->create_publisher<ros_path>("loop/path_odom", 1);
#ifdef __x86_64__
        pub_base_path = nh_->create_publisher<ros_path>("loop/base_path", 1);
        pub_match_img = nh_->create_publisher<ros_image>("loop/match_image", 5);
#endif

        auto handle_goal = [](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const loop_action::action::KeyFrameHandle::Goal> goal)
        {
            // 所有目标都被接受
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };
        auto handle_cancel = [](std::shared_ptr<rclcpp_action::ServerGoalHandle<loop_action::action::KeyFrameHandle>> goal_handle)
        {
            // 所有取消请求都被忽略
            return rclcpp_action::CancelResponse::REJECT;
        };
        auto handle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<loop_action::action::KeyFrameHandle>> goal_handle)
        {
            // 在目标被接受后立即执行你的函数
            std::thread([this, goal_handle]
                        { this->execute(goal_handle); })
                .detach();
        };
        this->KeyFramServer = rclcpp_action::create_server<loop_action::action::KeyFrameHandle>(nh_,
                                                                                                "loop/keyframe_action",
                                                                                                handle_goal,
                                                                                                handle_cancel,
                                                                                                handle_accepted);
#else
        pub_point_cloud = nh_.advertise<ros_pointcloud2>("loop/pointcloud", 10);
        pub_odometry = nh_.advertise<ros_odom>("loop/odometry", 10);
        pub_pg_path = nh_.advertise<ros_path>("loop/path_odom", 1);
#ifdef __x86_64__
        pub_base_path = nh_.advertise<ros_path>("loop/base_path", 1);
        pub_match_img = nh_.advertise<ros_image>("loop/match_image", 5);
#endif

        KeyFramServer = std::make_shared<actionlib::SimpleActionServer<loop_action::KeyFrameHandleAction>>(nh_, "loop/keyframe_action", boost::bind(&ROS_IO::execute, this, _1, KeyFramServer), false);
        KeyFramServer->start();
#endif
    }

    ~ROS_IO() {}

    void ros_topic_receive_init()
    {
#ifdef ROS2
        sub_image_ = nh_->create_subscription<ros_image>("/image_left", rclcpp::QoS(rclcpp::KeepLast(5)), std::bind(&ROS_IO::image_callback, this, std::placeholders::_1));
        sub_pose = nh_->create_subscription<ros_odom>("/keyframe_pose", rclcpp::QoS(rclcpp::KeepLast(5)), std::bind(&ROS_IO::pose_callback, this, std::placeholders::_1));
        sub_point = nh_->create_subscription<ros_pointcloud>("/keyframe_point", rclcpp::QoS(rclcpp::KeepLast(5)), std::bind(&ROS_IO::point_callback, this, std::placeholders::_1));
#else
        sub_image_ = nh_.subscribe("/image_left", 5, &ROS_IO::image_callback, this);
        sub_pose = nh_.subscribe("/keyframe_pose", 5, &ROS_IO::pose_callback, this);
        sub_point = nh_.subscribe("/keyframe_point", 5, &ROS_IO::point_callback, this);
#endif
    }

    void pub_loop_odom(const Eigen::Vector3d &vio_t, const Eigen::Quaterniond &vio_q, const double &time_stamp)
    {
        ros_odom odometry;
        odometry.header.stamp = sec2ros_time(time_stamp);
        odometry.header.frame_id = "odom";
        odometry.pose.pose.position.x = vio_t.x();
        odometry.pose.pose.position.y = vio_t.y();
        odometry.pose.pose.position.z = vio_t.z();
        odometry.pose.pose.orientation.x = vio_q.x();
        odometry.pose.pose.orientation.y = vio_q.y();
        odometry.pose.pose.orientation.z = vio_q.z();
        odometry.pose.pose.orientation.w = vio_q.w();
#ifdef ROS2
        pub_odometry->publish(odometry);
#else
        pub_odometry.publish(odometry);
#endif
    }

    void pub_loop_points(const point_type &point_msg, ros_pointcloud2 &cloud_msg)
    {
        cloud_msg.header.stamp = sec2ros_time(point_msg.timestamp);
        cloud_msg.header.frame_id = "odom";
        cloud_msg.point_step = 12;
        cloud_msg.fields = MakePointFields("xyz");
        cloud_msg.height = 1;
        cloud_msg.width = point_msg.points_3d.size();
        cloud_msg.data.resize(cloud_msg.width * cloud_msg.point_step);
        int i = 0;
        for (const auto& point : point_msg.points_3d) {
            auto* ptr = reinterpret_cast<float*>(cloud_msg.data.data() + i * cloud_msg.point_step);
            ptr[0] = point.x;
            ptr[1] = point.y;
            ptr[2] = point.z;
            i++;
        }
#ifdef ROS2
        pub_point_cloud->publish(cloud_msg);
#else
        pub_point_cloud.publish(cloud_msg);
#endif
    }

    void addBasePath(const double &time_stamp, const Eigen::Vector3d &P, const Eigen::Quaterniond &Q)
    {
        ros_pose pose_stamped;
        pose_stamped.header.stamp = sec2ros_time(time_stamp);
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = P.x();
        pose_stamped.pose.position.y = P.y();
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        base_path.poses.push_back(pose_stamped);
        base_path.header = pose_stamped.header;
    }

    void push_loop_path(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const double &time_stamp, const int &sequence)
    {
        ros_pose pose_stamped;
        pose_stamped.header.stamp = sec2ros_time(time_stamp);
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = P.x();
        pose_stamped.pose.position.y = P.y();
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        path[sequence].poses.push_back(pose_stamped);
        path[sequence].header = pose_stamped.header;
    }

    void pub_loop_path(const int &sequence)
    {
        for (int i = 1; i <= sequence; i++)
        {
#ifdef ROS2
            if (pub_pg_path->get_subscription_count())
            {
                pub_pg_path->publish(path[i]);
            }
#else
            if (pub_pg_path.getNumSubscribers())
            {
                pub_pg_path.publish(path[i]);
            }
#endif
        }

#ifdef __x86_64__

#ifdef ROS2
        if (pub_base_path->get_subscription_count())
        {
            pub_base_path->publish(base_path);
        }
#else
        if (pub_base_path.getNumSubscribers())
        {
            pub_base_path.publish(base_path);
        }
#endif

#endif
    }

    void pub_match_image(const cv::Mat &loop_match_img, double &time_stamp)
    {
        ros_image image_msg;
        image_msg.header.stamp = sec2ros_time(time_stamp);
        image_msg.header.frame_id = "odom";
        image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        image_msg.height = loop_match_img.rows;
        image_msg.width = loop_match_img.cols;
        image_msg.step = loop_match_img.cols * 3;
        image_msg.data.resize(image_msg.height * image_msg.step);
        memcpy(image_msg.data.data(), loop_match_img.data, image_msg.data.size());
        image_msg.is_bigendian = 0;
        
#ifdef __x86_64__

#ifdef ROS2
        pub_match_img->publish(image_msg);
#else
        pub_match_img.publish(image_msg);
#endif

#endif
    }

#ifdef ROS2
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
#else
    void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
#endif
    {
        image_type image_data;
        image_data.timestamp = ros_time2sec(image_msg->header.stamp);
        image_data.image = cv_bridge::toCvShare(image_msg)->image;

        if (add_image_)
        {
            add_image_(image_data);
        }
    }

#ifdef ROS2
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
#else
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
#endif
    {
        pose_type pose_data;
        pose_data.timestamp = ros_time2sec(pose_msg->header.stamp);
        pose_data.position = Eigen::Vector3d(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
        pose_data.orientation = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
        if (add_pose_)
        {
            add_pose_(pose_data);
        }
    }

#ifdef ROS2
    void point_callback(const sensor_msgs::msg::PointCloud::SharedPtr point_msg)
#else
    void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
#endif
    {
        point_type point_data;
        point_data.timestamp = ros_time2sec(point_msg->header.stamp);
        for (unsigned int i = 0; i < point_msg->points.size(); i++)
        {
            cv::Point3f p_3d;
            p_3d.x = point_msg->points[i].x;
            p_3d.y = point_msg->points[i].y;
            p_3d.z = point_msg->points[i].z;
            point_data.points_3d.push_back(p_3d);

            cv::Point2f p_2d_uv;
            p_2d_uv.x = point_msg->channels[i].values[2];
            p_2d_uv.y = point_msg->channels[i].values[3];
            point_data.points_2d_uv.push_back(p_2d_uv);
        }
        if (add_point_)
        {
            ros_pointcloud2 cloud_msg;
            add_point_(point_data, cloud_msg);
        }
    }

#ifdef ROS2
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<loop_action::action::KeyFrameHandle>> goal_handle)
    {
        rclcpp::Rate rate(5); // 200ms => 5Hz
        auto result = std::make_shared<loop_action::action::KeyFrameHandle::Result>();
        auto feedback = std::make_shared<loop_action::action::KeyFrameHandle::Feedback>();

        if (goal_handle->get_goal()->function == 2)
        {
            save_keyframe = true;
            while (!save_keyframe_done)
            {
                feedback->saving = 1;
                goal_handle->publish_feedback(feedback);
                rate.sleep();
            }
            save_keyframe_done = false;
            result->keyframe_num = keyframe_num;
            goal_handle->succeed(result);
        }
        // else
        // {
        //     loop.add_loopframe = true;
        //     // loop.keyframe_num++;
        //     std::cout << "keyframe num: " << loop.keyframe_num << std::endl;
        //     while (!loop.add_keyframe_done)
        //     {
        //         feedback->saving = 1;
        //         goal_handle->publish_feedback(feedback);
        //         rate.sleep();
        //     }
        //     loop.add_keyframe_done = false;
        //     result->keyframe_num = loop.keyframe_num;
        //     goal_handle->succeed(result);
        // }
    }
#else
    void execute(const loop_action::KeyFrameHandleGoalConstPtr &goal, std::shared_ptr<actionlib::SimpleActionServer<loop_action::KeyFrameHandleAction>> as)
    {
        ros::Rate r(5);
        loop_action::KeyFrameHandleFeedback feedback;
        loop_action::KeyFrameHandleResult result;
        if (goal->function == 2)
        {
            save_keyframe = true;
            while (!save_keyframe_done)
            {
                feedback.saving = 1;
                KeyFramServer->publishFeedback(feedback);
                r.sleep();
            }
            save_keyframe_done = false;
            result.keyframe_num = keyframe_num;
            KeyFramServer->setSucceeded(result);
        }
        // else
        // {
        //     loop.add_loopframe = true;
        //     // loop.keyframe_num++;
        //     ROS_INFO("keyframe num: %d", loop.keyframe_num);
        //     while (!loop.add_keyframe_done)
        //     {
        //         feedback.saving = 1;
        //         ptr_KF_server->publishFeedback(feedback);
        //         r.sleep();
        //     }
        //     loop.add_keyframe_done = false;
        //     result.keyframe_num = loop.keyframe_num;
        //     ptr_KF_server->setSucceeded(result);
        // }
    }
#endif

    bool save_keyframe{false};
    bool save_keyframe_done{false};
    unsigned int keyframe_num{0};
    std::string cfg_path_;
    ros_path path[10];
    ros_path base_path;

    std::function<void(image_type &)> add_image_;
    std::function<void(pose_type &)> add_pose_;
    std::function<void(point_type &, ros_pointcloud2 &)> add_point_;

private:
    int GetPointFieldDataTypeBytes(uint8_t dtype) noexcept {
        // INT8 = 1u,
        // UINT8 = 2u,
        // INT16 = 3u,
        // UINT16 = 4u,
        // INT32 = 5u,
        // UINT32 = 6u,
        // FLOAT32 = 7u,
        // FLOAT64 = 8u,
        switch (dtype) {
            case 0U:
            return 0;
            case 1U:  // int8
            case 2U:  // uint8
            return 1;
            case 3U:  // int16
            case 4U:  // uint16
            return 2;
            case 5U:  // int32
            case 6U:  // uint32
            case 7U:  // float32
            return 4;
            case 8U:  // float64
            return 8;
            default:
            return 0;
        }
    }

    inline ros_PointFields MakePointFields(const std::string& fstr) {
        ros_PointFields fields;
        fields.reserve(fstr.size());

        int offset{0};
        ros_pointfield field;

        for (auto s : fstr) {
            s = std::tolower(s);
            if (s == 'x' || s == 'y' || s == 'z') {
                field.name = s;
                field.offset = offset;
                field.datatype = ros_pointfield::FLOAT32;
                field.count = 1;
            } else if (s == 'i') {
                field.name = "intensity";
                field.offset = offset;
                field.datatype = ros_pointfield::FLOAT32;
                field.count = 1;
            } else continue;
            
            // update offset
            offset += GetPointFieldDataTypeBytes(field.datatype) * field.count;
            fields.push_back(field);
        }
        return fields;
    }
#ifdef ROS2
    double ros_time2sec(builtin_interfaces::msg::Time t)
    {
        return t.sec + t.nanosec * 1e-9;
    }

    builtin_interfaces::msg::Time sec2ros_time(const double &t)
    {
        return rclcpp::Time(t);
    }


    rclcpp::Publisher<ros_pointcloud2>::SharedPtr pub_point_cloud;
    rclcpp::Publisher<ros_odom>::SharedPtr pub_odometry;
    rclcpp::Publisher<ros_path>::SharedPtr pub_pg_path;
    rclcpp::Publisher<ros_path>::SharedPtr pub_base_path;
    rclcpp::Publisher<ros_image>::SharedPtr pub_match_img;

    rclcpp::Subscription<ros_image>::SharedPtr sub_image_;
    rclcpp::Subscription<ros_odom>::SharedPtr sub_vio;
    rclcpp::Subscription<ros_odom>::SharedPtr sub_pose;
    rclcpp::Subscription<ros_pointcloud>::SharedPtr sub_point;

    rclcpp_action::Server<loop_action::action::KeyFrameHandle>::SharedPtr KeyFramServer;

#else
    double ros_time2sec(ros::Time t)
    {
        return t.toSec();
    }

    ros::Time sec2ros_time(const double &t)
    {
        return ros::Time(t);
    }

    ros::Publisher pub_point_cloud;
    ros::Publisher pub_odometry;
    ros::Publisher pub_pg_path;
    ros::Publisher pub_base_path;
    ros::Publisher pub_match_img;

    ros::Subscriber sub_image_;
    ros::Subscriber sub_vio;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_point;

    std::shared_ptr<actionlib::SimpleActionServer<loop_action::KeyFrameHandleAction>> KeyFramServer;
#endif

    ros_nh nh_;
};
