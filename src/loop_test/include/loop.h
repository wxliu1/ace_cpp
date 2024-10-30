#pragma once
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <thread>
#include "keyframe.h"
#include "data_type.h"
#include <mutex>
#include <queue>

#define VERSION 20241019
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define LOOP_VER_STR TOSTRING(VERSION)

template <typename T>
inline void QuaternionInverse(const T q[4], T q_inverse[4])
{
    q_inverse[0] = q[0];
    q_inverse[1] = -q[1];
    q_inverse[2] = -q[2];
    q_inverse[3] = -q[3];
};

template <typename T>
T NormalizeAngle(const T &angle_degrees)
{
    if (angle_degrees > T(180.0))
        return angle_degrees - T(360.0);
    else if (angle_degrees < T(-180.0))
        return angle_degrees + T(360.0);
    else
        return angle_degrees;
};

class LoopClosure
{
public:
    LoopClosure(const std::string &cfg_path);
    ~LoopClosure();

    void ros_topic_receive_init();
    void Start();
    void Stop();
    void Reset();
    void add_left_image(const image_type img_left);
    void add_pose(pose_type &pose);
    void set_extrinsic(const Eigen::Matrix3d ric, const Eigen::Vector3d tic);
    void set_intrinsic(const intrinsic_type intrinsic);
    std::string loop_version() { return LOOP_VER_STR; }

private:
    void InitPoseGraph();
    void ReadParameters();
    void resetVocabulary();
    void loadVocabulary(std::string voc_path);
    void addKeyFrame(KeyFrame *cur_kf, bool add_brief);
    void updatePath();
    void savePoseGraph();
    void optimize6DoF();
    void loadPoseGraph();
    KeyFrame *getKeyFrame(int index);
    int detectLoop(KeyFrame *keyframe, int frame_index, bool add_brief);
    void addKeyFrameIntoVoc(KeyFrame *keyframe);
    void loadKeyFrame(KeyFrame *cur_kf);
    void process();
    void command();

    bool loop_on_flag{false};

    bool useOpenCL{false};
    // std::shared_ptr<ROS_IO> ros_io;
    std::thread measurement_process;
    std::thread keyboard_command_process;
    std::thread t_optimization;
    std::mutex m_buf;
    std::mutex m_process;
    std::mutex m_keyframelist;
    std::mutex m_optimize_buf;
    std::mutex m_path;
    std::mutex m_drift;

    bool LOAD_PREVIOUS_POSE_GRAPH = false;
    int ADD_KEYFRAME_MODE = 0;
    int sequence_cnt;
    int earliest_loop_index, global_index;
    int frame_index = 0;
    int sequence = 1;
    std::vector<bool> sequence_loop;
    std::string POSE_GRAPH_SAVE_PATH = "/root/";
    std::string vocabulary_file;
    cv::Mat mask;
    std::vector<cv::Point3f> point_3d;
    std::vector<cv::Point2f> point_2d_uv;
    std::vector<cv::KeyPoint> new_keypoints;
    
    Eigen::Vector3d last_t, last_ypr;
    Eigen::Vector3d t_drift;
    Eigen::Matrix3d r_drift;
    Eigen::Vector3d w_t_vio;
    Eigen::Matrix3d w_r_vio;

    BriefDatabase db;
    BriefVocabulary *voc;

    std::list<KeyFrame *> keyframelist;
    std::queue<int> optimize_buf;
    std::queue<image_type> image_buf;
    std::queue<pose_type> pose_buf;
    std::queue<point_type> point_buf;

    struct RelativeRTError
    {
        RelativeRTError(double t_x, double t_y, double t_z,
                        double q_w, double q_x, double q_y, double q_z,
                        double t_var, double q_var)
            : t_x(t_x), t_y(t_y), t_z(t_z),
              q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
              t_var(t_var), q_var(q_var) {}

        template <typename T>
        bool operator()(const T *const w_q_i, const T *ti, const T *w_q_j, const T *tj, T *residuals) const
        {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            T i_q_w[4];
            QuaternionInverse(w_q_i, i_q_w);

            T t_i_ij[3];
            ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
            residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
            residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

            T relative_q[4];
            relative_q[0] = T(q_w);
            relative_q[1] = T(q_x);
            relative_q[2] = T(q_y);
            relative_q[3] = T(q_z);

            T q_i_j[4];
            ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

            T relative_q_inv[4];
            QuaternionInverse(relative_q, relative_q_inv);

            T error_q[4];
            ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

            residuals[3] = T(2) * error_q[1] / T(q_var);
            residuals[4] = T(2) * error_q[2] / T(q_var);
            residuals[5] = T(2) * error_q[3] / T(q_var);

            return true;
        }

        static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                           const double q_w, const double q_x, const double q_y, const double q_z,
                                           const double t_var, const double q_var)
        {
            return (new ceres::AutoDiffCostFunction<
                    RelativeRTError, 6, 4, 3, 4, 3>(
                new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
        }

        double t_x, t_y, t_z, t_norm;
        double q_w, q_x, q_y, q_z;
        double t_var, q_var;
    };
};