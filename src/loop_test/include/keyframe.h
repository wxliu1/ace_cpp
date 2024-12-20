#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "../src/ThirdParty/camera_models/DS_cam.hpp"
#include "../src/ThirdParty/camera_models/Pinhole_cam.hpp"
#include "../src/utility/utility.h"
#include "../src/ThirdParty/DBoW/DBoW2.h"
#include "../src/ThirdParty/DVision/DVision.h"
#include "../src/ThirdParty/DBoW/TemplatedDatabase.h"
#include "../src/ThirdParty/DBoW/TemplatedVocabulary.h"
// #include "ros_interface.hpp"

#define MIN_LOOP_NUM 25

using namespace Eigen;
using namespace std;
using namespace DVision;

extern std::shared_ptr<DoubleSphereCamera> ds_camera;
extern std::shared_ptr<PinholeCamera> pinhole_camera;
extern double fx, fy, cx, cy;

extern bool DEBUG_IMAGE;
extern std::string BRIEF_PATTERN_FILE;
extern Eigen::Vector3d extr_tic;
extern Eigen::Matrix3d extr_ric;

class BriefExtractor
{
public:
	virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
	BriefExtractor(const std::string &pattern_file);

	DVision::BRIEF m_brief;
};

class KeyFrame
{
public:
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
			 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, int _sequence, vector<cv::KeyPoint> &new_keypoints);
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
			 cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1> &_loop_info,
			 vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors);
	bool findConnection(KeyFrame *old_kf);
	void computeWindowBRIEFPoint();
	void computeBRIEFPoint(vector<cv::KeyPoint> &new_keypoints);
	// void extractBrief();
	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
	bool searchInAera(const BRIEF::bitset window_descriptor,
					  const std::vector<BRIEF::bitset> &descriptors_old,
					  const std::vector<cv::KeyPoint> &keypoints_old,
					  const std::vector<cv::KeyPoint> &keypoints_old_norm,
					  cv::Point2f &best_match,
					  cv::Point2f &best_match_norm);
	void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
						  std::vector<cv::Point2f> &matched_2d_old_norm,
						  std::vector<uchar> &status,
						  const std::vector<BRIEF::bitset> &descriptors_old,
						  const std::vector<cv::KeyPoint> &keypoints_old,
						  const std::vector<cv::KeyPoint> &keypoints_old_norm);
	void FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
								const std::vector<cv::Point2f> &matched_2d_old_norm,
								vector<uchar> &status);
	void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
				   const std::vector<cv::Point3f> &matched_3d,
				   std::vector<uchar> &status,
				   Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);
	void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateLoop(Eigen::Matrix<double, 8, 1> &_loop_info);

	Eigen::Vector3d getLoopRelativeT();
	double getLoopRelativeYaw();
	Eigen::Quaterniond getLoopRelativeQ();

	double time_stamp;
	int index;
	int local_index;
	Eigen::Vector3d vio_T_w_i;
	Eigen::Matrix3d vio_R_w_i;
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d origin_vio_T;
	Eigen::Matrix3d origin_vio_R;
	cv::Mat image;
	vector<cv::Point3f> point_3d;
	vector<cv::Point2f> point_2d_uv;
	vector<cv::KeyPoint> keypoints;
	vector<cv::KeyPoint> keypoints_norm;
	vector<cv::KeyPoint> window_keypoints;
	vector<BRIEF::bitset> brief_descriptors;
	vector<BRIEF::bitset> window_brief_descriptors;
	std::shared_ptr<BriefExtractor> extractor;
	int sequence;

	bool has_loop;
	int loop_index;
	Eigen::Matrix<double, 8, 1> loop_info;

	// std::shared_ptr<ROS_IO> ros_io;
};
