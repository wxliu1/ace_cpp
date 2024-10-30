#include "loop.h"
#include "algo_loop.h"
/*
这个文件重新封装了算法的输入输出接口，在把算法打包成库后，只需要so文件和h文件即可合并算法到工程里面
*/

std::shared_ptr<LoopClosure> loop_algo;

void loop_algo_init(const ros_nh nh, const std::string &cfg_path)
{
    loop_algo = std::make_shared<LoopClosure>(nh, cfg_path);
}

void loop_algo_receive_topic_init()
{
    loop_algo->ros_topic_receive_init();
}

void loop_algo_start()
{
    loop_algo->Start();
}

void loop_algo_stop()
{
    loop_algo->Stop();
}

void loop_algo_reset()
{
    loop_algo->Reset();
}

void loop_add_left_image(const image_type &img_left)
{
    loop_algo->add_left_image(img_left);
}

void loop_add_feature_point(point_type &point, ros_pointcloud2 &pointcloud2_msg)
{
    loop_algo->add_feature_point(point, pointcloud2_msg);
}
void loop_add_pose(pose_type &pose)
{
    loop_algo->add_pose(pose);
}

void loop_set_extrinsic(const Eigen::Matrix3d ric, const Eigen::Vector3d tic)
{
    loop_algo->set_extrinsic(ric, tic);
}
void loop_set_intrinsic(const intrinsic_type intrinsic)
{
    loop_algo->set_intrinsic(intrinsic);
}

std::string loop_version()
{
    return loop_algo->loop_version();
}
