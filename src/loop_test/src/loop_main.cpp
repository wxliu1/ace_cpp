#include "loop.h"

#include <malloc.h>

#define buffer_size 30
#define SKIP_DISTANCE 0.1
#define SKIP_ANGLE 10

std::string pkg_path;
bool shutdown = false;

LoopClosure::LoopClosure(const std::string &cfg_path)
{
    pkg_path = cfg_path;
    // 获取support_files
    {
        vocabulary_file = pkg_path + "/support_files/brief_k10L6.bin";
        std::cout << "vocabulary_file: " << vocabulary_file << std::endl;
        // loadVocabulary(vocabulary_file);

        BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml";
        std::cout << "BRIEF_PATTERN_FILE: " << BRIEF_PATTERN_FILE << std::endl;
    }
}

LoopClosure::~LoopClosure()
{
    Stop();
}

void LoopClosure::InitPoseGraph()
{
    earliest_loop_index = -1;
    m_drift.lock();
    t_drift = Eigen::Vector3d(0, 0, 0);
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    m_drift.unlock();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);

    extr_ric = Eigen::Matrix3d::Identity();
    extr_tic = Eigen::Vector3d(0, 0, 0);

    last_t = Eigen::Vector3d(-100.0, -100.0, -100.0);
    last_ypr = Eigen::Vector3d(0, 0, 0);
}

void LoopClosure::Start()
{
    if (loop_on_flag)
        return; // 防止重复启动
    loop_on_flag = true;
    InitPoseGraph();
    ReadParameters();
    if (ADD_KEYFRAME_MODE)
    {
        // measurement_process = std::thread(&LoopClosure::process, this);
        // t_optimization = std::thread(&LoopClosure::optimize6DoF, this);
        keyboard_command_process = std::thread(&LoopClosure::command, this);

        // measurement_process.detach();
        // t_optimization.detach();
        keyboard_command_process.detach();
    }
}

void LoopClosure::Stop()
{
    if (!loop_on_flag)
        return;
    loop_on_flag = false;
    if(voc != nullptr)
    {
        resetVocabulary();
    }
}

void LoopClosure::resetVocabulary()
{
    // db.clear();
    db.clearAll();
    delete voc;
    voc = nullptr;
}

void LoopClosure::Reset()
{
    std::cout << "reset" << std::endl;
    m_process.lock();

    m_drift.lock();
    t_drift = Eigen::Vector3d(0, 0, 0);
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    m_drift.unlock();
    global_index = 0;
    frame_index = 0;
    sequence_cnt = 0;
    sequence = 1;
    sequence_loop.clear();
    sequence_loop.push_back(0);

    m_optimize_buf.lock();
    while (!optimize_buf.empty())
    {
        optimize_buf.pop();
    }
    m_optimize_buf.unlock();

    m_keyframelist.lock();
    for (auto it = keyframelist.begin(); it != keyframelist.end(); it++){
        delete *it;
    }
    keyframelist.clear();
    m_keyframelist.unlock();
    /*
malloc_trim 是 GNU C 库（glibc）中的一个函数，主要用于释放不再使用的内存回系统。它的作用是：

释放空闲内存：当程序在运行过程中分配了大量的内存后，又释放了一些内存，这些内存可能不会立即归还给操作系统，而是保存在堆中，以备未来分配。
malloc_trim 函数可以强制将这些不再需要的内存归还给操作系统，减少程序的内存占用。

控制内存碎片：在频繁分配和释放内存的场景中，堆中会产生很多小碎片，malloc_trim 通过回收空闲的内存块，有助于减小内存碎片，提高内存的利用效率。
     */
    malloc_trim(0);

    if(voc != nullptr)
    resetVocabulary();

    m_process.unlock();

    std::cout << "reset ok!" << std::endl;
}

void LoopClosure::add_left_image(const image_type image)
{
    m_buf.lock();
    if (image_buf.size() > buffer_size)
    {
        image_buf.pop();
    }
    image_buf.push(image);
    m_buf.unlock();
}

void LoopClosure::add_pose(pose_type &pose)
{
    m_buf.lock();
    if (pose_buf.size() > buffer_size)
    {
        pose_buf.pop();
    }
    pose_buf.push(pose);
    m_buf.unlock();

    pose.position = r_drift * (w_r_vio * pose.position + w_t_vio) + t_drift;
    pose.orientation = r_drift * (w_r_vio * pose.orientation);
}

void LoopClosure::set_extrinsic(const Eigen::Matrix3d ric, const Eigen::Vector3d tic)
{
    extr_tic = tic;
    extr_ric = ric;
}

void LoopClosure::set_intrinsic(const intrinsic_type intrinsic)
{
    if (intrinsic.cam_model == "pinhole")
    {
        pinhole_camera = std::make_shared<PinholeCamera>(intrinsic.projection_parameters[0],
                                                         intrinsic.projection_parameters[1],
                                                         intrinsic.projection_parameters[2],
                                                         intrinsic.projection_parameters[3],
                                                         intrinsic.distortion_parameters[0],
                                                         intrinsic.distortion_parameters[1],
                                                         intrinsic.distortion_parameters[2],
                                                         intrinsic.distortion_parameters[3],
                                                         intrinsic.distortion_parameters[4]);
    }
    else if (intrinsic.cam_model == "ds")
    {
        ds_camera = std::make_shared<DoubleSphereCamera>(intrinsic.projection_parameters[0],
                                                         intrinsic.projection_parameters[1],
                                                         intrinsic.projection_parameters[2],
                                                         intrinsic.projection_parameters[3],
                                                         intrinsic.distortion_parameters[0],
                                                         intrinsic.distortion_parameters[1]);
    }
    fx = intrinsic.projection_parameters[0];
    fy = intrinsic.projection_parameters[1];
    cx = intrinsic.projection_parameters[2];
    cy = intrinsic.projection_parameters[3];
}

void LoopClosure::ReadParameters()
{
    // 读取loop config
    {
        std::string config_file = pkg_path + "/config/sys.yaml";
        YAML::Node pr_loop_config = YAML::LoadFile(config_file);
        if (pr_loop_config["pose_graph_save_path"].Type() == YAML::NodeType::Scalar)
            POSE_GRAPH_SAVE_PATH = pr_loop_config["pose_graph_save_path"].as<std::string>();
        if (pr_loop_config["load_previous_pose_graph"].Type() == YAML::NodeType::Scalar)
            LOAD_PREVIOUS_POSE_GRAPH = pr_loop_config["load_previous_pose_graph"].as<bool>();
        if (pr_loop_config["add_keyframe_mode"].Type() == YAML::NodeType::Scalar)
            ADD_KEYFRAME_MODE = pr_loop_config["add_keyframe_mode"].as<int>();
        if (pr_loop_config["debug_image"].Type() == YAML::NodeType::Scalar)
            DEBUG_IMAGE = pr_loop_config["debug_image"].as<bool>();

        std::cout << "pose_graph_save_path: " << POSE_GRAPH_SAVE_PATH << std::endl;
        std::cout << "load_previous_pose_graph: " << LOAD_PREVIOUS_POSE_GRAPH << std::endl;
        std::cout << "add_keyframe_mode: " << ADD_KEYFRAME_MODE << std::endl;
    }

    FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        // loadPoseGraph();
        sequence_cnt = 1;
        m_process.unlock();
        printf("load pose graph finish\n");
    }
    else
    {
        printf("no previous pose graph\n");
    }
}

void LoopClosure::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void LoopClosure::optimize6DoF()
{
    while (loop_on_flag)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while (!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            // std::cout << "optimize pose graph" << std::endl;
            m_keyframelist.lock();
            KeyFrame *cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            double q_array[max_length][4];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            // ptions.minimizer_progress_to_stdout = true;
            // options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 20;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            // loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();

            list<KeyFrame *>::iterator it;
            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i][0] = tmp_q.w();
                q_array[i][1] = tmp_q.x();
                q_array[i][2] = tmp_q.y();
                q_array[i][3] = tmp_q.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {
                    problem.SetParameterBlockConstant(q_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                // add edge
                for (int j = 1; j < 5; j++)
                {
                    if (i - j >= 0 && sequence_array[i] == sequence_array[i - j])
                    {
                        Vector3d relative_t(t_array[i][0] - t_array[i - j][0], t_array[i][1] - t_array[i - j][1], t_array[i][2] - t_array[i - j][2]);
                        Quaterniond q_i_j = Quaterniond(q_array[i - j][0], q_array[i - j][1], q_array[i - j][2], q_array[i - j][3]);
                        Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                        relative_t = q_i_j.inverse() * relative_t;
                        Quaterniond relative_q = q_i_j.inverse() * q_i;
                        ceres::CostFunction *vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                   relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                   0.1, 0.01);
                        problem.AddResidualBlock(vo_function, NULL, q_array[i - j], t_array[i - j], q_array[i], t_array[i]);
                    }
                }

                // add loop edge

                if ((*it)->has_loop)
                {
                    if ((*it)->loop_index < first_looped_index)
                        continue;
                    // assert((*it)->loop_index >= first_looped_index);

                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    Quaterniond relative_q;
                    relative_q = (*it)->getLoopRelativeQ();
                    ceres::CostFunction *loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                 0.1, 0.01);
                    problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);
                }

                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            // std::cout << summary.BriefReport() << std::endl;

            // printf("pose optimization time: %f \n", tmp_t.toc());
            // for (int j = 0 ; j < i; j++)
            // {
            //     printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            // }

            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)->updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            r_drift = cur_r * vio_r.transpose();
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}

void LoopClosure::loadPoseGraph()
{
    FILE *pFile;
    std::string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen(file_path.c_str(), "r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    int loop_index;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1> loop_info;
    int cnt = 0;
    while (fscanf(pFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp,
                  &VIO_Tx, &VIO_Ty, &VIO_Tz,
                  &PG_Tx, &PG_Ty, &PG_Tz,
                  &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz,
                  &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz,
                  &loop_index,
                  &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3,
                  &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                  &keypoints_num) != EOF)
    {
        /*
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp,
                                    VIO_Tx, VIO_Ty, VIO_Tz,
                                    PG_Tx, PG_Ty, PG_Tz,
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz,
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz,
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3,
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;

        Eigen::Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Eigen::Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Eigen::Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Eigen::Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Eigen::Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1> loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }
        }
/**/
        // load keypoints, brief_descriptors
        std::string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        std::string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        std::vector<cv::KeyPoint> keypoints;
        std::vector<cv::KeyPoint> keypoints_norm;
        std::vector<DVision::BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            DVision::BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if (!fscanf(keypoints_file, "%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame *keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe);

    }
    fclose(pFile);

    printf("pose graph loading ok\n");
}

int LoopClosure::detectLoop(KeyFrame *keyframe, int index, bool add_brief)
{
    // first query; then add this frame into database!
    DBoW2::QueryResults ret;
    if (add_brief)
    {
        db.query(keyframe->brief_descriptors, ret, 4, index - 50);
        db.add(keyframe->brief_descriptors);
    }
    else
    {
        db.query(keyframe->brief_descriptors, ret, 4, -1);
    }

    // printf("add feature time: %f", t_add.toc());
    //  ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;

    // a good match with its nerghbour
    bool if1;
    if (add_brief)
    {
        if1 = (ret.size() >= 1 && ret[0].Score > 0.05);
    }
    else
    {
        if1 = (ret.size() >= 1 && ret[0].Score > 0.03);
    }
    if (if1)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            // std::cout << ret[i].Id << ": " << ret[i].Score << std::endl;
            // if (ret[i].Score > ret[0].Score * 0.3)
            // if (ret[i].Score > 0.015)
            if (ret[i].Score > 0.01)
            {
                find_loop = true;
            }
        }
    }

    bool if2;
    if (add_brief)
    {
        if2 = (find_loop && frame_index > 60);
    }
    else
    {
        if2 = (find_loop && frame_index > 0);
    }
    if (if2)
    {
        if (add_brief)
        {
            int min_index = -1;
            for (unsigned int i = 0; i < ret.size(); i++)
            {
                if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                    min_index = ret[i].Id;
            }
            return min_index;
        }
        else
        {
            return ret[0].Id;
        }
    }
    else
        return -1;
}

KeyFrame *LoopClosure::getKeyFrame(int index)
{
    //    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame *>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)
    {
        if ((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

void LoopClosure::addKeyFrameIntoVoc(KeyFrame *keyframe)
{
    db.add(keyframe->brief_descriptors);
}

void LoopClosure::loadKeyFrame(KeyFrame *cur_kf)
{
    cur_kf->index = global_index;
    global_index++;

    if(voc != nullptr)
    addKeyFrameIntoVoc(cur_kf);

    m_keyframelist.lock();

    Eigen::Vector3d P;
    Eigen::Matrix3d R;
    cur_kf->getPose(P, R);
    Eigen::Quaterniond Q{R};

    keyframelist.push_back(cur_kf);
    m_keyframelist.unlock();
}

void LoopClosure::addKeyFrame(KeyFrame *cur_kf, bool add_brief)
{
    // shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;
        sequence_loop.push_back(0);
        // w_t_vio = Eigen::Vector3d(0, 0, 0);
        // w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }

    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio * vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;

    int loop_index = -1;
    loop_index = detectLoop(cur_kf, cur_kf->index, add_brief);
    // std::cout << "cur index " << cur_kf->index << " match loop index " << loop_index << std::endl;

    if (loop_index != -1)
    {
        // printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame *old_kf = getKeyFrame(loop_index);

        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;

            shift_r = w_R_cur * vio_R_cur.transpose();
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;

            // shift vio pose of whole sequence to the world frame
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio * vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                list<KeyFrame *>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)
                {
                    if ((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio * vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;
            }
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }

    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};

    keyframelist.push_back(cur_kf);
    m_keyframelist.unlock();
}

void LoopClosure::updatePath()
{
    m_keyframelist.lock();
    list<KeyFrame *>::iterator it;

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
    }
    m_keyframelist.unlock();
}

void LoopClosure::savePoseGraph()
{
    m_keyframelist.lock();
    FILE *pFile;
    std::cout << "pose graph path: " << POSE_GRAPH_SAVE_PATH.c_str() << std::endl;
    std::cout << "pose graph saving... " << std::endl;
    std::string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen(file_path.c_str(), "w");
    if (pFile == NULL)
    {
        std::cout << "pFile: " << pFile << std::endl;
        return;
    }
    std::list<KeyFrame *>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        Eigen::Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Eigen::Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Eigen::Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Eigen::Vector3d PG_tmp_T = (*it)->T_w_i;

        fprintf(pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n", (*it)->index, (*it)->time_stamp,
                VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
                PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
                VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),
                PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),
                (*it)->loop_index,
                (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                (int)(*it)->keypoints.size());

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        // assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        if ((*it)->keypoints.size() != (*it)->brief_descriptors.size())
        {
            continue;
        }
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y,
                    (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);
    }
    fclose(pFile);

    m_keyframelist.unlock();
}

// void LoopClosure::command()
// {
//     while (loop_on_flag)
//     {
//         std::chrono::milliseconds dura1(15*1000);
//         // std::chrono::milliseconds dura1(2*1000);
//         std::this_thread::sleep_for(dura1);

//         Reset();

//         std::chrono::milliseconds dura2(15*1000);
//         // std::chrono::milliseconds dura2(6*1000);
//         std::this_thread::sleep_for(dura2);
//         Stop();
//         shutdown = true;
//     }
// }

void LoopClosure::command()//(void *pParam)
{

  std::cout << "1 command()" << std::endl;
  while (true)
  {
  #if 1//def _KEY_PRESS_
    // char c = getchar();
    // std::string strInput;
    // std::cin >> strInput;
    char c;
    std::cin >> c;
    
    if(c == 'v')
    {
        std::cout << "press 'v' to load vocabulary" << std::endl;
        loadVocabulary(vocabulary_file);
    }

    else if(c == 'p')
    {
        std::cout << "press 'p' to loadPoseGrpah" << std::endl;
        loadPoseGraph();
    }

    else if(c == 'r')
    {
        std::cout << "press 'r' to reset" << std::endl;
        Reset();
    }
    else if(c == 'q')
    {
        std::cout << "press 'q' to quit" << std::endl;
        Stop();
        shutdown = true;
        break ;

    }

    

  #endif

    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }

  std::cout << "2 command()" << std::endl;
}

void LoopClosure::process()
{
    while (loop_on_flag)
    {
        // 检查内存空间是否过小
        std::ifstream file("/proc/meminfo");
        if (file.is_open())
        {
            long MemAvailable = 0;
            std::string line;
            while (std::getline(file, line))
            {
                if (line.find("MemAvailable") != std::string::npos)
                {
                    size_t colonPos = line.find(':');
                    if (colonPos != std::string::npos)
                    {
                        MemAvailable = std::stol(line.substr(colonPos + 1));
                    }
                    break;
                }
            }
            file.close();

            if (MemAvailable < 512 * 1024)
            {
                std::chrono::milliseconds dura(500);
                std::this_thread::sleep_for(dura);
                continue;
            }
        }

        image_type image_msg;
        point_type point_msg;
        pose_type pose_msg;

        m_buf.lock();
        if (!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front().timestamp > pose_buf.front().timestamp)
            {
                pose_buf.pop();
            }
            else if (image_buf.front().timestamp > point_buf.front().timestamp)
            {
                point_buf.pop();
            }
            else if (image_buf.back().timestamp >= pose_buf.front().timestamp && point_buf.back().timestamp >= pose_buf.front().timestamp)
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front().timestamp < pose_msg.timestamp)
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front().timestamp < pose_msg.timestamp)
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        // 判断pose_msg是否为空
        if (!image_msg.image.empty())
        {
            Eigen::Matrix3d R = pose_msg.orientation.toRotationMatrix();
            Eigen::Vector3d ypr = Utility::R2ypr(R);
            double relative_yaw = Utility::normalizeAngle(ypr.x() - last_ypr.x());
            double relative_pitch = Utility::normalizeAngle(ypr.y() - last_ypr.y());
            double relative_roll = Utility::normalizeAngle(ypr.z() - last_ypr.z());

            if ((pose_msg.position - last_t).norm() < SKIP_DISTANCE &&
                fabs(relative_yaw) < SKIP_ANGLE &&
                fabs(relative_pitch) < SKIP_ANGLE &&
                fabs(relative_roll) < SKIP_ANGLE)
                continue;

            const int fast_th = 15;
            cv::FAST(image_msg.image, new_keypoints, fast_th, true);
            if (point_msg.points_3d.size() > 200) // 点太多的情况下只保留fast角点
            {
                point_3d.clear();
                point_2d_uv.clear();
                mask = cv::Mat(image_msg.image.rows, image_msg.image.cols, CV_8UC1, cv::Scalar(0));
                for (size_t i = 0; i < new_keypoints.size(); i++)
                {
                    cv::circle(mask, new_keypoints[i].pt, 2, 255, -1);
                }
                for (size_t i = 0; i < point_msg.points_3d.size(); i++)
                {
                    if (mask.at<uchar>((int)point_msg.points_2d_uv[i].y, (int)point_msg.points_2d_uv[i].x))
                    {
                        point_3d.push_back(point_msg.points_3d[i]);
                        point_2d_uv.push_back(point_msg.points_2d_uv[i]);
                    }
                }
            }
            else
            {
                point_3d = point_msg.points_3d;
                point_2d_uv = point_msg.points_2d_uv;
            }

            KeyFrame *keyframe = new KeyFrame(pose_msg.timestamp, frame_index, pose_msg.position, R, image_msg.image,
                                              point_3d, point_2d_uv, sequence, new_keypoints);
            m_process.lock();
            addKeyFrame(keyframe, !LOAD_PREVIOUS_POSE_GRAPH);
            m_process.unlock();
            frame_index++;
            last_t = pose_msg.position;
            last_ypr = ypr;
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv){
    std::string cfg_path = "..";

    std::shared_ptr<LoopClosure> loop_algo;
    loop_algo = std::make_shared<LoopClosure>(cfg_path);
    loop_algo->Start();

    while (!shutdown)
    {
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }

    return 0;
}
