//- Sophus::SO3d的成员函数log()返回的是旋转角度吗？
/*
Sophus::SO3d 的成员函数 log() 返回的不是直接的旋转角度，而是旋转的罗德里格斯参数（Rodrigues Parameters），即旋转向量。这是一个向量，它的方向表示旋转轴，大小表示旋转的角度（弧度制）。

具体说明：
log() 的作用：
SO3d::log() 将旋转矩阵或四元数转换为旋转向量（或称为旋转李代数元素）。该旋转向量的方向表示旋转轴，而其长度表示旋转的角度（弧度）。
具体地，如果旋转是由旋转矩阵 R 描述的，则：
log(R)=θv
 
其中 θ 是旋转角度（以弧度为单位），而 v 是旋转轴的单位向量。

旋转向量的解释：

向量的方向：旋转轴。
向量的长度：旋转的角度（弧度）。
例如，假设你有一个 SO3d 对象表示某个旋转：

Sophus::SO3d rotation;
Eigen::Vector3d log_map = rotation.log();
log_map 是一个旋转向量，它的大小是旋转的角度（弧度），方向是旋转轴。

示例：
假设你有一个旋转矩阵 R，其表示绕 z 轴旋转 90 度，可以通过 SO3d::log() 获取旋转向量：

Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
Sophus::SO3d rotation(R);
Eigen::Vector3d rotation_log = rotation.log();  // 返回一个旋转向量
在这个例子中，rotation_log 的长度为 π/2，方向是单位向量 [0,0,1]（表示绕 z 轴旋转）。

小结：
SO3d::log() 返回的是一个旋转向量，而不是单一的旋转角度。旋转向量的长度表示旋转的角度，方向表示旋转的轴。
*/


//- SO3d::log()返回的角度是ypr吗？
/*
Sophus::SO3d::log() 返回的旋转向量（或称为旋转李代数）并不是标准的 Yaw-Pitch-Roll (YPR) 角度，而是一个旋转向量（Rodrigues' rotation vector），它与 YPR 角度有不同的表示方式。

旋转向量与 YPR 角度的区别
旋转向量：SO3d::log() 返回的是一个向量，方向表示旋转轴，长度表示旋转角度（弧度）。如果旋转向量的长度为 θ，则该旋转表示绕旋转轴旋转角度 θ。

例如，如果你用 SO3d::log() 得到的旋转向量是 

v=[0,0,1] × π/2，这意味着绕 z 轴旋转了 90 度。

Yaw-Pitch-Roll (YPR)：YPR 是一种欧拉角表示方法，用三个角度（偏航角Yaw，俯仰角Pitch，滚转角Roll）来描述三维空间中的旋转。它们是绕固定坐标轴（通常是地理坐标系中的 x、y、z 轴）进行的旋转。

log() 返回的内容：
SO3d::log() 返回的旋转向量表示的是旋转的轴和角度，而不是绕固定轴的欧拉角（如 Yaw-Pitch-Roll）。具体来说：

旋转向量的方向表示旋转轴（单位向量）。
旋转向量的长度表示旋转角度（弧度）。
如何从旋转向量转换为 YPR 角度？
虽然 SO3d::log() 返回的旋转向量不是直接的 YPR 角度，但你可以通过一系列变换来从旋转矩阵（或旋转向量）提取 YPR 角度。例如，通过将 SO3d 对象转换为旋转矩阵后，再从矩阵中提取 YPR 角度。可以使用一些数学方法（如旋转矩阵到欧拉角的转换公式）来完成这个转换。

示例：从旋转矩阵到 YPR
假设你有一个旋转矩阵 R，你可以先通过 SO3d 的 log() 得到旋转向量，然后将旋转矩阵转换为 Yaw-Pitch-Roll 角度。

// 获取旋转矩阵
Eigen::Matrix3d R = rotation.matrix();

// 提取 Yaw, Pitch, Roll
double roll  = atan2(R(2,1), R(2,2)); // x轴的旋转
double pitch = -asin(R(2,0));          // y轴的旋转
double yaw   = atan2(R(1,0), R(0,0));  // z轴的旋转
小结：
SO3d::log() 返回的是一个旋转向量，它不是 Yaw-Pitch-Roll (YPR) 角度，而是绕某个轴旋转的角度。
要获取 YPR 角度，你需要首先将旋转矩阵提取出来，然后使用适当的算法将其转换为 Yaw-Pitch-Roll 角度。
 */

/*
李群和李代数：
SO(3);R(3*3)
SE(3);T(4*4)
李代数：

so(3);(3*1)=>三个旋转
se(3);(6*1)=>前三维平移，后三维旋转
定义：

Sophus::SO3 SO3_R(R ); //旋转矩阵定义李群SO(3)
Sophus::SO3 SO3_q(Q );//四元数定义李群SO(3)
Sophus::SE3 SE3_Rt(R, t);//R,t构造SE(3)
Sophus::SE3 SE3_qt(q,t); //q,t构造SE(3)
Sophus::Vector3d so3 = SO3_R.log();//李代数so3为李群SO(3)的对数映射
Sophus::Vector6d se3 = SE3_Rt.log();//李代数se(3) 是一个6维向量为李群SE3 的对数映射
SE3_Rt = se3.exp();//李群SE3是李代数se3的指数映射
SO3_R = so3.exp();//李群SO3是李代数so3的指数映射
 */

#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char*argv[])
{
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Sophus::SO3d rotation(R);
    Eigen::Vector3d rotation_log = rotation.log();  // 返回一个旋转向量
    std::cout << "rotation_log=" << rotation_log.transpose() << std::endl; // rotation_log=     0      0 1.5708
    std::cout << "rotation_log.norm=" << rotation_log.norm() << std::endl; // angle 大小

    Sophus::SE3d transform(R, Eigen::Vector3d(0, 0, 0)); // use R,t to construct SE3d object
    Eigen::Matrix<double, 6, 1> transform_se3 = transform.log(); // 前三维平移，后三维旋转
    // Eigen::Vector3d rotation_se3 = transform.log().template tail<3>();
    Eigen::Vector3d rotation_se3 = transform_se3.template tail<3>();
    std::cout << "rotation_se3=" << rotation_se3.transpose() << std::endl;

    // 获取旋转矩阵
    {
        Eigen::Matrix3d R = rotation.matrix();

        // 提取 Yaw, Pitch, Roll
        double roll  = atan2(R(2,1), R(2,2)); // x轴的旋转
        double pitch = -asin(R(2,0));          // y轴的旋转
        double yaw   = atan2(R(1,0), R(0,0));  // z轴的旋转
        std::cout << "roll=" << roll << " pitch=" << pitch << " yaw=" << yaw << std::endl;
    }

    return 0;
}