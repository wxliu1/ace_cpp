
#include <iostream>
#include <Eigen/Dense>

void get_imu_right_extrinsic(){
    Eigen::Matrix4d I2L,L2R;
/*    
    extrinsic_matrix cam_imu_extrinsic = sys_param->vio_get_CamToImu();
    I2L <<  cam_imu_extrinsic.r00, cam_imu_extrinsic.r01, cam_imu_extrinsic.r02,cam_imu_extrinsic.t0,
            cam_imu_extrinsic.r10, cam_imu_extrinsic.r11, cam_imu_extrinsic.r12,cam_imu_extrinsic.t1,
            cam_imu_extrinsic.r20, cam_imu_extrinsic.r21, cam_imu_extrinsic.r22,cam_imu_extrinsic.t2,
            0.0,0.0,0.0,1.0;
    L2R <<  1.0,0.0,0.0,-right_camera.P[3]/right_camera.P[0],
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;
*/
    I2L <<  9.9982561249955959e-01, -3.3336852175498738e-03, -1.8374741705687966e-02, -1.0162560183039057e-02,
            4.6026851707220203e-03, 9.9757421585676265e-01,6.9458614634790181e-02, -1.4670167521915003e-03,
            1.8098615391782733e-02, -6.9531075071764592e-02, 9.9741559528627033e-01, 3.2700939325780003e-04,
            0.0,0.0,0.0,1.0;
    
    L2R <<  1.0,0.0,0.0,0.04505367,
            0.0,1.0,0.0,0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;

    Eigen::Matrix4d I2R = I2L * L2R;

    //将计算出来的imu到右目外参写到文件里面
    std::cout << "============extrinsic I2R============" << std::endl;
    std::cout << I2R << std::endl;
/*
    //打包成话题发出去
    cam_imu_extrinsic_right.position.x = I2R(0,3);
    cam_imu_extrinsic_right.position.y = I2R(1,3);
    cam_imu_extrinsic_right.position.z = I2R(2,3);

    Eigen::Quaterniond q(I2R.block<3, 3>(0, 0));

    cam_imu_extrinsic_right.orientation.x = q.x();
    cam_imu_extrinsic_right.orientation.y = q.y();
    cam_imu_extrinsic_right.orientation.z = q.z();
    cam_imu_extrinsic_right.orientation.w = q.w();
    */

   Eigen::Quaterniond q_l(I2L.block<3, 3>(0, 0));
   Eigen::Quaterniond q_r(I2R.block<3, 3>(0, 0));

   std::cout << "q_l=" << q_l.coeffs() << std::endl;
   std::cout << "q_l.vec=" << q_l.vec() << std::endl;
   std::cout << q_l.w() << " " << q_l.x() << " " << q_l.y() << " " << q_l.z() << std::endl;

   std::cout << "q_r=" << q_r.coeffs() << std::endl;
   std::cout << "q_r.vec=" << q_r.vec() << std::endl;
   std::cout << q_r.w() << " " << q_r.x() << " " << q_r.y() << " " << q_r.z() << std::endl;
}

int main(int argc, char *argv[])
{
    get_imu_right_extrinsic();

    return 0;
}