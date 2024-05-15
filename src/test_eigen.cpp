// test eigen


#include <iostream>
#include <Eigen/Dense>

int main(int argc, char**argv)
{
    int row = 4;  
    int col = 5;  
    Eigen::MatrixXd matrixXd(row, col);  
    matrixXd << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20;  
    std::cout << matrixXd << std::endl << std::endl;

    // const auto Q1Jl = matrixXd.template block<2, 2>(0, 2).template triangularView<Eigen::Upper>();
    Eigen::Matrix2d Q1Jl = matrixXd.template block<2, 2>(0, 2).template triangularView<Eigen::Upper>();
    // Eigen::Matrix<double, 1, 1> Q1Jl = matrixXd.template block<1, 1>(0, 2).template triangularView<Eigen::Upper>();
    // auto  Q1Jl = matrixXd.template block<1, 1>(0, 2).template triangularView<Eigen::Upper>();
    // Eigen::Matrix2d Q1Jl = matrixXd.template block<2, 2>(0, 2);

    std::cout << "Q1Jl=" << Q1Jl << std::endl;

    Eigen::Matrix<double, 2, 1> n;
    n << 2, 8;
    std::cout << "n=" << n << std::endl;

    // 示例m.triangularView<Eigen::Upper>().solve(n)等价于m.inverse()*n
    // Eigen::Matrix<double, Eigen::Dynamic, 1> x1 = Q1Jl.solve(n);
    // Eigen::Matrix<double, Eigen::Dynamic, 1> x1 = Q1Jl.triangularView<Eigen::Upper>().solve(n);
    // std::cout << "x1=" << x1 << std::endl;


    Eigen::Matrix<double, Eigen::Dynamic, 1> x2 = Q1Jl.inverse() * n;
    std::cout << "x2=" << x2 << std::endl;


    matrixXd << Eigen::MatrixXd::Identity(row, col);
    std::cout << matrixXd << std::endl << std::endl;

    return 0;
}