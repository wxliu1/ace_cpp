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

    // 2024-9-24
    using MatX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
    using VecX = Eigen::Matrix<double, Eigen::Dynamic, 1>;
    Eigen::Matrix<double, 6, 36> Q2Jp;
    Q2Jp.setOnes();
    VecX tempVector;
    tempVector.resize(36 + 1);
    double* tempData = tempVector.data();
    double hCoeff=1.0;
    Q2Jp.bottomRightCorner(1, 30).applyHouseholderOnTheLeft(Q2Jp.col(5).tail(1 - 1),hCoeff, tempData + 5 + 1);
    std::cout << Q2Jp << std::endl;
    std::cout << "0 \n" << Q2Jp.bottomRightCorner(0, 0) << std::endl;
    std::cout << "1 \n" << Q2Jp.col(5).tail(1 - 1) << std::endl;
    std::cout << "2 \n" << Q2Jp.col(5).tail(2 - 1) << std::endl;
    std::cout << "3 \n" << Q2Jp.col(5).tail(3 - 1) << std::endl;
    // std::cout << Q2Jp.bottomRightCorner(-1, 30) << std::endl;
    try
    {
        /* code */
        std::cout << Q2Jp.bottomRightCorner(1, 37) << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    catch(...)
    {
        std::cout << "catch ....\n";
    }
    
    

    Eigen::Index marg_rank = 0;
    Eigen::Index total_rank = 0;
    {
    const double rank_threshold =
        std::sqrt(std::numeric_limits<double>::epsilon()); // epsilon=2.22045e-16一个很小的小量

    const Eigen::Index rows = Q2Jp.rows();
    const Eigen::Index cols = Q2Jp.cols();

    VecX tempVector;
    tempVector.resize(cols + 1);
    double* tempData = tempVector.data();

    for (Eigen::Index k = 0; k < cols && total_rank < rows; ++k) {
      Eigen::Index remainingRows = rows - total_rank;
      Eigen::Index remainingCols = cols - k - 1;

      double beta;
      double hCoeff;
      Q2Jp.col(k).tail(remainingRows).makeHouseholderInPlace(hCoeff, beta);

    //   if (std::abs(beta) > rank_threshold) {
      if (1) {
        Q2Jp.coeffRef(total_rank, k) = beta;

        // // Assertion failed 2024-9-20
        {
          char szLog[255] = { 0 };
          sprintf(szLog, "rows=%d,cols=%d,remainingRows=%d,remainingCols=%d, k=%d", rows, cols, remainingRows, remainingCols, k);
        //   wx::TFileSystemHelper::WriteLog(szLog);
            std::cout << szLog << std::endl;
        }
        // the end.
        Q2Jp.bottomRightCorner(remainingRows, remainingCols)
            .applyHouseholderOnTheLeft(Q2Jp.col(k).tail(remainingRows - 1),
                                       hCoeff, tempData + k + 1);
        // Q2r.tail(remainingRows)
        //     .applyHouseholderOnTheLeft(Q2Jp.col(k).tail(remainingRows - 1),
        //                                hCoeff, tempData + cols);
        total_rank++;
      } else {
          std::cout << "2 k=" << k << std::endl;
        Q2Jp.coeffRef(total_rank, k) = 0;
      }

      // Overwrite householder vectors with 0
      Q2Jp.col(k).tail(remainingRows - 1).setZero();

      
    }
  }

    return 0;
}