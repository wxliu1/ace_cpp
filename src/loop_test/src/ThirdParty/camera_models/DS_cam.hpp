#pragma once

#include "iostream"
#include "Eigen/Core"

class DoubleSphereCamera
{
public:
    DoubleSphereCamera(const double fx,
                       const double fy,
                       const double cx,
                       const double cy,
                       const double xi,
                       const double alpha)
    {
        param_[0] = fx;
        param_[1] = fy;
        param_[2] = cx;
        param_[3] = cy;
        param_[4] = xi;
        param_[5] = alpha;
    }

    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P)
    {
        const double &fx = param_[0];
        const double &fy = param_[1];
        const double &cx = param_[2];
        const double &cy = param_[3];

        const double &xi = param_[4];
        const double &alpha = param_[5];

        const double mx = (p[0] - cx) / fx;
        const double my = (p[1] - cy) / fy;

        const double r2 = mx * mx + my * my;

        const bool is_valid =
            !static_cast<bool>(alpha > double(0.5) &&
                               (r2 >= double(1) / (double(2) * alpha - double(1))));

        const double xi2_2 = alpha * alpha;
        const double xi1_2 = xi * xi;

        const double sqrt2 = sqrt(double(1) - (double(2) * alpha - double(1)) * r2);

        const double norm2 = alpha * sqrt2 + double(1) - alpha;

        const double mz = (double(1) - xi2_2 * r2) / norm2;
        const double mz2 = mz * mz;

        const double norm1 = mz2 + r2;
        const double sqrt1 = sqrt(mz2 + (double(1) - xi1_2) * r2);
        const double k = (mz * xi + sqrt1) / norm1;

        P << k * mx, k * my, k * mz - xi;

        P[0] /= P[2];
        P[1] /= P[2];
        P[2] = 1.0;
    }

private:
    double param_[6];
};