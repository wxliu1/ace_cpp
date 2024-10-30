#pragma once

#include "iostream"
#include "Eigen/Core"

class PinholeCamera
{
public:
    PinholeCamera(const double fx,
                  const double fy,
                  const double cx,
                  const double cy,
                  const double p1 = 0.0,
                  const double p2 = 0.0,
                  const double k1 = 0.0,
                  const double k2 = 0.0,
                  const double k3 = 0.0)
    {
        param_[0] = fx;
        param_[1] = fy;
        param_[2] = cx;
        param_[3] = cy;
        param_[4] = p1;
        param_[5] = p2;
        param_[6] = k1;
        param_[7] = k2;
        param_[8] = k3;
    }

    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P)
    {
        // 提取相机参数
        const double &fx = param_[0];
        const double &fy = param_[1];
        const double &cx = param_[2];
        const double &cy = param_[3];
        const double &p1 = param_[4];
        const double &p2 = param_[5];
        const double &k1 = param_[6];
        const double &k2 = param_[7];
        const double &k3 = param_[8];

        double x = (p[0] - cx) / fx;
        double y = (p[1] - cy) / fy;

        double r2 = x * x + y * y;

        double kr = (1 + (k1 + (k2 + k3 * r2) * r2) * r2);
        double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        x = x * kr + dx;
        y = y * kr + dy;
        P << x, y, 1.0;
    }

private:
    double param_[9];
};