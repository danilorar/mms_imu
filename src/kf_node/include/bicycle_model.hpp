#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>


class BicycleModel
{
public:
    BicycleModel();

    Eigen::Matrix2d computeA(double vx) const;
    Eigen::Vector2d computeB() const;
    Eigen::Matrix2d computeC(double vx) const;
    Eigen::Vector2d computeD() const;

private:
    double m_;
    double Iz_;
    double lf_;
    double lr_;
    double Cf_;
    double Cr_;
    double vx_min_;
};

#endif
