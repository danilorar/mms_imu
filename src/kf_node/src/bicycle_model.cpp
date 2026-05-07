// linearized bicycle model state space 

#include "bicycle_model.hpp"
#include <algorithm>
#include <cmath>

BicycleModel::BicycleModel(){

    // constant placeholders
    m_ = 2600.0;;  // kg,
    Iz_ = 4500.0;  // kg m^2
    lf_ = 1.49225; // m
    lr_ = 1.49225; // m
    Cf_ = 80000.0; // N/rad
    Cr_ = 80000.0; // N/rad
    vx_min_ = 1.0; // avoid division by zero
}

// methods
Eigen::Matrix2d BicycleModel::computeA(double vx) const 
{
    const double vx_safe = std::max(std::abs(vx), vx_min_);

    Eigen::Matrix2d A; 
    A <<  -(Cf_ + Cr_) / (m_ * vx_safe),
    -((Cf_ * lf_ - Cr_ * lr_) / (m_ * vx_safe)) - vx_safe,
    -(Cf_ * lf_ - Cr_ * lr_) / (Iz_ * vx_safe),
    -(Cf_ * lf_ * lf_ + Cr_ * lr_ * lr_) / (Iz_ * vx_safe);

    return A; 
};

Eigen::Vector2d BicycleModel::computeB() const
{
    Eigen::Vector2d B;
    B << Cf_ / m_,
    Cf_ * lf_ / Iz_; 
    return B; 
};

Eigen::Matrix2d BicycleModel::computeC(double vx) const
{
    const double vx_safe = std::max(std::abs(vx), vx_min_);

    Eigen::Matrix2d C;

    C << -(Cf_ + Cr_) / (m_ * vx_safe),
        (-Cf_ * lf_ + Cr_ * lr_) / (m_ * vx_safe),
        0.0,
        1.0;

    return C;
}

Eigen::Vector2d BicycleModel::computeD() const
{
    Eigen::Vector2d D;
    D << Cf_ / m_,
    0;

    return D; 
};
