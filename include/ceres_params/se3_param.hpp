#pragma once

#include <ceres/ceres.h>

#include <sophus/se3.hpp>


/*
    SE(3) Parametrization such as: T + dT = Exp(dT) * T
*/
struct AutoDiffLocalLeftSE3_Kernel
{
  template<typename T>
  bool operator()(const T* _x, const T* _delta, T* _x_plus_delta) const 
  {
    Eigen::Map<const Sophus::SE3<T>> ini_T(_x);
    Eigen::Map<const Eigen::Matrix<T,6,1>> lie_delta(_delta);

    Eigen::Map<Sophus::SE3<T>> opt_T(_x_plus_delta);
    opt_T = Sophus::SE3<T>::exp(lie_delta) * ini_T;

    return true;
  }
};

using AutoDiffLocalLeftSE3 = 
  ceres::AutoDiffLocalParameterization<AutoDiffLocalLeftSE3_Kernel,7,6>;