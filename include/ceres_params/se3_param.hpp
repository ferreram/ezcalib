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

// struct AutoDiffLeftSE3RelativePoseError
// {
//   AutoDiffLeftSE3RelativePoseError(const Sophus::SE3d &_Tc0ci)
//     : m_Tc0ci_meas(_Tc0ci)
//   {}

//   template <typename T>
//   bool operator()(const T *const _Tcoci_param,
//                   T *_err) const
//   {
//     Eigen::Map<const Sophus::SE3<T>> Tc0ci(_Tcoci_param);

//     Sophus::SE3<T> err = (Tc0ci * m_Tc0ci_meas);

//     Eigen::Map<Eigen::Matrix<T,6,1>> _err = err.log();

//     return true;
//   }

//   Sophus::SE3d m_Tc0ci_meas;
// };
