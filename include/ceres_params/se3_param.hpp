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

struct AutoDiffReprojErr_Kernel
{
  AutoDiffReprojErr_Kernel(const double _u, const double _v,
                          const Eigen::Vector3d& _wpt)
      : m_px(_u,_v)
      , m_wpt(_wpt)
  {}

  template <typename T>
  bool operator()(const T *const _focal_param,
                  const T *const _pp_param,
                  const T *const _Tcw_param,
                  T *_err) const
  {
    // Focal, Principal Point, Dist
    const T fx = _focal_param[0];
    const T fy = _focal_param[1];

    const T cx = _pp_param[0];
    const T cy = _pp_param[1];

    Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

    // Compute reproj err
    const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
    const T invz = T(1. / campt[2]);

    const T u = T(campt[0]) * invz * fx + cx;
    const T v = T(campt[1]) * invz * fy + cy;

    _err[0] = u - T(m_px[0]);
    _err[1] = v - T(m_px[1]);

    return true;
  }

  Eigen::Vector2d m_px;
  Eigen::Vector3d m_wpt;
};

using AutoDiffReprojErr = 
  ceres::AutoDiffCostFunction<AutoDiffReprojErr_Kernel, 2, 2, 2, 7>;
