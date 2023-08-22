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


// ===================================
//    Analyticial Parametrization!
// ===================================


/*
    SE(3) Parametrization such as:
    1. T + dT = Exp(dT) * T
    2. T o X = T^(-1) * X (i.e. T: cam -> world)
*/
class SE3LeftParameterization : public ceres::LocalParameterization
{
public:
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const
  {
    Eigen::Map<const Sophus::SE3d> T(x);
    Eigen::Map<const Sophus::Vector6d> vdelta(delta);

    Eigen::Map<Sophus::SE3d> upT(x_plus_delta);

    // Left update
    upT = Sophus::SE3d::exp(vdelta) * T;

    return true;
  }

  virtual bool ComputeJacobian(const double *x,
                               double *jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);

    // The Jacobian is set w.r.t. to the 6D delta_param jacobian set in Evaluate()
    J.topRows<6>().setIdentity();
    J.bottomRows<1>().setZero();

    return true;
  }

  virtual int GlobalSize() const { return 7; }
  virtual int LocalSize() const { return 6; }
};



// struct AutoDiffReprojErr_Kernel
// {
//   AutoDiffReprojErr_Kernel(const double _u, const double _v,
//                           const Eigen::Vector3d& _wpt)
//       : m_px(_u,_v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
//     const T invz = T(1. / campt[2]);

//     const T u = T(campt[0]) * invz * fx + cx;
//     const T v = T(campt[1]) * invz * fy + cy;

//     _err[0] = u - m_px.cast<T>()[0];
//     _err[1] = v - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };

// using AutoDiffReprojErr = 
//   ceres::AutoDiffCostFunction<AutoDiffReprojErr_Kernel, 2, 2, 2, 7>;
