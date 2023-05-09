// #pragma once

// #include <ceres/ceres.h>

// #include <sophus/se3.hpp>


// /*
//     SE(3) Parametrization such as: T + dT = Exp(dT) * T
// */
// struct AutoDiffLocalLeftSE3_Kernel
// {
//   template<typename T>
//   bool operator()(const T* _x, const T* _delta, T* _x_plus_delta) const 
//   {
//     Eigen::Map<const Sophus::SE3<T>> ini_T(_x);
//     Eigen::Map<const Eigen::Matrix<T,6,1>> lie_delta(_delta);

//     Eigen::Map<Sophus::SE3<T>> opt_T(_x_plus_delta);
//     opt_T = Sophus::SE3<T>::exp(lie_delta) * ini_T;

//     return true;
//   }
// };

// using AutoDiffLocalLeftSE3 = 
//   ceres::AutoDiffLocalParameterization<AutoDiffLocalLeftSE3_Kernel,7,6>;


// struct AutoDiffRad1Calib_Kernel
// {
//   AutoDiffRad1Calib_Kernel(const double _u, const double _v,
//                            const Eigen::Vector3d& _wpt)
//       : m_px(_u,_v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _dist_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     const T k1 = _dist_param[0];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
//     const T invz = T(1. / campt[2]);

//     const T x = T(campt[0]) * invz;
//     const T y = T(campt[1]) * invz;

//     const T x2 = x*x;
//     const T y2 = y*y;

//     const T r2 = x2 + y2;

//     const T D = T(1.) + r2*k1;

//     Eigen::Matrix<T,2,1> pred;
//     pred << fx * x * D + cx,
//             fy * y * D + cy;

//     _err[0] = pred[0] - m_px.cast<T>()[0];
//     _err[1] = pred[1] - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };


// struct AutoDiffRad2Calib_Kernel
// {
//   AutoDiffRad2Calib_Kernel(const double _u, const double _v,
//                            const Eigen::Vector3d& _wpt)
//       : m_px(_u,_v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _dist_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     const T k1 = _dist_param[0];
//     const T k2 = _dist_param[1];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
//     const T invz = T(1. / campt[2]);

//     const T x = T(campt[0]) * invz;
//     const T y = T(campt[1]) * invz;

//     const T x2 = x*x;
//     const T y2 = y*y;

//     const T r2 = x2 + y2;

//     const T D = T(1.) + r2*(k1 + k2*r2);

//     Eigen::Matrix<T,2,1> pred;
//     pred << fx * x * D + cx,
//             fy * y * D + cy;

//     _err[0] = pred[0] - m_px.cast<T>()[0];
//     _err[1] = pred[1] - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };


// struct AutoDiffRad3Calib_Kernel
// {
//   AutoDiffRad3Calib_Kernel(const double _u, const double _v,
//                            const Eigen::Vector3d& _wpt)
//       : m_px(_u,_v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _dist_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     const T k1 = _dist_param[0];
//     const T k2 = _dist_param[1];
//     const T k3 = _dist_param[2];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
//     const T invz = T(1. / campt[2]);

//     const T x = T(campt[0]) * invz;
//     const T y = T(campt[1]) * invz;

//     const T x2 = x*x;
//     const T y2 = y*y;

//     const T r2 = x2 + y2;
//     const T r4 = r2*r2;

//     const T D = T(1.) + r2*(k1 + k2*r2 + k3*r4);

//     Eigen::Matrix<T,2,1> pred;
//     pred << fx * x * D + cx,
//             fy * y * D + cy;

//     _err[0] = pred[0] - m_px.cast<T>()[0];
//     _err[1] = pred[1] - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };


// struct AutoDiffRadTan4Calib_Kernel
// {
//   AutoDiffRadTan4Calib_Kernel(const double _u, const double _v,
//                               const Eigen::Vector3d& _wpt)
//       : m_px(_u, _v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _dist_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     const T k1 = _dist_param[0];
//     const T k2 = _dist_param[1];
//     const T p1 = _dist_param[2];
//     const T p2 = _dist_param[3];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
//     const T invz = T(1. / campt[2]);

//     const T x = T(campt[0]) * invz;
//     const T y = T(campt[1]) * invz;

//     const T x2 = x*x;
//     const T y2 = y*y;
//     const T xy_2 = T(2.)*x*y;

//     const T r2 = x2 + y2;

//     const T D = T(1.) + r2*(k1 + k2*r2);

//     const T xd = x*D + p1*xy_2 + p2*(r2 +  T(2.)*x2);
//     const T yd = y*D + p2*xy_2 + p1*(r2 +  T(2.)*y2);

//     Eigen::Matrix<T,2,1> pred;
//     pred << fx * xd + cx,
//             fy * yd + cy;

//     _err[0] = pred[0] - m_px.cast<T>()[0];
//     _err[1] = pred[1] - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };


// struct AutoDiffRadTan5Calib_Kernel
// {
//   AutoDiffRadTan5Calib_Kernel(const double _u, const double _v,
//                               const Eigen::Vector3d& _wpt)
//       : m_px(_u, _v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _dist_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     const T k1 = _dist_param[0];
//     const T k2 = _dist_param[1];
//     const T k3 = _dist_param[2];
//     const T p1 = _dist_param[3];
//     const T p2 = _dist_param[4];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
//     const T invz = T(1. / campt[2]);

//     const T x = T(campt[0]) * invz;
//     const T y = T(campt[1]) * invz;

//     const T x2 = x*x;
//     const T y2 = y*y;
//     const T xy_2 = T(2.)*x*y;

//     const T r2 = x2 + y2;
//     const T r4 = r2*r2;

//     const T D = T(1.) + r2*(k1 + k2*r2 + k3*r4);

//     const T xd = x*D + p1*xy_2 + p2*(r2 +  T(2.)*x2);
//     const T yd = y*D + p2*xy_2 + p1*(r2 +  T(2.)*y2);

//     Eigen::Matrix<T,2,1> pred;
//     pred << fx * xd + cx,
//             fy * yd + cy;

//     _err[0] = pred[0] - m_px.cast<T>()[0];
//     _err[1] = pred[1] - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };


// struct AutoDiffRadTan8Calib_Kernel
// {
//   AutoDiffRadTan8Calib_Kernel(const double _u, const double _v,
//                               const Eigen::Vector3d& _wpt)
//       : m_px(_u, _v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _dist_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     const T k1 = _dist_param[0];
//     const T k2 = _dist_param[1];
//     const T k3 = _dist_param[2];
//     const T k4 = _dist_param[3];
//     const T k5 = _dist_param[4];
//     const T k6 = _dist_param[5];
//     const T p1 = _dist_param[6];
//     const T p2 = _dist_param[7];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
//     const T invz = T(1. / campt[2]);

//     const T x = T(campt[0]) * invz;
//     const T y = T(campt[1]) * invz;

//     const T x2 = x*x;
//     const T y2 = y*y;
//     const T xy_2 = T(2.)*x*y;

//     const T r2 = x2 + y2;
//     const T r4 = r2*r2;

//     const T D = (T(1.) + r2*(k1 + k2*r2 + k3*r4)) / (T(1.) + r2*(k4 + k5*r2 + k6*r4));

//     const T xd = x*D + p1*xy_2 + p2*(r2 +  T(2.)*x2);
//     const T yd = y*D + p2*xy_2 + p1*(r2 +  T(2.)*y2);

//     Eigen::Matrix<T,2,1> pred;
//     pred << fx * xd + cx,
//             fy * yd + cy;

//     _err[0] = pred[0] - m_px.cast<T>()[0];
//     _err[1] = pred[1] - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };


// struct AutoDiffKB4Calib_Kernel
// {
//   AutoDiffKB4Calib_Kernel(const double _u, const double _v,
//                           const Eigen::Vector3d& _wpt)
//       : m_px(_u, _v)
//       , m_wpt(_wpt)
//   {}

//   template <typename T>
//   bool operator()(const T *const _focal_param,
//                   const T *const _pp_param,
//                   const T *const _dist_param,
//                   const T *const _Tcw_param,
//                   T *_err) const
//   {
//     // Focal, Principal Point, Dist
//     const T fx = _focal_param[0];
//     const T fy = _focal_param[1];

//     const T cx = _pp_param[0];
//     const T cy = _pp_param[1];

//     const T k1 = _dist_param[0];
//     const T k2 = _dist_param[1];
//     const T k3 = _dist_param[2];
//     const T k4 = _dist_param[3];

//     Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

//     // Compute reproj err
//     const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt.cast<T>();
//     const T invz = T(1. / campt[2]);

//     const T x = T(campt[0]) * invz;
//     const T y = T(campt[1]) * invz;

//     const T r2 = x*x + y*y;
//     const T r = ceres::sqrt(r2);

//     const T theta = ceres::atan(r);
//     const T theta2 = theta*theta;
//     const T theta4 = theta2*theta2;
//     const T theta6 = theta4*theta2;

//     const T D = theta*(T(1.) + theta2*(k1 + theta2*k2 + theta4*k3 + theta6*k4));

//     const T D_r = D / r;

//     const T xd = D_r * x;
//     const T yd = D_r * y;

//     Eigen::Matrix<T,2,1> pred;
//     pred << fx * xd + cx,
//             fy * yd + cy;

//     _err[0] = pred[0] - m_px.cast<T>()[0];
//     _err[1] = pred[1] - m_px.cast<T>()[1];

//     return true;
//   }

//   Eigen::Vector2d m_px;
//   Eigen::Vector3d m_wpt;
// };