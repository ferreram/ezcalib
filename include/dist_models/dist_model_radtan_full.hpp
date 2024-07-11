#pragma once

#include "dist_model_base.hpp"


class RadTan8DistParam : public DistParam
{
public:

  RadTan8DistParam(const bool _use_mono_focal)
    : DistParam(8, _use_mono_focal)
  {}

  RadTan8DistParam(const double _k1, const double _k2, const double _k3,
                   const double _k4, const double _k5, const double _k6,
                   const double _p1, const double _p2,
                   const bool _use_mono_focal)
    : DistParam(8, _use_mono_focal)
    , m_k1(_k1), m_k2(_k2), m_k3(_k3)
    , m_k4(_k4), m_k5(_k5), m_k6(_k6)
    , m_p1(_p1), m_p2(_p2)
  {}

  void displayParams() const override
  {
    std::cout << "\n Distortion k1 / k2 / k3 : " << m_k1 << " / " << m_k2 << " / " << m_k3;
    std::cout << "\n Distortion k4 / k5 / k6 : " << m_k4 << " / " << m_k5 << " / " << m_k6;
    std::cout << "\n Distortion p1 / p2 : " << m_p1 << " / " << m_p2 << "\n\n";
  }

  Eigen::Vector2d distortCamPoint(const double _x, const double _y) const override
  {
    const double x2 = _x*_x;
    const double y2 = _y*_y;

    const double xy_2 = 2.*_x*_y;
    
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;

    const double D = (1. + r2*(m_k1 + m_k2*r2 + m_k3*r4)) 
                    / (1. + r2*(m_k4 + m_k5*r2 + m_k6*r4));

    const double xd = _x*D + m_p1*xy_2 + m_p2*(r2 +  2.*x2);
    const double yd = _y*D + m_p2*xy_2 + m_p1*(r2 +  2.*y2);

    return Eigen::Vector2d(xd, yd);
  }

  Eigen::Vector2d distortCamPoint(const Eigen::Vector2d& _cam_pt) const override
  {
    return distortCamPoint(_cam_pt[0],_cam_pt[1]);
  }

  Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
  {
    const double inv_z = 1. / _cam_pt[2];
    return distortCamPoint(_cam_pt[0]*inv_z,_cam_pt[1]*inv_z);
  }

  ceres::CostFunction*
  createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
  {
    if (m_use_mono_focal)
    {
      return  new ceres::AutoDiffCostFunction<
                  AutoDiffRadTan8CalibMonoFocal_Kernel, 2, 1, 2, 8, 7>(
                      new AutoDiffRadTan8CalibMonoFocal_Kernel( _u, _v, _wpt));
    }
    
    return  new ceres::AutoDiffCostFunction<
                AutoDiffRadTan8Calib_Kernel, 2, 2, 2, 8, 7>(
                    new AutoDiffRadTan8Calib_Kernel( _u, _v, _wpt));
  }

  void
  resetParameters(const std::vector<double> _v_dist_coefs) override
  {
    assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
    m_k1 = _v_dist_coefs[0];
    m_k2 = _v_dist_coefs[1];
    m_k3 = _v_dist_coefs[2];
    m_k4 = _v_dist_coefs[3];
    m_k5 = _v_dist_coefs[4];
    m_k6 = _v_dist_coefs[5];
    m_p1 = _v_dist_coefs[6];
    m_p2 = _v_dist_coefs[7];
  }

  std::vector<double> 
  getDistParameters() const
  {
    return {m_k1, m_k2, m_k3, m_k4, m_k5, m_k6, m_p1, m_p2};
  }

private:
  double m_k1=0., m_k2=0., m_k3=0.;
  double m_k4=0., m_k5=0., m_k6=0.;
  double m_p1=0., m_p2=0.;

  struct AutoDiffRadTan8Calib_Kernel
  {
    AutoDiffRadTan8Calib_Kernel(const double _u, const double _v,
                                const Eigen::Vector3d& _wpt)
        : m_px(_u, _v)
        , m_wpt(_wpt)
    {}

    template <typename T>
    bool operator()(const T *const _focal_param,
                    const T *const _pp_param,
                    const T *const _dist_param,
                    const T *const _Tcw_param,
                    T *_err) const
    {
      // Focal, Principal Point, Dist
      const T fx = _focal_param[0];
      const T fy = _focal_param[1];

      const T cx = _pp_param[0];
      const T cy = _pp_param[1];

      const T k1 = _dist_param[0];
      const T k2 = _dist_param[1];
      const T k3 = _dist_param[2];
      const T k4 = _dist_param[3];
      const T k5 = _dist_param[4];
      const T k6 = _dist_param[5];
      const T p1 = _dist_param[6];
      const T p2 = _dist_param[7];

      Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

      // Compute reproj err
      const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
      const T invz = T(1. / campt[2]);

      const T x = T(campt[0]) * invz;
      const T y = T(campt[1]) * invz;

      const T x2 = x*x;
      const T y2 = y*y;
      const T xy_2 = T(2.)*x*y;

      const T r2 = x2 + y2;
      const T r4 = r2*r2;

      const T D = (T(1.) + r2*(k1 + k2*r2 + k3*r4)) / (T(1.) + r2*(k4 + k5*r2 + k6*r4));

      const T xd = x*D + p1*xy_2 + p2*(r2 +  T(2.)*x2);
      const T yd = y*D + p2*xy_2 + p1*(r2 +  T(2.)*y2);

      Eigen::Matrix<T,2,1> pred;
      pred << fx * xd + cx,
              fy * yd + cy;

      _err[0] = pred[0] - m_px.cast<T>()[0];
      _err[1] = pred[1] - m_px.cast<T>()[1];

      return true;
    }

    Eigen::Vector2d m_px;
    Eigen::Vector3d m_wpt;
  };

  struct AutoDiffRadTan8CalibMonoFocal_Kernel
  {
    AutoDiffRadTan8CalibMonoFocal_Kernel(const double _u, const double _v,
                                         const Eigen::Vector3d& _wpt)
        : m_px(_u, _v)
        , m_wpt(_wpt)
    {}

    template <typename T>
    bool operator()(const T *const _focal_param,
                    const T *const _pp_param,
                    const T *const _dist_param,
                    const T *const _Tcw_param,
                    T *_err) const
    {
      // Focal, Principal Point, Dist
      const T f = _focal_param[0];

      const T cx = _pp_param[0];
      const T cy = _pp_param[1];

      const T k1 = _dist_param[0];
      const T k2 = _dist_param[1];
      const T k3 = _dist_param[2];
      const T k4 = _dist_param[3];
      const T k5 = _dist_param[4];
      const T k6 = _dist_param[5];
      const T p1 = _dist_param[6];
      const T p2 = _dist_param[7];

      Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

      // Compute reproj err
      const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
      const T invz = T(1. / campt[2]);

      const T x = T(campt[0]) * invz;
      const T y = T(campt[1]) * invz;

      const T x2 = x*x;
      const T y2 = y*y;
      const T xy_2 = T(2.)*x*y;

      const T r2 = x2 + y2;
      const T r4 = r2*r2;

      const T D = (T(1.) + r2*(k1 + k2*r2 + k3*r4)) / (T(1.) + r2*(k4 + k5*r2 + k6*r4));

      const T xd = x*D + p1*xy_2 + p2*(r2 +  T(2.)*x2);
      const T yd = y*D + p2*xy_2 + p1*(r2 +  T(2.)*y2);

      Eigen::Matrix<T,2,1> pred;
      pred << f * xd + cx,
              f * yd + cy;

      _err[0] = pred[0] - m_px.cast<T>()[0];
      _err[1] = pred[1] - m_px.cast<T>()[1];

      return true;
    }

    Eigen::Vector2d m_px;
    Eigen::Vector3d m_wpt;
  };

};

