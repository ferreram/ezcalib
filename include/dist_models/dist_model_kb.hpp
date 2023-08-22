#pragma once

#include "dist_model_base.hpp"


class KB4DistParam : public DistParam
{
public:

  KB4DistParam()
    : DistParam(4)
  {}

  KB4DistParam(const double _k1, const double _k2, 
               const double _k3, const double _k4)
    : DistParam(4)
    , m_k1(_k1), m_k2(_k2)
    , m_k3(_k3), m_k4(_k4)
  {}

  void displayParams() const override
  {
    std::cout << "\n Distortion k1 / k2 / k3 / k4 : " << m_k1 << " / " << m_k2 << " / " << m_k3 << " / " << m_k4 << "\n\n";
  }

  Eigen::Vector2d distortCamPoint(const double _x, const double _y) const override
  {
    const double r2 = _x*_x + _y*_y;

    const double r = std::sqrt(r2);

    const double theta = std::atan(r);
    const double theta2 = theta*theta;
    const double theta4 = theta2*theta2;
    const double theta6 = theta4*theta2;

    const double D = theta*(1. + theta2*(m_k1 + theta2*m_k2 + theta4*m_k3 + theta6*m_k4));

    const double D_r = D / r;

    return Eigen::Vector2d(_x*D_r, _y*D_r);
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
  createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt, const bool _use_autodiff) const override
  {
    return  new ceres::AutoDiffCostFunction<
                AutoDiffKB4Calib_Kernel, 2, 2, 2, 4, 7>(
                    new AutoDiffKB4Calib_Kernel(_u, _v, _wpt));
  }
  
  void
  resetParameters(const std::vector<double> _v_dist_coefs) override
  {
    assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
    m_k1 = _v_dist_coefs[0];
    m_k2 = _v_dist_coefs[1];
    m_k3 = _v_dist_coefs[2];
    m_k4 = _v_dist_coefs[3];
  }

  std::vector<double> 
  getDistParameters() const
  {
    return {m_k1, m_k2, m_k3, m_k4};
  }

private:
  double m_k1=0., m_k2=0.;
  double m_k3=0., m_k4=0.;

  struct AutoDiffKB4Calib_Kernel
  {
    AutoDiffKB4Calib_Kernel(const double _u, const double _v,
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

      Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

      // Compute reproj err
      const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt.cast<T>();
      const T invz = T(1. / campt[2]);

      const T x = T(campt[0]) * invz;
      const T y = T(campt[1]) * invz;

      const T r2 = x*x + y*y;
      const T r = ceres::sqrt(r2);

      const T theta = ceres::atan(r);
      const T theta2 = theta*theta;
      const T theta4 = theta2*theta2;
      const T theta6 = theta4*theta2;

      const T D = theta*(T(1.) + theta2*(k1 + theta2*k2 + theta4*k3 + theta6*k4));

      const T D_r = D / r;

      const T xd = D_r * x;
      const T yd = D_r * y;

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
  
};
