#pragma once

#include "dist_model_base.hpp"


class Rad2DistParam : public DistParam
{
public:

  Rad2DistParam()
    : DistParam(2)
  {}

  Rad2DistParam(const double _k1, const double _k2)
    : DistParam(2)
    , m_k1(_k1), m_k2(_k2)
  {}

  void displayParams() const override
  {
    std::cout << "\n Distortion k1 / k2 : " << m_k1 << " / " << m_k2 << "\n\n";
  }

  Eigen::Vector2d distortCamPoint(const double _x, const double _y) const override
  {
    const double r2 = _x*_x + _y*_y;

    const double D = 1. + r2*(m_k1 + m_k2*r2);
    return Eigen::Vector2d(_x*D, _y*D);
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
    return  new ceres::AutoDiffCostFunction<
                AutoDiffRad2Calib_Kernel, 2, 2, 2, 2, 7>(
                    new AutoDiffRad2Calib_Kernel( _u, _v, _wpt));
  }
  
  void
  resetParameters(const std::vector<double> _v_dist_coefs) override
  {
    assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
    m_k1 = _v_dist_coefs[0];
    m_k2 = _v_dist_coefs[1];
  }

  std::vector<double> 
  getDistParameters() const
  {
    return {m_k1, m_k2};
  }

private:
  double m_k1=0., m_k2=0.;

  struct AutoDiffRad2Calib_Kernel
  {
    AutoDiffRad2Calib_Kernel(const double _u, const double _v,
                            const Eigen::Vector3d& _wpt)
        : m_px(_u,_v)
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

      Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

      // Compute reproj err
      const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
      const T invz = T(1. / campt[2]);

      const T x = T(campt[0]) * invz;
      const T y = T(campt[1]) * invz;

      const T x2 = x*x;
      const T y2 = y*y;

      const T r2 = x2 + y2;

      const T D = T(1.) + r2*(k1 + k2*r2);

      Eigen::Matrix<T,2,1> pred;
      pred << fx * x * D + cx,
              fy * y * D + cy;

      _err[0] = pred[0] - m_px.cast<T>()[0];
      _err[1] = pred[1] - m_px.cast<T>()[1];

      return true;
    }

    Eigen::Vector2d m_px;
    Eigen::Vector3d m_wpt;
  };

};
