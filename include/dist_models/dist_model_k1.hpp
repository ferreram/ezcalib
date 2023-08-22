#pragma once

#include "dist_model_base.hpp"


class Rad1DistParam : public DistParam
{
public:

  Rad1DistParam()
    : DistParam(1)
  {}

  Rad1DistParam(const double _k1)
    : DistParam(1)
    , m_k1(_k1)
  {}

  void displayParams() const override
  {
    std::cout << "\n Distortion k1 : " << m_k1 << "\n\n";
  }

  Eigen::Vector2d distortCamPoint(const double _x, const double _y) const override
  {
    const double r2 = _x*_x + _y*_y;

    const double D = 1. + r2*m_k1;
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
  createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt, const bool _use_autodiff) const override
  {
    if (_use_autodiff)
    {
      return  new ceres::AutoDiffCostFunction<
                  AutoDiffRad1Calib_Kernel, 2, 2, 2, 1, 7>(
                      new AutoDiffRad1Calib_Kernel(_u, _v, _wpt));
    }

    return new ReprojectionError(_u, _v, _wpt);
  }
  
  void
  resetParameters(const std::vector<double> _v_dist_coefs) override
  {
    assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
    m_k1 = _v_dist_coefs[0];
  }
  
  std::vector<double> 
  getDistParameters() const
  {
    return {m_k1};
  }

private:
  double m_k1 = 0.;

  struct AutoDiffRad1Calib_Kernel
  {
    AutoDiffRad1Calib_Kernel(const double _u, const double _v,
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

      Eigen::Map<const Sophus::SE3<T>> Tcw(_Tcw_param);

      // Compute reproj err
      const Eigen::Matrix<T,3,1> campt = Tcw * m_wpt;
      const T invz = T(1. / campt[2]);

      const T x = T(campt[0]) * invz;
      const T y = T(campt[1]) * invz;

      const T x2 = x*x;
      const T y2 = y*y;

      const T r2 = x2 + y2;

      const T D = T(1.) + r2*k1;

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

  class ReprojectionError : public ceres::SizedCostFunction<2, 2, 2, 1, 7>
  {
  public:
    ReprojectionError(const double _u, const double _v,
                      const Eigen::Vector3d& _wpt)
        : m_px(_u,_v)
        , m_wpt(_wpt)
    {}

    // virtual bool Evaluate(double const *const *parameters,
    //                       double *residuals,
    //                       double **jacobians) const;

    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const
    {
      // Focal, Principal Point, Dist
      const double fx = parameters[0][0];
      const double fy = parameters[0][1];

      const double cx = parameters[1][0];
      const double cy = parameters[1][1];

      const double k1 = parameters[2][0];

      Eigen::Map<const Sophus::SE3d> Tcw(parameters[3]);

      // Compute reproj err
      const Eigen::Vector3d campt = Tcw * m_wpt;
      const double invz = 1. / campt[2];

      const double x = campt[0] * invz;
      const double y = campt[1] * invz;

      const double x2 = x*x;
      const double y2 = y*y;

      const double r2 = x2 + y2;

      const double D = 1. + r2*k1;

      const Eigen::Vector2d pred(fx * x * D + cx, 
                                 fy * y * D + cy);
      
      Eigen::Map<Eigen::Vector2d> err(residuals);

      err = pred - m_px;

      if (jacobians != nullptr)
      {
        if (jacobians[0] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J_focal(jacobians[0]);
          J_focal.setZero();

          J_focal(0,0) = x * D;
          J_focal(1,1) = y * D;
        }
        if (jacobians[1] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J_pp(jacobians[1]);
          J_pp.setIdentity();
        }
        if (jacobians[2] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2, 1>> J_dist(jacobians[2]);

          // k1
          J_dist <<  fx * x * r2,
                     fy * y * r2;
        }
        if (jacobians[3] != nullptr)
        {
          Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_se3pose(jacobians[3]);
          J_se3pose.setZero();

          const double invz2 = invz * invz;

          Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_proj;
          
          J_proj << invz, 0., -campt[0] * invz2, 
                    0., invz, -campt[1] * invz2;

          Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J_cam_dist;

          const double dr2_dx = 2.*x;
          const double dr2_dy = 2.*y;

          J_cam_dist << D + x * dr2_dx * k1, x * dr2_dy * k1,
                        y * dr2_dx * k1    , D + y * dr2_dy * k1;

          Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J_cam_calib;
          
          J_cam_calib << fx, 0.,
                         0., fy;

          Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam_proj;
          J_cam_proj.noalias() = J_cam_calib * J_cam_dist * J_proj;

          // Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam_proj;
          // J_cam_proj << invz * fx, 0., -campt[0] * invz2 * fx,
          //               0., invz * fy, -campt[1] * invz2 * fy;

          // The Jacobian is set w.r.t. to the 6D delta_param which starts with the trans coefs
          J_se3pose.block<2, 3>(0, 0) = J_cam_proj;
          J_se3pose.block<2, 3>(0, 3).noalias() = -1. * J_cam_proj * Sophus::SO3d::hat(campt);
        }
      }

      return true;
    }

    private:
      Eigen::Vector2d m_px;
      Eigen::Vector3d m_wpt;
  };
};
