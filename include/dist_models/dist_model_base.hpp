#pragma once

#include "ceres_params/se3_param.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

class DistParam
{
public:

  virtual ~DistParam() 
  {}

  virtual void displayParams() const = 0;

  virtual Eigen::Vector2d distortCamPoint(const double _x, const double _y) const = 0;
  virtual Eigen::Vector2d distortCamPoint(const Eigen::Vector2d& _cam_pt) const = 0;
  virtual Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const = 0;

  Eigen::Vector2d undistortCamPoint(const Eigen::Vector3d& _cam_pt) const
  {
    const double inv_z = 1. / _cam_pt[2];
    return undistortCamPoint(_cam_pt[0]*inv_z,_cam_pt[1]*inv_z);
  }

  Eigen::Vector2d undistortCamPoint(const Eigen::Vector2d& _cam_pt) const
  {
    return undistortCamPoint(_cam_pt[0],_cam_pt[1]);
  }

  Eigen::Vector2d undistortCamPoint(const double _campt_x, const double _campt_y) const
  {
    // Parameters for Newton iteration using numerical differentiation with
    // central differences, 100 iterations should be enough even for complex
    // camera models with higher order terms.
    const size_t kNumIterations = 100;
    const double kMaxStepNorm = 1e-10;
    const double kRelStepSize = 1e-6;

    const Eigen::Vector2d x0(_campt_x,_campt_y);
    Eigen::Vector2d x(x0);
    Eigen::Matrix2d J;

    for (size_t i = 0; i < kNumIterations; ++i) 
    {
      const double step0 = std::max(std::numeric_limits<double>::epsilon(),
                                    std::abs(kRelStepSize * x[0]));
      const double step1 = std::max(std::numeric_limits<double>::epsilon(),
                                    std::abs(kRelStepSize * x[1]));
      
      const Eigen::Vector2d dx = distortCamPoint(x[0],x[1]) - x;
      const Eigen::Vector2d dx0b = distortCamPoint(x[0]-step0,x[1]-step0) - x;
      const Eigen::Vector2d dx0f = distortCamPoint(x[0]+step0,x[1]+step0) - x;
      const Eigen::Vector2d dx1b = distortCamPoint(x[0]-step1,x[1]-step1) - x;
      const Eigen::Vector2d dx1f = distortCamPoint(x[0]+step1,x[1]+step1) - x;

      J(0, 0) = 1. + (dx0f[0] - dx0b[0]) / (2. * step0);
      J(0, 1) = (dx1f[0] - dx1b[0]) / (2. * step1);
      J(1, 0) = (dx0f[1] - dx0b[1]) / (2. * step0);
      J(1, 1) = 1. + (dx1f[1] - dx1b[1]) / (2. * step1);
      const Eigen::Vector2d step_x = J.inverse() * (x + dx - x0);
      x -= step_x;
      if (step_x.squaredNorm() < kMaxStepNorm) {
        break;
      }
    }

    return x;
  }

  virtual ceres::CostFunction*
  createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt, const bool _use_autodiff) const = 0;

  virtual void
  resetParameters(const std::vector<double> _v_dist_coefs) = 0;

  virtual std::vector<double> getDistParameters() const = 0;

  int getNumberOfParameters() const
  {
    return m_nb_params;
  }

protected:
  DistParam(const int _nb_params)
    : m_nb_params(_nb_params)
  {}

  int m_nb_params = -1;
};
