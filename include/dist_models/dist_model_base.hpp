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

  virtual Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const = 0;

  virtual ceres::CostFunction*
  createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const = 0;

  virtual void
  resetParameters(const std::vector<double> _v_dist_coefs) = 0;

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
