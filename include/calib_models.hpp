#pragma once

// #include "dist_models/dist_models.hpp"

#include <iostream>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>

#include <sophus/se3.hpp>

class IntrinsicsParam
{
public:
  IntrinsicsParam(const double _img_width,
                  const double _img_height,
                  const double _fx, const double _fy,
                  const double _cx, const double _cy)
    : m_img_width(_img_width), m_img_height(_img_height)
    , m_fx(_fx), m_fy(_fy), m_cx(_cx), m_cy(_cy)
    , m_fx_std(0.), m_fy_std(0.), m_cx_std(0.), m_cy_std(0.)
    , m_use_mono_focal(false)
  {
    setCalibMatrix();
  }

  IntrinsicsParam(const double _img_width,
                  const double _img_height,
                  const double _f,
                  const double _cx, const double _cy,
                  const bool _use_mono_focal = true)
    : m_img_width(_img_width), m_img_height(_img_height)
    , m_fx(_f), m_fy(_f), m_cx(_cx), m_cy(_cy)
    , m_fx_std(0.), m_fy_std(0.), m_cx_std(0.), m_cy_std(0.)
    , m_use_mono_focal(_use_mono_focal)
  {
    setCalibMatrix();
  }

  void displayParams() const
  {
    if (m_use_mono_focal)
      std::cout << "\n Focal: " << m_fx;
    else
      std::cout << "\n Focal: " << m_fx << " / " << m_fy;
      
    std::cout << "\n PrincipalPoint: " << m_cx << " / " << m_cy << "\n\n";
  }

  void displayParamsWithStd() const
  {
    if (m_fx_std == 0. && m_fy_std == 0. && m_cx_std == 0. && m_cy_std == 0.)
    {
      displayParams();
      return;
    }

    if (m_use_mono_focal)
      std::cout << "\n Focal: " << m_fx << " +/- " << m_fx_std;
    else
      std::cout << "\n Focal: " << m_fx << " / " << m_fy << " +/- " << m_fx_std << " / " << m_fy_std;
      
    std::cout << "\n PrincipalPoint: " << m_cx << " / " << m_cy << " +/- " << m_cx_std << " / " << m_cy_std << "\n\n";
  }

  Eigen::Vector3d projImageToCam(const Eigen::Vector2d& _px) const
  {
    return m_invK * _px.homogeneous();
  }

  Eigen::Vector3d projImageToCam(const double _u, const double _v) const
  {
    return projImageToCam(Eigen::Vector2d(_u,_v));
  }

  Eigen::Vector2d projCamToImage(const Eigen::Vector3d& _un_cam_pt) const
  {
    const double inv_z = 1. / _un_cam_pt[2];
    return projCamToImage(Eigen::Vector2d(_un_cam_pt[0]*inv_z,
                                          _un_cam_pt[1]*inv_z));
  }

  Eigen::Vector2d projCamToImage(const Eigen::Vector2d& _un_cam_pt_norm) const
  {
    return Eigen::Vector2d(m_fx*_un_cam_pt_norm[0] + m_cx, 
                           m_fy*_un_cam_pt_norm[1] + m_cy);
  }

  bool isPointInImage(const Eigen::Vector2d& _px) const 
  {
    return isPointInImage(_px[0], _px[1]);
  }

  bool isPointInImage(const double _u, const double _v) const 
  {
    // The 0.5 offset is due to OpenCV coords,
    // which are (0.,0.) at center of the top left-pixel
    static const double img_cols_border = static_cast<double>(m_img_width) - 0.5;
    static const double img_rows_border = static_cast<double>(m_img_height) - 0.5;

    return (_u > -0.5 && _v > -0.5 && _u < img_cols_border && _v < img_rows_border);
  }

  bool isPointInImage(const float _u, const float _v) const 
  {
    // The 0.5 offset is due to OpenCV coords,
    // which are (0.,0.) at center of the top left-pixel
    static const float img_cols_border = static_cast<float>(m_img_width) - 0.5f;
    static const float img_rows_border = static_cast<float>(m_img_height) - 0.5f;

    return (_u > -0.5f && _v > -0.5f && _u < img_cols_border && _v < img_rows_border);
  }

  void resetParameters(const std::vector<double> _vfocal,
                       const std::vector<double> _vprincipal_point)
  {
    if (m_use_mono_focal)
    {
      m_fx = _vfocal.at(0); m_fy = _vfocal.at(0);
    }
    else
    {
      m_fx = _vfocal.at(0); m_fy = _vfocal.at(1);
    }
    
    m_cx = _vprincipal_point.at(0); 
    m_cy = _vprincipal_point.at(1);
    
    setCalibMatrix();
  }

  void resetParameters(const double _fx, const double _fy,
                       const double _cx, const double _cy)
  {
    m_fx = _fx; m_fy = _fy;
    m_cx = _cx; m_cy = _cy;
    
    setCalibMatrix();
  }

  void resetParameters(const double _f, const double _cx, const double _cy)
  {
    m_fx = _f; m_fy = _f;
    m_cx = _cx; m_cy = _cy;
    
    setCalibMatrix();
  }

  std::vector<double> getFocal() const
  {
    if (m_use_mono_focal) 
      return {m_fx};

    return {m_fx, m_fy};
  }

  std::vector<double> getPrincipalPoint() const
  {
    return {m_cx, m_cy};
  }

  double getMeanFocal() const
  {
    if (m_use_mono_focal)
      return m_fx;
    
    return (m_fx + m_fy) * 0.5;
  }

  std::vector<double> getParameters() const
  {
    return {m_fx, m_fy, m_cx, m_cy};
  }

  std::vector<double> getParametersStd() const
  {
    return {m_fx_std, m_fy_std, m_cx_std, m_cy_std};
  }

  void setFocalStd(const double _cov_f)
  {
    m_fx_std = std::sqrt(_cov_f);
    m_fy_std = m_fx_std;
  }

  void setFocalStd(const double _cov_fx, const double _cov_fy)
  {
    m_fx_std = std::sqrt(_cov_fx);
    m_fy_std = std::sqrt(_cov_fy);
  }

  void setPPStd(const double _cov_cx, const double _cov_cy)
  {
    m_cx_std = std::sqrt(_cov_cx);
    m_cy_std = std::sqrt(_cov_cy);
  }
  
  void setFocalStd(const std::vector<double>& _cov_focal)
  {
    m_fx_std = std::sqrt(_cov_focal[0]);
    if (!m_use_mono_focal && _cov_focal.size() > 1)
    {
      m_fy_std = std::sqrt(_cov_focal[3]);
    }
    else
    {
      m_fy_std = m_fx_std;
    }
  }

  void setPPStd(const std::vector<double>& _cov_pp)
  {
    m_cx_std = std::sqrt(_cov_pp[0]);
    m_cy_std = std::sqrt(_cov_pp[3]);
  }

  double m_img_width, m_img_height;

  double m_fx, m_fy;
  double m_cx, m_cy;

  double m_fx_std, m_fy_std;
  double m_cx_std, m_cy_std;

  bool m_use_mono_focal;

private:
  void setCalibMatrix()
  {
    m_K << m_fx, 0., m_cx, 
           0., m_fy, m_cy, 
           0., 0., 1.;

    m_invK = m_K.inverse();
  }
  
  Eigen::Matrix3d m_K, m_invK;
};
