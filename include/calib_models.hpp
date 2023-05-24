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
  {
    setCalibMatrix();
  }

  void displayParams() const
  {
    std::cout << "\n Focal fx / fy : " << m_fx << " / " << m_fy;
    std::cout << "\n PrincipalPoint cx / cy : " << m_cx << " / " << m_cy << "\n\n";
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
    static const double img_cols_border = m_img_width - 0.5;
    static const double img_rows_border = m_img_height - 0.5;

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

  void resetParameters(const double _fx, const double _fy,
                       const double _cx, const double _cy)
  {
    m_fx = _fx; m_fy = _fy;
    m_cx = _cx; m_cy = _cy;
    
    setCalibMatrix();
  }

  std::vector<double> getParameters()
  {
    return {m_fx, m_fy, m_cx, m_cy};
  }

  double m_img_width, m_img_height;

  double m_fx, m_fy;
  double m_cx, m_cy;

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
