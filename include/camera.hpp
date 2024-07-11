#pragma once

#include "calib_models.hpp"
#include "dist_models/dist_models.hpp"

#include <iostream>
#include <vector>
#include <memory>

#include <sophus/se3.hpp>

#include <opencv2/core.hpp>

struct CalibFrame
{
  CalibFrame(const std::string& _img_name,
            const double _timestamp,
            const std::vector<cv::Point2f>& _v_corners_px,
            const Sophus::SE3d& _T_world_2_cam)
    : m_img_name(_img_name)
    , m_timestamp(_timestamp)
    , m_v_corners_px(std::move(_v_corners_px))
    , m_T_world_2_cam(_T_world_2_cam)
  {}

  std::string m_img_name = "";
  double m_timestamp = -1.;
  std::vector<cv::Point2f> m_v_corners_px;
  Sophus::SE3d m_T_world_2_cam;

  // Reproj err
  double rmse_err = -1.;
};

class Camera
{
  // EZCalibrator should process Camera objects!
public:
  Camera(const std::string& _config_file_path);

  Camera(const std::string& _input_images_path,
         const bool _use_mono_focal,
         const std::string& _dist_model, 
         const double _prior_fov);

  void setupInitialDistortion();
  void setupInitialCalibration(const cv::Size& _img_size);

  Eigen::Vector2d undistortPx(const cv::Point2f& _corner_pt) const;

  void displayCalibrationParameters() const;
  void displayIntrinsicParameters() const;
  void displayDistortionParameters() const;
  void displayCameraInfo() const;
  
  int m_id = -1;

  std::string m_input_images_path;

  std::unique_ptr<IntrinsicsParam> m_pcalib_params;
  std::unique_ptr<DistParam> m_pdist_params;

  std::vector<CalibFrame> m_v_calib_frames;

  Sophus::SE3d m_T_cam0_2_cam;

  bool m_use_mono_focal = true;
  std::string m_dist_model = "";
  double m_prior_fov_deg = -1.;
};
