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
  // EZMonoCalibrator should process Camera objects!
public:
  Camera(const std::string& _config_file_path);

  void setupInitialDistortion(const std::string& _dist_model);
  void setupInitialCalibration(const cv::Size& _img_size);

  void displayCalibrationParameters() const;

  int m_id = -1;

  std::string m_input_images_path;

  std::unique_ptr<IntrinsicsParam> m_pcalib_params;
  std::unique_ptr<DistParam> m_pdist_params;

  std::vector<CalibFrame> m_v_calib_frames;

  double m_prior_fov_deg = -1.;
};
