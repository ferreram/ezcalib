#pragma once

#include "calib_models.hpp"
#include "dist_models/dist_models.hpp"

#include "target_detectors/aprilgrid_detector.hpp"
#include "target_detectors/chessboard_detector.hpp"

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
};


class EZMonoCalibrator
{
public:
  EZMonoCalibrator(const std::string& _config_file_path);

  EZMonoCalibrator(const int _dist_model,
                  const int _calib_tgt_nrows, 
                  const int _calib_tgt_ncols,
                  const double _calib_tgt_square_size,
                  const double _prior_fov_deg = 70.,
                  const bool _debug_display = false);
  
  ~EZMonoCalibrator();

  bool processImage(const cv::Mat& _in_img, const std::string &_img_name = "", const double _timestamp = -1.);

  void computeCalibration();

  void computeStereoCalibration();

  void displayCalibrationParameters()
  {
    std::cout << "\nIntrinsic Parameters:\n";
    m_pcalib_params->displayParams();

    std::cout << "\nDistortion Parameters:\n";
    m_pdist_params->displayParams();
  }

private:
  void setupCalibrationProblem(const std::string& _config_file_path);
  void setupInitialDistortion(const std::string& _dist_model);
  void setupInitialCalibration(const cv::Size& _img_size);

  bool computeP3PRansac(const std::vector<cv::Point2f>& _v_px_obs,
                        const std::vector<Eigen::Vector3d>& _v_wpts,
                        const IntrinsicsParam& _calib_params,
                        Sophus::SE3d& _T_world_2_cam,
                        std::vector<int>& _v_inliers);

  // CalibTarget m_calib_target;
  std::vector<CalibFrame> m_v_calib_frames;

  double m_prior_fov_deg;

  std::unique_ptr<IntrinsicsParam> m_pcalib_params;
  std::unique_ptr<DistParam> m_pdist_params;

  std::unique_ptr<CalibDetector> m_pcalib_detector;
  
  bool m_debug_display;
};
