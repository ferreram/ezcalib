#pragma once

#include "camera.hpp"

#include "target_detectors/aprilgrid_detector.hpp"
#include "target_detectors/chessboard_detector.hpp"


class EZCalibrator
{
public:
  EZCalibrator(const std::string& _config_file_path);

  void runCalibration(Camera* _pcamera);

  void runMultiCameraCalib(std::vector<Camera>& _v_cameras);

private:

  bool processImage(const cv::Mat& _in_img, const std::string &_img_name = "", const double _timestamp = -1.);

  void computeCalibration();
  void refineCalibration();

  void setupCalibrationProblem(const std::string& _config_file_path);

  bool computeP3PRansac(const std::vector<cv::Point2f>& _v_px_obs,
                        const std::vector<Eigen::Vector3d>& _v_wpts,
                        const IntrinsicsParam& _calib_params,
                        Sophus::SE3d& _T_world_2_cam,
                        std::vector<int>& _v_inliers);

  Camera* m_pcamera = nullptr;

  std::unique_ptr<CalibDetector> m_pcalib_detector;
  
  bool m_debug_display;
};
