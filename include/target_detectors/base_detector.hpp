#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

class CalibDetector
{
public:

  virtual ~CalibDetector() 
  {
    if (m_debug_display)
    {
      cv::destroyWindow(m_debug_win_name);
    }
  }

  virtual bool detectTarget(const cv::Mat& _img, std::vector<cv::Point2f>& _v_corners_pts) = 0;

  virtual const std::vector<Eigen::Vector3d>& getTargetCoords() const = 0;

protected:
  CalibDetector(const bool _debug_display)
    : m_debug_display(_debug_display)
  {
    if (m_debug_display)
    {
      cv::namedWindow(m_debug_win_name, cv::WINDOW_AUTOSIZE);
    }
  }

  virtual void setupCalibTarget() = 0;

  bool m_debug_display = false;
  std::string m_debug_win_name = "Debug Calib Detector";
};
