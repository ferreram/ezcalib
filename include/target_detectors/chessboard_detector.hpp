#pragma once

#include "base_detector.hpp"

#include <opencv2/calib3d.hpp>

class ChessboardCalibDetector : public CalibDetector
{
public:
  ChessboardCalibDetector(const int _nb_rows, 
                          const int _nb_cols,
                          const double _square_size,
                          const bool _debug_display)
    : CalibDetector(_debug_display)
    , m_nb_rows(_nb_rows)
    , m_nb_cols(_nb_cols)
    , m_nb_tags(_nb_rows*_nb_cols)
    , m_square_size(_square_size)
    , m_cv_board_pattern_size(_nb_cols-1,_nb_rows-1)
  {
    assert(m_nb_rows > 0 && m_nb_cols > 0);
    assert(m_square_size > 0.);

    setupCalibTarget();
  }

  bool 
  detectTarget(const cv::Mat& _img, std::vector<cv::Point2f>& _v_corners_pts)
  {
    const bool success = cv::findChessboardCorners(
                                  _img, 
                                  m_cv_board_pattern_size, 
                                  _v_corners_pts, 
                                  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                  + cv::CALIB_CB_FILTER_QUADS + cv::CALIB_CB_FAST_CHECK);
    
    if (success)
    {
      static const cv::TermCriteria TERM_CRIT(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.05);
      cv::cornerSubPix(_img, _v_corners_pts, cv::Size(5,5), cv::Size(-1,-1), TERM_CRIT);
    }

    if (m_debug_display)
    {
      debugDisplay(_img, _v_corners_pts, success);
    }


    return success;
  }

  const std::vector<Eigen::Vector3d>& 
  getTargetCoords() const
  {
    return m_vtgt_coords;
  }

  void
  displayInfo() const
  {
    std::cout << "\n====================================\n";
    std::cout << "\tProvided Calibration problem:\n";
    std::cout << "\n- Target type: Chessboard";
    std::cout << "\n- Number of rows: " << m_nb_rows;
    std::cout << "\n- Number of cols: " << m_nb_cols;
    std::cout << "\n- Size of a square: " << m_square_size << " m\n";
    std::cout << "\n====================================\n";
  }

private:
  void 
  setupCalibTarget()
  {
    m_vtgt_coords.reserve((m_nb_rows-1)*(m_nb_cols-1));

    // Following the corners ordering of OpenCV findChessboardCorners()
    for (int r = 1; r < m_nb_rows; ++r)
    {
      for (int c = m_nb_cols-1; c > 0; --c)
      {
        m_vtgt_coords.push_back(Eigen::Vector3d(-c*m_square_size, r*m_square_size, 1.));
      }
    }
  }

  void debugDisplay(const cv::Mat &_img, 
                    const std::vector<cv::Point2f>& _v_corners_pts, 
                    const bool _success) const
  {
    cv::Mat img_2_draw;
    if (_img.channels() > 1)
    {
      img_2_draw = _img;
    }
    else
    {
      cv::cvtColor(_img, img_2_draw, cv::COLOR_GRAY2BGR);
    }
    cv::drawChessboardCorners(img_2_draw, 
                              m_cv_board_pattern_size, 
                              _v_corners_pts, 
                              _success);
    cv::imshow(m_debug_win_name, img_2_draw);
    cv::waitKey(500);
  }

  int m_nb_rows = 0;
  int m_nb_cols = 0;
  int m_nb_tags = 0;

  double m_square_size = 0.;
  cv::Size m_cv_board_pattern_size;

  std::vector<Eigen::Vector3d> m_vtgt_coords;
};