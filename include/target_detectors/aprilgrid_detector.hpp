#pragma once

#include "base_detector.hpp"

#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>

class AprilTagCalibDetector : public CalibDetector
{
public:
  AprilTagCalibDetector(const int _nb_rows, 
                        const int _nb_cols,
                        const double _tag_size,
                        const double _tag_space,
                        const bool _debug_display)
    : CalibDetector(_debug_display)
    , m_detector(AprilTags::tagCodes36h11)
    , m_nb_rows(_nb_rows)
    , m_nb_cols(_nb_cols)
    , m_nb_tags(_nb_rows*_nb_cols)
    , m_tag_size(_tag_size)
    , m_tag_space(_tag_space)
  {
    assert(m_nb_rows > 0 && m_nb_cols > 0);
    assert(m_tag_size > 0. && m_tag_space > 0.);

    setupCalibTarget();
  }

  bool detectTarget(const cv::Mat& _img, std::vector<cv::Point2f>& _v_corners_pts)
  {
    const std::vector<AprilTags::TagDetection> vdet = m_detector.extractTags(_img);

    std::vector<cv::Point2f> vcorners(4*m_nb_tags, cv::Point2f(-1.f,-1.f));

    for (const auto& det : vdet)
    {
      // Filter non existing tags
      if (det.id >= m_nb_tags)
      {
        continue;
      }

      std::vector<cv::Point2f> vraw_corners;
      vraw_corners.reserve(4ul);
      for (size_t i=0ul; i < 4ul; ++i)
      {
        vraw_corners.push_back(cv::Point2f(det.p[i].first,det.p[i].second));
      }

      static const cv::TermCriteria TERM_CRIT(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 0.05);
      cv::cornerSubPix(_img, vraw_corners, cv::Size(5,5), cv::Size(-1,-1), TERM_CRIT);

      for (size_t i=0ul; i < 4ul; ++i)
      {
        vcorners[det.id*4ul+i] = vraw_corners[i];
      }
    }

    static const size_t success_threshold = (m_nb_tags / 2) - 1;

    if (m_debug_display)
    {
      debugDisplay(_img, vdet, vdet.size() > success_threshold);
    }

    _v_corners_pts.swap(vcorners);

    return vdet.size() > success_threshold;
  }

  const std::vector<Eigen::Vector3d>& getTargetCoords()
  {
    return m_vtgt_coords;
  }

private:
  void setupCalibTarget()
  {
    m_vtgt_coords.reserve(4*m_nb_tags);

    const double tag_spacing = m_tag_size * m_tag_space;
    const double tags_offset = m_tag_size + tag_spacing;

    // Following the corners ordering of AprilTag Lib 
    // (start at bottom-left and then go counter clock-wise till top-left)
    const Eigen::Vector3d corner1(0., 0., 1.);
    const Eigen::Vector3d corner2(m_tag_size, 0., 1.);
    const Eigen::Vector3d corner3(m_tag_size, -m_tag_size, 1.);
    const Eigen::Vector3d corner4(0., -m_tag_size, 1.);

    for (int r = 0; r < m_nb_rows; ++r)
    {
      for (int c = 0; c < m_nb_cols; ++c)
      {
        const Eigen::Vector3d corners_offset(c*tags_offset,-r*tags_offset,0.);
        m_vtgt_coords.push_back(corner1 + corners_offset);
        m_vtgt_coords.push_back(corner2 + corners_offset);
        m_vtgt_coords.push_back(corner3 + corners_offset);
        m_vtgt_coords.push_back(corner4 + corners_offset);
      }
    }
  }

  void debugDisplay(const cv::Mat &_img, 
                    const std::vector<AprilTags::TagDetection> &_vdet, 
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
    
    for (const auto& det : _vdet)
    {
      if (!_success)
      {
        cv::putText(img_2_draw, 
                    "Fail to detect enough tags!", 
                    cv::Point2f(50.f,50.f), 
                    cv::FONT_HERSHEY_SIMPLEX, 
                    1, 
                    cv::Scalar(0,0,255), 
                    3); 
      }

      cv::putText(img_2_draw, 
                  std::to_string(det.id), 
                  cv::Point2f(det.cxy.first,det.cxy.second), 
                  cv::FONT_HERSHEY_SIMPLEX, 
                  1, 
                  cv::Scalar(0,0,255), 
                  3);
      
      static std::vector<cv::Scalar> vcols;
      if (vcols.empty())
      {
        vcols.push_back(cv::Scalar(255,0,0));
        vcols.push_back(cv::Scalar(0,255,0));
        vcols.push_back(cv::Scalar(0,0,255));
        vcols.push_back(cv::Scalar(128,32,255));
      }

      for (size_t i=0ul; i < 4ul; ++i)
      {
        cv::circle(img_2_draw, cv::Point2f(det.p[i].first,det.p[i].second), 5, vcols[i], -1);
      }
    }

    cv::imshow(m_debug_win_name, img_2_draw);
    cv::waitKey(500);
  }

  AprilTags::TagDetector m_detector;

  int m_nb_rows = 0;
  int m_nb_cols = 0;
  int m_nb_tags = 0;

  const double m_tag_size = 0.; // Size of a tag
  const double m_tag_space = 0.; // Ratio of space between two tags (tag_space = tag_size / space_size)

  std::vector<Eigen::Vector3d> m_vtgt_coords;
};