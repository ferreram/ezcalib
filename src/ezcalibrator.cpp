#include "ezcalibrator.hpp"

#include <set>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;


EZCalibrator::EZCalibrator(const std::string& _config_file_path)
{
  setupCalibrationProblem(_config_file_path);
}

void 
EZCalibrator::runCalibration(Camera* _pcamera)
{
  // Set m_pcamera to point on current camera to calibrate
  m_pcamera = _pcamera; 

  // Get image files in ascending order
  std::set<fs::path> set_ordered_images;

  for (const auto& file_path : fs::directory_iterator(m_pcamera->m_input_images_path))
  {
    const std::string& file_ext = file_path.path().extension();
    if (file_ext == ".png" || file_ext == ".jpg" || file_ext == ".jpeg" || file_ext == ".PNG" || file_ext == ".JPG" || file_ext == ".JPEG")
    {
      set_ordered_images.insert(file_path);
    }
  }

  // Process every images
  int nb_consec_bad_img = 0;
  for (auto it = set_ordered_images.begin(); it != set_ordered_images.end(); ++it)
  {
    std::cout << "\nAdding image : " << it->c_str() << "\n";

    const cv::Mat gray_img = cv::imread(it->c_str(), cv::IMREAD_GRAYSCALE);

    if (processImage(gray_img, it->filename()))
    {
      nb_consec_bad_img = 0;
    }
    else
    {
      ++nb_consec_bad_img;
      for (int i=0; i < nb_consec_bad_img / 2; ++i)
      {
        ++it;
      }
      std::cout << "\nNo target detected!  Going to skip the next " << nb_consec_bad_img / 2 << " images for speed-up\n";
    }
  }
  
  // Calibrate
  computeCalibration();
  // refineCalibration();

  // Leave Camera pointer
  m_pcamera = nullptr;
}

bool 
EZCalibrator::processImage(const cv::Mat& _in_img, const std::string &_img_name /*= ""*/, const double _timestamp /*= -1.*/)
{
  if (_in_img.empty())
    return false;

  if (m_pcamera->m_pcalib_params == nullptr)
  {
    m_pcamera->setupInitialCalibration(_in_img.size());
  }

  std::vector<cv::Point2f> v_corners_pts;

  const bool success = m_pcalib_detector->detectTarget(_in_img, v_corners_pts);
  
  if (success)
  {
    Sophus::SE3d T_world_2_cam;
    std::vector<int> vinliers;

    const bool p3p_success = computeP3PRansac(v_corners_pts, 
                                              m_pcalib_detector->getTargetCoords(), 
                                              *m_pcamera->m_pcalib_params, 
                                              T_world_2_cam,
                                              vinliers);

    if (p3p_success)
    {
      m_pcamera->m_v_calib_frames.emplace_back(_img_name, _timestamp, v_corners_pts, T_world_2_cam);
    }
  }

  return success;
}


void
EZCalibrator::computeCalibration()
{
  // Setup Calibration Optimization Problem!
  // =======================================
  ceres::Problem problem;
  ceres::LossFunctionWrapper* loss_function;
  loss_function = new ceres::LossFunctionWrapper(new ceres::CauchyLoss(1.0), ceres::TAKE_OWNERSHIP);

  // Setup Intrinsic & Distortion parameters
  std::vector<double> opt_focal_param = m_pcamera->m_pcalib_params->getFocal();
  std::vector<double> opt_pp_param = m_pcamera->m_pcalib_params->getPrincipalPoint();
  std::vector<double> opt_dist_param = m_pcamera->m_pdist_params->getDistParameters();
  
  // Setup Intrinsic & Distortion parameters
  problem.AddParameterBlock(opt_focal_param.data(), opt_focal_param.size());
  problem.AddParameterBlock(opt_pp_param.data(), opt_pp_param.size());
  problem.AddParameterBlock(opt_dist_param.data(), opt_dist_param.size());

  const std::vector<Eigen::Vector3d> &v_tgt_coords = m_pcalib_detector->getTargetCoords();

  // Setup residuals
  const size_t nb_views = m_pcamera->m_v_calib_frames.size();
  const size_t nb_residuals = nb_views * v_tgt_coords.size();

  std::vector<double> vini_err;
  vini_err.reserve(nb_residuals);

  for (auto& calib_frame : m_pcamera->m_v_calib_frames)
  {
    // auto to handle LocalParametrization / Manifold changes in Ceres
    auto *local_param = new AutoDiffLocalLeftSE3();
    problem.AddParameterBlock(calib_frame.m_T_world_2_cam.data(), 7, local_param);

    for (size_t j = 0ul; j < v_tgt_coords.size(); ++j)
    {
      if (!m_pcamera->m_pcalib_params->isPointInImage(calib_frame.m_v_corners_px[j].x, 
                                                      calib_frame.m_v_corners_px[j].y))
      {
        continue;
      }

      ceres::CostFunction* f = 
          m_pcamera->m_pdist_params->createCeresCostFunction(calib_frame.m_v_corners_px[j].x, 
                                                             calib_frame.m_v_corners_px[j].y,
                                                             v_tgt_coords[j]);

      problem.AddResidualBlock(
                  f, loss_function,
                  opt_focal_param.data(), 
                  opt_pp_param.data(),
                  opt_dist_param.data(),
                  calib_frame.m_T_world_2_cam.data());

      const Eigen::Vector2d px_obs(calib_frame.m_v_corners_px[j].x,calib_frame.m_v_corners_px[j].y);
      const Eigen::Vector2d dist_cam_pt = m_pcamera->m_pdist_params->distortCamPoint(calib_frame.m_T_world_2_cam * v_tgt_coords[j]);
      const Eigen::Vector2d px_proj = m_pcamera->m_pcalib_params->projCamToImage(dist_cam_pt);

      vini_err.push_back((px_obs - px_proj).norm());
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  options.num_threads = 8;
  options.max_num_iterations = 1000;

  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  
  std::cout << "\n\n=========================================\n";
  std::cout << "     Ceres based Calibration Optimization";
  std::cout << "\n=========================================\n";

  std::cout << "\n0. Initial Parameters to Optimize";
  std::cout << "\n=================================\n";
  
  m_pcamera->m_pcalib_params->displayParamsWithStd();
  m_pcamera->m_pdist_params->displayParamsWithStd();

  std::cout << "\n\n1. Optimize Focal & Camera Poses";
  std::cout << "\n=================================\n";

  // Optimize Focal & Poses only first
  problem.SetParameterBlockConstant(opt_pp_param.data());
  problem.SetParameterBlockConstant(opt_dist_param.data());

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;
  
  m_pcamera->m_pcalib_params->resetParameters(opt_focal_param, opt_pp_param);
  m_pcamera->m_pdist_params->resetParameters(opt_dist_param);

  m_pcamera->m_pcalib_params->displayParamsWithStd();
  m_pcamera->m_pdist_params->displayParamsWithStd();

  std::cout << "\n\n2. Optimize Focal & Distortion & Camera Poses";
  std::cout << "\n==============================================\n";

  problem.SetParameterBlockVariable(opt_dist_param.data());
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  m_pcamera->m_pcalib_params->resetParameters(opt_focal_param, opt_pp_param);
  m_pcamera->m_pdist_params->resetParameters(opt_dist_param);

  m_pcamera->m_pcalib_params->displayParamsWithStd();
  m_pcamera->m_pdist_params->displayParamsWithStd();
  
  std::cout << "\n\n3. Optimize Focal & Principal Point & Distortion & Camera Poses";
  std::cout << "\n================================================================\n";

  problem.SetParameterBlockVariable(opt_pp_param.data());
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  m_pcamera->m_pcalib_params->resetParameters(opt_focal_param, opt_pp_param);
  m_pcamera->m_pdist_params->resetParameters(opt_dist_param);

  std::vector<double> vopt_err;
  vopt_err.reserve(vini_err.size());

  std::vector<double> vfinal_err;
  vfinal_err.reserve(vini_err.size());

  for (auto& calib_frame : m_pcamera->m_v_calib_frames)
  {
    const std::vector<cv::Point2f>& v_corners_pts = calib_frame.m_v_corners_px;

    double rmse_err = 0.;
    int nb_err = 0;

    for (size_t j = 0ul; j < v_tgt_coords.size(); ++j)
    {
      if (!m_pcamera->m_pcalib_params->isPointInImage(v_corners_pts[j].x,
                                                      v_corners_pts[j].y))
      {
        continue;
      }

      const Eigen::Vector2d px_obs(calib_frame.m_v_corners_px[j].x,calib_frame.m_v_corners_px[j].y);
      const Eigen::Vector2d dist_cam_pt = m_pcamera->m_pdist_params->distortCamPoint(calib_frame.m_T_world_2_cam * v_tgt_coords[j]);
      const Eigen::Vector2d px_proj = m_pcamera->m_pcalib_params->projCamToImage(dist_cam_pt);

      vopt_err.push_back((px_obs - px_proj).norm());

      rmse_err += vopt_err.back() * vopt_err.back();
      ++nb_err;

      if (vopt_err.back() < 1.)
      {
        vfinal_err.push_back(vopt_err.back());
      }
    }

    if (nb_err > 0)
    {
      calib_frame.rmse_err = std::sqrt(rmse_err / static_cast<double>(nb_err));
    }
    else
    {
      calib_frame.rmse_err = std::numeric_limits<double>::max();
    }
  }

  cv::Scalar vini_err_mean, vini_err_std;
  cv::meanStdDev(vini_err, vini_err_mean, vini_err_std);

  cv::Scalar vopt_err_mean, vopt_err_std;
  cv::meanStdDev(vopt_err, vopt_err_mean, vopt_err_std);

  cv::Scalar vfinal_err_mean, vfinal_err_std;
  cv::meanStdDev(vfinal_err, vfinal_err_mean, vfinal_err_std);

  std::sort(vini_err.begin(), vini_err.end());
  std::sort(vopt_err.begin(), vopt_err.end());
  std::sort(vfinal_err.begin(), vfinal_err.end());

  std::cout << "\n\nIni Obs reproj error (Mean / Std / Med) : " << vini_err_mean[0] << " / " << vini_err_std[0] << " / " << vini_err[vini_err.size() / 2] << "\n\n";;
  std::cout << "Opt Obs reproj error (Mean / Std / Med) : " << vopt_err_mean[0] << " / " << vopt_err_std[0] << " / " << vopt_err[vopt_err.size() / 2] << "\n\n";
  std::cout << "Final Obs reproj error (Mean / Std / Med) : " << vfinal_err_mean[0] << " / " << vfinal_err_std[0] << " / " << vfinal_err[vfinal_err.size() / 2] << "\n\n";

  std::vector<const double*> v_params;
  v_params.reserve(opt_focal_param.size()+opt_pp_param.size()+opt_dist_param.size());
  
  v_params.push_back(opt_focal_param.data());
  v_params.push_back(opt_pp_param.data());
  v_params.push_back(opt_dist_param.data());

  ceres::Covariance::Options cov_options;
  ceres::Covariance ceres_covariance(cov_options);
  const bool cov_success = ceres_covariance.Compute(v_params, &problem);

  if (cov_success)
  {
    std::vector<double> vcov_focal(opt_focal_param.size()*opt_focal_param.size());
    std::vector<double> vcov_pp(opt_pp_param.size()*opt_pp_param.size());
    std::vector<double> vcov_dist(opt_dist_param.size()*opt_dist_param.size());

    ceres_covariance.GetCovarianceBlock(opt_focal_param.data(),opt_focal_param.data(),vcov_focal.data());
    ceres_covariance.GetCovarianceBlock(opt_pp_param.data(),opt_pp_param.data(),vcov_pp.data());
    ceres_covariance.GetCovarianceBlock(opt_dist_param.data(),opt_dist_param.data(),vcov_dist.data());

    m_pcamera->m_pcalib_params->setFocalStd(vcov_focal);
    m_pcamera->m_pcalib_params->setPPStd(vcov_pp);
    m_pcamera->m_pdist_params->setDistParamsStd(vcov_dist);

    const Eigen::Map<Eigen::MatrixXd> cov_focal_mat(vcov_focal.data(),opt_focal_param.size(),opt_focal_param.size());
    const Eigen::Map<Eigen::Matrix2d> cov_pp_mat(vcov_pp.data());
    const Eigen::Map<Eigen::MatrixXd> cov_dist_mat(vcov_dist.data(),opt_dist_param.size(),opt_dist_param.size());

    // std::cout << "\n\nCovariance on focal : \n" << cov_focal_mat;
    // std::cout << "\n\nCovariance on PP : \n" << cov_pp_mat;
    // std::cout << "\n\nCovariance on dist : \n" << cov_dist_mat;
    // std::cout << "\n\n";
  }
  else
  {
    std::cout << "\nCovariance Estimation Failed!\n";
    std::cout << "The Jacobian is most likely rank deficient!\n\n";
  }

  m_pcamera->m_pcalib_params->displayParamsWithStd();
  m_pcamera->m_pdist_params->displayParamsWithStd();

  std::cout << "\n\nDone!\n\n";
  
  return;
}


void
EZCalibrator::refineCalibration()
{
  // Setup Calibration Optimization Problem!
  // =======================================
  ceres::Problem problem;
  ceres::LossFunctionWrapper* loss_function;
  loss_function = new ceres::LossFunctionWrapper(new ceres::CauchyLoss(1.0), ceres::TAKE_OWNERSHIP);

  // Setup Intrinsic & Distortion parameters
  // std::array<double,2> opt_focal_param = {m_pcamera->m_pcalib_params->m_fx, m_pcamera->m_pcalib_params->m_fy};
  // std::array<double,2> opt_pp_param = {m_pcamera->m_pcalib_params->m_cx, m_pcamera->m_pcalib_params->m_cy};
  std::vector<double> opt_focal_param = m_pcamera->m_pcalib_params->getFocal();
  std::vector<double> opt_pp_param = m_pcamera->m_pcalib_params->getPrincipalPoint();
  std::vector<double> opt_dist_param = m_pcamera->m_pdist_params->getDistParameters();
  
  // Setup Intrinsic & Distortion parameters
  problem.AddParameterBlock(opt_focal_param.data(), opt_focal_param.size());
  problem.AddParameterBlock(opt_pp_param.data(), opt_pp_param.size());
  problem.AddParameterBlock(opt_dist_param.data(), opt_dist_param.size());

  const std::vector<Eigen::Vector3d> &v_tgt_coords = m_pcalib_detector->getTargetCoords();

  // Setup residuals
  const size_t nb_views = m_pcamera->m_v_calib_frames.size();
  const size_t nb_residuals = nb_views * v_tgt_coords.size();

  std::vector<double> vini_err;
  vini_err.reserve(nb_residuals);

  for (auto& calib_frame : m_pcamera->m_v_calib_frames)
  {
    if (calib_frame.rmse_err > 1.)
    {
      continue;
    }

    // auto to handle LocalParametrization / Manifold changes in Ceres
    auto *local_param = new AutoDiffLocalLeftSE3();
    problem.AddParameterBlock(calib_frame.m_T_world_2_cam.data(), 7, local_param);

    for (size_t j = 0ul; j < v_tgt_coords.size(); ++j)
    {
      if (!m_pcamera->m_pcalib_params->isPointInImage(calib_frame.m_v_corners_px[j].x, 
                                                      calib_frame.m_v_corners_px[j].y))
      {
        continue;
      }

      ceres::CostFunction* f = 
          m_pcamera->m_pdist_params->createCeresCostFunction(calib_frame.m_v_corners_px[j].x, 
                                                            calib_frame.m_v_corners_px[j].y,
                                                            v_tgt_coords[j]);

      // const ceres::ResidualBlockId res_id =
      problem.AddResidualBlock(
                  f, loss_function,
                  opt_focal_param.data(), 
                  opt_pp_param.data(),
                  opt_dist_param.data(),
                  calib_frame.m_T_world_2_cam.data());

      const Eigen::Vector2d px_obs(calib_frame.m_v_corners_px[j].x,calib_frame.m_v_corners_px[j].y);
      const Eigen::Vector2d dist_cam_pt = m_pcamera->m_pdist_params->distortCamPoint(calib_frame.m_T_world_2_cam * v_tgt_coords[j]);
      const Eigen::Vector2d px_proj = m_pcamera->m_pcalib_params->projCamToImage(dist_cam_pt);

      vini_err.push_back((px_obs - px_proj).norm());
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  options.num_threads = 8;
  options.max_num_iterations = 1000;

  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  
  std::cout << "\n\n=========================================\n";
  std::cout << "     Ceres based Calibration Refinement";
  std::cout << "\n=========================================\n";

  std::cout << "\n0. Initial Parameters to Refine";
  std::cout << "\n=================================\n";
  
  m_pcamera->m_pcalib_params->displayParamsWithStd();
  m_pcamera->m_pdist_params->displayParamsWithStd();

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;
  
  m_pcamera->m_pcalib_params->resetParameters(opt_focal_param, opt_pp_param);
  m_pcamera->m_pdist_params->resetParameters(opt_dist_param);

  std::cout << "\n1. Refined Calibration Parameters";
  std::cout << "\n=================================\n";

  std::vector<double> vopt_err;
  vopt_err.reserve(vini_err.size());

  std::vector<double> vfinal_err;
  vfinal_err.reserve(vini_err.size());

  for (auto& calib_frame : m_pcamera->m_v_calib_frames)
  {
    if (calib_frame.rmse_err > 1.)
    {
      continue;
    }

    const std::vector<cv::Point2f>& v_corners_pts = calib_frame.m_v_corners_px;

    double rmse_err = 0.;
    int nb_err = 0;

    for (size_t j = 0ul; j < v_tgt_coords.size(); ++j)
    {
      if (!m_pcamera->m_pcalib_params->isPointInImage(v_corners_pts[j].x,
                                                      v_corners_pts[j].y))
      {
        continue;
      }

      const Eigen::Vector2d px_obs(calib_frame.m_v_corners_px[j].x,calib_frame.m_v_corners_px[j].y);
      const Eigen::Vector2d dist_cam_pt = m_pcamera->m_pdist_params->distortCamPoint(calib_frame.m_T_world_2_cam * v_tgt_coords[j]);
      const Eigen::Vector2d px_proj = m_pcamera->m_pcalib_params->projCamToImage(dist_cam_pt);

      vopt_err.push_back((px_obs - px_proj).norm());

      rmse_err += vopt_err.back() * vopt_err.back();
      ++nb_err;

      if (vopt_err.back() < 1.)
      {
        vfinal_err.push_back(vopt_err.back());
      }
    }

    if (nb_err > 0)
    {
      calib_frame.rmse_err = std::sqrt(rmse_err / static_cast<double>(nb_err));
    }
    else
    {
      calib_frame.rmse_err = std::numeric_limits<double>::max();
    }
  }

  cv::Scalar vini_err_mean, vini_err_std;
  cv::meanStdDev(vini_err, vini_err_mean, vini_err_std);

  cv::Scalar vopt_err_mean, vopt_err_std;
  cv::meanStdDev(vopt_err, vopt_err_mean, vopt_err_std);

  cv::Scalar vfinal_err_mean, vfinal_err_std;
  cv::meanStdDev(vfinal_err, vfinal_err_mean, vfinal_err_std);

  std::sort(vini_err.begin(), vini_err.end());
  std::sort(vopt_err.begin(), vopt_err.end());
  std::sort(vfinal_err.begin(), vfinal_err.end());

  std::cout << "\n\nIni Obs reproj error (Mean / Std / Med) : " << vini_err_mean[0] << " / " << vini_err_std[0] << " / " << vini_err[vini_err.size() / 2] << "\n\n";;
  std::cout << "Opt Obs reproj error (Mean / Std / Med) : " << vopt_err_mean[0] << " / " << vopt_err_std[0] << " / " << vopt_err[vopt_err.size() / 2] << "\n\n";
  std::cout << "Final Obs reproj error (Mean / Std / Med) : " << vfinal_err_mean[0] << " / " << vfinal_err_std[0] << " / " << vfinal_err[vfinal_err.size() / 2] << "\n\n";

  std::vector<const double*> v_params;
  v_params.reserve(opt_focal_param.size()+opt_pp_param.size()+opt_dist_param.size());
  
  v_params.push_back(opt_focal_param.data());
  v_params.push_back(opt_pp_param.data());
  v_params.push_back(opt_dist_param.data());

  ceres::Covariance::Options cov_options;
  ceres::Covariance ceres_covariance(cov_options);
  const bool cov_success = ceres_covariance.Compute(v_params, &problem);

  if (cov_success)
  {
    std::vector<double> vcov_focal(opt_focal_param.size()*opt_focal_param.size());
    std::vector<double> vcov_pp(opt_pp_param.size()*opt_pp_param.size());
    std::vector<double> vcov_dist(opt_dist_param.size()*opt_dist_param.size());

    ceres_covariance.GetCovarianceBlock(opt_focal_param.data(),opt_focal_param.data(),vcov_focal.data());
    ceres_covariance.GetCovarianceBlock(opt_pp_param.data(),opt_pp_param.data(),vcov_pp.data());
    ceres_covariance.GetCovarianceBlock(opt_dist_param.data(),opt_dist_param.data(),vcov_dist.data());

    m_pcamera->m_pcalib_params->setFocalStd(vcov_focal);
    m_pcamera->m_pcalib_params->setPPStd(vcov_pp);
    m_pcamera->m_pdist_params->setDistParamsStd(vcov_dist);

    const Eigen::Map<Eigen::MatrixXd> cov_focal_mat(vcov_focal.data(),opt_focal_param.size(),opt_focal_param.size());
    const Eigen::Map<Eigen::Matrix2d> cov_pp_mat(vcov_pp.data());
    const Eigen::Map<Eigen::MatrixXd> cov_dist_mat(vcov_dist.data(),opt_dist_param.size(),opt_dist_param.size());

    // std::cout << "\n\nCovariance on focal : \n" << cov_focal_mat;
    // std::cout << "\n\nCovariance on PP : \n" << cov_pp_mat;
    // std::cout << "\n\nCovariance on dist : \n" << cov_dist_mat;
    // std::cout << "\n\n";
  }
  else
  {
    std::cout << "\nCovariance Estimation Failed!\n";
    std::cout << "The Jacobian is most likely rank deficient!\n\n";
  }

  m_pcamera->m_pcalib_params->displayParamsWithStd();
  m_pcamera->m_pdist_params->displayParamsWithStd();

  std::cout << "\n\nDone!\n\n";

  return;
}

// Compute Tcw with P3P-RANSAC OpenCV Impl.
bool 
EZCalibrator::computeP3PRansac(
  const std::vector<cv::Point2f>& _v_px_obs,
  const std::vector<Eigen::Vector3d>& _v_wpts,
  const IntrinsicsParam& _calib_params,
  Sophus::SE3d& _T_world_2_cam,
  std::vector<int>& _v_inliers
)
{
  assert(_v_px_obs.size() == _v_wpts.size());

  std::vector<cv::Point2f> v_unpx;
  std::vector<cv::Point3f> v_wpts;
  v_unpx.reserve(_v_px_obs.size());
  v_wpts.reserve(_v_px_obs.size());

  for (size_t i = 0ul; i < _v_px_obs.size(); ++i)
  {
    if (_calib_params.isPointInImage(_v_px_obs[i].x, _v_px_obs[i].y))
    {
      const Eigen::Vector3f unpx = _calib_params.projImageToCam(Eigen::Vector2d(_v_px_obs[i].x,_v_px_obs[i].y)).cast<float>();
      v_unpx.push_back(cv::Point2f(unpx[0],unpx[1]));
      v_wpts.push_back(cv::Point3f(_v_wpts[i][0],_v_wpts[i][1],_v_wpts[i][2]));
    }
  }

  if (v_unpx.size() < 32ul && m_pcalib_detector->getTargetCoords().size() >= 32ul)
  {
    std::cout << "Not enough measurements for P3P... Skipping frame!\n";
    return false;
  }

  cv::Mat rvec, tvec;
  const bool success = 
        cv::solvePnPRansac(v_wpts, 
                          v_unpx, 
                          cv::Mat::eye(3,3,CV_32F), 
                          cv::Mat(), 
                          rvec, tvec, false, 
                          1000, 4.0 / _calib_params.getMeanFocal(), 0.999, 
                          _v_inliers,
                          #if CV_VERSION_MAJOR < 4 
                            cv::SOLVEPNP_P3P 
                          #else
                            cv::SOLVEPNP_AP3P
                          #endif
                          );
  
  std::cout << "\n >>>>>>>> P3P nb inliers : " << _v_inliers.size() << " out of " << v_wpts.size() << " pts\n";

  if (!success || (_v_inliers.size() < 32ul && m_pcalib_detector->getTargetCoords().size() >= 32ul))
  {
    std::cout << "P3P failed... Skipping frame!\n";
    return false;
  }

  cv::Mat cvR;
  cv::Rodrigues(rvec, cvR);

  Eigen::Vector3f tcw;
  Eigen::Matrix3f Rcw;

  cv::cv2eigen(cvR, Rcw);
  cv::cv2eigen(tvec, tcw);

  _T_world_2_cam = Sophus::SE3d(Sophus::SO3d::fitToSO3(Rcw.cast<double>()), tcw.cast<double>());

  return true;
}


void
EZCalibrator::runMultiCameraCalib(std::vector<Camera>& _v_cameras)
{
  // Setup Calibration Optimization Problem!
  // =======================================
  ceres::Problem problem;
  ceres::LossFunctionWrapper* loss_function;
  loss_function = new ceres::LossFunctionWrapper(new ceres::CauchyLoss(1.0), ceres::TAKE_OWNERSHIP);

  const size_t nb_cams = _v_cameras.size();

  std::vector<std::vector<double>> v_focal_param(nb_cams-1);
  std::vector<std::vector<double>> v_pp_param(nb_cams-1);
  std::vector<std::vector<double>> v_dist_param(nb_cams-1);

  const std::vector<Eigen::Vector3d> &v_tgt_coords = m_pcalib_detector->getTargetCoords();

  std::vector<Sophus::SE3d> v_opt_Tcic0;
  v_opt_Tcic0.reserve(nb_cams-1);

  const auto& cam0 = _v_cameras[0];

  std::size_t nb_good_pairs = 0;

  // Setup Intrinsic & Distortion parameters
  for (size_t i = 1ul; i < nb_cams; ++i)
  {
    const auto& camj = _v_cameras[i];

    v_focal_param[i-1] = camj.m_pcalib_params->getFocal();
    v_pp_param[i-1] = camj.m_pcalib_params->getPrincipalPoint();
    v_dist_param[i-1] = camj.m_pdist_params->getDistParameters();

    problem.AddParameterBlock(v_focal_param[i-1].data(), v_focal_param[i-1].size());
    problem.AddParameterBlock(v_pp_param[i-1].data(), v_pp_param[i-1].size());
    problem.AddParameterBlock(v_dist_param[i-1].data(), v_dist_param[i-1].size());

    problem.SetParameterBlockConstant(v_focal_param[i-1].data());
    problem.SetParameterBlockConstant(v_pp_param[i-1].data());
    problem.SetParameterBlockConstant(v_dist_param[i-1].data());

    std::array<std::vector<double>,6ul> tab_Tcic0_v_coefs;
    for (size_t j=0ul; j < 6ul; ++j)
    {
      tab_Tcic0_v_coefs[j].reserve(cam0.m_v_calib_frames.size());
    }

    for (size_t j=0ul; j < cam0.m_v_calib_frames.size(); ++j)
    {
      const auto& calib_frame0 = cam0.m_v_calib_frames[i];
      if (calib_frame0.rmse_err > 1.)
      {
        continue;
      }

      for (size_t k=0ul; k < camj.m_v_calib_frames.size(); ++k)
      {
        const auto& calib_framej = camj.m_v_calib_frames[k];
        if (calib_frame0.m_img_name == calib_framej.m_img_name)
        {
          ++nb_good_pairs;
          const Sophus::SE3d Tcic0 = calib_framej.m_T_world_2_cam * calib_frame0.m_T_world_2_cam.inverse();
          const Sophus::Vector6d T_log = Tcic0.log();
          for (size_t l=0ul; l < 6ul; ++l)
          {
            tab_Tcic0_v_coefs[l].push_back(T_log[l]);
          }
          break;
        }
        else if (calib_framej.m_img_name > calib_frame0.m_img_name)
        {
          break;
        }
      }
    }

    const size_t med_idx = tab_Tcic0_v_coefs[0].size() / 2ul;
    Sophus::Vector6d med_T_log;
    for (size_t j=0ul; j < 6ul; ++j)
    {
      std::sort(tab_Tcic0_v_coefs[j].begin(), tab_Tcic0_v_coefs[j].end());
      med_T_log[j] = tab_Tcic0_v_coefs[j][med_idx];
    }

    // const Sophus::SE3d Tc0w = cam0.m_v_calib_frames[0].m_T_world_2_cam;
    // const Sophus::SE3d Tciw = camj.m_v_calib_frames[0].m_T_world_2_cam;
    // v_opt_Tcic0.push_back(Tciw * Tc0w.inverse());
    v_opt_Tcic0.push_back(Sophus::SE3d::exp(med_T_log));

    // auto to handle LocalParametrization / Manifold changes in Ceres
    auto *local_param = new AutoDiffLocalLeftSE3();
    problem.AddParameterBlock(v_opt_Tcic0.back().data(), 7, local_param);
  }

  std::cout << "\nMulti-Camera Calib -- Nb good pairs : " << nb_good_pairs << "\n";

  for (size_t i = 0ul; i < cam0.m_v_calib_frames.size(); ++i)
  {
    const auto& calib_frame0 = cam0.m_v_calib_frames[i];

    std::vector<Eigen::Vector3d> cam0_tgt_coords;
    cam0_tgt_coords.reserve(v_tgt_coords.size());
    for (const auto& tgt_coord : v_tgt_coords)
    {
      cam0_tgt_coords.push_back(calib_frame0.m_T_world_2_cam * tgt_coord);
    }

    if (calib_frame0.rmse_err > 1.)
    {
      continue;
    }

    for (size_t j = 1ul; j < nb_cams; ++j)
    {
      auto& camj = _v_cameras[j];

      for (const auto& calib_framej : camj.m_v_calib_frames)
      {
        if (calib_framej.rmse_err > 1.)
        {
          continue;
        }
        
        if (calib_frame0.m_img_name == calib_framej.m_img_name)
        {
          for (size_t k = 0ul; k < v_tgt_coords.size(); ++k)
          {
            if (!camj.m_pcalib_params->isPointInImage(calib_framej.m_v_corners_px[k].x, 
                                                      calib_framej.m_v_corners_px[k].y))
            {
              continue;
            }

            ceres::CostFunction* f = 
                camj.m_pdist_params->createCeresCostFunction(calib_framej.m_v_corners_px[k].x, 
                                                             calib_framej.m_v_corners_px[k].y,
                                                             cam0_tgt_coords[k]);

            problem.AddResidualBlock(
                        f, loss_function,
                        v_focal_param[j-1].data(), 
                        v_pp_param[j-1].data(),
                        v_dist_param[j-1].data(),
                        v_opt_Tcic0[j-1].data());
          }

          break;
        }
        else if (calib_framej.m_img_name > calib_frame0.m_img_name)
        {
          break;
        }
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  options.num_threads = 8;
  options.max_num_iterations = 1000;

  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  
  std::cout << "\n\n=========================================\n";
  std::cout << "     Ceres based Extrinsic Calibration";
  std::cout << "\n=========================================\n";

  std::cout << "\n0. Initial Parameters";
  std::cout << "\n=================================\n";
  
  for (size_t i=0; i < nb_cams; ++i)
  {
    std::cout << "\nCam #" << i << ":\n";

    _v_cameras[i].m_pcalib_params->displayParamsWithStd();
    _v_cameras[i].m_pdist_params->displayParamsWithStd();

    if (i > 0)
    {
      std::cout << "\nCam0 to Cam" << i << " 3x4 transformation:\n";
      std::cout << v_opt_Tcic0[i-1].matrix3x4() << "\n\n";
    }
  }

  std::cout << "\n" << v_opt_Tcic0[0].translation().transpose() << "\n";
  std::cout << v_opt_Tcic0[0].rotationMatrix() << "\n";

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;
  
  std::cout << "\n1. Optimized Parameters ";
  std::cout << "\n=================================\n";

  for (size_t i=0ul; i < nb_cams; ++i)
  {
    std::cout << "\nCam #" << i << ":\n";

    _v_cameras[i].m_pcalib_params->displayParamsWithStd();
    _v_cameras[i].m_pdist_params->displayParamsWithStd();

    if (i > 0)
    {
      std::cout << "\nCam0 to Cam" << i << " 3x4 transformation:\n";
      std::cout << v_opt_Tcic0[i-1].matrix3x4() << "\n\n";

      _v_cameras[i].m_T_cam0_2_cam = v_opt_Tcic0[i-1];
    }
  }
}

void 
EZCalibrator::setupCalibrationProblem(const std::string& _config_file_path)
{
  const cv::FileStorage fsSettings(_config_file_path, cv::FileStorage::READ);

  if(!fsSettings.isOpened()) 
  {
    std::cerr << "Failed to open settings file...";
    exit(-1);
  }

  const std::string target_type = fsSettings["target_type"].string();

  m_debug_display = static_cast<int>(fsSettings["debug"]);

  if (target_type == "aprilgrid")
  {
    const int nb_rows = fsSettings["target_nb_rows"];
    const int nb_cols = fsSettings["target_nb_cols"];

    const double tag_size = fsSettings["tag_size"];
    const double tag_space = fsSettings["tag_space"];

    m_pcalib_detector = std::make_unique<AprilTagCalibDetector>(nb_rows,
                                                                nb_cols,
                                                                tag_size,
                                                                tag_space,
                                                                m_debug_display);
  }
  else if (target_type == "chessboard")
  {
    const int nb_rows = fsSettings["target_nb_rows"];
    const int nb_cols = fsSettings["target_nb_cols"];

    const double square_size = fsSettings["square_size"];

    m_pcalib_detector = std::make_unique<ChessboardCalibDetector>(nb_rows,
                                                                  nb_cols,
                                                                  square_size,
                                                                  m_debug_display);
  }
  else
  {
    std::cerr << "\nERROR!  Not implemented target type!\n";
    std::cerr << "\nProvided target type is : " << target_type;
    std::cerr << "\nImplemented target types are : chessboard / aprilgrid\n";
    exit(-1);
  }

  m_pcalib_detector->displayInfo();

  cv::waitKey(2000);
}
