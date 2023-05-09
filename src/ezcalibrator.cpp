#include "ezcalibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

EZMonoCalibrator::EZMonoCalibrator(const std::string& _config_file_path)
{
  setupCalibrationProblem(_config_file_path);
}

EZMonoCalibrator::~EZMonoCalibrator()
{}


bool 
EZMonoCalibrator::processImage(const cv::Mat& _in_img, const std::string &_img_name /*= ""*/, const double _timestamp /*= -1.*/)
{
  if (_in_img.empty())
    return false;

  if (m_pcalib_params == nullptr)
  {
    setupInitialCalibration(_in_img.size());
  }

  std::vector<cv::Point2f> v_corners_pts;

  const bool success = m_pcalib_detector->detectTarget(_in_img, v_corners_pts);
  
  if (success)
  {
    Sophus::SE3d T_world_2_cam;
    std::vector<int> vinliers;

    const bool p3p_success = computeP3PRansac(v_corners_pts, 
                                              m_pcalib_detector->getTargetCoords(), 
                                              *m_pcalib_params, 
                                              T_world_2_cam,
                                              vinliers);

    if (p3p_success)
    {
      m_v_calib_frames.emplace_back(_img_name, _timestamp, v_corners_pts, T_world_2_cam);
    }
  }

  return success;
}


void
EZMonoCalibrator::computeCalibration()
{
  // Setup Calibration Optimization Problem!
  // =======================================
  ceres::Problem problem;
  ceres::LossFunctionWrapper* loss_function;
  loss_function = new ceres::LossFunctionWrapper(new ceres::CauchyLoss(1.0), ceres::TAKE_OWNERSHIP);

  // Setup Intrinsic & Distortion parameters
  std::array<double,2> opt_focal_param = {m_pcalib_params->m_fx, m_pcalib_params->m_fy};
  std::array<double,2> opt_pp_param = {m_pcalib_params->m_cx, m_pcalib_params->m_cy};
  std::vector<double>  opt_dist_param(m_pdist_params->getNumberOfParameters(), 0.);

  // Setup Intrinsic & Distortion parameters
  problem.AddParameterBlock(opt_focal_param.data(), 2);
  problem.AddParameterBlock(opt_pp_param.data(), 2);
  problem.AddParameterBlock(opt_dist_param.data(), opt_dist_param.size());

  std::vector<Eigen::Vector3d> v_tgt_coords = m_pcalib_detector->getTargetCoords();

  // Setup calibration board 3D points
  for (size_t i = 0; i < v_tgt_coords.size(); ++i)
  {
    problem.AddParameterBlock(v_tgt_coords[i].data(), 3);
    problem.SetParameterBlockConstant(v_tgt_coords[i].data());
  }

  // Setup residuals
  const size_t nb_views = m_v_calib_frames.size();
  const size_t nb_residuals = nb_views * v_tgt_coords.size();

  // std::vector<ceres::ResidualBlockId> v_res_ids;
  // v_res_ids.reserve(nb_residuals);

  std::vector<double> vini_err;
  vini_err.reserve(nb_residuals);

  for (auto& calib_frame : m_v_calib_frames)
  {
    ceres::LocalParameterization *local_param = new AutoDiffLocalLeftSE3();
    problem.AddParameterBlock(calib_frame.m_T_world_2_cam.data(), 7, local_param);

    for (size_t j = 0ul; j < v_tgt_coords.size(); ++j)
    {
      if (!m_pcalib_params->isPointInImage(calib_frame.m_v_corners_px[j].x, 
                                           calib_frame.m_v_corners_px[j].y))
      {
        continue;
      }

      ceres::CostFunction* f = 
          m_pdist_params->createCeresCostFunction(calib_frame.m_v_corners_px[j].x, 
                                                  calib_frame.m_v_corners_px[j].y,
                                                  v_tgt_coords[j]);

      // const ceres::ResidualBlockId res_id =
      problem.AddResidualBlock(
                  f, loss_function,
                  opt_focal_param.data(), 
                  opt_pp_param.data(),
                  opt_dist_param.data(),
                  calib_frame.m_T_world_2_cam.data());
      
      // v_res_ids.push_back(res_id);

      vini_err.push_back((Eigen::Vector2d(calib_frame.m_v_corners_px[j].x,calib_frame.m_v_corners_px[j].y) 
                            - m_pcalib_params->projCamToImage(calib_frame.m_T_world_2_cam * v_tgt_coords[j])).norm());
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  options.num_threads = 8;
  options.max_num_iterations = 1000;
  options.function_tolerance = 1e-10;

  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  
  std::cout << "\n\n=================================\n";
  std::cout << "     Ceres based Optimization";
  std::cout << "\n=================================\n";

  std::cout << "\n0. Initial Values to Optimize";
  std::cout << "\n=================================\n";
  
  m_pcalib_params->displayParams();
  m_pdist_params->displayParams();


  std::cout << "\n\n1. Optimize Focal & Camera Poses";
  std::cout << "\n=================================\n";

  // Optimize Focal & Poses only first
  problem.SetParameterBlockConstant(opt_pp_param.data());
  problem.SetParameterBlockConstant(opt_dist_param.data());

  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;
  
  m_pcalib_params->resetParameters(opt_focal_param[0], opt_focal_param[1], 
                                   opt_pp_param[0], opt_pp_param[1]);
  m_pdist_params->resetParameters(opt_dist_param);

  m_pcalib_params->displayParams();
  m_pdist_params->displayParams();

  std::cout << "\n\n2. Optimize Focal & Distortion & Camera Poses";
  std::cout << "\n==============================================\n";

  problem.SetParameterBlockVariable(opt_dist_param.data());
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  m_pcalib_params->resetParameters(opt_focal_param[0], opt_focal_param[1], 
                                   opt_pp_param[0], opt_pp_param[1]);
  m_pdist_params->resetParameters(opt_dist_param);

  m_pcalib_params->displayParams();
  m_pdist_params->displayParams();
  
  std::cout << "\n\n3. Optimize Focal & Principal Point & Distortion & Camera Poses";
  std::cout << "\n================================================================\n";

  problem.SetParameterBlockVariable(opt_pp_param.data());
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  m_pcalib_params->resetParameters(opt_focal_param[0], opt_focal_param[1], 
                                   opt_pp_param[0], opt_pp_param[1]);
  m_pdist_params->resetParameters(opt_dist_param);

  m_pcalib_params->displayParams();
  m_pdist_params->displayParams();

  std::vector<double> vopt_err;
  vopt_err.reserve(vini_err.size());

  std::vector<double> vfinal_err;
  vfinal_err.reserve(vini_err.size());

  for (const auto& calib_frame : m_v_calib_frames)
  {
    const std::vector<cv::Point2f>& v_corners_pts = calib_frame.m_v_corners_px;

    for (size_t j = 0ul; j < v_tgt_coords.size(); ++j)
    {
      if (!m_pcalib_params->isPointInImage(v_corners_pts[j].x,
                                           v_corners_pts[j].y))
      {
        continue;
      }

      vopt_err.push_back((Eigen::Vector2d(v_corners_pts[j].x,v_corners_pts[j].y) 
                            - m_pcalib_params->projCamToImage(calib_frame.m_T_world_2_cam * v_tgt_coords[j])).norm());

      if (vopt_err.back() < 0.5)
      {
        vfinal_err.push_back(vopt_err.back());
      }
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
  ceres_covariance.Compute(v_params, &problem);

  double cov_focal[2*2];
  double cov_pp[2*2];
  double cov_dist[opt_dist_param.size()*opt_dist_param.size()];

  ceres_covariance.GetCovarianceBlock(opt_focal_param.data(),opt_focal_param.data(),cov_focal);
  ceres_covariance.GetCovarianceBlock(opt_pp_param.data(),opt_pp_param.data(),cov_pp);
  ceres_covariance.GetCovarianceBlock(opt_dist_param.data(),opt_dist_param.data(),cov_dist);

  const Eigen::Map<Eigen::Matrix2d> cov_focal_mat(cov_focal);
  const Eigen::Map<Eigen::Matrix2d> cov_pp_mat(cov_pp);
  const Eigen::Map<Eigen::MatrixXd> cov_dist_mat(cov_dist,opt_dist_param.size(),opt_dist_param.size());

  std::cout << "\n\nCovariance on focal : \n" << cov_focal_mat;
  std::cout << "\n\nCovariance on PP : \n" << cov_pp_mat;
  std::cout << "\n\nCovariance on dist : \n" << cov_dist_mat;

  std::cout << "\n\nDone!\n\n";
  
  return;
}


// Compute Tcw with P3P-RANSAC OpenCV Impl.
bool 
EZMonoCalibrator::computeP3PRansac(
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

  if (v_unpx.size() < 48ul)
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
                          1000, 4.0 / _calib_params.m_fx, 0.999, 
                          _v_inliers,
                          CV_VERSION_MAJOR < 4 ? cv::SOLVEPNP_P3P 
                                               : cv::SOLVEPNP_AP3P);
  
  std::cout << "\n >>>>>>>> P3P nb inliers : " << _v_inliers.size() << " out of " << v_wpts.size() << " pts\n";

  if (!success || _v_inliers.size() < 48ul)
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
EZMonoCalibrator::setupCalibrationProblem(const std::string& _config_file_path)
{
  const cv::FileStorage fsSettings(_config_file_path, cv::FileStorage::READ);

  if(!fsSettings.isOpened()) 
  {
    std::cerr << "Failed to open settings file...";
    exit(-1);
  }

  std::string dist_model;
  dist_model.assign(fsSettings["dist_model"]);

  setupInitialDistortion(dist_model);

  m_prior_fov_deg = fsSettings["prior_fov"];

  std::string target_type;
  target_type.assign(fsSettings["target_type"]);

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

    std::cout << "\n====================================\n";
    std::cout << "\tProvided Calibration problem:\n";
    std::cout << "\n- Target type: AprilGrid";
    std::cout << "\n- Number of rows: " << nb_rows;
    std::cout << "\n- Number of cols: " << nb_cols;
    std::cout << "\n- Size of a tag: " << tag_size << " m";
    std::cout << "\n- Space ratio between tags: " << tag_space << "(i.e. spacing of: " << tag_size*tag_space << " m)\n";
    std::cout << "\n- Distortion model to be used: " << dist_model;
    std::cout << "\n- Provided prior FOV: " << m_prior_fov_deg << " degree";
    std::cout << "\n====================================\n";
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
    
    std::cout << "\n====================================\n";
    std::cout << "\tProvided Calibration problem:\n";
    std::cout << "\n- Target type: Chessboard";
    std::cout << "\n- Number of rows: " << nb_rows;
    std::cout << "\n- Number of cols: " << nb_cols;
    std::cout << "\n- Size of a square: " << square_size << " m\n";
    std::cout << "\n- Distortion model to be used: " << dist_model;
    std::cout << "\n- Provided prior FOV: " << m_prior_fov_deg << " degree";
    std::cout << "\n====================================\n";
  }
  else
  {
    std::cerr << "\nERROR!  Not implemented target type!\n";
    std::cerr << "\nProvided target type is : " << target_type;
    std::cerr << "\nImplemented target types are : chessboard / aprilgrid\n";
    exit(-1);
  }

  std::cout << "\nDistortion model initialized as:\n"; 
  m_pdist_params->displayParams();

  cv::waitKey(2000);
}


void
EZMonoCalibrator::setupInitialCalibration(const cv::Size& _img_size)
{
  const double img_width = static_cast<double>(_img_size.width);
  const double img_height = static_cast<double>(_img_size.height);

  const double prior_cx = img_width / 2.;
  const double prior_cy = img_height / 2.;

  const double PI = 2. * std::asin(1.);
  const double DEG2RAD = PI / 180.;

  const double half_fov_rad = m_prior_fov_deg / 2. * DEG2RAD;

  const double inv_tan_half_fov = 1. / std::tan(half_fov_rad);
  const double prior_focal = std::max(prior_cx * inv_tan_half_fov, prior_cy * inv_tan_half_fov);
  
  m_pcalib_params = std::make_unique<IntrinsicsParam>(img_width, img_height, prior_focal, prior_focal, prior_cx, prior_cy);
}


void
EZMonoCalibrator::setupInitialDistortion(const std::string& _dist_model)
{
  if (_dist_model == "rad1")
  {
    m_pdist_params = std::make_unique<Rad1DistParam>();
  }
  else if (_dist_model == "rad2")
  {
    m_pdist_params = std::make_unique<Rad2DistParam>();
  }
  else if (_dist_model == "rad3")
  {
    m_pdist_params = std::make_unique<Rad3DistParam>();
  }
  else if (_dist_model == "radtan4")
  {
    m_pdist_params = std::make_unique<RadTan4DistParam>();
  }
  else if (_dist_model == "radtan5")
  {
    m_pdist_params = std::make_unique<RadTan5DistParam>();
  }
  else if (_dist_model == "radtan8")
  {
    m_pdist_params = std::make_unique<RadTan8DistParam>();
  }
  else if (_dist_model == "kannalabrandt")
  {
    m_pdist_params = std::make_unique<KB4DistParam>();
  }
  else
  {
    std::cerr << "\nNot implemented distortion model!\n";
    exit(-1);
  }
}