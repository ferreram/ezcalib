#include "camera.hpp"


Camera::Camera(const std::string& _config_file_path)
{
  const cv::FileStorage fsSettings(_config_file_path, cv::FileStorage::READ);

  if(!fsSettings.isOpened()) 
  {
    std::cerr << "Failed to open settings file...";
    exit(-1);
  }

  const std::string dist_model = fsSettings["dist_model"].string();
  // dist_model.assign(fsSettings["dist_model"]);

  setupInitialDistortion(dist_model);

  m_prior_fov_deg = fsSettings["prior_fov"].real();

  std::cout << "\nDistortion model initialized as:\n"; 
  m_pdist_params->displayParams();

  m_input_images_path = fsSettings["input_images_folder"].string();
  std::cout << "\nGoing to take images from folder: " << m_input_images_path;
}

void 
Camera::setupInitialDistortion(const std::string& _dist_model)
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

void 
Camera::setupInitialCalibration(const cv::Size& _img_size)
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
Camera::displayCalibrationParameters() const
{
  std::cout << "\nIntrinsic Parameters:\n";
  m_pcalib_params->displayParams();

  std::cout << "\nDistortion Parameters:\n";
  m_pdist_params->displayParams();
}
