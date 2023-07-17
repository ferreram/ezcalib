#include "camera.hpp"

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

Camera::Camera(const std::string& _config_file_path)
{
  const cv::FileStorage fsSettings(_config_file_path, cv::FileStorage::READ);

  if(!fsSettings.isOpened()) 
  {
    std::cerr << "Failed to open settings file...";
    exit(-1);
  }

  m_dist_model = fsSettings["dist_model"].string();
  m_prior_fov_deg = fsSettings["prior_fov"].real();
  m_input_images_path = fsSettings["input_images_folder"].string();

  if (m_prior_fov_deg < 1.)
  {
    std::cerr << "Provided prior field of view should be positive!  Wrong value here: " << m_prior_fov_deg << "\n";
    exit(-1);
  }

  if (!fs::exists(m_input_images_path))
  {
    std::cerr << "Provided image folder: " << m_input_images_path << " does not exist!\n";
    exit(-1);
  }

  setupInitialDistortion(m_dist_model);

  displayCameraInfo();
}

Camera::Camera(const std::string& _input_images_path,
               const std::string& _dist_model, 
               const double _prior_fov)
  : m_input_images_path(_input_images_path)
  , m_dist_model(_dist_model)
  , m_prior_fov_deg(_prior_fov)
{
  if (m_prior_fov_deg < 1.)
  {
    std::cerr << "Provided prior field of view should be positive!  Wrong value here: " << m_prior_fov_deg << "\n";
    exit(-1);
  }

  if (!fs::exists(m_input_images_path))
  {
    std::cerr << "Provided image folder: " << m_input_images_path << " does not exist!\n";
    exit(-1);
  }

  setupInitialDistortion(m_dist_model);

  displayCameraInfo();
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
    std::cerr << "\nThe provided distortion model: " << _dist_model << " is not implemented!\n";
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


Eigen::Vector2d 
Camera::undistortPx(const cv::Point2f& _corner_pt) const
{
  return  m_pcalib_params->projCamToImage(
              m_pdist_params->undistortCamPoint(
                  m_pcalib_params->projImageToCam(_corner_pt.x,
                                                  _corner_pt.y)
              )
          );
}


void 
Camera::displayCalibrationParameters() const
{
  displayIntrinsicParameters();
  displayDistortionParameters();
}


void 
Camera::displayIntrinsicParameters() const
{
  if (m_pcalib_params != nullptr)
  {
    std::cout << "\nIntrinsic Parameters:\n";
    m_pcalib_params->displayParams();
  }
  else
  {
    std::cout << "\nIntrinsic Parameters not intialized yet.\n";
  }
}


void 
Camera::displayDistortionParameters() const
{
  if (m_pdist_params != nullptr)
  {
    std::cout << "\nDistortion Parameters:\n";
    m_pdist_params->displayParams();
  }
  else
  {
    std::cout << "\nDistortion Parameters not intialized yet.\n";
  }
}


void 
Camera::displayCameraInfo() const
{
  std::cout << "\n========================\n";
  std::cout << "     Camera Info";
  std::cout << "\n========================\n";

  std::cout << "\nCamera related to images from: " << m_input_images_path << "\n";
  std::cout << "\nPrior Field-of-View set as: " << m_prior_fov_deg << "degrees.\n";

  std::cout << "\nCurrent Parameters are:\n";
  std::cout << "----------------------------\n";
  displayCalibrationParameters();
}
