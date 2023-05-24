#include "ezcalibrator.hpp"

#include <opencv2/core/eigen.hpp>

void 
setupCameras(const std::string& _config_file_path, 
             std::vector<Camera>& _v_cameras)
{
  const cv::FileStorage fsSettings(_config_file_path, cv::FileStorage::READ);

  if(!fsSettings.isOpened()) 
  {
    std::cerr << "Failed to open settings file...";
    exit(-1);
  }

  const cv::FileNode node_input_images = fsSettings["input_images_folder"];

  if (node_input_images.isSeq())
  {
    const size_t nb_cams = node_input_images.size();


    const cv::FileNode node_dist_models = fsSettings["dist_model"];
    const cv::FileNode node_prior_fovs = fsSettings["prior_fov"];

    if (node_dist_models.size() != nb_cams)
    {
      std::cerr << "\nError!\n";
      std::cerr << "\nConfig file must have as many dist models as provided input image folders!\n";
      exit(-1);
    }
    else if (node_prior_fovs.size() != nb_cams)
    {
      std::cerr << "\nError!\n";
      std::cerr << "\nConfig file must have as many prior fovs as provided input image folders!\n";
      exit(-1);
    }

    _v_cameras.reserve(nb_cams);

    for (size_t i=0; i < nb_cams; ++i)
    {
      _v_cameras.emplace_back(node_input_images[i].string(),
                              node_dist_models[i].string(),
                              node_prior_fovs[i].real());
    }
  }
  else
  {
    _v_cameras.emplace_back(_config_file_path);
  }
  
  cv::waitKey(2000);
}


void 
writeMultiCameras(const std::vector<Camera>& _v_cameras,
                  const std::string& _out_cam_path = "cam_calib.yaml")
{
  cv::FileStorage cam_calib_file(_out_cam_path, cv::FileStorage::WRITE);

  if(!cam_calib_file.isOpened()) 
  {
    std::cerr << "Failed to create " << _out_cam_path << " calib file...";
    exit(-1);
  }

  cam_calib_file.writeComment("Camera Calibration Parameters\n");
  for (size_t i=0; i < _v_cameras.size(); ++i)
  {
    const auto& cam = _v_cameras[i];

    cam_calib_file.writeComment("Camera #" + std::to_string(i));
    cam_calib_file.write("cam" + std::to_string(i), "");

    cam_calib_file.write("  input_images_folder", cam.m_input_images_path);
    cam_calib_file.write("  dist_model", cam.m_dist_model);

    const auto vcalib_params = cam.m_pcalib_params->getParameters();
    cam_calib_file.write("  intrisics_Parameters", cv::Mat(vcalib_params).t());

    const auto vdist_coefs = cam.m_pdist_params->getDistParameters();
    cam_calib_file.write("  distortion_Coefs", cv::Mat(vdist_coefs).t());

    if (i > 0)
    {
      const Eigen::Matrix<double,3,4> T_cam0_2_cam = cam.m_T_cam0_2_cam.matrix3x4();
      cv::Mat T;
      cv::eigen2cv(T_cam0_2_cam, T);
      cam_calib_file.writeComment("Transformation from cam0 frame to cam #" + std::to_string(i) + " frame.");
      cam_calib_file.write("  T_cam0_2_cam", T);
    }
  }

  std::cout << "\nCameras Calibration Results written to: " << _out_cam_path << "!\n\n";
}


int main(int argc, char* argv[])
{
  std::cout << "\n======================================\n";
  std::cout << "\tEZCalib - Multi-Camera Calibration";
  std::cout << "\n======================================\n";

  if (argc < 2)
  {
    std::cout << "\nUsage: ./ezcalib calib_config_path)\n";
    exit(-1);
  }

  // Setup Calib Problem
  const std::string calib_config_path = argv[1];

  EZMonoCalibrator ezmono_calibrator(calib_config_path);

  std::vector<Camera> v_cameras; 
  setupCameras(calib_config_path, v_cameras);

  for (auto& camera : v_cameras)
  {
    ezmono_calibrator.runCalibration(&camera);
  }

  if (v_cameras.size() > 1)
  {
    ezmono_calibrator.runMultiCameraCalib(v_cameras);

    writeMultiCameras(v_cameras);
  }
  else
  {
    v_cameras[0].writeCameraCalib();
  }

  return 0;
}