#include "ezcalibrator.hpp"


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

  // If only one camera!
  if (!node_input_images.isSeq())
  {
    _v_cameras.emplace_back(_config_file_path);
    
    cv::waitKey(2000);
  }
  else
  {
    // Multi-camera
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

      cv::waitKey(2000);
    }
  }
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

  ezmono_calibrator.runMultiCameraCalib(v_cameras);

  return 0;
}