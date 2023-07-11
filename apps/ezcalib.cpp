#include "ezcalibrator.hpp"

#include <opencv2/core/eigen.hpp>

#include <fstream>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

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
writeCamerasCalib(const std::vector<Camera>& _v_cameras,
                  const std::string& _out_cam_path = "cam_calib.yaml")
{
  fs::path out_cam_path(_out_cam_path);

  if (!out_cam_path.has_extension())
  {
    out_cam_path += ".yaml";
  }
  else if (out_cam_path.extension() != ".yaml")
  {
    out_cam_path.replace_extension("yaml");

    std::cout << "\n\nProvided cam path was not a yaml file: " << _out_cam_path;
    std::cout << "\nSwitching to new cam path: " << out_cam_path;
  }
  
  if (fs::exists(out_cam_path))
  {
    fs::remove(out_cam_path);
  }

  std::ofstream cam_calib_file(out_cam_path);

  if(!cam_calib_file.is_open()) 
  {
    std::cerr << "Failed to create " << out_cam_path << " calib file...";
    exit(-1);
  }

  cam_calib_file << std::fixed << std::setprecision(9);

  cam_calib_file << "%YAML:1.0\n";
  cam_calib_file << "---\n\n";

  cam_calib_file << "#Camera Calibration Parameters\n\n";
  for (size_t i=0; i < _v_cameras.size(); ++i)
  {
    const auto& cam = _v_cameras[i];

    cam_calib_file << "#Camera #" + std::to_string(i) + "\n";

    cam_calib_file << "cam" + std::to_string(i) << ":\n";

    cam_calib_file << "  input_images_folder: " << cam.m_input_images_path << "\n";
    cam_calib_file << "  dist_model: " << cam.m_dist_model << "\n";
    
    const auto vcalib_params = cam.m_pcalib_params->getParameters();
    cam_calib_file << "  intrisics_parameters: ["; 
    for (size_t j=0; j < vcalib_params.size(); ++j)
    {
      if (j < vcalib_params.size()-1)
        cam_calib_file << vcalib_params[j] << ", ";
      else
        cam_calib_file << vcalib_params[j] << "]\n";
    }

    const auto vdist_coefs = cam.m_pdist_params->getDistParameters();
    cam_calib_file << "  distortion_coefs: ["; 
    for (size_t j=0; j < vdist_coefs.size(); ++j)
    {
      if (j < vdist_coefs.size()-1)
        cam_calib_file << vdist_coefs[j] << ", ";
      else
        cam_calib_file << vdist_coefs[j] << "]\n";
    }

    if (i > 0)
    {
      const Eigen::Matrix<double,3,4> T_cam0_2_cam = cam.m_T_cam0_2_cam.matrix3x4();

      cam_calib_file << "  #Transformation from cam0 frame to cam #" + std::to_string(i) + " frame.\n";
      cam_calib_file << "  T_cam0_2_cam: [";
      for (int r=0; r < 3; ++r)
      {
        if (r > 0)
          cam_calib_file << "                ";
        for (int c=0; c < 4; ++c)
        {
          if (r == 2 && c == 3)
            cam_calib_file << T_cam0_2_cam(r,c) << "]\n";
          else
            cam_calib_file << T_cam0_2_cam(r,c) << ", ";
        }
        cam_calib_file << "\n";
      }
    }
  }

  std::cout << "\nCameras Calibration Results written to: " << out_cam_path << "!\n\n";
}


int main(int argc, char* argv[])
{
  std::cout << "\n===============================================\n";
  std::cout << "\tEZCalib - Multi-Camera Calibration";
  std::cout << "\n===============================================\n";

  if (argc < 2)
  {
    std::cout << "\nUsage: ./ezcalib calib_config_path (optionnal: out_calib_file_path)\n";
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
  }

  if (argc > 2)
  {
    const std::string out_calib_path = argv[2];
    writeCamerasCalib(v_cameras, out_calib_path);
  }
  else
    writeCamerasCalib(v_cameras);

  return 0;
}