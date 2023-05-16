#include "ezcalibrator.hpp"

int main(int argc, char* argv[])
{
  std::cout << "\n======================================\n";
  std::cout << "\tEZCalib - Stereo Calibration";
  std::cout << "\n======================================\n";

  if (argc < 2)
  {
    std::cout << "\nUsage: ./ezcalib_test calib_config_path)\n";
    exit(-1);
  }

  // Setup Calib Problem
  const std::string calib_config_path1 = argv[1];

  const cv::FileStorage fsSettings(calib_config_path1, cv::FileStorage::READ);

  if(!fsSettings.isOpened()) 
  {
    std::cerr << "Failed to open settings file...";
    exit(-1);
  }
  
  // const std::string input_folder_path = fsSettings["input_images_folder"].string();
  // std::cout << "\nInput folder path : " << input_folder_path << "\n";

  const cv::FileNode node_input_images = fsSettings["input_images_folder"];
  const cv::FileNode node_dist_models = fsSettings["dist_model"];

  std::cout << "\n node_input_images is Seq ? => " << node_input_images.isSeq() << "\n";
  std::cout << "\n node_input_images size ? => " << node_input_images.size() << "\n";

  std::cout << "\n node_dist_models is Seq ? => " << node_dist_models.isSeq() << "\n";
  std::cout << "\n node_dist_models size ? => " << node_dist_models.size() << "\n";

  for (size_t i=0; i < node_dist_models.size(); ++i)
  {
    std::cout << "\n node_input_images #" << i << " : " << node_input_images[i].string() << "\n";
    std::cout << "\n node_dist_models #" << i << " : " << node_dist_models[i].string() << "\n";

  }

  return 0;
}