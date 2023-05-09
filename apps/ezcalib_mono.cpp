#include "ezcalibrator.hpp"

#include <set>
#include <experimental/filesystem>

#include <opencv2/imgcodecs.hpp>


namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[])
{
  std::cout << "\n======================================\n";
  std::cout << "\tEZCalib - Monocular Calibration";
  std::cout << "\n======================================\n";

  if (argc < 2)
  {
    std::cout << "\nUsage: ./ezcalib_mono img_folder calib_config_path)\n";
    exit(-1);
  }

  // Setup Calib Problem
  const std::string input_images_path = argv[1];
  const std::string calib_config_path = argv[2];

  EZMonoCalibrator ezmono_calibrator(calib_config_path);

  // Get image files in ascending order
  std::set<fs::path> set_ordered_images;

  for (const auto& file_path : fs::directory_iterator(input_images_path))
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
      if (ezmono_calibrator.processImage(gray_img))
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
  
  // Optimize the calibration parameters
  ezmono_calibrator.computeCalibration();

  return 0;
}