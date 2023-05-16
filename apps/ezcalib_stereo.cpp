#include "ezcalibrator.hpp"

int main(int argc, char* argv[])
{
  std::cout << "\n======================================\n";
  std::cout << "\tEZCalib - Stereo Calibration";
  std::cout << "\n======================================\n";

  if (argc < 2)
  {
    std::cout << "\nUsage: ./ezcalib_stereo calib_config_path)\n";
    exit(-1);
  }

  // Setup Calib Problem
  const std::string calib_config_path1 = argv[1];
  const std::string calib_config_path2 = argv[2];

  std::vector<Camera> v_cameras;
  v_cameras.emplace_back(calib_config_path1);
  v_cameras.emplace_back(calib_config_path2);

  EZMonoCalibrator ezmono_calibrator(calib_config_path1);

  ezmono_calibrator.runCalibration(&v_cameras[0]);
  ezmono_calibrator.runCalibration(&v_cameras[1]);

  ezmono_calibrator.runMultiCameraCalib(v_cameras);

  return 0;
}