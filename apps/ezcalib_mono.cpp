#include "ezcalibrator.hpp"

int main(int argc, char* argv[])
{
  std::cout << "\n======================================\n";
  std::cout << "\tEZCalib - Monocular Calibration";
  std::cout << "\n======================================\n";

  if (argc < 2)
  {
    std::cout << "\nUsage: ./ezcalib_mono calib_config_path)\n";
    exit(-1);
  }

  // Setup Calib Problem
  const std::string calib_config_path = argv[1];

  Camera camera(calib_config_path);

  EZMonoCalibrator ezmono_calibrator(calib_config_path);

  ezmono_calibrator.runCalibration(&camera);

  camera.writeCameraCalib("cam0_calib.yaml");

  return 0;
}