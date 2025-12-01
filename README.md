# EZCALIB

Camera calibration toolbox.

Currently implemented distortion models:

* Radial 1 coef. : k1
* Radial 2 coef. : k1, k2
* Radial 3 coef. : k1, k2, k3

* Radial-Tangential 4 coef. : k1, k2, p1, p2
* Radial-Tangential 5 coef. : k1, k2, k3, p1, p2

* Radial-Tangential 8 coef. : k1, k2, k3, k4, k5, k6, p1, p2

* Kannala-Brandt (large FOV) model : k1, k2, k3, k4

Multi-camera calibration (stereo and N-cameras):

* All cameras are registered with respect to cam0
* At this point, only stereo has been evaluated yet and multi-cameras calibration requires that all cameras have an overlapping view with cam0

---

## Usage

``` bash
$ ./ezcalib config_file_path.yaml  (optionnal: out_calib_file_path)
```

We provide different config files as examples:
- calib_aprilgrid_mono_config.yaml
- calib_chessboard_mono_config.yaml
- calib_chessboard_stereo_config.yaml
- ...

For multi-camera setups, we exepect as input one folder per camera with the same image names for images taken simultaneously. 
For instance, for a stereo setup you must provide:

- left_cam:
  - image000.png
  - image001.png
  - image002.png
  - ...
  - imageXXX.png

- right_cam:
  - image000.png
  - image001.png
  - image002.png
  - ...
  - imageXXX.png

Where each pair of imageXXX.png from the different folders will be considered as a stereo pair (i.e. images taken simultaneously).

---

## Dependencies


* OpenCV 3 or 4
* Ceres (v2.1 or higher) : https://github.com/ceres-solver/ceres-solver
* Sophus : https://github.com/strasdat/Sophus

---

## Install

For convenience, we provide a script to build the application:

``` bash
$ chmod +x build.sh
$ ./build.sh
```

If you wish to use your own version of Opencv, Ceres and Sophus, you can skip this step and build manually:

``` bash
$ mkdir build
$ cd build
$ cmake ..
$ make -j$(nproc)
```


---

## To come

* More images input (videos, rosbags, ...)
* Better multi-camera calibration
