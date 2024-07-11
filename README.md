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

---

## Dependencies


* OpenCV 3 or 4
* Ceres : https://github.com/ceres-solver/ceres-solver
* Sophus : https://github.com/strasdat/Sophus

Note: OpenCV 3 should also work, modifiy the CMakeLists.txt accordingly if you do not have OpenCV 4.  The current implementation does not yet follow the new Manifold paradigm but still relies on the LocalParametrization.



---

## Install

For convenience, we provide compatible version of ceres and sophus in the thirdparty folder.
You can build them as follows:

``` bash
$ chmod +x build_thirdparty.sh
$ ./build_thirdparty.sh
```

If you wish to use your own version of ceres and sophus, you can skip this step.

Note: If using Ubuntu 20.04 or higher, you might have to remove the "FindTBB.cmake" file in the "ceres-solver/cmake/" folder.

Then, you can simply build ezcalib as follows:

``` bash
$ mkdir build
$ cd build/
$ cmake ..
$ make -j4
```

---

## To come

* More images input (videos, rosbags, ...)
* Better multi-camera calibration
