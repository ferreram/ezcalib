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

* Only stereo has been evaluated yet
* All cameras are registered with respect to cam0
* At this point, multi-cameras calibration requires that all cameras have an overlapping view with cam0

# Usage

``` bash
./ezcalib config_file_path.yaml
```

## Dependencies

* OpenCV 4
* Ceres : https://github.com/ceres-solver/ceres-solver
* Sophus : https://github.com/strasdat/Sophus

Note: OpenCV 3 should also work, modifiy the CMakeLists.txt accordingly if you do not have OpenCV 4.  The current implementation does not yet follow the new Manifold paradigm but still relies on the LocalParametrization.

## To come

* More images input (videos, rosbags, ...)
* Better multi-camera calibration
