#!/bin/bash

echo ""
echo "Building EZCALIB!"
echo ""

# This script builds third-party libraries for Ubuntu 20.04 / 22.04.
dir_path=$(pwd)

sudo apt update
sudo apt install -y build-essential cmake git

# Eigen3
sudo apt install -y libeigen3-dev

echo ""
echo "Do you need to install opencv? If you already have opencv installed, you can skip this step."
read -p "Install opencv? (y/n): " install_opencv
if [[ "$install_opencv" == "y" || "$install_opencv" == "yes" ]]; then
    echo ""
    echo "Installing OpenCV!"
    echo ""

    sudo apt install libopencv-dev python3-opencv
else
    echo ""
    echo "Skipping OpenCV installation."
fi


echo ""
echo "Building Ceres Solver!"
echo ""

# google-glog + gflags
sudo apt install -y libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt install -y libatlas-base-dev
# SuiteSparse (optional)
sudo apt install -y libsuitesparse-dev

cd thirdparty
git clone https://github.com/ceres-solver/ceres-solver
cd ceres-solver
git checkout 2.2.0
mkdir build
mkdir install
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../install/" -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TESTING=OFF
make -j$(nproc) install
cd $dir_path

echo ""
echo "Building Sophus lib!"
echo ""

cd thirdparty
git clone https://github.com/strasdat/Sophus
cd Sophus
mkdir build
mkdir install
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../install/" -DBUILD_SOPHUS_TESTS=OFF
make -j$(nproc) install
cd $dir_path


echo ""
echo "Building Ezcalib app!"
echo ""

mkdir build
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../install/"
make -j$(nproc)
