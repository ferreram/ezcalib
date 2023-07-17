#!/bin/bash

echo ""
echo "Building Sophus lib!"
echo ""

cd thirdparty/Sophus

mkdir build
mkdir install
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../install/"
make -j4 install
cd ../../..

echo ""
echo "Building Ceres lib!"
echo ""

cd thirdparty/ceres-solver
mkdir build
mkdir install
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../install/" -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=ON
make -j4 install
cd ../../..
