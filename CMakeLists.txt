cmake_minimum_required(VERSION 2.8.3)
project(ezcalib)

if( ${CMAKE_VERSION} VERSION_LESS "3.8.2" )
    set(CMAKE_CXX_STANDARD 14)
else()
    set(CMAKE_CXX_STANDARD 17)
endif()

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

##################
## Dependencies ##
##################

find_package(Boost REQUIRED COMPONENTS
            program_options)

find_package(Eigen3 REQUIRED)

if (Eigen3_FOUND)
  message(STATUS "Found Eigen3 lib : ${Eigen3_VERSION} - ${EIGEN3_INCLUDE_DIRS}")
endif (Eigen3_FOUND)

find_package(OpenCV 4 REQUIRED)

if (OpenCV_FOUND)
  message(STATUS "Found OpenCV lib : ${OpenCV_VERSION} - ${OpenCV_INCLUDE_DIRS}")
endif (OpenCV_FOUND)

find_package(Sophus REQUIRED)

find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "Found OpenMP ${OPENMP_CXX_VERSION} ${OpenMP_C_VERSION}!")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########

add_subdirectory(ethz_apriltag2)

add_executable(${PROJECT_NAME}_mono apps/ezcalib_mono.cpp
              src/ezcalibrator.cpp)

target_include_directories(${PROJECT_NAME}_mono
  PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/ethz_apriltag2/include
)

target_link_libraries(${PROJECT_NAME}_mono
  PRIVATE
    Ceres::ceres
    ${OpenCV_LIBS}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    stdc++fs
    ethz_apriltag2
  INTERFACE
    Eigen3::Eigen
    Sophus::Sophus
)

add_executable(${PROJECT_NAME}_stereo apps/ezcalib_stereo.cpp
              src/ezcalibrator.cpp)
