cmake_minimum_required(VERSION 2.8.12)
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
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3")

##################
## Dependencies ##
##################

find_package(Boost REQUIRED COMPONENTS
            program_options)


find_package(Eigen3 REQUIRED)

if (Eigen3_FOUND)
  message(STATUS "Found Eigen3 lib : ${Eigen3_VERSION} - ${EIGEN3_INCLUDE_DIRS}")
endif (Eigen3_FOUND)

find_package(OpenCV REQUIRED)

if(OpenCV_VERSION_MAJOR LESS 3)
  message( FATAL_ERROR "OpenCV 3 or 4 is required!  Current version : ${OpenCV_VERSION}" )
endif()


#Sophus
# First search for a custom build of Sophus within this project
find_package(Sophus NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/build/)
find_package(Sophus REQUIRED)

if (Sophus_FOUND)
  message("Found Sophus lib : ${Sophus_VERSION} - ${Sophus_DIR}")
endif (Sophus_FOUND)


# Ceres
# First search for a custom build of Ceres within this project
find_package(Ceres NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/thirdparty/ceres-solver/install/)
find_package(Ceres REQUIRED)

if (Ceres_FOUND)
  message("Found Ceres lib : ${CERES_VERSION} - ${Ceres_DIR}")
endif (Ceres_FOUND)

# OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "Found OpenMP ${OPENMP_CXX_VERSION} ${OpenMP_C_VERSION}!")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###########
## Build ##
###########

add_subdirectory(thirdparty/backward-cpp)

add_subdirectory(ethz_apriltag2)

add_executable(${PROJECT_NAME} apps/ezcalib.cpp
              src/ezcalibrator.cpp
              src/camera.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/ethz_apriltag2/include
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Backward::Interface
  PRIVATE
    Ceres::ceres
    ${OpenCV_LIBS}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    stdc++fs
    ethz_apriltag2
    Sophus::Sophus
  INTERFACE
    Eigen3::Eigen
)       