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

# find_package(Boost REQUIRED COMPONENTS
#             program_options)


find_package(Eigen3 REQUIRED)

if (Eigen3_FOUND)
  message(STATUS "Found Eigen3 lib : ${Eigen3_VERSION} - ${EIGEN3_INCLUDE_DIRS}")
endif (Eigen3_FOUND)

find_package(OpenCV REQUIRED)

if(OpenCV_VERSION_MAJOR LESS 3)
  message(FATAL_ERROR "OpenCV 3 or 4 is required!  Current version : ${OpenCV_VERSION}" )
endif()


#Sophus
# First search for a custom build of Sophus within this project
find_package(Sophus REQUIRED
  HINTS ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/install/share/sophus/cmake
)

if (Sophus_FOUND)
  message("Found Sophus lib : ${Sophus_VERSION} - ${Sophus_DIR}")
endif (Sophus_FOUND)


# Ceres
# First search for a custom build of Ceres within this project
find_package(Ceres REQUIRED
  HINTS ${PROJECT_SOURCE_DIR}/thirdparty/ceres-solver/install/lib/cmake/Ceres/
)

if (Ceres_FOUND)
  message(STATUS "Found Ceres lib : ${CERES_VERSION} - ${Ceres_DIR}")
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

file(GLOB ETHZ_ATAG2_FILES "thirdparty/ethz_apriltag2/src/*.cc")
add_library(ethz_apriltag2_lib ${ETHZ_ATAG2_FILES})

target_include_directories(ethz_apriltag2_lib
  PUBLIC
    thirdparty/ethz_apriltag2/include
)

target_link_libraries(ethz_apriltag2_lib
  PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

add_executable(${PROJECT_NAME} apps/ezcalib.cpp
              src/ezcalibrator.cpp
              src/camera.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    Ceres::ceres
    ${OpenCV_LIBS}
    stdc++fs
    ethz_apriltag2_lib
    Sophus::Sophus
  INTERFACE
    Eigen3::Eigen
)       


# --- Installation Configuration ---

# --- RPATH Configuration for the Executable ---
# Tells the linker that the executable will look for shared libraries 
# in the 'lib' folder sibling to the 'bin' folder where the executable resides.
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    INSTALL_RPATH_USE_LINK_PATH FALSE 
    # Use $ORIGIN which is the directory of the executable at runtime.
)

# --- Installation Rules ---
message(STATUS ${PROJECT_NAME} " install path: " ${CMAKE_INSTALL_PREFIX})

# 1. Install the Executable to the 'bin' folder of the staging directory
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(FILES
        $<TARGET_FILE:opencv_core>
        $<TARGET_FILE:opencv_core>
        $<TARGET_FILE:opencv_imgproc>
        $<TARGET_FILE:opencv_calib3d>
        $<TARGET_FILE:opencv_imgcodecs>
        $<TARGET_FILE:opencv_highgui>
        # Add all other OpenCV modules used (e.g., highgui, features2d, etc.)
        DESTINATION lib
    )
# 2. Install the Dependent Libraries
# This is where you list all non-system shared libraries needed by your executable.
# They go into the 'lib' directory, which is the path defined in your RPATH.

# Optional: Install any necessary resource files (e.g., images, configs)
# install(DIRECTORY ${CMAKE_SOURCE_DIR}/settings 
#     DESTINATION ${CMAKE_INSTALL_PREFIX}
# )
# install(DIRECTORY ${CMAKE_SOURCE_DIR}/resources 
#     DESTINATION ${CMAKE_INSTALL_PREFIX}
# )
