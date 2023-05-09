cmake_minimum_required(VERSION 2.8)

project(ethz_apriltag2)

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

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "Found OpenMP ${OPENMP_CXX_VERSION} ${OpenMP_C_VERSION}!")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED NO_DEFAULT_PATH PATHS /home/maxime/libraries/opencv_4/install/)


#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME}  ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${OpenCV_LIBS}
  INTERFACE
    Eigen3::Eigen
)
