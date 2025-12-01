cmake_minimum_required(VERSION 2.8.12)

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
set(CMAKE_CXX_FLAGS_RELEASE "-march=native -O3")

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "Found OpenMP ${OPENMP_CXX_VERSION} ${OpenMP_C_VERSION}!")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

if(OpenCV_VERSION_MAJOR LESS 3)
  message( FATAL_ERROR "OpenCV 3 or 4 is required!  Current version : ${OpenCV_VERSION}" )
endif()

if (Eigen3_FOUND)
  message(STATUS "Found Eigen3 lib : ${Eigen3_VERSION} - ${EIGEN3_INCLUDE_DIRS}")
endif (Eigen3_FOUND)

if (OpenCV_FOUND)
  message(STATUS "Found OpenCV lib : ${OpenCV_VERSION} - ${OpenCV_INCLUDE_DIRS}")
endif (OpenCV_FOUND)


#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME} SHARED 
            ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${OpenCV_LIBS}
  # INTERFACE
  
    Eigen3::Eigen
)
