cmake_minimum_required(VERSION 3.17)
project(VO)

set(CMAKE_CXX_STANDARD 14)

############### dependencies ######################
# Eigen
include_directories("/usr/local/include/eigen3")

# OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

#PCL



add_executable(VO main.cpp src/frame.cpp include/frame.h src/features.cpp include/features.h src/mappoint.cpp include/mappoint.h src/map.cpp include/map.h)
target_link_libraries(VO ${OpenCV_LIBS},${Sophus_LIBRARIES})
