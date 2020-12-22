cmake_minimum_required(VERSION 3.15)
project(VO)

set(CMAKE_BUILD_TYPE Release)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_STANDARD 14)

############### dependencies ######################
# Eigen
include_directories("/usr/local/include/eigen3")

# OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# g2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(G2O REQUIRED)

include_directories(${G20_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        #        ${GTEST_BOTH_LIBRARIES}
                ${GLOG_LIBRARIES}
        #        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        )


add_executable(VO main.cpp
        src/frame.cpp include/frame.h
        src/features.cpp include/features.h
        src/mappoint.cpp include/mappoint.h
        src/map.cpp include/map.h
        src/frontend.cpp include/frontend.h
        src/config.cpp include/config.h
        include/camera.h src/camera.cpp
        include/algorithm.h
        src/backend.cpp include/backend.h
        src/viewer.cpp include/viewer.h src/visual_odmetry.cpp include/visual_odmetry.h src/dataset.cpp include/dataset.h)
target_link_libraries(VO ${THIRD_PARTY_LIBS})
