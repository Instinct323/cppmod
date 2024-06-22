cmake_minimum_required(VERSION 3.10)
project(main)

set(CMAKE_CXX_STANDARD 17)
# todo: -pg
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# ROS-noetic header
include_directories(/opt/ros/noetic/include)
# ROS-humble header
file(GLOB ROS2_HEADER /opt/ros/humble/include/*)
include_directories(${ROS2_HEADER})
# CMake module
list(APPEND CMAKE_MODULE_PATH
        /usr/local/share/g2o/cmake
)

# packages
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# all library
set(LIBRARY
        glog::glog
        fmt::fmt
        ${OpenCV_LIBS}
        yaml-cpp
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_TYPES_SLAM3D}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CSPARSE_EXTENSION}
)

# subdirectory
add_subdirectory(zjcv)

# test
add_executable(test main.cpp)
target_link_libraries(test ${LIBRARY})
