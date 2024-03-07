cmake_minimum_required(VERSION 3.10)
project(main)

# ROS-noetic header
include_directories(/opt/ros/noetic/include)

# ROS-humble header
file(GLOB ROS2_HEADER /opt/ros/humble/include/*)
include_directories(${ROS2_HEADER})

# CMake module
list(APPEND CMAKE_MODULE_PATH
        /usr/local/share/g2o/cmake
)

# C++ 17
set(CMAKE_CXX_STANDARD 17)
# todo: -pg
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# source, header
file(GLOB SRC_FILES *.cpp)
message(STATUS "${PROJECT_NAME} SRC_FILES: ${SRC_FILES}")
include_directories(utils)

# packages
find_package(Glog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)

# g2o library
find_package(G2O REQUIRED)
set(G2O_LIBS
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_TYPES_SLAM3D}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CSPARSE_EXTENSION}
)

# executable
add_executable(main ${SRC_FILES})
target_link_libraries(main
        glog::glog
        fmt::fmt
        ${G2O_LIBS}
        ${OpenCV_LIBS}
)
