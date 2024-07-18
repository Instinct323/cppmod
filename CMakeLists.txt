cmake_minimum_required(VERSION 3.10)
project(main)

set(CMAKE_CXX_STANDARD 17)
# todo: -pg
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# CMake module
list(APPEND CMAKE_MODULE_PATH
        /home/workbench/cppmod/cmake
)

# ROS-noetic header
include_directories(/opt/ros/noetic/include)
# ROS-humble header
file(GLOB ROS2_HEADER /opt/ros/humble/include/*)
include_directories(${ROS2_HEADER})

find_package(utils REQUIRED)
find_package(zjcv REQUIRED)

# my library
add_library(utils STATIC ${UTILS_SRC})
add_library(zjcv STATIC ${ZJCV_SRC} ${UTILS_SRC})
target_link_libraries(utils ${UTILS_LIBS})
target_link_libraries(zjcv ${UTILS_LIBS})

# subdirectory
add_subdirectory(ORB-slam3)

# test
add_executable(test main.cpp)
target_link_libraries(test utils ${UTILS_LIBS})
