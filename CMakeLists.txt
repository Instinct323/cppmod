cmake_minimum_required(VERSION 3.10)
project(main)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# CMake module
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# ROS-noetic header
include_directories(/opt/ros/noetic/include)
# ROS-foxy header
# file(GLOB ROS2_HEADER /opt/ros/foxy/include/*)
# include_directories(${ROS2_HEADER})

# My Libraries
find_package(utils REQUIRED)
if (utils_FOUND)
  add_library(utils STATIC ${utils_SRC})
  target_link_libraries(utils ${utils_LIBS})
endif ()

find_package(zjcv REQUIRED)
if (zjcv_FOUND)
  add_library(zjcv STATIC ${zjcv_SRC} ${utils_SRC})
  target_link_libraries(zjcv ${utils_LIBS})
endif ()

# demo
add_executable(demo demo.cpp)
target_link_libraries(demo utils)

# code insight
option(CODE_INSIGHT "Code Insight" OFF)
if (CODE_INSIGHT)

endif ()
