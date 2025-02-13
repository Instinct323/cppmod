cmake_minimum_required(VERSION 3.10)
project(main)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# CMake module
list(APPEND CMAKE_MODULE_PATH
        /media/tongzj/disk/Workbench/cppmod/cmake
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
target_link_libraries(zjcv ${UT-ILS_LIBS})

# test
add_executable(test main.cpp)
target_link_libraries(test utils ${ZJCV_LIBS})

# code insight
option(CODE_INSIGHT "Code Insight" OFF)
if (CODE_INSIGHT)
  
endif ()
