cmake_minimum_required(VERSION 3.10)
project(slam)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

# C++ 11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# package
find_package(fmt REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main
        fmt::fmt
        glog::glog
        ${OpenCV_LIBS})
