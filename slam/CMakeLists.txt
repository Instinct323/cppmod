cmake_minimum_required(VERSION 3.10)
project(slam)

# Path
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)
include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
message(STATUS "${PROJECT_NAME} SRC_FILES: ${SRC_FILES}")

# C++ 11
set(CMAKE_CXX_STANDARD 11)
# todo: -pg
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# package
find_package(fmt REQUIRED)
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)

# g2o library
set(G2O_LIBS
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_TYPES_SLAM3D}
        ${G2O_SOLVER_CSPARSE}
        ${G2O_SOLVER_CSPARSE_EXTENSION}
)

add_executable(slam ${SRC_FILES})
target_link_libraries(slam
        glog::glog
        fmt::fmt
        ${G2O_LIBS}
        ${OpenCV_LIBS}
)
