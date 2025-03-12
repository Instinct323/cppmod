if (NOT utils_FOUND)

  # CMake module
  list(APPEND CMAKE_MODULE_PATH
          /usr/local/share/g2o/cmake
  )

  # packages
  find_package(G2O REQUIRED)
  find_package(Glog REQUIRED)
  find_package(fmt REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(Pangolin REQUIRED)
  find_package(realsense2 REQUIRED)
  find_package(yaml-cpp REQUIRED)

  # all library
  set(utils_LIBS
          glog::glog
          fmt::fmt
          ${OpenCV_LIBS}
          ${Pangolin_LIBRARIES}
          ${realsense2_LIBRARY}
          yaml-cpp

          -lboost_chrono
          -lboost_system
          -lboost_thread

          ${G2O_CORE_LIBRARY}
          ${G2O_SOLVER_CSPARSE}
          ${G2O_SOLVER_CSPARSE_EXTENSION}
          ${G2O_STUFF_LIBRARY}
          ${G2O_TYPES_SBA}
          ${G2O_TYPES_SLAM3D}
  )

  # header
  find_path(utils_INCLUDE_DIR
          NAMES utils/glog.hpp
          PATHS ${CMAKE_SOURCE_DIR}/utils/include
          /usr/local/include
          /usr/include
          NO_DEFAULT_PATH
  )

  # source
  file(GLOB utils_SRC ${CMAKE_SOURCE_DIR}/utils/src/*.cpp)

  # message
  if (utils_INCLUDE_DIR AND utils_SRC)
    message(STATUS "Found utils: ${utils_INCLUDE_DIR}")
    include_directories(${utils_INCLUDE_DIR})
    set(utils_SRC ${utils_SRC})
    set(utils_FOUND TRUE)
  else ()
    message(FATAL_ERROR "utils not found")
    set(utils_FOUND FALSE)
  endif ()

endif ()



