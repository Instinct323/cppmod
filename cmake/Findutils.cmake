if (NOT UTILS_FOUND)

  # CMake module
  list(APPEND CMAKE_MODULE_PATH
          /usr/local/share/g2o/cmake
  )

  # packages
  find_package(G2O REQUIRED)
  find_package(Glog REQUIRED)
  find_package(fbow REQUIRED)
  find_package(fmt REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(Pangolin REQUIRED)
  find_package(yaml-cpp REQUIRED)

  # all library
  set(UTILS_LIBS
          glog::glog
          ${fbow_LIBS}
          fmt::fmt
          ${OpenCV_LIBS}
          ${Pangolin_LIBRARIES}
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
  find_path(UTILS_INCLUDE_DIR
          NAMES utils/glog.hpp
          PATHS ${CMAKE_SOURCE_DIR}/utils/include
          /usr/local/include
          /usr/include
          NO_DEFAULT_PATH
  )

  # source
  file(GLOB UTILS_SRC ${CMAKE_SOURCE_DIR}/utils/src/*.cpp)

  # message
  if (UTILS_INCLUDE_DIR AND UTILS_SRC)
    message(STATUS "Found utils: ${UTILS_INCLUDE_DIR}")
    include_directories(${UTILS_INCLUDE_DIR})
    set(UTILS_SRC ${UTILS_SRC})
    set(UTILS_FOUND TRUE)
  else ()
    message(FATAL_ERROR "utils not found")
    set(UTILS_FOUND FALSE)
  endif ()

endif ()



