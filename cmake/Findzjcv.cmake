if (NOT ZJCV_FOUND)

  # packages
  find_package(utils REQUIRED)

  # all library
  set(ZJCV_LIBS
          ${UTILS_LIBS}
  )

  # header
  find_path(ZJCV_INCLUDE_DIR
          NAMES zjcv/camera.hpp
          PATHS ${CMAKE_SOURCE_DIR}/zjcv/include
          /usr/local/include
          /usr/include
          NO_DEFAULT_PATH
  )

  # source
  file(GLOB ZJCV_SRC ${CMAKE_SOURCE_DIR}/zjcv/src/*.cpp)

  # message
  if (ZJCV_INCLUDE_DIR AND ZJCV_SRC)
    message(STATUS "Found zjcv: ${ZJCV_INCLUDE_DIR}")
    include_directories(${ZJCV_INCLUDE_DIR})
    set(ZJCV_SRC ${ZJCV_SRC})
    set(ZJCV_FOUND TRUE)
  else ()
    message(FATAL_ERROR "zjcv not found")
    set(ZJCV_FOUND FALSE)
  endif ()

endif ()
