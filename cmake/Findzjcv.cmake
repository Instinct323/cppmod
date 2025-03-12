if (NOT zjcv_FOUND)

  # packages
  find_package(utils REQUIRED)

  # all library
  set(zjcv_LIBS
          ${UTILS_LIBS}
  )

  # header
  find_path(zjcv_INCLUDE_DIR
          NAMES zjcv/camera.hpp
          PATHS ${CMAKE_SOURCE_DIR}/zjcv/include
          /usr/local/include
          /usr/include
          NO_DEFAULT_PATH
  )

  # source
  file(GLOB zjcv_SRC ${CMAKE_SOURCE_DIR}/zjcv/src/*.cpp)

  # message
  if (zjcv_INCLUDE_DIR AND zjcv_SRC)
    message(STATUS "Found zjcv: ${zjcv_INCLUDE_DIR}")
    include_directories(${zjcv_INCLUDE_DIR})
    set(zjcv_SRC ${zjcv_SRC})
    set(zjcv_FOUND TRUE)
  else ()
    message(FATAL_ERROR "zjcv not found")
    set(zjcv_FOUND FALSE)
  endif ()

endif ()
