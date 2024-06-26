project(utils)

include_directories(include)
aux_source_directory(src/utils SRC)
add_library(${PROJECT_NAME} STATIC ${SRC})
target_link_libraries(${PROJECT_NAME} ${LIBRARY})
