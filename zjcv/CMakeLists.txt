project(zjcv)

include_directories(include)
aux_source_directory(src SRC)
message(STATUS "Source Files: ${SRC}")

add_library(${PROJECT_NAME} STATIC ${SRC})
add_executable(tumvi example/slam/TUM-VI.cpp)
target_link_libraries(tumvi ${PROJECT_NAME} ${LIBRARY})
