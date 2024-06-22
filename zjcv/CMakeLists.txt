project(zjcv)

include_directories(include)
aux_source_directory(src SRC)
message(STATUS "Source Files: ${SRC}")

add_executable(tumvi example/slam/TUM-VI.cpp ${SRC})
target_link_libraries(tumvi ${LIBRARY})
