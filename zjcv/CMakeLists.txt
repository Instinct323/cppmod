project(zjcv)

include_directories(include)
aux_source_directory(src SRC)
add_library(${PROJECT_NAME} STATIC ${SRC})

add_executable(tumvi example/slam/TUM-VI.cpp)
target_link_libraries(tumvi utils ${PROJECT_NAME} ${LIBRARY})
