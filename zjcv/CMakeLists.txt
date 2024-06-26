project(zjcv)

include_directories(include)
file(GLOB ZJCV_SRC src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${ZJCV_SRC} ${UTILS_SRC})

add_executable(tumvi example/slam/TUM-VI.cpp)
target_link_libraries(tumvi ${PROJECT_NAME} ${LIBRARY})
