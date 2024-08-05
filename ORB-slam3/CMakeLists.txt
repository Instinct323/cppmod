project(ORBslam3)

include_directories(include)
file(GLOB SLAM_SRC src/*.cpp)

# add_executable(tumvi example/TUM-VI.cpp ${SLAM_SRC})
# target_link_libraries(tumvi zjcv ${UTILS_LIBS})

add_executable(kitti example/TUM-VI.cpp ${SLAM_SRC})
target_link_libraries(kitti zjcv ${UTILS_LIBS})
