cmake_minimum_required(VERSION 3.0.2)
project(lidar_object_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS false)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  custom_msgs
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)


catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(lidar_object_tracking
  src/tracking/Track2D.cpp
  src/tracking/kalman.cpp
  src/tracking/Hungarian.cpp
)
add_library(lidar_object_tracking_ros src/lidar_object_tracking_ros.cpp)
target_link_libraries(lidar_object_tracking_ros
  lidar_object_tracking
  ${catkin_LIBRARIES}
)

add_library(lidar_track_visualization_ros src/lidar_track_vis_ros.cpp)



add_executable(lidar_object_tracking_node src/lidar_object_tracking_node.cpp)
target_link_libraries(lidar_object_tracking_node
  lidar_object_tracking_ros
  ${catkin_LIBRARIES}
)

add_executable(lidar_track_visualization_node src/lidar_track_vis_node.cpp)
target_link_libraries(lidar_track_visualization_node
  lidar_track_visualization_ros
  ${catkin_LIBRARIES}
)


