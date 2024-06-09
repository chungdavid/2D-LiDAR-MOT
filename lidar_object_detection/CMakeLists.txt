cmake_minimum_required(VERSION 3.0.2)
project(lidar_object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS false)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    custom_msgs
    laser_geometry
    pcl_ros
    pcl_conversions
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(lidar_preprocessing_ros src/lidar_preprocessing_ros.cpp)

add_library(lidar_object_detection src/detection/Detection2D.cpp)
add_library(lidar_object_detection_ros src/lidar_object_detection_ros.cpp)
target_link_libraries(lidar_object_detection_ros
    lidar_object_detection
    ${catkin_LIBRARIES}
)

add_library(lidar_detection_visualization_ros src/lidar_detection_vis_ros.cpp)



add_executable(lidar_preprocessing_node src/lidar_preprocessing_node.cpp)
target_link_libraries(lidar_preprocessing_node
    lidar_preprocessing_ros
    ${catkin_LIBRARIES}
)

add_executable(lidar_object_detection_node src/lidar_object_detection_node.cpp)
target_link_libraries(lidar_object_detection_node
    lidar_object_detection_ros
    ${catkin_LIBRARIES}
)

add_executable(lidar_detection_visualization_node src/lidar_detection_vis_node.cpp)
target_link_libraries(lidar_detection_visualization_node
    lidar_detection_visualization_ros
    ${catkin_LIBRARIES}
)


