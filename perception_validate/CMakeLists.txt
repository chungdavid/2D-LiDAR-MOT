cmake_minimum_required(VERSION 3.0.2)
project(perception_validate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS false)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    custom_msgs
    std_msgs
    gazebo_msgs
)


catkin_package()


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(perception_metrics_ros
    src/perception_metrics_ros.cpp
)

add_executable(perception_metrics_node src/perception_metrics_node.cpp)

target_link_libraries(perception_metrics_node
    perception_metrics_ros
    ${catkin_LIBRARIES}
)


