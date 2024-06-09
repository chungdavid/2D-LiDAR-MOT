#include <ros/ros.h>

#include "lidar_object_detection/lidar_detection_vis_ros.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_detection_visualization_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    LidarDetectionVisualizationRos lidar_detection_visualization_ros(nh);

    ros::spin();

    return 0;
}