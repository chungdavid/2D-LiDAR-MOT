#include <ros/ros.h>

#include "lidar_object_detection/lidar_object_detection_ros.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_object_detection_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    LidarObjectDetectionRos lidar_object_detection_ros(nh);

    ros::spin();
    
    return 0;
}