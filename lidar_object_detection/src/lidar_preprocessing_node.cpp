#include <ros/ros.h>

#include "lidar_object_detection/lidar_preprocessing_ros.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_preprocessing_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    LidarPreprocessingRos lidar_preprocessing_ros(nh);

    ros::spin();
    
    return 0;
}