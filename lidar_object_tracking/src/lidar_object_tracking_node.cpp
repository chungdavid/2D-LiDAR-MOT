#include <ros/ros.h>

#include "lidar_object_tracking/lidar_object_tracking_ros.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_object_tracking_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    LidarObjectTrackingRos lidar_object_tracking_ros(nh);

    ros::spin();

    return 0;
}