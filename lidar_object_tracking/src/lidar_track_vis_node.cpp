#include <ros/ros.h>

#include "lidar_object_tracking/lidar_track_vis_ros.hpp"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_track_visualization_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    LidarTrackVisualizationRos lidar_track_visualization_ros(nh);

    ros::spin();
    
    return 0;
}