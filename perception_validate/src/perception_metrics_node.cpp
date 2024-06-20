#include <ros/ros.h>

#include "perception_validate/perception_metrics_ros.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "perception_metrics_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    PerceptionMetricsRos perception_metrics_ros(nh);
    
    ros::spin();

    return 0;
}