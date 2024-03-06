#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

void lidarObjectDetectionPipeline(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("I heard: something");
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "/lidar/hokuyo/objects_cpp");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/lidar/hokuyo/pointcloud2_preprocessed", 1000, lidarObjectDetectionPipeline);
    ros::spin();

    return 0;
}