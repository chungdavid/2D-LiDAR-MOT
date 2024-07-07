#pragma once

#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class LidarPreprocessingRos {
private:
    ros::Subscriber input_scan_sub_;
    ros::Publisher output_cloud_pub_;

    laser_geometry::LaserProjection projector_;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor_; //voxel gid filter
    pcl::PassThrough<pcl::PCLPointCloud2> pass_; //pass through filter

    float sor_leaf_size_;
    float pass_upper_x_;
    float pass_lower_x_;
    float pass_upper_y_;
    float pass_lower_y_;
    
    std::string lidar_frame_;

public:
    LidarPreprocessingRos(ros::NodeHandle& nh);

    bool init(ros::NodeHandle& nh);

    void lidarPreprocessingPipeline(const sensor_msgs::LaserScan::ConstPtr& msg);
};