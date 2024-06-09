#pragma once

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class LidarObjectDetectionRos {
private:
    ros::Subscriber input_cloud_sub_;
    ros::Publisher output_detections_pub_;
    ros::Publisher vis_cluster_markers_pub_;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;

    std::string lidar_frame_;

public:
    LidarObjectDetectionRos(ros::NodeHandle& nh);

    void lidarObjectDetectionPipeline(const pcl::PCLPointCloud2ConstPtr& msg);
    void visualizeClusterMarkers(std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_ptr, uint64_t pcl_time_stamp);
};

