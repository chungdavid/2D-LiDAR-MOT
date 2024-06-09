#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include "detection/Detection2D.hpp"
#include <custom_msgs/Detection2DArray.h>

#include "lidar_object_detection/lidar_object_detection_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;

LidarObjectDetectionRos::LidarObjectDetectionRos(ros::NodeHandle& nh)
 : input_cloud_sub_(nh.subscribe("/lidar/hokuyo/pointcloud2_preprocessed_cropped", QUEUE_SIZE, &LidarObjectDetectionRos::lidarObjectDetectionPipeline, this)),
   output_detections_pub_(nh.advertise<custom_msgs::Detection2DArray>( "/lidar/hokuyo/detections_2d", QUEUE_SIZE)),
   vis_cluster_markers_pub_(nh.advertise<visualization_msgs::MarkerArray>( "/visualization/lidar_cluster_markers", QUEUE_SIZE)),
   kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>), lidar_frame_("hokuyo_link")
 {
    ec_.setClusterTolerance (0.10); // 5cm
    ec_.setMinClusterSize (15);
    ec_.setSearchMethod (kd_tree_);
 }

void LidarObjectDetectionRos::lidarObjectDetectionPipeline(const pcl::PCLPointCloud2ConstPtr& msg) {
    //convert msg to pcl::PointXYZ to enable use with other PCL modules
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*msg, *xyz_cloud);

    //If no point clouds, don't do anything
    
    //clustering
    kd_tree_->setInputCloud (xyz_cloud);
    ec_.setInputCloud (xyz_cloud);
    std::vector<pcl::PointIndices> cluster_indices; //vector containing clusters of indices
    ec_.extract (cluster_indices);
    
    //perform lshape fitting
    custom_msgs::Detection2DArray detection_msg_array;
    std::vector<Detection2D> detections;
    for (const auto& cluster : cluster_indices) {
        Eigen::MatrixXf cluster_matrix(2, cluster.indices.size()); //note that PCL assumes floats
        int p = 0;
        for (const auto& idx : cluster.indices) {
            cluster_matrix(0, p) = (*xyz_cloud)[idx].x;
            cluster_matrix(1, p) = (*xyz_cloud)[idx].y;
            ++p;
        }
        
        Detection2D detection_obj = Detection2D(cluster_matrix);
        detections.push_back(detection_obj); //only for visualization
        //package the detection into a message
        custom_msgs::Detection2D detection_msg;
        detection_msg.header.frame_id = lidar_frame_;
        detection_msg.length = detection_obj.length_;
        detection_msg.width = detection_obj.width_;
        detection_msg.theta = detection_obj.theta_;
        detection_msg.position.x = detection_obj.position_[0];
        detection_msg.position.y = detection_obj.position_[1];
        detection_msg.position.z = 0.0;
        detection_msg.header.stamp.sec = (*msg).header.stamp / 1000000000;
        detection_msg.header.stamp.nsec = (*msg).header.stamp % 1000000000;
        for(auto& corner : detection_obj.corner_list_) {
            geometry_msgs::Point point;
            point.x = corner.first;
            point.y = corner.second;
            point.z = 0;
            detection_msg.corners.push_back(point);
        }
        detection_msg_array.detections.push_back(detection_msg);
    }

    //publish the detections
    output_detections_pub_.publish(detection_msg_array);

    visualizeClusterMarkers(cluster_indices, xyz_cloud, (*msg).header.stamp);
}

void LidarObjectDetectionRos::visualizeClusterMarkers(std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_ptr, uint64_t pcl_time_stamp){
    visualization_msgs::MarkerArray marker_array;
    int i = 0;
    for (const auto& cluster : cluster_indices) {
        Eigen::Vector4f min_p, max_p;
        pcl::getMinMax3D(*xyz_cloud_ptr, cluster, min_p, max_p);
        visualization_msgs::Marker marker;
        marker.header.frame_id = lidar_frame_;
        marker.header.stamp.sec = pcl_time_stamp / 1000000000;
        marker.header.stamp.nsec = pcl_time_stamp % 1000000000;
        marker.ns = "cluster marker";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = i;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0;
        marker.color.a = 0.35;
        Eigen::Vector4f mid_p = (min_p + max_p) / 2; 
        marker.pose.position.x = mid_p[0]; 
        marker.pose.position.y = mid_p[1];
        marker.pose.position.z = mid_p[2];
        marker.scale.x = max_p[0] - min_p[0];
        marker.scale.y = max_p[1] - min_p[1];
        marker.scale.z = 0.001;
        marker_array.markers.push_back(marker);
        ++i;
    }
    vis_cluster_markers_pub_.publish(marker_array);
}
