#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include "lidar_object_detection/detection/Detection2D.hpp"
#include <custom_msgs/Detection2DArray.h>

#include "lidar_object_detection/lidar_object_detection_ros.hpp"

LidarObjectDetectionRos::LidarObjectDetectionRos(ros::NodeHandle& nh)
    : kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
{
    if(!init(nh)) {
        ros::requestShutdown();
    }
}

bool LidarObjectDetectionRos::init(ros::NodeHandle& nh) {
    // set lidar frame using rosparams
    nh.param("/lidar_perception/frame", lidar_frame_, std::string("lidar_link"));

    // init subscribers and publishers using rosparams
    std::string in_topic;
    std::string out_topic;
    int sub_queue_size;
    int pub_queue_size;
    nh.param("/lidar_perception/preprocessing/out_topic", in_topic, std::string("/lidar/pointcloud2_preprocessed"));
    nh.param("/lidar_perception/detection/out_topic", out_topic, std::string("/lidar/detections_2d"));
    nh.param("/lidar_perception/detection/sub_queue_size", sub_queue_size, 1);
    nh.param("/lidar_perception/detection/pub_queue_size", pub_queue_size, 5);

    input_cloud_sub_ = nh.subscribe(in_topic, sub_queue_size, &LidarObjectDetectionRos::lidarObjectDetectionPipeline, this);
    output_detections_pub_ = nh.advertise<custom_msgs::Detection2DArray>(out_topic, pub_queue_size);
    // vis_cluster_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/lidar/cluster_markers", pub_queue_size);
    
    // init detection parameters using rosparams
    nh.param("/lidar_perception/detection/params/ec_cluster_tolerance", ec_cluster_tolerance_, 0.10f);
    nh.param("/lidar_perception/detection/params/ec_min_cluster_size", ec_min_cluster_size_, 15);
    nh.param("/lidar_perception/detection/params/ec_max_cluster_size", ec_max_cluster_size_, 20);
    
    ec_.setClusterTolerance (ec_cluster_tolerance_);
    ec_.setMinClusterSize (ec_min_cluster_size_);
    ec_.setMaxClusterSize (ec_max_cluster_size_);
    ec_.setSearchMethod (kd_tree_);

    return true;
}

void LidarObjectDetectionRos::lidarObjectDetectionPipeline(const pcl::PCLPointCloud2ConstPtr& msg) {
    //convert msg to pcl::PointXYZ to enable use with other PCL modules
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*msg, *xyz_cloud);

    //clustering
    kd_tree_->setInputCloud (xyz_cloud);
    ec_.setInputCloud (xyz_cloud);
    std::vector<pcl::PointIndices> cluster_indices; //vector containing clusters of indices
    ec_.extract (cluster_indices);

    // remove the clusters that come from wall points
    // this is a naive method, it just removes clusters that are above a certain length
    for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin(); it != cluster_indices.end();) {
        Eigen::Vector4f min_p, max_p;
        pcl::getMinMax3D(*xyz_cloud, *it, min_p, max_p);
        if(std::abs(max_p[0]-min_p[0]) > 1 || std::abs(max_p[1]-min_p[1]) > 1) {
            cluster_indices.erase(it);
        } else {
            ++it;
        }
    }
    
    // create the message that will store the detections
    custom_msgs::Detection2DArray detection_msg_array;
    detection_msg_array.header.frame_id = lidar_frame_;
    detection_msg_array.header.stamp.sec = (*msg).header.stamp / 1000000000;
    detection_msg_array.header.stamp.nsec = (*msg).header.stamp % 1000000000;
    
    //perform lshape fitting
    for (const auto& cluster : cluster_indices) {
        Eigen::MatrixXf cluster_matrix(2, cluster.indices.size()); //note that PCL assumes floats
        int p = 0;
        for (const auto& idx : cluster.indices) {
            cluster_matrix(0, p) = (*xyz_cloud)[idx].x;
            cluster_matrix(1, p) = (*xyz_cloud)[idx].y;
            ++p;
        }
        
        Detection2D detection_obj = Detection2D(cluster_matrix);
        
        //package the detection into a message
        custom_msgs::Detection2D detection_msg;
        detection_msg.length = detection_obj.length_;
        detection_msg.width = detection_obj.width_;
        detection_msg.theta = detection_obj.theta_;
        detection_msg.position.x = detection_obj.position_[0];
        detection_msg.position.y = detection_obj.position_[1];
        detection_msg.position.z = 0.0;
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

    // Visualize the clusters
    // visualizeClusterMarkers(cluster_indices, xyz_cloud, (*msg).header.stamp);
}

// void LidarObjectDetectionRos::visualizeClusterMarkers(std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_ptr, uint64_t pcl_time_stamp){
//     visualization_msgs::MarkerArray marker_array;
//     int i = 0;
//     for (const auto& cluster : cluster_indices) {
//         Eigen::Vector4f min_p, max_p;
//         pcl::getMinMax3D(*xyz_cloud_ptr, cluster, min_p, max_p);
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = lidar_frame_;
//         marker.header.stamp.sec = pcl_time_stamp / 1000000000;
//         marker.header.stamp.nsec = pcl_time_stamp % 1000000000;
//         marker.ns = "cluster marker";
//         marker.type = visualization_msgs::Marker::CUBE;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.id = i;
//         marker.pose.orientation.w = 1.0;
//         marker.color.r = 1.0;
//         marker.color.a = 0.35;
//         Eigen::Vector4f mid_p = (min_p + max_p) / 2; 
//         marker.pose.position.x = mid_p[0]; 
//         marker.pose.position.y = mid_p[1];
//         marker.pose.position.z = mid_p[2];
//         marker.scale.x = max_p[0] - min_p[0];
//         marker.scale.y = max_p[1] - min_p[1];
//         marker.scale.z = 0.001;
//         marker.lifetime = ros::Duration(0.25);
//         marker_array.markers.push_back(marker);
//         ++i;
//     }
//     vis_cluster_markers_pub_.publish(marker_array);
// }
