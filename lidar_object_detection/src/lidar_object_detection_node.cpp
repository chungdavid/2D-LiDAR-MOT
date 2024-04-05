#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include "detection/Detection2D.hpp"
#include <custom_msgs/Detection2DArray.h>

class LidarObjectDetectionNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_cloud_sub_;
    ros::Publisher output_detections_pub_;

    ros::Publisher vis_cluster_markers_pub_;
    ros::Publisher vis_detections_pub_;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;

    std::string lidar_frame_;
    bool visualization_enabled_;

public:
    LidarObjectDetectionNode() : kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>) {
        input_cloud_sub_ = nh_.subscribe("/lidar/hokuyo/pointcloud2_preprocessed_cropped", 1000, &LidarObjectDetectionNode::lidarObjectDetectionPipeline, this);
        output_detections_pub_ = nh_.advertise<custom_msgs::Detection2DArray>( "/lidar/hokuyo/detections_2d", 0);

        ec_.setClusterTolerance (0.10); // 5cm
        ec_.setMinClusterSize (15);
        ec_.setSearchMethod (kd_tree_);

        lidar_frame_ = "hokuyo";
        nh_.getParam("/lidar_object_detection/visualization_enabled", visualization_enabled_);
        if(visualization_enabled_) {
            ROS_INFO("Visualiation has been enabled for lidar_object_detection_node.");
            vis_cluster_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization/lidar_cluster_markers", 0);
            vis_detections_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization/lidar_detections", 0);
        } else {
            ROS_INFO("Visualiation has been disabled for lidar_object_detection_node.");
        }
    }

    void lidarObjectDetectionPipeline(const pcl::PCLPointCloud2ConstPtr& msg)
    {
        //convert msg to pcl::PointXYZ to enable use with other PCL modules
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*msg, *xyz_cloud);

        //clustering
        kd_tree_->setInputCloud (xyz_cloud);
        ec_.setInputCloud (xyz_cloud);
        std::vector<pcl::PointIndices> cluster_indices; //vector containing clusters of indices
        ec_.extract (cluster_indices);
        
        //perform lshape fitting
        custom_msgs::Detection2DArray detection_msg_array;
        std::vector<Detection2D> detections;
        for (const auto& cluster : cluster_indices) {
            Eigen::MatrixXf cluster_matrix(2, cluster.indices.size());
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
            detection_msg.length = detection_obj.getLength();
            detection_msg.width = detection_obj.getWidth();
            Eigen::Quaternionf quat = detection_obj.getRotation();
            detection_msg.pose.orientation.w = quat.w();
            detection_msg.pose.orientation.x = quat.x();
            detection_msg.pose.orientation.y = quat.y() ;
            detection_msg.pose.orientation.z = quat.z() ;
            Eigen::Vector2f pos = detection_obj.getPosition();
            detection_msg.pose.position.x = pos(0);
            detection_msg.pose.position.y = pos(1);
            detection_msg.pose.position.z = 0.0;
            detection_msg_array.detections.push_back(detection_msg);
        }

        //publish the detections
        output_detections_pub_.publish(detection_msg_array);

        if(visualization_enabled_) {
            //visualize the clusters
            visualizeClusterMarkers(cluster_indices, xyz_cloud);
            //visualize the detections
            visualizeDetections(detections);
        }
    }

    void visualizeClusterMarkers(std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_ptr) {
        visualization_msgs::MarkerArray marker_array;
        int i = 0;
        for (const auto& cluster : cluster_indices) {
            Eigen::Vector4f min_p, max_p;
            pcl::getMinMax3D(*xyz_cloud_ptr, cluster, min_p, max_p);
            visualization_msgs::Marker marker;
            marker.header.frame_id = lidar_frame_;
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

    void visualizeDetections(std::vector<Detection2D> detections) {
        visualization_msgs::MarkerArray marker_array;
        int i = 0;
        //for detection boxes (width, length, rotation)
        for(const auto& detection : detections) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = lidar_frame_;
            marker.ns = "detections";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i;
            Eigen::Quaternionf quaternion = detection.getRotation();
            marker.pose.orientation.w = quaternion.w();
            marker.pose.orientation.x = quaternion.x();
            marker.pose.orientation.y = quaternion.y();
            marker.pose.orientation.z = quaternion.z();
            marker.color.r = 1.0;
            marker.color.a = 0.35;
            marker.pose.position.x = detection.getPosition()(0); 
            marker.pose.position.y = detection.getPosition()(1);
            marker.pose.position.z = 0.0;
            marker.scale.x = detection.getWidth();
            marker.scale.y = detection.getLength();
            marker.scale.z = 0.001;
            marker_array.markers.push_back(marker);
            ++i;
        }
        //for rectangle contours
        for(const auto& detection : detections) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = lidar_frame_;
            marker.ns = "rectangle contours";
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i;
            marker.pose.orientation.w = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;
            marker.scale.x = 0.02;
            geometry_msgs::Point p;
            std::vector<std::pair<float,float>> corners = detection.getCorners();
            for(int i = 0; i < 4; ++i) {
                p.x = corners[i].first;
                p.y = corners[i].second;
                marker.points.push_back(p);
            }
            p.x = corners[0].first;
            p.y = corners[0].second;
            marker.points.push_back(p);
            marker_array.markers.push_back(marker);
            ++i;
        }
        vis_detections_pub_.publish(marker_array);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_object_detection_node", ros::init_options::AnonymousName);
    LidarObjectDetectionNode lidar_object_detection_node;
    ros::spin();
    return 0;
}