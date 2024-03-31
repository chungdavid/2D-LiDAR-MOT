#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include "detection/Detection2D.hpp"

// #include <vector>

class LidarObjectDetectionNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;

    ros::Publisher vis_cluster_markers_pub_;
    ros::Publisher vis_detections_pub_;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;

    std::string lidar_frame_;

public:
    LidarObjectDetectionNode() : kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>) {
        input_sub_ = nh_.subscribe("/lidar/hokuyo/pointcloud2_preprocessed_cropped", 1000, &LidarObjectDetectionNode::lidarObjectDetectionPipeline, this);

        vis_cluster_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization/lidar_cluster_markers", 0);
        vis_detections_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization/lidar_detections", 0);

        ec_.setClusterTolerance (0.10); // 5cm
        ec_.setMinClusterSize (15);
        ec_.setSearchMethod (kd_tree_);

        lidar_frame_ = "hokuyo";
    }

    void lidarObjectDetectionPipeline(const pcl::PCLPointCloud2ConstPtr& msg)
    {
        //convert msg to pcl::PointXYZ to enable use with other PCL modules
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*msg, *xyz_cloud);
        // ROS_INFO("Height of input cloud: %d. Width of input cloud: %d",msg->height,msg->width);
        // ROS_INFO("Size of pcl::PointCloud<pcl::PointXYZ>: %ld", xyz_cloud_ptr->size());

        //clustering
        kd_tree_->setInputCloud (xyz_cloud);
        ec_.setInputCloud (xyz_cloud);
        std::vector<pcl::PointIndices> cluster_indices; //vector containing clusters of indices
        ec_.extract (cluster_indices);
        
        //perform lshape fitting
        std::vector<Detection2D> detections;
        for (const auto& cluster : cluster_indices) {
            int num_points = cluster.indices.size(); //num points in cluster 
            Eigen::MatrixXf cluster_matrix(2, num_points);
            int p = 0;
            for (const auto& idx : cluster.indices) {
                cluster_matrix(0, p) = (*xyz_cloud)[idx].x;
                cluster_matrix(1, p) = (*xyz_cloud)[idx].y;
                ++p;
            }
            detections.push_back(Detection2D(cluster_matrix));
        }
        
        //visualize the clusters
        visualizeClusterMarkers(cluster_indices, xyz_cloud);
        //visualize the detections (rects)
        visualizeDetections(detections);

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
        for(const auto& detection : detections) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = lidar_frame_;
            marker.ns = "detections";
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 0.02;

            geometry_msgs::Point p;
            std::array<float, 4> pX = detection.getRectPointsX();
            std::array<float, 4> pY = detection.getRectPointsY(); 
            // ROS_INFO("The points of rect %d are: (%f, %f) (%f, %f) (%f, %f) (%f, %f)", i, pX[0], pY[0], pX[1], pY[1], pX[2], pY[2], pX[3], pY[3]);

            for(int i = 0; i < 4; ++i) {
                p.x = pX[i];
                p.y = pY[i];
                marker.points.push_back(p);
            }
            p.x = pX[0];
            p.y = pY[0];
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