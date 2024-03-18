#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

#include "perception/LShapeFitting.hpp"

class LidarObjectDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    // ros::Publisher output_pub_;
    ros::Publisher vis_cluster_pub_; //for visualization
    ros::Publisher vis_rect_pub_; //for visualization
    ros::Publisher vis_cluster_markers_pub_;
    LShapeFitting l_shape_fitter_;//l shape fitting class

public:
    LidarObjectDetector() {
        input_sub_ = nh_.subscribe("/lidar/hokuyo/pointcloud2_preprocessed_cropped", 1000, &LidarObjectDetector::lidarObjectDetectionPipeline, this);
        // output_pub_ = nh_.advertise<pcl::PCLPointCloud2>("/lidar/hokuyo/objects", 1);
        vis_cluster_pub_ = nh_.advertise<pcl::PCLPointCloud2>( "/visualization/lidar_clusters", 0);
        vis_cluster_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "/visualization/lidar_cluster_markers", 0);
        vis_rect_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization/lidar_rects", 0);
    }

    void lidarObjectDetectionPipeline(const pcl::PCLPointCloud2ConstPtr& msg)
    {
        //convert msg to pcl::PointXYZ to enable use with other PCL modules
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*msg, *xyz_cloud_ptr);
        // ROS_INFO("Height of input cloud: %d. Width of input cloud: %d",msg->height,msg->width);
        // ROS_INFO("Size of pcl::PointCloud<pcl::PointXYZ>: %ld", xyz_cloud_ptr->size());

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);
        kd_tree->setInputCloud (xyz_cloud_ptr);
        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster_indices; //vector containing clusters of indices
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // specify euclidean cluster parameters
        ec.setClusterTolerance (0.05); // 5cm
        ec.setMinClusterSize (6);
        ec.setSearchMethod (kd_tree);
        ec.setInputCloud (xyz_cloud_ptr);
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract (cluster_indices);
        // ROS_INFO("There are %ld clusters in this scan.",cluster_indices.size());
        
        //view the clusters in rviz
        // visualizeClusters(cluster_indices, xyz_cloud_ptr);
        visualizeClusterMarkers(cluster_indices, xyz_cloud_ptr);

        //get each cluster into a matrix  m = [[x1,x2],
        //                                     [x2,x3]]
        //fit a rectangle to it and append it to a vector
        std::vector<RectangleData> rects;
        for (const auto& cluster : cluster_indices) {
            int num_points = cluster.indices.size(); //num points in cluster 
            Eigen::MatrixXf cluster_matrix(2, num_points);
            int p = 0;
            for (const auto& idx : cluster.indices) {
                cluster_matrix(0, p) = (*xyz_cloud_ptr)[idx].x;
                cluster_matrix(1, p) = (*xyz_cloud_ptr)[idx].y;
                ++p;
            }
            RectangleData output_rect;
            l_shape_fitter_.fitRect(cluster_matrix, output_rect);
            rects.push_back(output_rect);
        }

        visualizeRects(rects);
    }

    void visualizeClusters(std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_ptr) {
        // assigns each cluster a different color so it can be visualized in rviz
        //this function can be improved... the cluster visibility isn't great
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud_clusters_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        int j = 0;
        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
            //generate random RGB value
            std::uint8_t r = 255 * (j%2);
            std::uint8_t g = 0;
            std::uint8_t b = 255 - r;
            ++j;

            for (const auto& idx : cluster.indices) {
                pcl::PointXYZRGB xyzrgb_point; 
                xyzrgb_point.x = (*xyz_cloud_ptr)[idx].x;
                xyzrgb_point.y = (*xyz_cloud_ptr)[idx].y;
                xyzrgb_point.z = (*xyz_cloud_ptr)[idx].z;
                xyzrgb_point.r = r;
                xyzrgb_point.g = g;
                xyzrgb_point.b = b;
                cloud_cluster_ptr->push_back(xyzrgb_point);
            }
            *all_cloud_clusters_ptr += *cloud_cluster_ptr;
        }
        pcl::PCLPointCloud2 objects;
        pcl::toPCLPointCloud2(*all_cloud_clusters_ptr, objects);
        objects.header.frame_id = "hokuyo";
        vis_cluster_pub_.publish(objects);
    }

    void visualizeClusterMarkers(std::vector<pcl::PointIndices>& cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud_ptr) {
        visualization_msgs::MarkerArray marker_array;
        int i = 0;
        for (const auto& cluster : cluster_indices) {
            Eigen::Vector4f min_p, max_p;
            pcl::getMinMax3D(*xyz_cloud_ptr, cluster, min_p, max_p);
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = "hokuyo";
            marker.header.stamp = ros::Time::now();
            marker.ns = "cluster marker";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
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

    void visualizeRects(std::vector<RectangleData>& rects) {
        visualization_msgs::MarkerArray marker_array;
        int i = 0;
        for(const auto& rect : rects) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "hokuyo";
            marker.header.stamp = ros::Time::now();
            marker.ns = "rectangle";
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 0.02;

            geometry_msgs::Point p;
            std::array<float, 4> pX = rect.getRectPointsX();
            std::array<float, 4> pY = rect.getRectPointsY(); 
            ROS_INFO("The points of rect %d are: (%f, %f) (%f, %f) (%f, %f) (%f, %f)", i, pX[0], pY[0], pX[1], pY[1], pX[2], pY[2], pX[3], pY[3]);

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
        vis_rect_pub_.publish(marker_array);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_object_detection", ros::init_options::AnonymousName);
    LidarObjectDetector lidar_object_detector;
    ros::spin();
    return 0;
}