#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/io.h>


class LidarObjectDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    ros::Publisher vis_pub_; //for visualization

public:
    LidarObjectDetector() {
        input_sub_ = nh_.subscribe("/lidar/hokuyo/pointcloud2_preprocessed", 1000, &LidarObjectDetector::lidarObjectDetectionPipeline, this);
        output_pub_ = nh_.advertise<pcl::PCLPointCloud2>("/lidar/hokuyo/objects", 1);
        // vis_pub_ = nh.advertise<visualization_msgs::Marker>( "/visualization/lidar_clusters", 0 );
    }

    void lidarObjectDetectionPipeline(const pcl::PCLPointCloud2ConstPtr& msg)
    {
        //convert msg to pcl::PointXYZ to enable use with other PCL modules
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*msg, *xyz_cloud_ptr);
        ROS_INFO("Height of input cloud: %d. Width of input cloud: %d",msg->height,msg->width);
        ROS_INFO("Size of pcl::PointCloud<pcl::PointXYZ>: %ld", xyz_cloud_ptr->size());

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);
        kd_tree->setInputCloud (xyz_cloud_ptr);
        // create the extraction object for the clusters
        std::vector<pcl::PointIndices> cluster_indices; //vector containing clusters of indices
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // specify euclidean cluster parameters
        ec.setClusterTolerance (0.05); // 5cm
        ec.setMinClusterSize (4);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (kd_tree);
        ec.setInputCloud (xyz_cloud_ptr);
        // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
        ec.extract (cluster_indices);
        ROS_INFO("There are %ld clusters in this scan.",cluster_indices.size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_clusters_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_ptr (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : cluster.indices) {
                cloud_cluster_ptr->push_back((*xyz_cloud_ptr)[idx]);
            }
            cloud_cluster_ptr->width = cloud_cluster_ptr->size();
            cloud_cluster_ptr->width = 1;
            cloud_cluster_ptr->is_dense = true;
            // ROS_INFO("This cluster has %ld points.", cloud_cluster_ptr->size());
            *all_cloud_clusters_ptr += *cloud_cluster_ptr;
        }

        // //using this code for now so we can differentiate the clusters by colour
        // //for some reason no points show up on rviz, might be because of how I'm setting the x,y,z values
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud_clusters_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        // for (const auto& cluster : cluster_indices)
        // {
        //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        //     int r = rand() % 255;
        //     int g = rand() % 255;
        //     int b = rand() % 255;

        //     for (const auto& idx : cluster.indices) {
        //         float x = (*xyz_cloud_ptr)[idx].x;
        //         float y = (*xyz_cloud_ptr)[idx].y;
        //         float z = (*xyz_cloud_ptr)[idx].z;
        //         pcl::PointXYZRGB* xyzrgb_point = new pcl::PointXYZRGB(x, y, z);
        //         xyzrgb_point->r = r;
        //         xyzrgb_point->g = g;
        //         xyzrgb_point->b = b;
        //         cloud_cluster_ptr->push_back(*xyzrgb_point);
        //     }
        //     cloud_cluster_ptr->width = cloud_cluster_ptr->size();
        //     cloud_cluster_ptr->width = 1;
        //     cloud_cluster_ptr->is_dense = true;
        //     ROS_INFO("This cluster has %ld points.", cloud_cluster_ptr->size());

        //     *all_cloud_clusters_ptr += *cloud_cluster_ptr;
        // }

        pcl::PCLPointCloud2 objects;
        pcl::toPCLPointCloud2(*all_cloud_clusters_ptr, objects);
        objects.header.frame_id = "hokuyo";
        output_pub_.publish(objects);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_object_detection", ros::init_options::AnonymousName);
    LidarObjectDetector lidar_object_detector;
    ros::spin();
    return 0;
}