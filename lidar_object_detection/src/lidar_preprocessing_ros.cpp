#include "lidar_object_detection/lidar_preprocessing_ros.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

constexpr uint32_t QUEUE_SIZE = 5u;

LidarPreprocessingRos::LidarPreprocessingRos(ros::NodeHandle& nh)
    : input_scan_sub_(nh.subscribe("/scan", QUEUE_SIZE, &LidarPreprocessingRos::lidarPreprocessingPipeline, this)),
      output_cloud_pub_(nh.advertise<pcl::PCLPointCloud2>("/lidar/hokuyo/pointcloud2_preprocessed", QUEUE_SIZE)),
      output_cloud_pub_cropped_(nh.advertise<pcl::PCLPointCloud2>("/lidar/hokuyo/pointcloud2_preprocessed_cropped", QUEUE_SIZE)),
      lidar_frame_("hokuyo_link")
{
    sor_.setLeafSize (0.01, 0.01, 0.01); //1 cm
}

void LidarPreprocessingRos::lidarPreprocessingPipeline(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //convert LaserScan to PointCloud2
    //outliers are already removed in this step
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*msg, cloud);
    cloud.header.frame_id = lidar_frame_;

    //convert from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    //this is just done for completeness. sensor_msgs::PointCloud2 can be treated as a pcl::PointCloud2 wihout any conversion
    //there is an underlying templating mechanism that serializes sensor_msgs::PointCloud2 into pcl::PCLPointCloud2
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud, *pcl_cloud);
    // ROS_INFO("Num points starting: %d",pcl_cloud->width * pcl_cloud->height);

    //voxel grid filter for downsampling
    pcl::PCLPointCloud2::Ptr pcl_cloud_filtered(new pcl::PCLPointCloud2);
    sor_.setInputCloud (pcl_cloud);
    sor_.filter (*pcl_cloud_filtered);
    // ROS_INFO("Num points after voxel filter: %d", cloud_filtered.width * cloud_filtered.height);
    
    //crop the lidar data to a specific region
    pcl::PCLPointCloud2::Ptr pcl_cloud_cropped(new pcl::PCLPointCloud2);
    //filter along y
    pass_.setInputCloud(pcl_cloud_filtered); 
    pass_.setFilterFieldName ("y");
    pass_.setFilterLimits (-1.5,1.5);
    pass_.filter (*pcl_cloud_cropped);
    //filter along x
    pass_.setInputCloud(pcl_cloud_cropped); 
    pass_.setFilterFieldName ("x");
    pass_.setFilterLimits (0.25, 2);
    pass_.filter (*pcl_cloud_cropped);
    
    //bilateral or gaussian filter?

    //this is just to double check that the time stamp is preserved after all the conversions we did   
    // std::cout << "Timestamp of messsage in: "<< (*msg).header.stamp.sec << (*msg).header.stamp.nsec << "       Timestamp of message out: " << (*pcl_cloud_cropped).header.stamp << "\n"; 
    //..looks like converting to PCLPointCloud2 changes the stamp from nano seconds to milliseconds

    //publish the preprocessed LiDAR data
    // output_cloud_pub_.publish(*pcl_cloud_filtered);  
    //publish the preprocessed and cropped lidar data
    output_cloud_pub_cropped_.publish(*pcl_cloud_cropped);
}