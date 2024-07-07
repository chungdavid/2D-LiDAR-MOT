#include "lidar_object_detection/lidar_preprocessing_ros.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

LidarPreprocessingRos::LidarPreprocessingRos(ros::NodeHandle& nh) {
    if(!init(nh)) {
        ros::requestShutdown();
    }
}

bool LidarPreprocessingRos::init(ros::NodeHandle& nh) {
    // set lidar frame using rosparams
    nh.param("/lidar_perception/frame", lidar_frame_, std::string("hokuyo_link"));
    
    // init publisher and subscribers using rosparams
    std::string in_topic;
    std::string out_topic;
    int sub_queue_size;
    int pub_queue_size;
    nh.param("/sensors/hokuyo/topic", in_topic, std::string("/scan"));
    nh.param("/lidar_perception/preprocessing/out_topic", out_topic, std::string("/lidar/pointcloud2_preprocessed"));
    nh.param("/lidar_perception/preprocessing/sub_queue_size", sub_queue_size, 1);
    nh.param("/lidar_perception/preprocessing/pub_queue_size", pub_queue_size, 5);
    
    input_scan_sub_ = nh.subscribe(in_topic, sub_queue_size, &LidarPreprocessingRos::lidarPreprocessingPipeline, this);
    output_cloud_pub_ = nh.advertise<pcl::PCLPointCloud2>(out_topic, pub_queue_size);
    
    // set preprocessing parameters using rosparams
    nh.param("/lidar_perception/preprocessing/params/sor_leaf_size", sor_leaf_size_, 0.05f);
    nh.param("/lidar_perception/preprocessing/params/pass_upper_x", pass_upper_x_, 0.05f);
    nh.param("/lidar_perception/preprocessing/params/pass_lower_x", pass_lower_x_, -0.05f);
    nh.param("/lidar_perception/preprocessing/params/pass_upper_y", pass_upper_y_, 0.05f);
    nh.param("/lidar_perception/preprocessing/params/pass_lower_y", pass_lower_y_, -0.05f);

    sor_.setLeafSize (sor_leaf_size_, sor_leaf_size_, sor_leaf_size_);
    
    return true;
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
    pass_.setFilterLimits (pass_lower_y_, pass_upper_y_);
    pass_.filter (*pcl_cloud_cropped);
    //filter along x
    pass_.setInputCloud(pcl_cloud_cropped); 
    pass_.setFilterFieldName ("x");
    pass_.setFilterLimits (pass_lower_x_, pass_upper_x_);
    pass_.filter (*pcl_cloud_cropped);
    
    //bilateral or gaussian filter?

    //this is just to double check that the time stamp is preserved after all the conversions we did   
    // std::cout << "Timestamp of messsage in: "<< (*msg).header.stamp.sec << (*msg).header.stamp.nsec << "       Timestamp of message out: " << (*pcl_cloud_cropped).header.stamp << "\n"; 
    //..looks like converting to PCLPointCloud2 changes the stamp from nano seconds to milliseconds

    //publish the preprocessed and cropped lidar data
    output_cloud_pub_.publish(*pcl_cloud_cropped);
}