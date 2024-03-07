#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>


class LidarPreprocessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    laser_geometry::LaserProjection projector_;
    std::string lidar_frame_; //we have to specify this at the beginnign (during preprocessing) because the topic we read the raw hokuyo scan from does not have a frame id

public:
    LidarPreprocessor() {
        input_sub_ = nh_.subscribe("/scan", 1000, &LidarPreprocessor::lidarPreprocessingPipeline, this);
        output_pub_ = nh_.advertise<pcl::PCLPointCloud2>("/lidar/hokuyo/pointcloud2_preprocessed", 1);
        lidar_frame_ = "hokuyo";
    }

    void lidarPreprocessingPipeline(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        //convert LaserScan to PointCloud2
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*msg, cloud);
        cloud.header.frame_id = lidar_frame_;

        //convert from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
        //this is just done for completeness. sensor_msgs::PointCloud2 can be treated as a pcl::PointCloud2 wihout any conversion
        //there is an underlying templating mechanism that serializes sensor_msgs::PointCloud2 into pcl::PCLPointCloud2
        pcl::PCLPointCloud2* pcl_cloud = new pcl::PCLPointCloud2; 
        pcl_conversions::toPCL(cloud, *pcl_cloud);
        pcl::PCLPointCloud2ConstPtr cloudPtr(pcl_cloud);
        // ROS_INFO("Num points starting: %d",pcl_cloud->width * pcl_cloud->width);

        //voxel grid filter
        pcl::PCLPointCloud2 cloud_filtered;
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (0.01, 0.01, 0.01);
        sor.filter (cloud_filtered);
        // ROS_INFO("Num points after voxel filter: %d", cloud_filtered.width * cloud_filtered.width);

        output_pub_.publish(cloud_filtered);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_preprocessing", ros::init_options::AnonymousName);
    LidarPreprocessor lidar_preprocessor;
    ros::spin();
    return 0;
}