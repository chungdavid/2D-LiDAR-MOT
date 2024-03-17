#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>


class LidarPreprocessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    ros::Publisher output_pub_cropped_;
    laser_geometry::LaserProjection projector_;
    std::string lidar_frame_; //we have to specify this at the beginning (during preprocessing) because the topic we read the raw hokuyo scan from does not have a frame id

public:
    LidarPreprocessor() {
        input_sub_ = nh_.subscribe("/scan", 1000, &LidarPreprocessor::lidarPreprocessingPipeline, this);
        output_pub_ = nh_.advertise<pcl::PCLPointCloud2>("/lidar/hokuyo/pointcloud2_preprocessed", 1);
        output_pub_cropped_ = nh_.advertise<pcl::PCLPointCloud2>("/lidar/hokuyo/pointcloud2_preprocessed_cropped", 1);
        lidar_frame_ = "hokuyo";
    }

    void lidarPreprocessingPipeline(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        //convert LaserScan to PointCloud2
        //outliers are already removed in this step
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*msg, cloud);
        cloud.header.frame_id = lidar_frame_;

        //convert from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
        //this is just done for completeness. sensor_msgs::PointCloud2 can be treated as a pcl::PointCloud2 wihout any conversion
        //there is an underlying templating mechanism that serializes sensor_msgs::PointCloud2 into pcl::PCLPointCloud2
        pcl::PCLPointCloud2* pcl_cloud = new pcl::PCLPointCloud2; 
        pcl_conversions::toPCL(cloud, *pcl_cloud);
        pcl::PCLPointCloud2ConstPtr pcl_cloud_ptr(pcl_cloud);
        // ROS_INFO("Num points starting: %d",pcl_cloud->width * pcl_cloud->height);

        //voxel grid filter for downsampling
        pcl::PCLPointCloud2::Ptr pcl_cloud_filtered_ptr(new pcl::PCLPointCloud2);
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (pcl_cloud_ptr);
        sor.setLeafSize (0.01, 0.01, 0.01); //1 cm
        sor.filter (*pcl_cloud_filtered_ptr);
        // ROS_INFO("Num points after voxel filter: %d", cloud_filtered.width * cloud_filtered.height);
        
        //crop the lidar data to a specific region
        pcl::PCLPointCloud2::Ptr pcl_cloud_cropped_ptr(new pcl::PCLPointCloud2);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        //filter along y
        pass.setInputCloud(pcl_cloud_filtered_ptr); 
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-1.5, 1.5);
        pass.filter (*pcl_cloud_cropped_ptr);
        //filter along x
        pass.setInputCloud(pcl_cloud_cropped_ptr); 
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-0.5, 3);
        pass.filter (*pcl_cloud_cropped_ptr);
        
        //bilateral or gaussian filter?

        //publish the preprocessed LiDAR data
        output_pub_.publish(*pcl_cloud_filtered_ptr);
        //publish the preprocessed and cropped lidar data
        output_pub_cropped_.publish(*pcl_cloud_cropped_ptr);
        
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_preprocessing", ros::init_options::AnonymousName);
    LidarPreprocessor lidar_preprocessor;
    ros::spin();
    return 0;
}