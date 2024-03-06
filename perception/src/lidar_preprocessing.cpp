#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>



class LidarPreprocessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;
    laser_geometry::LaserProjection projector_;
    std::string lidar_frame_;

public:
    LidarPreprocessor() {
        input_sub_ = nh_.subscribe("/scan", 1000, &LidarPreprocessor::lidarPreprocessingPipeline, this);
        output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar/hokuyo/pointcloud2_preprocessed", 1);
        lidar_frame_ = "hokuyo";
    }

    void lidarPreprocessingPipeline(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*msg, cloud);
        cloud.header.frame_id = lidar_frame_;
        output_pub_.publish(cloud);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_preprocessing", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    LidarPreprocessor lidar_preprocessor;
    ros::spin();
    return 0;
}