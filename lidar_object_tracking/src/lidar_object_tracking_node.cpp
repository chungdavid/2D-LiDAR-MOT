#include <ros/ros.h>

#include <custom_msgs/Detection2DArray.h>


class LidarObjectTrackingNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_detections_sub_;
    // ros::Publisher output_tracks_pub_;

    std::string lidar_frame_;

public:
    LidarObjectTrackingNode() {
        input_detections_sub_ = nh_.subscribe("/lidar/hokuyo/detections_2d", 1000, &LidarObjectTrackingNode::lidarObjectTrackingPipeline, this);
        // output_tracks_pub_ = nh_.advertise<custom_msgs::Tracks2DArray>( "/lidar/hokuyo/tracks_2d", 0);
        lidar_frame_ = "hokuyo";
    }

    void lidarObjectTrackingPipeline(const custom_msgs::Detection2DArray::ConstPtr& msg) {
        ROS_INFO("Message received");
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_object_tracking_node", ros::init_options::AnonymousName);
    LidarObjectTrackingNode lidar_object_tracking_node;
    ros::spin();
    return 0;
}