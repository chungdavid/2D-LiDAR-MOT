#pragma once

#include <ros/ros.h>

#include <custom_msgs/Detection2DArray.h>

class LidarDetectionVisualizationRos {
private:
    ros::Subscriber input_detections_sub_;
    ros::Publisher vis_detections_pub_;

public:
    LidarDetectionVisualizationRos(ros::NodeHandle& nh);

    bool init(ros::NodeHandle& nh);

    void visualizeLidarDetections(const custom_msgs::Detection2DArray::ConstPtr& msg);
};