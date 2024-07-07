#pragma once

#include <ros/ros.h>

#include <custom_msgs/Track2DArray.h>

class LidarTrackVisualizationRos {
private:
    ros::Subscriber input_tracks_sub_;
    ros::Publisher vis_tracks_pub_;

public:
    LidarTrackVisualizationRos(ros::NodeHandle& nh);

    bool init(ros::NodeHandle& nh);

    void visualizeLidarTracks(const custom_msgs::Track2DArray::ConstPtr& msg);
};