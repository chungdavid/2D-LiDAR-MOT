#pragma once

#include <ros/ros.h>

#include <custom_msgs/Detection2DArray.h>
#include "tracking/Track2D.hpp"
#include "tracking/Hungarian.hpp"

class LidarObjectTrackingRos {
private:
    ros::Subscriber input_detections_sub_;
    ros::Publisher output_tracks_pub_;

    HungarianAlgorithm hung_algo_;

    std::vector<Track2D> tracks_;
    unsigned int track_num_;
    double dist_threshold_;

    std::string lidar_frame_;


public:
    LidarObjectTrackingRos(ros::NodeHandle& nh);

    void lidarObjectTrackingPipeline(const custom_msgs::Detection2DArray::ConstPtr& msg);

    double compute_sqr_distance(Track2D& track, custom_msgs::Detection2D detection) const;
    
};