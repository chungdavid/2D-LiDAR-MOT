#pragma once

#include <ros/ros.h>

#include <custom_msgs/Track2DArray.h>

class PerceptionMetricsRos {
private:
    ros::Subscriber input_tracks_sub_;
    ros::Publisher output_gospa_pub_;
    ros::ServiceClient gazebo_model_states_client_;
    std::vector<std::string> actors_; // hard code here for now

public:
    PerceptionMetricsRos(ros::NodeHandle& nh);

    void calculatePerceptionMetrics(const custom_msgs::Track2DArray::ConstPtr& msg);

    void calculateGospaMetric();

};