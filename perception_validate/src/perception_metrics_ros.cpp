#include <ros/ros.h>

#include "perception_validate/perception_metrics_ros.hpp"
#include <std_msgs/Float32.h>
#include "gazebo_msgs/GetModelState.h"

constexpr uint32_t QUEUE_SIZE = 5u;

PerceptionMetricsRos::PerceptionMetricsRos(ros::NodeHandle& nh)
    : input_tracks_sub_(nh.subscribe("/lidar/hokuyo/tracks_2d", QUEUE_SIZE, &PerceptionMetricsRos::calculatePerceptionMetrics, this)),
      output_gospa_pub_(nh.advertise<std_msgs::Float32>("/gazebo/lidar/hokuyo/tracks_2d/gospa", QUEUE_SIZE)),
      gazebo_model_states_client_(nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state")),
      actors_({"animated_box_circles_1", "animated_box_straight_1", "animated_box_straight_2", "static_box_1"}) //hard code for now
    {
        ;
    }

void PerceptionMetricsRos::calculatePerceptionMetrics(const custom_msgs::Track2DArray::ConstPtr& msg) {
    
    // Get the state of each actor in the sim
    for(std::string& actor : actors_) {
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = actor;
        srv.request.relative_entity_name = "hokuyo_link";
        
        if (gazebo_model_states_client_.call(srv)) {
            ROS_INFO("  Position: x=%f, y=%f, z=%f", srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z);
            ROS_INFO("  Orientation: x=%f, y=%f, z=%f, w=%f", srv.response.pose.orientation.x, srv.response.pose.orientation.y, srv.response.pose.orientation.z, srv.response.pose.orientation.w);

            calculateGospaMetric();
        } else {
            ROS_ERROR("Failed to call service for %s", actor.c_str());
            return;
        }

    }
    

}

void PerceptionMetricsRos::calculateGospaMetric() {
    //TO DO
}
