#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include "lidar_object_detection/lidar_detection_vis_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;

LidarDetectionVisualizationRos::LidarDetectionVisualizationRos(ros::NodeHandle& nh)
    : input_detections_sub_(nh.subscribe("/lidar/hokuyo/detections_2d", QUEUE_SIZE, &LidarDetectionVisualizationRos::visualizeLidarDetections, this)),
      vis_detections_pub_(nh.advertise<visualization_msgs::MarkerArray>( "/visualization/lidar_detections", QUEUE_SIZE))
    {
        ;
    }

void LidarDetectionVisualizationRos::visualizeLidarDetections(const custom_msgs::Detection2DArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray marker_array;
    int i = 0;
    //for detection boxes (width, length, rotation)
    for(const auto& detection : msg->detections) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = detection.header.frame_id;
        marker.header.stamp = detection.header.stamp;
        marker.ns = "detections";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = i;

        // Eigen::Quaternionf rot_quat(cosf(-detection.theta_/2.0F), 0.0F, 0.0F, sinf(-detection.theta_/2.0F)); //w, x, y, z
        marker.pose.orientation.w = cosf(-detection.theta/2.0F);
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = sinf(-detection.theta/2.0F);

        marker.color.r = 1.0;
        marker.color.a = 0.35;
        marker.pose.position.x = detection.position.x; 
        marker.pose.position.y = detection.position.y;
        marker.pose.position.z = detection.position.z;
        marker.scale.x = detection.width;
        marker.scale.y = detection.length;
        marker.scale.z = 0.001;
        marker_array.markers.push_back(marker);
        ++i;
    }
    //for rectangle contours
    for(const auto& detection : msg->detections) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = detection.header.frame_id;
        marker.header.stamp = detection.header.stamp;
        marker.ns = "rectangle contours";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = i;
        marker.pose.orientation.w = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.5;
        marker.scale.x = 0.02;
        geometry_msgs::Point p;
        for(int i = 0; i < 4; ++i) {
            p.x = detection.corners[i].x;
            p.y = detection.corners[i].y;
            marker.points.push_back(p);
        }
        p.x = detection.corners[0].x;
        p.y = detection.corners[0].y;
        marker.points.push_back(p);
        marker_array.markers.push_back(marker);
        ++i;
    }
    vis_detections_pub_.publish(marker_array);
}