#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include "lidar_object_tracking/lidar_track_vis_ros.hpp"

LidarTrackVisualizationRos::LidarTrackVisualizationRos(ros::NodeHandle& nh) {
    if(!init(nh)) {
        ros::requestShutdown;
    }
}

bool LidarTrackVisualizationRos::init(ros::NodeHandle& nh) {
    // initialize publisher and subscriber using rosparams
    std::string in_topic;
    std::string vis_topic;
    int sub_queue_size;
    int pub_queue_size;
    nh.param("/lidar_perception/tracking/out_topic", in_topic, std::string("/lidar/tracks_2d"));
    nh.param("/lidar_perception/tracking/vis_topic", vis_topic, std::string("/lidar/tracks_2d_markers"));
    nh.param("/lidar_perception/tracking/sub_queue_size", sub_queue_size, 1);
    nh.param("/lidar_perception/tracking/pub_queue_size", pub_queue_size, 5);

    input_tracks_sub_ = nh.subscribe(in_topic, sub_queue_size, &LidarTrackVisualizationRos::visualizeLidarTracks, this);
    vis_tracks_pub_ = nh.advertise<visualization_msgs::MarkerArray>(vis_topic, pub_queue_size);

    return true;
}


void LidarTrackVisualizationRos::visualizeLidarTracks(const custom_msgs::Track2DArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray marker_array;

    for(auto& track_msg : msg->tracks) {
        //bounding box marker
        visualization_msgs::Marker track_box_marker; //visualize the detection
        track_box_marker.header.frame_id = track_msg.header.frame_id;
        track_box_marker.header.stamp = track_msg.header.stamp;
        track_box_marker.ns = "track_box";
        track_box_marker.id = track_msg.id;
        track_box_marker.type = visualization_msgs::Marker::CUBE;
        track_box_marker.action = visualization_msgs::Marker::ADD;
        track_box_marker.pose.orientation.w = cosf(-track_msg.theta/2.0F);
        track_box_marker.pose.orientation.x = 0;
        track_box_marker.pose.orientation.y = 0;
        track_box_marker.pose.orientation.z = sinf(-track_msg.theta/2.0F);
        track_box_marker.color.r = 1.0;
        track_box_marker.color.a = 1.0;
        track_box_marker.pose.position.x = track_msg.position.x; 
        track_box_marker.pose.position.y = track_msg.position.y;
        track_box_marker.pose.position.z = 0;
        track_box_marker.scale.x = track_msg.width;
        track_box_marker.scale.y = track_msg.length;
        track_box_marker.scale.z = 0.001;
        track_box_marker.lifetime = ros::Duration(0.25);
        marker_array.markers.push_back(track_box_marker);

        //arrow marker
        visualization_msgs::Marker track_arrow_marker; //visualize the velocity
        track_arrow_marker.header.frame_id = track_msg.header.frame_id;
        track_arrow_marker.header.stamp = track_msg.header.stamp;
        track_arrow_marker.ns = "track_arrow";
        track_arrow_marker.id = track_msg.id;
        track_arrow_marker.type = visualization_msgs::Marker::ARROW;
        track_arrow_marker.action = visualization_msgs::Marker::ADD;
        track_arrow_marker.color.b = 1.0;
        track_arrow_marker.color.a = 1.0;
        track_arrow_marker.scale.x = 0.025;    //Shaft diameter of the arrow
        track_arrow_marker.scale.y = 0.05;    //Head  diameter of the arrow
        track_arrow_marker.pose.orientation.w = 1.0; 
        geometry_msgs::Point p;
        p.x = track_msg.position.x;
        p.y = track_msg.position.y;
        p.z = 0;
        track_arrow_marker.points.push_back(p);
        p.x = track_msg.position.x + track_msg.velocity.x;
        p.y = track_msg.position.y + track_msg.velocity.y;
        p.z = 0;
        track_arrow_marker.points.push_back(p);
        track_arrow_marker.lifetime = ros::Duration(0.25);
        marker_array.markers.push_back(track_arrow_marker);

        //track_id marker
        visualization_msgs::Marker track_id_marker; //visualize the track id at the center of the object            
        track_id_marker.header.frame_id = track_msg.header.frame_id;
        track_id_marker.header.stamp = track_msg.header.stamp;
        track_id_marker.ns = "track_id";
        track_id_marker.id = track_msg.id;
        track_id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        track_id_marker.action = visualization_msgs::Marker::ADD;
        track_id_marker.pose.orientation.w = 1.0;    
        track_id_marker.scale.z = 0.1;
        track_id_marker.color.a = 1.0;
        track_id_marker.color.g = 1;
        track_id_marker.pose.position.x = track_msg.position.x;   
        track_id_marker.pose.position.y = track_msg.position.y;   
        track_id_marker.pose.position.z = 0.2;
        track_id_marker.text = std::to_string(track_msg.id);
        track_id_marker.lifetime = ros::Duration(0.25);
        marker_array.markers.push_back(track_id_marker);
    }        

    vis_tracks_pub_.publish(marker_array);
}