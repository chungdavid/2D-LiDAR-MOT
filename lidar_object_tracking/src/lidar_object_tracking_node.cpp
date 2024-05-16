#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <custom_msgs/Detection2DArray.h>
#include <custom_msgs/Track2DArray.h>
#include "tracking/Track2D.hpp"
#include "tracking/hungarian-cpp/Hungarian.hpp"

class LidarObjectTrackingNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber input_detections_sub_;
    ros::Publisher output_tracks_pub_;

    std::string lidar_frame_;

    std::vector<Track2D> tracks_;
    unsigned int track_num_;
    double dist_threshold_;

    HungarianAlgorithm hung_algo_;

public:
    LidarObjectTrackingNode() 
        : nh_(),
        input_detections_sub_(nh_.subscribe("/lidar/hokuyo/detections_2d", 1000, &LidarObjectTrackingNode::lidarObjectTrackingPipeline, this)),
        output_tracks_pub_(nh_.advertise<custom_msgs::Track2DArray>( "/lidar/hokuyo/tracks_2d", 0)),
        lidar_frame_("hokuyo"), track_num_(0), dist_threshold_(0.5)
    {
        ;
    }

    void lidarObjectTrackingPipeline(const custom_msgs::Detection2DArray::ConstPtr& msg) {
        int num_tracks = tracks_.size();
        int num_detections = msg->detections.size();
        ros::Time curr_stamp = msg->detections[0].header.stamp;
        double dt = 0.1; //static for now, will make dynamic later

        //init tracks if they don't already exist
        //all tracks are unconfirmed and not coasted
        if(num_tracks == 0) {
            for(auto& detection : msg->detections) {
                tracks_.push_back(Track2D(
                    track_num_,
                    detection.position.x,
                    detection.position.y,
                    detection.theta,
                    detection.length,
                    detection.width,
                    dt
                ));
                ++track_num_;
            }
            return;
        }

        //predict next state for each track
        //all tracks are set to coasted
        for(auto& track : tracks_) {
            track.kalmanPredict();
        }
        
        //loop through each current track, find distance between track and new detections
        //currently using euclidean distance as cost metric, explore DIoU in the future
        std::vector<std::vector<double>> cost_matrix(num_tracks, std::vector<double>(num_detections));
        for(int i=0; i<num_tracks; ++i) {
            for(int j=0; j<num_detections; ++j) {
                cost_matrix[i][j] = compute_distance(tracks_[i], msg->detections[j]); 
            }
        }

        //use Hungarian Algorithm to associate tracks with detections if distance is below threshold
        std::vector<int> assignments;
        hung_algo_.Solve(cost_matrix, assignments);

        std::vector<bool> tracks_matched(num_tracks, false);
        std::vector<bool> detections_matched(num_detections, false);
        std::vector<pair<int,int>> matches;
        for(int i=0; i<num_tracks;++i) {
            if(assignments[i] != -1) {//track is assigned
                //check if distance between tracks and assigned detections is below threshold
                if(cost_matrix[i][assignments[i]] < dist_threshold_) {
                    //below threshold, add to list of matched pairs
                    matches.push_back(std::pair<int,int>(i,assignments[i]));
                    tracks_matched[i] = true;
                    detections_matched[assignments[i]] = true;
                }
            }
        }

        //kalmanUpdate() to update assigned tracks with new detections
        //assigned tracks are not coasted anymore, while existing tracks that are unmatched remain coasted
        for(int i=0; i<matches.size(); ++i) {
            custom_msgs::Detection2D detection = msg->detections[matches[i].second];
            tracks_[matches[i].first].kalmanUpdate(
                detection.position.x,
                detection.position.y,
                detection.theta,
                detection.length,
                detection.width
            );
        }

        //detections that were not assigned to existing tracks are added as new tracks
        //these tracks are unconfirmed and not coasted
        for(int i = 0; i<detections_matched.size(); ++i) {
            if(detections_matched[i] == false) {
                custom_msgs::Detection2D detection_to_add = msg->detections[i];
                tracks_.push_back(Track2D(
                    track_num_,
                    detection_to_add.position.x,
                    detection_to_add.position.y,
                    detection_to_add.theta,
                    detection_to_add.length,
                    detection_to_add.width,
                    dt
                ));
                ++track_num_;
            }
        }

        //update each track's detection history
        //note: if the track is currently coasted, that means a new detection didn't associate with it
        //if the track is not coasted, that means a detection was associated with it, or it is a new detection
        //tracks that are coasted for 4 time steps are deleted
        for (auto& track : tracks_){
            track.updateDetectionHistory();
            //TODO - delete tracks that have coasted for too long
            // if(track.isDeletable()) {
            //     std::swap(track, tracks_.back());
            //     tracks_.pop_back();
            // }
        }

        //populate detections message with confirmed tracks
        custom_msgs::Track2DArray confirmed_tracks_msg_array;
        //publish only confirmed tracks
        for(auto& track : tracks_) {
            if(track.isConfirmed()) {
                custom_msgs::Track2D confirmed_track_msg;
                confirmed_track_msg.header.frame_id = lidar_frame_;
                confirmed_track_msg.header.stamp = curr_stamp;
                confirmed_track_msg.length = track.length_;
                confirmed_track_msg.width = track.width_;
                confirmed_track_msg.position.x = track.position_[0];
                confirmed_track_msg.position.y = track.position_[1];
                confirmed_track_msg.position.z = 0;
                confirmed_track_msg.velocity.x = track.velocity_[0];
                confirmed_track_msg.velocity.y = track.velocity_[1];
                confirmed_track_msg.velocity.z = 0;
                confirmed_track_msg.theta = track.theta_; 
                confirmed_track_msg.id = track.track_id_;
                confirmed_tracks_msg_array.tracks.push_back(confirmed_track_msg);
            }
        }

        output_tracks_pub_.publish(confirmed_tracks_msg_array);
    }

    double compute_distance(Track2D& track, custom_msgs::Detection2D detection) {
        double diff_x = detection.position.x - track.position_[0];
        double diff_y = detection.position.y - track.position_[1]; 
        return diff_x*diff_x + diff_y*diff_y; //using squared distance
    }
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_object_tracking_node", ros::init_options::AnonymousName);
    LidarObjectTrackingNode lidar_object_tracking_node;
    ros::spin();
    return 0;
}