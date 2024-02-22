#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg

class LidarPreprocessor:
    laserscan_topic = '/scan' #the topic to read LaserScan messages from
    pointcloud2_topic = 'preprocessed_lidar_points_xyz' #the topic to publish PointCloud2 messages to

    def __init__(self):
        self.pub = rospy.Publisher(self.pointcloud2_topic, PointCloud2)
        self.sub = rospy.Subscriber(self.laserscan_topic, LaserScan, self.lidar_preprocessing_pipeline)
        self.projector = lg.LaserProjection() #http://wiki.ros.org/laser_geometry#Python_Usage

    def lidar_preprocessing_pipeline(self, msg):
        #project to xyz coordinates and publish
        pc2 = self.projector.projectLaser(msg)
        pc2.header.frame_id = 'base_laser'

        self.pub.publish(pc2)


if __name__ == "__main__":
    rospy.init_node('lidar_preprocessing', anonymous=True)
    LidarPreprocessor()
    rospy.spin()
