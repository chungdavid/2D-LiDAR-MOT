#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg

class LidarPreprocessor:
    laserscan_topic = '/scan' #the topic to read LaserScan messages from
    pointcloud2_topic = '/lidar/hokuyo/pointcloud2_preprocessed' #the topic to publish PointCloud2 messages to
    CHANNEL_OPTIONS = (0x01|0x02|0x04) #01=intensities, 02=index,04=distance,08=timestamp

    def __init__(self):
        self.pub = rospy.Publisher(self.pointcloud2_topic, PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber(self.laserscan_topic, LaserScan, self.lidar_preprocessing_pipeline)
        self.projector = lg.LaserProjection() #http://wiki.ros.org/laser_geometry#Python_Usage

    def lidar_preprocessing_pipeline(self, msg):
        #project to xyz coordinates
        #also removes points greater than max_range and less than min_range [0.02m to 30.0m]
        pc2 = self.projector.projectLaser(msg, channel_options=self.CHANNEL_OPTIONS)
        pc2.header.frame_id = 'hokuyo'

        #downsampling not required because subsequent clustering algorithm accounts for distance of points frrom LiDAR

        #ROI cropping not needed since we want the entire view

        # rospy.loginfo_once(msg)
        # rospy.loginfo_once(pc2)
        self.pub.publish(pc2)


if __name__ == "__main__":
    rospy.init_node('lidar_preprocessing', anonymous=True)
    LidarPreprocessor()
    rospy.spin()
