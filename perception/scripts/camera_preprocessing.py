#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard PointCloud2 data!")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("preprocessed_lidar_points_xyz", PointCloud2, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()