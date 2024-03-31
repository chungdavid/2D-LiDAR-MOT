# #!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Image, PointCloud2

# class LidarToCameraProjector:
#     camera_topic = '/zed2/zed_node/left/image_rect_color'
#     lidar_topic = '/lidar/hokuyo/pointcloud2_preprocessed
#     pub_topic = '/calibration/lidar_to_camera_projection'

#     def __init__(self):
#         self.pub = rospy.Publisher(self.pub_topic, Image, queue_size=10)
#         self.sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self.project_lidar_to_camera)

#     def project_lidar_to_camera(self, msg):
#         rospy.loginfo(rospy.get_caller_id() + "I heard Image data!")

#         preprocessed_frame = msg
#         preprocessed_frame.header.frame_id = 'zed_left'
#         self.pub.publish(preprocessed_frame)

# if __name__ == "__main__":
#     rospy.init_node('camera_preprocessing', anonymous=True)
#     CameraPreprocessor()
#     rospy.spin()