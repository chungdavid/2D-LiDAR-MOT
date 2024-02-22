#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class CameraPreprocessor:
    camera_topic = '/zed2/zed_node/left/image_rect_color'
    preprocessed_camera_topic = '/camera/zed_left/image_preprocessed'

    def __init__(self):
        self.pub = rospy.Publisher(self.preprocessed_camera_topic, Image, queue_size=10)
        self.sub = rospy.Subscriber(self.camera_topic, Image, self.camera_preprocessing_pipeline)

    def camera_preprocessing_pipeline(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard Image data!")

        preprocessed_frame = msg
        preprocessed_frame.header.frame_id = 'zed_left'
        self.pub.publish(preprocessed_frame)

if __name__ == "__main__":
    rospy.init_node('camera_preprocessing', anonymous=True)
    CameraPreprocessor()
    rospy.spin()