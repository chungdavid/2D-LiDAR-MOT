#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import ros_numpy.point_cloud2
from enum import Enum
import numpy as np

class LidarObjectDetector:
    pointcloud2_topic = '/lidar/hokuyo/pointcloud2_preprocessed' #the topic to read PointCloud2 messages from
    hokuyo_objects_topic = '/lidar/hokuyo/objects'

    class Criteria(Enum):
        AREA = 1
        CLOSENESS = 2
        VARIANCE = 3

    def __init__(self):
        self.pub = rospy.Publisher(self.hokuyo_objects_topic, String, queue_size=10) #make custom message
        self.sub = rospy.Subscriber(self.pointcloud2_topic, PointCloud2, self.lidar_object_detection_pipeline)

        #: Fitting criteria parameter
        self.criteria = self.Criteria.VARIANCE
        #: Minimum distance for closeness criteria parameter [m]
        self.min_dist_of_closeness_criteria = 0.01
        #: Angle difference parameter [deg]
        self.d_theta_deg_for_search = 1.0
        #: Range segmentation parameter [m]
        self.R0 = 0.02
        #: Range segmentation parameter [m]
        self.Rd = 0.001

    def lidar_object_detection_pipeline(self, msg):
        pc2_array_np = ros_numpy.point_cloud2.pointcloud2_to_array(msg) #numpy.void array
        
        pc2_x = pc2_array_np['x']
        pc2_y = pc2_array_np['y']
        pc2_distances = pc2_array_np['distances']

        clusters = self.cluster_lidar_points(pc2_x, pc2_y, pc2_distances) #array
        # rospy.loginfo_once(pc2_array_np.dtype)

        self.pub.publish("hello")

    def find_neighbours(self, point, num_points, ox, oy, od):
        radius = self.R0 + self.Rd * od[point] #radius required between points to be in cluster
        neighbours = []
        for i in range(num_points):
            dist = np.hypot(ox[point] - ox[i], oy[point] - oy[i])
            if dist <= radius:
                neighbours.append(i)
        return neighbours

    #adaptive range segmentation
    def cluster_lidar_points(self, ox, oy, od):
        num_points = len(ox)
        # rospy.loginfo(num_points) #test
        labels = np.full(num_points, 0, np.uint8) #each point has a label, 0 means the point hasn't been touched yet
        cluster_list = []
        # rospy.loginfo(cluster_list) #test
        # rospy.loginfo(labels) #test
        
        for i in range(num_points):
            # if labels[i] != 0: #point has already been checked
            #     continue
            # cluster = [i] #create cluster with current point in it
            # labels[i] = 1 #claim the current point
            neighbours = self.find_neighbours(i, num_points, ox, oy, od) #note that neighbours contain the current point

        #     j = 0
        #     while j < len(neighbours):
        #         point = neighbours[j]
        #         if labels[point]==0: #point is not claimed
        #             labels[point]=1 #claim the point
        #             cluster.append(point) #add the point to the cluster
        #             point_neighbours = self.find_neighbours(point, num_points, ox, oy, od)
        #             neighbours = neighbours + point_neighbours
        #         j+=1
        #     cluster_list.append(cluster)
        
        # rospy.loginfo(cluster_list)
        # rospy.loginfo(labels)
        # rospy.loginfo("Cluster finished!")

        #options to imporve clustering - downsample using voxel grid
        #use dbscan instead of euclidean clustering
        #write in c++ instead of python
        rospy.loginfo("hello")
        return cluster_list


    def fit_l_shape(sef, clusters):
        pass
        #return marker array message


if __name__ == "__main__":
    rospy.init_node('lidar_object_detection', anonymous=True)
    LidarObjectDetector()
    rospy.spin()
