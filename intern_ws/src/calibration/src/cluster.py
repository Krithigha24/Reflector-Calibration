#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np
#from calibration.msg import Cluster

THRESHOLD = rospy.get_param('cluster_threshold')  # Minimum points for a main cluster. Adjust as needed.

def convert_laser_scan_to_points(scan_msg):
    """
    Converts laser scan message to a list of (x, y) points and respective angle of each point in angle_array.
    """
    points, angle_array = [], []
    angle = scan_msg.angle_min

    for r in scan_msg.ranges:
        if r != 0.0:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append([x, y])
            angle_array.append(angle)
        angle += scan_msg.angle_increment
    return points, angle_array

def perform_dbscan_clustering(points):
    """
    Performs DBSCAN clustering on the given points and returns the cluster labels.
    """
    dbscan = DBSCAN(eps=0.01, min_samples=2)
    labels = dbscan.fit_predict(points)

    # Identify the main clusters
    cluster_sizes = np.bincount(labels[labels >= 0])
    main_cluster_labels = np.where(cluster_sizes > THRESHOLD)[0]
    # Change the labels to -1 for non-main clusters
    labels = [-1 if l not in main_cluster_labels else l for l in labels]

    return labels

def find_label_indices(input_array):
    indices = {}

    label = None
    start_index = None
    end_index = None

    for i in range(len(input_array)):
        if input_array[i] != -1:
            if label is None:
                label = input_array[i]
                start_index = i
                end_index = i
            elif input_array[i] == label:
                end_index = i
            else:
                indices[label] = (start_index, end_index)
                label = input_array[i]
                start_index = i
                end_index = i

    if label is not None:
        indices[label] = (start_index, end_index)

    return indices

def process_laser_scan(scan_msg):
    """
    Processes the laser scan message by converting it to points, performing DBSCAN clustering,
    and publishing the cluster information.
    """
    points, angle_array = convert_laser_scan_to_points(scan_msg)
    labels = perform_dbscan_clustering(points)

    #rospy.loginfo("Points array:")
    #rospy.loginfo(points)  # Print the points array using rospy.loginfo()
    rospy.loginfo("Labels array:")
    rospy.loginfo(labels)  # Print the points array using rospy.loginfo()
    #rospy.loginfo("Angle array:")
    #rospy.loginfo(angle_array)  # Print the points array using rospy.loginfo()
    indices = find_label_indices(labels)
    # Print the results
    for label, (start_index, end_index) in indices.items():
        rospy.loginfo("Label {} start index = {}, end index = {}".format(label, start_index, end_index))
        average = sum(angle_array[start_index:end_index+1]) / len(angle_array[start_index:end_index+1])
        rospy.loginfo("Angle {}".format(average))
        mp_x = (points[start_index][0] + points[end_index][0])/2 
        mp_y = (points[start_index][1] + points[end_index][1])/2 
        distance = math.sqrt(mp_x**2 + mp_y**2)
        rospy.loginfo("distance {}".format(distance))

    #cluster_msg = Cluster()
    #cluster_msg.angle_min = msg.angle_min
    #cluster_msg.angle_increment = msg.angle_increment
    #cluster_msg.index = index_array
    #cluster_msg.label = labels

    #pub.publish(cluster_msg)

def main():
    rospy.init_node('cluster')
    rospy.Subscriber('/average_scan', LaserScan, process_laser_scan)
    #global pub
    #pub = rospy.Publisher('/cluster_points', Cluster, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
