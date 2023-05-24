#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np
from calibration.msg import Cluster

THRESHOLD = 15  # Minimum points for a main cluster. Adjust as needed.

def convert_laser_scan_to_points(msg, index_array): #x_array, y_array):
    """
    Converts laser scan message to a list of (x, y) points and updates the x_array and y_array.
    """
    points = []
    angle = msg.angle_min

    for index, r in enumerate(msg.ranges):
        if r != 0.0:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append([x, y])
            index_array.append(index)
        angle += msg.angle_increment
    return points

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

def process_laser_scan(msg):
    """
    Processes the laser scan message by converting it to points, performing DBSCAN clustering,
    and publishing the cluster information.
    """
    index_array = []

    points = convert_laser_scan_to_points(msg, index_array)#, x_array, y_array)
    rospy.loginfo("Points array:")
    rospy.loginfo(points)  # Print the points array using rospy.loginfo()
    labels = perform_dbscan_clustering(points)

    cluster_msg = Cluster()
    #cluster_msg.angle_min = msg.angle_min
    #cluster_msg.angle_increment = msg.angle_increment
    cluster_msg.index = index_array
    cluster_msg.label = labels

    pub.publish(cluster_msg)

def main():
    rospy.init_node('cluster')
    rospy.Subscriber('/average_scan', LaserScan, process_laser_scan)
    global pub
    pub = rospy.Publisher('/cluster_points', Cluster, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
