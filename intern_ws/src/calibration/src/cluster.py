#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np
from calibration.msg import Cluster

def convert_laser_scan_to_points(msg, x_array, y_array):
    points = []
    angle = msg.angle_min

    for r in msg.ranges:
        if r != 0.0:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points.append([x, y])
            x_array.append(x)
            y_array.append(y)
        angle += msg.angle_increment
    return points

def perform_dbscan_clustering(points):
    dbscan = DBSCAN(eps=0.01, min_samples=2)
    labels = dbscan.fit_predict(points)
    return labels

def process_laser_scan(msg):
    x_array = []
    y_array = []

    
    points = convert_laser_scan_to_points(msg, x_array, y_array)
    rospy.loginfo("Points array:")
    rospy.loginfo(points)  # Print the points array using rospy.loginfo()
    labels = perform_dbscan_clustering(points)

    cluster_msg = Cluster()
    cluster_msg.angle_min = msg.angle_min
    cluster_msg.angle_increment = msg.angle_increment
    cluster_msg.x = x_array
    cluster_msg.y = y_array
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
