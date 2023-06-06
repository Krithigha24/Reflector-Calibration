#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np

class ReflectorCoordinate:
    def __init__(self, mainClusterThreshold, reflectorRadius, DBSCANeps, DBSCANminClusterPts):
        self.mainClusterThreshold = mainClusterThreshold
        self.reflectorRadius = reflectorRadius
        self.DBSCANeps = DBSCANeps
        self.DBSCANminClusterPts = DBSCANminClusterPts
        self.filteredScan_subscriber = rospy.Subscriber('/filtered_scan', LaserScan, self.process_laser_scan)
    
    # Converts filtered laser scan message to a list of (x, y) points and respective angle of each point in angle_array.
    def filterScan_to_cartesian_and_angle(self, scan_msg):
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
    
      # Euclidian distance from point1 to point2, or from point1 to (0,0) if point2 input is not specified
    def dist_point2point(self, point1, point2=(0, 0)):
        Px = (point1[0]-point2[0]) ** 2
        Py = (point1[1]-point2[1]) ** 2
        return math.sqrt(Px + Py)
    
    def filterScanPts_adjacent_distances(self, points):
        adjacent_distances = []
        for i in range(len(points) - 1):
            adjacent_distances.append(self.dist_point2point(points[i],points[i+1]))
        return adjacent_distances
    
    '''# Compute chord length between two data points
    def chord_btw2points(self, point1, point2, angle_increment):
        dist1 = self.dist_point2point(point1)
        dist2 = self.dist_point2point(point2)
        # Calculate the length of the opposite side which is chord length
        chord_length = math.sqrt(dist1**2 + dist2**2 - 2*dist1*dist2*math.cos(angle_increment))
        return chord_length
    
    def filterScanPts_adjacent_chordLengths(self, points, angle_increment):
        adjacent_chords = []
        for i in range(len(points) - 1):
            adjacent_chords.append(self.chord_btw2points(points[i],points[i+1], angle_increment))
        return adjacent_chords'''
    
    # Performs DBSCAN clustering on the given points and returns the cluster labels.
    def perform_dbscan_clustering(self, points):
        dbscan = DBSCAN(eps=self.DBSCANeps, min_samples= self.DBSCANminClusterPts)
        labels = dbscan.fit_predict(points)
        return labels
    
    # From perform_dbscan_clustering function results, Identify the MAIN clusters from noise clusters 
    def remove_noise_from_DBSCANresults(self, labels):
        # Identify the main clusters
        cluster_sizes = np.bincount(labels[labels >= 0])
        main_cluster_labels = np.where(cluster_sizes > self.mainClusterThreshold )[0]
        # Change the labels to -1 for noise clusters
        filtered_labels = [-1 if l not in main_cluster_labels else l for l in labels]
        return filtered_labels

    # stores the start and end indices of each MAIN cluster, along with the positions of -1 values within each cluster.
    # The output type is a dictionary where the keys represent the unique labels found in the input array, 
    # and the values are tuples containing the start index, end index, and positions of -1 values (if any) within each cluster.
    def cluster_startEndNegativeIndices(self, input_array):
        indicedCluster = {}  # Dictionary to store the label indices
        label = None  # Current label being processed
        start_index = None  # Start index of the current label cluster
        end_index = None  # End index of the current label cluster
        for i in range(len(input_array)):
            if input_array[i] != -1:
                if label is None:
                    label = input_array[i]
                    start_index = i
                elif label != input_array[i]:
                     # If a new label is encountered, store the previous label cluster indices
                    indicedCluster[label] = (start_index, end_index)
                    label = input_array[i]
                    start_index = i
                end_index = i
        # Store the last label cluster indices if it exists
        if label is not None:
           indicedCluster[label] = (start_index, end_index)
        # Update the dictionary values to include positions of -1 values within each cluster
        for label, (start, end) in indicedCluster.items():
            indicedCluster[label] = (start, end) + tuple(idx for idx in range(start, end + 1) if input_array[idx] == -1)
        return indicedCluster


    def process_laser_scan(self, scan_msg):
        points, angle_array = self.filterScan_to_cartesian_and_angle(scan_msg)
        adjacent_distances = self.filterScanPts_adjacent_distances(points)
        rospy.loginfo("adjacent distances btw points in filtered scan:")
        rospy.loginfo(adjacent_distances)  
        rospy.loginfo("max dist= {}, min dist = {}".format(max(adjacent_distances),min(adjacent_distances)))
        #adjacent_chords = self.filterScanPts_adjacent_chordLengths(points,scan_msg.angle_increment)
        #rospy.loginfo("adjacent_chords btw points in filtered scan:")
        #rospy.loginfo(adjacent_chords)  

        """labels = self.perform_dbscan_clustering(points)
        filtered_labels = self.remove_noise_from_DBSCANresults(labels)

        rospy.loginfo("Labels array:")
        rospy.loginfo(labels)  # Print the labels array using rospy.loginfo()

        rospy.loginfo("Filtered Labels array:")
        rospy.loginfo(filtered_labels)  # Print the labels array using rospy.loginfo()

        indices = find_label_indices(labels)
        # Print the results of each MAIN cluster 
        for label, (start_index, end_index) in indices.items():
            rospy.loginfo("Label {} start index = {}, end index = {}".format(label, start_index, end_index))
            # Angle of the reflector wrt to lidar
            average = sum(angle_array[start_index:end_index+1]) / len(angle_array[start_index:end_index+1])
            rospy.loginfo("Angle {}".format(average))
            # Midpoint coordinates of the the reflector ARC
            arcmp_x = (points[start_index][0] + points[end_index][0])/2
            arcmp_y = (points[start_index][1] + points[end_index][1])/2
            rospy.loginfo("arc_x = {}, arc_y = {}".format(arcmp_x,arcmp_y))
            
            # Find coordinate on the cylindrical relfector that has the same x coordinates as the midpoint
            circum_coord = reflector_matchX(arcmp_x, points)
            rospy.loginfo("circum_x = {}, circum_y = {}".format(circum_coord[0],circum_coord[1]))

            # Move radius distance to locate midpoint coordinates of reflector 
            mp_x = circum_coord[0]

            if(circum_coord[1] < arcmp_y):
                mp_y = circum_coord[1] + RADIUS 
            else:
                mp_y = circum_coord[1] - RADIUS 
            
            
            rospy.loginfo("x = {}, y = {}".format(mp_x,mp_y))
            # Euclidean distance from midpoint coordinate to lidar (0,0)
            distance = euclidean_distance(mp_x,mp_y) 
            rospy.loginfo("METHOD 1 Distance {}".format(distance))

        #rospy.loginfo("points = {}".format(points))

def reflector_matchX(mp_x, points):
    closest_coordinate = None
    closest_distance = float('inf')
    
    for point in points:
        distance = abs(mp_x - point[0])
        if distance < closest_distance:
            closest_coordinate = point
            closest_distance = distance
    
    return closest_coordinate"""

    

if __name__ == '__main__':
    mainClusterThreshold = rospy.get_param('cluster_threshold') # Minimum points for a MAIN cluster density
    reflectorRadius = rospy.get_param('reflector_radius') # Known radius of cyindrical reflector 
    DBSCANeps = rospy.get_param('DBSCAN_epsilon') # Maximum distance between two points for them to be considered as part of same cluster
    DBSCANminClusterPts = rospy.get_param('DBSCAN_minpts') # Minimum number of points required to form a cluster

    rospy.init_node('reflector_coordinate')
    ReflectorCoordinate(mainClusterThreshold, reflectorRadius, DBSCANeps, DBSCANminClusterPts)
    rospy.spin()
   

