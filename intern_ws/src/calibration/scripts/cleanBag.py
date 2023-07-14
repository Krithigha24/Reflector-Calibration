#!/usr/bin/env python3
import os
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import math
from circle_fit import *

class RosbagCleaner:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.max_dist_from_lidar = 3.0 #m
        self.radius = 0.045 #m
        self.lidar_ang_res = 0.25 #deg
        self.ang_span = 2 * math.atan(self.radius/self.max_dist_from_lidar)
        self.raw_min_samples = (self.ang_span)/(self.lidar_ang_res * (math.pi/180))
        self.min_samples = math.ceil((self.ang_span)/(self.lidar_ang_res * (math.pi/180)))
        self.eps = (self.ang_span/self.raw_min_samples) * self.max_dist_from_lidar
        
    def filterScan_to_cartesian(self, scan_msg):
        points_info = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            points_info.append([x, y])
            angle += scan_msg.angle_increment
        return np.array(points_info)
        
    def perform_dbscan(self, points_info):
         # Check if there are any points with [0, 0] coordinates
        zero_points_mask = np.all(points_info == [0.0, 0.0], axis=1)
        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=self.eps , min_samples=self.min_samples)
        clusters = dbscan.fit_predict(points_info)
        # Assign the points with [0, 0] coordinates to the noise cluster (-1)
        clusters[zero_points_mask] = -1
        # Retrieve the cluster points
        cluster_points = []
        print("the unique labels are ", np.unique(clusters))
        for label in np.unique(clusters):
            if label != -1:  # Exclude noise points
                points = points_info[clusters == label]
                cluster_points.append((label,points)) 
        return np.array(clusters), cluster_points

    def clean_rosbags(self):
        # Iterate over each file in the folder
        for filename in os.listdir(self.folder_path):
            if filename.endswith('.bag') and filename.startswith('bs_'):  
                
                bag_path = os.path.join(self.folder_path, filename)
                bag = rosbag.Bag(bag_path)
                output_bag_path = os.path.join(self.folder_path, "cl_" + filename)
                output_bag = rosbag.Bag(output_bag_path, 'w')
                for topic, msg, t in bag.read_messages(topics=['/segmented_scan']):
                    
                    points_info = self.filterScan_to_cartesian(msg)
                    #print(len(points_info))
                    cluster, cluster_points = self.perform_dbscan(points_info)
                    # Print the cluster points
                    # diff = 1e99
                    main_cluster = 0
                    for i,(label, points) in enumerate(cluster_points):
                        print(f"Cluster {label}:")
                        print(points)
                        print("--------------------")
                        # xc, yc, r, sigma = kmh(points)
                        # print("radius is", r)
                        # if abs(self.radius - r) < diff:
                        #     main_cluster = label
                        #     diff = abs(self.radius - r)
                        #     print("diff is", diff)
                        # print(r)
                        # print("--------------------")
                    # print(main_cluster)

                    # Retrieve the corresponding values from array1
                    # print(main_cluster)
                    # #print(cluster)
                    # # print(list((np.array(msg.ranges))))
                    # msg.ranges = (np.array(msg.ranges)[np.where(cluster == main_cluster)]).tolist()
                    # msg.intensities = (np.array(msg.intensities)[np.where(cluster == main_cluster)]).tolist()
                    # # print(msg.intensities)
                    # output_bag.write('/cleaned_scan', msg, t)
                    # print(msg.ranges)
                # Close the bag after processing
                bag.close()
                output_bag.close()
        
# Get the current user's home directory
home_directory = os.path.expanduser("~")
# Define the relative path to the folder
relative_path = 'Reflector-Calibration/intern_ws/src/calibration/test_bags'
# Combine the home directory and the relative path
folder_path = os.path.join(home_directory, relative_path)
np.set_printoptions(threshold=np.inf)
# Create an instance of RosbagProcessor and process the ROS bags
cleaner = RosbagCleaner(folder_path)
cleaner.clean_rosbags()

################ AFTER BINARY SEGMENTATION, THERE ARE STILL SOME OTHER NOISY CLUSTERS ######################
########## THIS CODE CLEANS UP AND ISOLATES THE CLUSTER THAT BELONGS TO CYLINDRICAL REFLECTOR ONLY #########

    # I WANT TO INCLUDE A DBSCAN FUNCTION HERE THAT TAKES IN points_info AS INPUT
    # PERFORMS CLUSTERING ON THE (X.Y) COORDINATES IN points_info 
    # THE PARAMETERS OF THE DBSCAN ARE DETERMINED AS FOLLOWS:
    
    # SO AFTER SEGMENTATION STILL GOT SOME NOISY DATA POINTS
    # ASSUME THAT the reflector can be anywhere within 0.1m to 3.0m
    # Angular Resolution is 0.25 degrees/ 0.004363323096185923 rad
    # STEP 1 : Calculate angular span covered by the reflector 
    # i.e extent of the angular range that the cylindrical reflector occupies 
    # when viewed from the LiDAR sensor's perspective.
    # angular_span = 2 * arctan(radius / distance from lidar)
    # 2 * arctan(0.045 / 3.0) = 0.0299977503 rad
    # STEP 2 : Calculate the number of data points captured by the LiDAR
    # num_data_points = angular_span / angular_resolution
    # 0.0299977503 / 0.004363323096185923 = 6.8749 data points 
    # STEP 3 : approximate adjacent distance between the points
    # angular_distance = angular_span / num_data_points
    # 0.0299977503/ 6.8749 = 0.0043633726
    # linear_distance = angular_distance * distance
    # 0.0043633726 * 3.0 = 0.01309011781m
    # Worst case reflector at 3.0m [Perform DBSCAN with the following parameters]
    # Adjacent distance between points : 0.014m (round up)
    # Number of data points captured : 7 (round up)
    
    # Ok so a few potential clusters will be formed, some noise points eliminated in the process
    # I propose doing circle fit with the points using python circle_fit library 
    # the circle fit with the radius closest to the actual radius will be
    # the cluster belonging to the cylindrical reflector