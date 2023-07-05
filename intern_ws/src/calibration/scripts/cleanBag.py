#!/usr/bin/env python3
import os
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt
import re

class RosbagCleaner:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        
    def filterScan_to_cartesian_and_angle(self, scan_msg):
        points_info = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if r != 0.0:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                dist_to_lidar = self.dist_point2point((x,y))
                points_info.append([x, y, angle, dist_to_lidar])
            angle += scan_msg.angle_increment
        return points_info

    def clean_rosbags(self):
        # Iterate over each file in the folder
        for filename in os.listdir(self.folder_path):
            if filename.endswith('.bag') and filename.startswith('bs_'):  # Check if it's a ROS bag file
                bag_path = os.path.join(self.folder_path, filename)
                # Open the ROS bag
                bag = rosbag.Bag(bag_path)
                
                # Extract the measured distance to lidar from the filename
                measured_dist_to_lidar = int(self.extract_dist_from_filename(filename))
                
                computed_dist_array = []
                
                # Process the messages in the bag
                for topic, msg, t in bag.read_messages(topics=['/segmented_scan']):
                    # Perform computations with the messages
                    points_info = self.filterScan_to_cartesian_and_angle(msg)
                    
                    # Sort smallest to largest value of dist to lidar 
                    points_info.sort(key=lambda point: point[3])
                    
                    # Extract the first num_points (x, y) coordinates
                    coordinates = [(point[0], point[1]) for point in points_info[:self.num_normal_pts]]

                    # Calculate the average of the coordinates
                    average_x, average_y = np.mean(coordinates, axis=0)
                    
                    center_coordinates = self.calculate_coordinates_on_line(self.radius, (average_x,average_y))
                    computed_dist_to_lidar = self.dist_point2point(center_coordinates)
                    computed_dist_array.append(computed_dist_to_lidar)
                    
                    if len(computed_dist_array) == self.num_scans:
                        break
                
                #Compute mean
                mean_computed_dist = np.mean(computed_dist_array)
                print(mean_computed_dist)
                # Add the point to the plot
                ax.plot(measured_dist_to_lidar / 1000.0, mean_computed_dist, 'ro')
                        
                # Close the bag after processing
                bag.close()
            
         # Show the plot
        plt.show()
        
# Get the current user's home directory
home_directory = os.path.expanduser("~")
# Define the relative path to the folder
relative_path = 'Reflector-Calibration/intern_ws/src/calibration/test_bags'
# Combine the home directory and the relative path
folder_path = os.path.join(home_directory, relative_path)

# Create an instance of RosbagProcessor and process the ROS bags
processor = RosbagCleaner(folder_path)
processor.clean_rosbags()

