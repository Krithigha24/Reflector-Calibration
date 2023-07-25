# After converting the rosbags into binary segmented data by running binsegBag.py script,
# 9. Navigate to the directory where this script is by typing "scripts"
#    I added this line alias scripts='cd ~/Reflector-Calibration/intern_ws/src/calibration/scripts' in bashrc file
#    for easier directory path navigation
# 10. Run the script by using python3 processBag.py

#!/usr/bin/env python3
import os
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt
import re

class RosbagProcessor:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.num_normal_pts = 2 # number of closest points to lidar to average
        self.radius = 0.045 # fixed radius(m) of cylindrical marker ±0.0005 m 
        
    # Euclidian distance from point1 to point2, or from point1 to (0,0) if point2 input is not specified
    def dist_point2point(self, point1, point2=(0, 0)):    
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        
    def filterScan_to_cartesian(self, scan_msg):
        points_info = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if r != 0.0:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                dist_to_lidar = self.dist_point2point((x,y))
                points_info.append([x, y, dist_to_lidar])
            angle += scan_msg.angle_increment
        return points_info
    
    def calculate_coordinates_on_line(self, distance, known_point, start_point=(0,0)):
        line_vector = np.array(known_point) - np.array(start_point)
        normalized_vector = line_vector / np.linalg.norm(line_vector)
        desired_point = known_point + (normalized_vector * distance)
        return desired_point
    
    def extract_dist_from_filename(self,string):
        # Define the regular expression pattern to match a number
        pattern = r'\d+'
        # Find all matches of the pattern in the string
        matches = re.findall(pattern, string)
        # Extract the first matched number (if any)
        if matches:
            number = int(matches[0])
            return number
        else:
            return None

    def process_rosbags(self):
         # Initialize the plot
        fig, ax = plt.subplots()
        ax.set_xlabel('Measured Distance to lidar (m)')
        ax.set_ylabel('Error (m)')
        
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
                    points_info = self.filterScan_to_cartesian(msg)
                    
                    # Sort smallest to largest value of dist to lidar 
                    points_info.sort(key=lambda point: point[2])
                    
                    # Extract the first num_points (x, y) coordinates
                    coordinates = [(point[0], point[1]) for point in points_info[:self.num_normal_pts]]

                    # Calculate the average of the coordinates
                    average_x, average_y = np.mean(coordinates, axis=0)
                    
                    center_coordinates = self.calculate_coordinates_on_line(self.radius, (average_x,average_y))
                    computed_dist_to_lidar = self.dist_point2point(center_coordinates)
                    computed_dist_array.append(computed_dist_to_lidar)
                                    
                #Compute mean
                mean_computed_dist = np.mean(computed_dist_array)
                #print(mean_computed_dist)
                # Add the point to the plot
                ax.plot(measured_dist_to_lidar / 1000.0, mean_computed_dist-measured_dist_to_lidar/ 1000.0, 'ro')
                        
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
processor = RosbagProcessor(folder_path)
processor.process_rosbags()

