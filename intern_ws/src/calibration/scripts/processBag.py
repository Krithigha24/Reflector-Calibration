import os
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt

class RosbagProcessor:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.num_normal_pts = 2 # number of closest points to lidar to average
        self.radius = 0.045 # fixed radius(m) of cylindrical marker ±0.0005 m 
        
    # Euclidian distance from point1 to point2, or from point1 to (0,0) if point2 input is not specified
    def dist_point2point(self, point1, point2=(0, 0)):    
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        
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
    
    def calculate_coordinates_on_line(self, distance, known_point, start_point=(0,0)):
        line_vector = np.array(known_point) - np.array(start_point)
        normalized_vector = line_vector / np.linalg.norm(line_vector)
        desired_point = known_point + (normalized_vector * distance)
        return desired_point

    def process_rosbags(self):
         # Initialize the plot
        fig, ax = plt.subplots()
        ax.set_xlabel('Measured Distance to lidar(m)')
        ax.set_ylabel('Mean Computed Distance to lidar(m)')
        
        # Iterate over each file in the folder
        for filename in os.listdir(self.folder_path):
            if filename.endswith('.bag'):  # Check if it's a ROS bag file
                bag_path = os.path.join(self.folder_path, filename)
                
                # Extract the measured distance to lidar from the filename
                file_parts = filename.split('_')
                measured_dist_to_lidar = int(file_parts[1].split('.')[0])  # Extract the number and remove the extension
                
                # Open the ROS bag
                bag = rosbag.Bag(bag_path)
                
                computed_dist_array = []
                
                # Process the messages in the bag
                for topic, msg, t in bag.read_messages(topics=['/scan']):
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
                
                #Compute mean
                mean_computed_dist = np.mean(computed_dist_array)
                
                # Add the point to the plot
                ax.plot(measured_dist_to_lidar / 1000.0, mean_computed_dist, 'ro')
                        
                # Close the bag after processing
                bag.close()
            
         # Show the plot
        plt.show()
        
# Specify the folder path containing the ROS bags
folder_path = '/home/krithigha/Reflector-Calibration/intern_ws/src/calibration/test_bags'

# Create an instance of RosbagProcessor and process the ROS bags
processor = RosbagProcessor(folder_path)
processor.process_rosbags()

