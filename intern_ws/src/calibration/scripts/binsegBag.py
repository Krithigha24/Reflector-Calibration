# After collecting rosbag data using roslaunch calibration collectData.launch,
# 7. Navigate to the directory where this script is by typing "scripts"
#    I added this line alias scripts='cd ~/Reflector-Calibration/intern_ws/src/calibration/scripts' in bashrc file
#    for easier directory path navigation
# 8. Run the script by using python3 binsegBag.py

#!/usr/bin/env python3
import os
import rosbag
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import re

class IntensityBinarySegmentation:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.reflectivity_threshold = None
        self.num_scans = 30
        
    def segment(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        intensities = np.array(scan_msg.intensities)
        mask = intensities > self.reflectivity_threshold
        filtered_ranges = np.where(mask, ranges, 0.0)
        filtered_intensities = np.where(mask, intensities, 0.0)
        scan_msg.ranges = filtered_ranges.tolist()
        scan_msg.intensities = filtered_intensities.tolist()
        return scan_msg
    
    def find_binseg_thresh(self,data, k=0.5):
        #Determine the reflectivity threshold using the IQR method

        #The IQR method calculates interqurtile range within which most values 
        #fall. Values outside this range are considered outliers

        #Smaller value of k (eg. k < 1) more sensitive to outlier, less strict
        #Larger value of k (e.g. k > 1) stricter

        # Calculate the lower and upper quartiles
        q1 = np.percentile(data, 25)
        q3 = np.percentile(data, 75)
        # Calculate the interquartile range (IQR)
        iqr = q3 - q1
        # Find the significantly large values that fall outside the range
        upper_bound = q3 + k * iqr
        outliers = [val for val in data if val > upper_bound]
        return min(outliers)
    
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
        
    def segment_rosbags(self):
        # Iterate over each file in the folder
        for filename in os.listdir(self.folder_path):
            
            if filename.endswith('.bag'):  # Check if it's a ROS bag file
                
                 # Extract the measured distance to lidar from the filename
                measured_dist_to_lidar = int(self.extract_dist_from_filename(filename))
                
                input_bag_path = os.path.join(self.folder_path, filename)
                output_bag_path = os.path.join(self.folder_path, "bs_" + filename)
                input_bag = rosbag.Bag(input_bag_path)
                output_bag = rosbag.Bag(output_bag_path, 'w')
                
                self.reflectivity_threshold = None
                scan_count = 0
                
                for topic, msg, t in input_bag.read_messages(topics=['/scan']):
                    scan_count += 1
                    
                    if self.reflectivity_threshold is None:
                        if measured_dist_to_lidar > 1000:
                            self.reflectivity_threshold = int(self.find_binseg_thresh(msg.intensities,k=2.0))
                        else:
                            self.reflectivity_threshold = int(self.find_binseg_thresh(msg.intensities))
                    
                    segmented_scan_msg = self.segment(msg)
                    output_bag.write('/segmented_scan', segmented_scan_msg, t)
                    
                    if scan_count == self.num_scans:
                        break
                    
            input_bag.close()
            output_bag.close()
            
# Get the current user's home directory
home_directory = os.path.expanduser("~")
# Define the relative path to the folder
relative_path = 'Reflector-Calibration/intern_ws/src/calibration/test_bags'
# Combine the home directory and the relative path
folder_path = os.path.join(home_directory, relative_path)

# Create an instance of IntensityBinarySegmentationr and process the ROS bags
segmentation = IntensityBinarySegmentation(folder_path)
segmentation.segment_rosbags()
