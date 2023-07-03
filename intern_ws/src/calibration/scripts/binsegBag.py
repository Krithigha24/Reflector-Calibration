#!/usr/bin/env python3

import os
import rosbag
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class IntensityBinarySegmentation:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.reflectivity_threshold = None
        
    def segment(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        intensities = np.array(scan_msg.intensities)
        mask = intensities > self.reflectivity_threshold
        filtered_ranges = np.where(mask, ranges, 0.0)
        filtered_intensities = np.where(mask, intensities, 0.0)
        scan_msg.ranges = filtered_ranges.tolist()
        scan_msg.intensities = filtered_intensities.tolist()
        return scan_msg
    
    def find_binseg_thresh(self,data, k=1.2):
        #DETERMINING REFLECTIVITY THRESHOLD:

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
    
    def segment_rosbags(self):
        # Iterate over each file in the folder
        for filename in os.listdir(self.folder_path):
            if filename.endswith('.bag'):  # Check if it's a ROS bag file
                input_bag_path = os.path.join(self.folder_path, filename)
                output_bag_path = os.path.join(self.folder_path, "bs_" + filename)
                
                input_bag = rosbag.Bag(input_bag_path)
                output_bag = rosbag.Bag(output_bag_path, 'w')
                
                self.reflectivity_threshold = None
                
                for topic, msg, t in input_bag.read_messages(topics=['/scan']):
                    if self.reflectivity_threshold is None:
                        self.reflectivity_threshold = self.find_binseg_thresh(msg.intensities)
                    
                    segmented_scan_msg = self.segment(msg)
                    output_bag.write('/segmented_scan', segmented_scan_msg, t)
            input_bag.close()
            output_bag.close()

# Specify the folder path containing the ROS bags
folder_path = '/home/krithigha/Reflector-Calibration/intern_ws/src/calibration/test_bags'

# Create an instance of IntensityBinarySegmentationr and process the ROS bags
segmentation = IntensityBinarySegmentation(folder_path)
segmentation.segment_rosbags()
