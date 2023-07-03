#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class IntensityBinarySegmentation:
    def __init__(self):
        self.REFLECTIVITY_THRESHOLD = rospy.get_param('intensity_threshold') # Intensity threshold to idenity points belonging to cylindrical reflector
        self.segmented_scan_pub = rospy.Publisher('/segmented_scan', LaserScan, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)      
       
    def scan_callback(self,scan_msg):
        self.REFLECTIVITY_THRESHOLD = rospy.get_param('intensity_threshold') 
        #self.find_binseg_thresh(scan_msg.intensities)
        
        # Convert ranges and intensities to numpy arrays
        ranges = np.array(scan_msg.ranges)
        intensities = np.array(scan_msg.intensities)

        # Create a mask based on intensities above the threshold
        mask = intensities > self.REFLECTIVITY_THRESHOLD

        # Filter ranges and intensities using the mask
        filtered_ranges = np.where(mask, ranges, 0.0)
        filtered_intensities = np.where(mask, intensities, 0.0)

        # Update the LaserScan message with the filtered values
        scan_msg.ranges = filtered_ranges.tolist()
        scan_msg.intensities = filtered_intensities.tolist()

        # Publish the filtered LaserScan message to a new topic
        self.segmented_scan_pub.publish(scan_msg)
        
if __name__ == '__main__':
    rospy.init_node('intensity_binary_segmentation')
    IntensityBinarySegmentation()
    rospy.spin()