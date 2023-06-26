#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class IntensityBinarySegmentation:
    def __init__(self, REFLECTIVITY_THRESHOLD):
        self.REFLECTIVITY_THRESHOLD = REFLECTIVITY_THRESHOLD
        self.segmented_scan_pub = rospy.Publisher('/segmented_scan', LaserScan, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scanCallback, queue_size=10)
    
    #THIS FUNCTION IS NOT USED, THE VALUE IS TAKEN FROM YAML FILE  
    def find_binseg_thresh(self,data, k=2):
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
        rospy.loginfo(min(outliers)) 
        return min(outliers)
       
    def scan_callback(self,scan_msg):
        #self.REFLECTIVITY_THRESHOLD = self.find_binseg_thresh(scan_msg.intensities)

        # Convert ranges and intensities to numpy arrays
        ranges = np.array(scan_msg.ranges)
        intensities = np.array(scan_msg.intensities)

        # Create a mask based on intensities above the threshold
        mask = intensities > REFLECTIVITY_THRESHOLD

        # Filter ranges and intensities using the mask
        filtered_ranges = np.where(mask, ranges, 0.0)
        filtered_intensities = np.where(mask, intensities, 0.0)

        # Update the LaserScan message with the filtered values
        scan_msg.ranges = filtered_ranges.tolist()
        scan_msg.intensities = filtered_intensities.tolist()

        # Publish the filtered LaserScan message to a new topic
        self.segmented_scan_pub.publish(scan_msg)
        
if __name__ == '__main__':
    REFLECTIVITY_THRESHOLD = rospy.get_param('intensity_threshold') # Intensity threshold to idenity points belonging to cylindrical reflector

    rospy.init_node('intensity_binary_segmentation')
    IntensityBinarySegmentation(REFLECTIVITY_THRESHOLD)
    rospy.spin()
    