#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

REFLECTIVITY_THRESHOLD = 2000

"""
DETERMINING REFLECTIVITY THRESHOLD:

The IQR method calculates interqurtile range within which most values 
fall. Values outside this range are considered outliers

Smaller value of k (eg. k < 1) more sensitive to outlier, less strict
Larger value of k (e.g. k > 1) stricter
"""
# def find_binseg_thresh(data, k=2):
#     # Calculate the lower and upper quartiles
#     q1 = np.percentile(data, 25)
#     q3 = np.percentile(data, 75)
    
#     # Calculate the interquartile range (IQR)
#     iqr = q3 - q1
    
#     # Find the significantly large values that fall outside the range
#     upper_bound = q3 + k * iqr
#     outliers = [val for val in data if val > upper_bound]
#     rospy.loginfo(min(outliers)) 
#     return min(outliers)
    
def scanCallback(scan_msg):
    ranges = np.array(scan_msg.ranges)
    intensities = np.array(scan_msg.intensities)

    mask = intensities > REFLECTIVITY_THRESHOLD
    filtered_ranges = np.where(mask, ranges, 0.0)
    filtered_intensities = np.where(mask, intensities, 0.0)

    scan_msg.ranges = filtered_ranges.tolist()
    scan_msg.intensities = filtered_intensities.tolist()

    # Publish the filtered LaserScan message to a new topic
    filtered_scan_pub.publish(scan_msg)

def main():
    rospy.init_node('binseg')
   
    # Create a publisher for the filtered LaserScan message
    global filtered_scan_pub
    filtered_scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)

    # Subscribe to the LaserScan topic
    sub = rospy.Subscriber('/scan', LaserScan, scanCallback, queue_size=10)

    # Start the ROS node
    rospy.spin()

if __name__ == "__main__":
    main()
