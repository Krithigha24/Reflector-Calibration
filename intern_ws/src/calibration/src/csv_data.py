#!/usr/bin/env python3
import rospy
import math
import os 
from sensor_msgs.msg import LaserScan
import numpy as np
import csv

class CsvData:
    def __init__(self):
        self.filteredScan_subscriber = rospy.Subscriber('/filtered_scan', LaserScan, self.filterScanToCSV)
        self.csv_file = 'scan_data.csv'
        self.msg_no = 0

    # Euclidian distance from point1 to point2, or from point1 to (0,0) if point2 input is not specified
    def dist_point2point(self, point1, point2=(0, 0)):
        Px = (point1[0]-point2[0]) ** 2
        Py = (point1[1]-point2[1]) ** 2
        return math.sqrt(Px + Py)
        
    def filterScan_to_cartesian_and_angle(self, scan_msg):
        points_info = []
        prev_point = None  # Initialize the previous point variable
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if r != 0.0:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                if prev_point is not None:
                    # Compute the Euclidean distance between the current point and the previous point
                    adjacent_dist = self.dist_point2point((x,y),prev_point)
                else:
                    adjacent_dist = 0
                points_info.append([x, y, angle, adjacent_dist])
                # Update the previous point
                prev_point = (x, y)
            angle += scan_msg.angle_increment
        return points_info
    
    def filterScanToCSV(self, scan_msg):
        points_info = self.filterScan_to_cartesian_and_angle(scan_msg)
        if not os.path.isfile(self.csv_file):  # Check if the CSV file exists
            with open(self.csv_file, 'a') as file:
                writer = csv.writer(file)
                writer.writerow(['MSG_NO', 'X_COORD', 'Y_COORD', 'ANGLE','ADJACENT_DIST'])  # Write the heading row

       
        with open(self.csv_file, 'a') as file:
            writer = csv.writer(file)
            for point_info in points_info:
                writer.writerow([self.msg_no, point_info[0], point_info[1], point_info[2],point_info[3]])
        self.msg_no += 1

if __name__ == '__main__':
    rospy.init_node('csv_data')
    CsvData()
    rospy.spin()

   

