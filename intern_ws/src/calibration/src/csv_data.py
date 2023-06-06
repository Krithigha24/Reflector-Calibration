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

    def filterScan_to_cartesian_and_angle(self, scan_msg):
        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if r != 0.0:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
            angle += scan_msg.angle_increment
        return points

    def filterScanToCSV(self, scan_msg):
        points = self.filterScan_to_cartesian_and_angle(scan_msg)
        if not os.path.isfile(self.csv_file):  # Check if the CSV file exists
            with open(self.csv_file, 'a') as file:
                writer = csv.writer(file)
                writer.writerow(['MSG_NO', 'X_COORD', 'Y_COORD'])  # Write the heading row

        with open(self.csv_file, 'a') as file:
            writer = csv.writer(file)
            for point in points:
                writer.writerow([self.msg_no, point[0], point[1]])
        self.msg_no += 1

if __name__ == '__main__':
    rospy.init_node('csv_data')
    CsvData()
    rospy.spin()

   

