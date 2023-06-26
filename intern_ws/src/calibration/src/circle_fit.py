#!/usr/bin/env python3
import rospy
import math
import os 
from sensor_msgs.msg import LaserScan
import numpy as np
import csv

class CircleFit:
    def __init__(self):
        self.filteredScan_subscriber = rospy.Subscriber('/arc_scan', LaserScan, self.fit_circle_to_arc_points)
        
    #STEP 1
    def compute_centroid_coordinate(self, data_points):
        centroid = np.mean(data_points, axis=0)
        return centroid
    
    #STEP 2
    def shift_coordinates(self, data_points, centroid):
        shifted_points = data_points - centroid
        return shifted_points
    
    #STEP 3
    def compute_z_values(self, data_points):
        x_values = data_points[:, 0]
        y_values = data_points[:, 1]
        z_values = x_values ** 2 + y_values ** 2
        return z_values
    
    #STEP 4
    def compute_array_mean(self, array):
        # Calculate the mean of the array
        mean = np.mean(array)
        # Return the mean
        return mean
    
    #STEP 5 & 6
    def compute_moment_matrix(self, data_points, z_values):
        n = len(z_values)
        ones_column = np.ones((n, 1))
        Z = np.column_stack((z_values, data_points, ones_column))
        Z_transpose = np.transpose(Z)
        M = (1/n) * np.dot(Z_transpose, Z)
        return M
    
    #STEP 7 and 8
    def compute_constraint_matrix(self, z_values):
        z_mean = self.compute_array_mean(z_values)
        H = np.array([[8*z_mean, 0, 0, 2],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [2, 0, 0, 0]])
        H_inv= np.array([[0, 0, 0, 0.5],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0.5, 0, 0, -2*z_mean]])
        return H, H_inv
    
    
    #STEPS 9, 10, 11
    def fit_circle_to_arc_points(self, scan_msg):
        
        # Compute SVD of matrix Z
        U, sigma, Vt = np.linalg.svd(Z)

        # Check the smallest singular value
        if sigma[3] < 1e-12:
            # Compute A as the 4th column of V
            A = Vt[3]
        else:
            # Compute Y using V, sigma, and Vt
            Y = np.dot(np.dot(Vt.T, np.diag(sigma)), Vt)
            # Compute Q
            Q = np.dot(np.dot(Y, H), Y.T)
            # Compute eigenvalues and eigenvectors of Q
            eigenvalues, eigenvectors = np.linalg.eig(Q)
            # Find the eigenvector corresponding to the smallest positive eigenvalue
            smallest_positive_eigenvalue_index = np.where(eigenvalues > 0)[0][0]
            A_star = eigenvectors[:, smallest_positive_eigenvalue_index]
            # Solve YA = A_star for A
            A = np.linalg.solve(Y, A_star)

# Output the A matrix
print(A)

    
    


if __name__ == '__main__':
    rospy.init_node('circle_fit')
    CircleFit()
    rospy.spin()
