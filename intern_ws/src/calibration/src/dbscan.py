#!/usr/bin/env python3

import rospy
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import plotly.express as px
from sklearn.metrics import silhouette_score as ss
from sklearn.cluster import DBSCAN
import itertools

class DBSCAN:
    def __init__(self, BAG_NAME, BAG_MESSAGE_NO):
        self.BAG_NAME = BAG_NAME
        self.BAG_MESSGAE_NO = BAG_MESSAGE_NO
        df = pd.read_csv('?????.csv') #GET THIS FROM SOME FIXED LOCATION!, READ FROM BAGS FOLDER AND SAVE TO "CSV" FOLDER
        # Filter rows where 'MSG_NO' column has value BAG_MESSAGE_NO
        df = df[df['MSG_NO'] == self.BAG_MESSGAE_NO]
        self.MIN_EPS = df[df['ADJACENT_DIST'] > 0]['ADJACENT_DIST'].min()
        self.MAX_EPS = df['ADJACENT_DIST'].max()
        # Select the 'X_COORD' and 'Y_COORD' columns
        self.XY = (df[['X_COORD', 'Y_COORD']]).to_numpy()
        # Access the values in 'X_COORD' and 'Y_COORD' column
        self.X,self.Y = df.X_COORD, df.Y_COORD
    
    def grid_search(self):
        epsilons = np.linspace(self.MIN_EPS, self.MAX_EPS, num=60)
        min_samples = np.arange(2,3, step=1)
        combinations = list(itertools.product(epsilons, min_samples))
        scores = [] #list of scores for each combination
        all_labels_list = [] #labels for each combination
        
        for i, (eps, num_samples) in enumerate(combinations):
          dbscan_cluster_model = DBSCAN(eps=eps,min_samples=num_samples).fit(my_data)
          labels = dbscan_cluster_model.labels_
          labels_set = set(labels)
          num_clusters = len(labels_set)

          if(num_clusters != 4):
            scores.append(-10)
            all_labels_list.append('bad')
            c = (eps, num_samples)
            print(f"Combination {c} on iteration {i+1} of {N} has {num_clusters} clusters. Moving on")
            continue

          scores.append(ss(self.XY,labels))
          all_labels_list.append(labels)
          print(f"Index: {i}, Score: {scores[-1]}, Labels: {all_labels_list[-1]}, NumClusters: {num_clusters}")

        best_index = np.argmax(scores)
        best_parameters = combinations[best_index]
        best_labels = all_labels_list[best_index]
        best_score = scores[best_index]
        
        return best_parameters, best_labels, best_score
if __name__ == '__main__':
    BAG_NAME = rospy.get_param('bag_name') # Intensity threshold to idenity points belonging to cylindrical reflector
    BAG_MESSAGE_NO = rospy.get_param('bag_message_no') # Intensity threshold to idenity points belonging to cylindrical reflector
    rospy.init_node('dbscan')
    DBSCAN(BAG_NAME,BAG_MESSAGE_NO)
    rospy.spin()



