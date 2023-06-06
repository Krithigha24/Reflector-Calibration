import numpy as np
import math
from fractions import Fraction
from scipy.odr import *

class featuresDetection:
    def __init__(self):
        #variables
        self.EPSILON = 10
        self.DELTA = 501
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None 
        self.NP = len(self.LASERPOINTS) - 1
        self.LMIN = 20 #minimum length of a line segment
        self.LR = 0 #real length of a line segment
        self.PR = 0 # the number of laser points contained in the line segment
    
    # euclidian distance from point1 to point2
    def dist_point2point(self, point1, point2):
        Px = (point1[0] - point2[0]) ** 2
        Py = (point1[1] - point2[1]) ** 2
        return math.sqrt(Px + Py)
    
    # distance point to line written in the general form
    def dist_point2line(Self, params, point):
        A, B, C = params
        distance = abs(A * point[0] + B * point[1] + C ) / math.sqrt(A ** 2 + B **2)
        return distance
    
    #general form to slope-intercept form
    def lineform_G2SI(self, A, B, C):
        m = -A / B
        b = -C / B
        return m, b
    
    #slope-intercept to general form
    def lineform_si2G(self, m, b):
        A, B, C = -m, 1, -b
        if A < 0:
            A, B, C  

