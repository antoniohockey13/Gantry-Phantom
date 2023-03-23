# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 11:07:51 2023

@author: Antonio
"""
import numpy as np
import math
def generate_trajectory(x_i: float, y_i: float, x_f: float, y_f: float,       \
                        max_separation: float = 0.001):
    num_x = abs(x_f-x_i)/max_separation
    num_y = abs(y_f-y_i)/max_separation
    if num_x > num_y:
        num_points = int(math.ceil(num_x))
    else:
        num_points = int(math.ceil(num_y))

    x = np.linspace(x_i, x_f, num_points, dtype = float)
    y = np.linspace(y_i, y_f, num_points, dtype = float)
    points = []
    for i in range(len(x)):
        ipoint = [x[i], y[i]]
        points.append(ipoint)
    points = np.array(points)
    print(points[0])
    print(points[0,0])
    print(points[0,1])
    return points
