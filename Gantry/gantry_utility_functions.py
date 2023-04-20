# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 11:07:51 2023

@author: Antonio
"""
import numpy as np
import pandas as pd
import math


def generate_trajectory(x_i: float, y_i: float, x_f: float, y_f: float,       \
                        max_separation: float = 0.001) -> np.ndarray:
    """
    Generate a trajectory in a straight line between two given points with a
    maximum separation allowed.

    Parameters:
    ----------
    x_i : float
        The x-coordinate of the starting point.
    y_i : float
        The y-coordinate of the starting point.
    x_f : float
        The x-coordinate of the end point.
    y_f : float
        The y-coordinate of the end point.
    max_separation : float, optional
        The maximum allowed separation between two consecutive points in the
        trajectory. Default is 0.001.

    Returns:
    -------
    points : np.array
        A two-dimensional numpy array with shape (n_points, 2), where n_points
        is the number of points in the trajectory. The first column contains
        the x-coordinates and the second column contains the y-coordinates.

    Note:
    -----
    The function uses numpy.linspace to generate the x and y coordinates,
    and then converts them to a numpy array of points.
    """
    num_x = abs(x_f - x_i) / max_separation
    num_y = abs(y_f - y_i) / max_separation
    if num_x > num_y:
        num_points = int(math.ceil(num_x))
    else:
        num_points = int(math.ceil(num_y))

    x = np.linspace(x_i, x_f, num_points, dtype=float)
    y = np.linspace(y_i, y_f, num_points, dtype=float)
    points = []
    for i in range(len(x)):
        ipoint = [x[i], y[i]]
        points.append(ipoint)
    points = np.array(points)
    return points


def digest_input(file: str):
    """
    Parameters
    ----------
    file : str
        Name of the file wanted to be read.

    Returns
    -------
    x_source : float
        X Position of the source in gantry coordinates
    y_source : float
        Y Position of the source in gantry coordinates
    x_gantry : float
        X position of the middle of the gantry's movement
    y_gantry : float
        Y position of the middle of the gantry's movement
    distancia_minima : float
        Minimum distance between gantry and source
    x_min : float
        Minimum value of the X position for the gantry
    x_max : float
     Maximum value of the X position for the gantry
    y_min : float
        Minimum value of the Y position for the gantry
    y_max : float
        Maximum value of the Y position for the gantry

    """

    x_source = 169 #centre of the phantom in coordinate x
    y_source = 150 #centre of the phantom in coordinate y
    distancia_minima =  2 #2 mm of radio
    x_min = 1
    x_max = 344
    y_min = 1
    y_max = 345

    #   We open the file from which we are going to read the input
    try:

        df = pd.read_fwf(file, header = None)

        for ifile in range(df.shape[0]):
            if df[0].str.split().str[0].str[0] [ifile] == '#':
                df2 = df.drop(ifile, axis = 0, inplace = False)

        x_source = float(df2.loc[df2[0] == 'x_source'][1])
        y_source = float(df2.loc[df2[0] == 'y_source'][1])
        distancia_minima = float(df2.loc[df2[0] == 'distancia_minima'][1])
        x_min = float(df2.loc[df2[0] == 'x_min'][1])
        y_min = float(df2.loc[df2[0] == 'y_min'][1])
        x_max = float(df2.loc[df2[0] == 'x_max'][1])
        y_max = float(df2.loc[df2[0] == 'y_max'][1])

    except IOError:
        print(f"The file {file} does not exist")
    x_gantry = (x_max-x_min)/2
    y_gantry = (y_max-y_min)/2
    return(x_source, y_source, x_gantry, y_gantry, distancia_minima, x_min,   \
           x_max, y_min, y_max)
