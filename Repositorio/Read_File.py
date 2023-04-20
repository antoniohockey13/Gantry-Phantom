# -*- coding: utf-8 -*-
"""
Created on Fri Mar 10 12:18:00 2023

@author: Antonio
"""
import pandas as pd

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

        df = pd.read_fwf('parametos.txt', header = None)

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
