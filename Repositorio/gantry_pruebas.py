# -*- coding: utf-8 -*-
"""
Created on Tue Mar 21 11:48:11 2023

@author: Antonio
"""
#%% Read file viejo
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
        with open(file, "r") as input_file:
            for line in input_file :
                # lines are separated by blank spaces
                words = line.split()
                # if the line is only a blanck line, we discard it and
                # proceed with the next one
                if len(words) > 1 :
                    if words[0].lower() == "x_source":
                        x_source = float(words[1])
                    if words[0].lower() == "y_source":
                        y_source = float(words[1])
                    if words[0].lower() == "distancia_minima":
                        distancia_minima = float(words[1])
                    if words[0].lower() == "x_min":
                        x_min = float(words[1])
                    if words[0].lower() == "x_max":
                        x_max = float(words[1])
                    if words[0].lower() == "y_min":
                        y_min = float(words[1])
                    if words[0].lower() == "y_max":
                        y_max = float(words[1])

    except IOError:
        print(f"The file {file} does not exist")
    x_gantry = (x_max-x_min)/2
    y_gantry = (y_max-y_min)/2
    return(x_source, y_source, x_gantry, y_gantry, distancia_minima, x_min,   \
           x_max, y_min, y_max)
#%% Pruebas read file pandas
import pandas as pd

df = pd.read_fwf('parametos.txt', header = None)

for ifile in range(df.shape[0]):
    print(df[0].str.split().str[0].str[0].iloc[ifile])
    if df[0].str.split().str[0].str[0] [ifile] == '#':
        df2 = df.drop(ifile, axis = 0, inplace = False)

x_source = float(df2.loc[df2[0] == 'x_source'][1])
y_source = float(df2.loc[df2[0] == 'y_source'][1])
distancia_minima = float(df2.loc[df2[0] == 'distancia_minima'][1])
x_min = float(df2.loc[df2[0] == 'x_min'][1])
y_min = float(df2.loc[df2[0] == 'y_min'][1])
x_max = float(df2.loc[df2[0] == 'x_max'][1])
y_max = float(df2.loc[df2[0] == 'y_max'][1])
