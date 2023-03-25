# -*- coding: utf-8 -*-
"""
Created on Fri Mar 24 11:53:49 2023

@author: Antonio
"""
import os as _os

def get_actual_absolute_path():
    return _os.getcwd()

def go_to_phantom_absolute_path():
    _os.chdir('C:\\Users\\Antonio\\OneDrive\\Escritorio1\\Clase\\Universidad' \
              '\\Prácticas IFCA\\Phantom')


def change_to_directory(directory: str = 'Gantry'):
    go_to_phantom_absolute_path()
    _os.chdir(directory)
