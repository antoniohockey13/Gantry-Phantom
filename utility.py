# -*- coding: utf-8 -*-
"""
Created on Fri Mar 24 11:53:49 2023

@author: Antonio
"""
import os as _os

def get_actual_absolute_path():
    """
    Returns the actual path as a str
    """
    return _os.getcwd()

def go_to_phantom_absolute_path():
    """
    Change directory to Phantom

    """
    path = findfile('Gantry_Phantom.py', 'C:\\')
    path = _os.path.split(path)
    _os.chdir(path[0])



def change_to_directory(directory: str = 'Gantry'):
    """
    Change to given directory

    Parameters
    ----------
    directory : str, optional
        Name of the directory. The default is 'Gantry'.
    """
    go_to_phantom_absolute_path()
    _os.chdir(directory)

def findfile(name: str, path: str = '/'):
    """
    Search for the absolute path of a given file

    Parameters
    ----------
    name : str
        Name of the file.
    path : str
        Begin of the path, the closer it is to the final path the faster the
        funcion works. DEFAULT '/'

    Returns
    -------
    str
        Absolute path of the file.

    """
    for dirpath, dirname, filename in _os.walk(path):
        if name in filename:
            return _os.path.join(dirpath, name)
go_to_phantom_absolute_path()
