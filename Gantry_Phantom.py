# -*- coding: utf-8 -*-
"""
Created on Tue Mar 21 10:04:32 2023

@author: Antonio
"""
import time
import numpy as np
import Gantry
import utility
import Acquisition

class Gantry_Phantom(Gantry.Gantry):
    """
    This class represents a Gantry_Phantom object that inherits from the
    Superior.Gantry class. It provides methods for moving the gantry in a
    linear or circular motion, and taking measurements using the gantry.
    """

    def __init__(self, path: str = 'COM3', baud: int = 115200,                \
                 file: str = 'parametros.txt'):
        """
        Initializes a new instance of the Gantry_Phantom class.

        Parameters:
        path (str): The path to the serial port used to communicate with the
                    gantry.
        baud (int): The baud rate used for communication.
        file (str): The name of the file containing the parameters for the
                    gantry.
        """
        utility.go_to_phantom_absolute_path()
        utility.change_to_directory('Gantry')
        super().__init__(path, baud, file)
        utility.go_to_phantom_absolute_path()

    def all_measurements(self, num_radius: int, n_measures: int):
        """
        Parameters
        ----------
        num_radius : int
            Number of circles in which gantry will measure.
        n_measures : int
            The number of measurements to take in each circle.
        """
        radio_max = self.y_source - self.y_min
        radius = np.linspace(self.distancia_minima, radio_max, num_radius)
        for iradio in radius:
            self.measurements(iradio, n_measures)


    def move_linear(self, x: float, y: float, feed_rate: int = 400):
        """
        Moves the gantry in a linear motion to the specified coordinates.

        Parameters
        ----------
        x : float
            The x-coordinate to move to.
        y : float
            The y-coordinate to move to..
        feed_rate : int, optional
            The feed rate at which to move the gantry in mm/min. The default is 400.
        """
        self.move_wrt_source(x_wrts=x, y_wrts=y, feed_rate=feed_rate)

    def measurements(self, radius: float, n_measures: int):
        """
        Makes circular movements and takes measurements at regular intervals.

        Parameters:
        -----------
        radius: float
            The radius of the circle to move along
        n_measures: int
            The number of measurements to take
        """
        angle = 2 * np.pi / n_measures
        self.set_radius(radius)
        Acquisition.Measure.measures(1)
        for i in range(1, n_measures):
            self.move_circular(angle)
            Acquisition.Measure.measures(1)

    def set_radius(self, radius: float):
        """
        Sets the radius for circular movements.

        Parameters:
        -----------
        radius: float
            The radius of the circle to move along
        """
        self.set_initial_position(radius)

    def move_circular(self, angle: float):
        """
        Moves the device circularly by a specified angle.

        Parameters:
        -----------
        angle: float
            The angle to move the device by, in radians
        """
        print('Move circular')
        self.arco_giro(angle)
