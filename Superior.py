# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 10:53:04 2023

@author: antia
"""
import numpy as np
import Gantry_Interface
import Read_File

class Gantry(Gantry_Interface.Interface):
    """
    This class is a superior module of Gantry_Interface
    """

    def __init__(self, path: str = 'COM4', baud: int =115200,                 \
                 file: str = 'parametos.txt'):
        """
        Initialization tasks

        Parameters
        ----------
        path : str, optional
            The serial port device node living under /dev.
            e.g. /dev/ttyACM0 or /dev/ttyUSB0.
            The default is 'COM3'.
        baud : int, optional
            The baud rate. The default is 115200.
        file : str, optional
            Name of the file where all parameter will be read

        """
        self.x_source, self.y_source, self.x_gantry, self.y_gantry,           \
            self.distancia_minima, self.x_min, self.x_max, self.y_min,        \
                self.y_max = Read_File.digest_input(file)
        super().__init__(path, baud)

        self.connect() #the laptop connects with the gantry
        self.position_wrts = self.position_source()
        self.position = self.get_position()
        self.other_commands('G17')
        self.other_commands('G21')
        self.other_commands('G90')
        self.other_commands('G55')
        self.other_commands('G94')



        self.inside_limits = False
        # Used to know if the movement is inside limits, updated in
        # check_limits()

        self.safety_distance = False
        # Used to know if the movement is outside the source range, updated in
        # check_safety_distance()



    def disconnect(self):
        """
        Close the device node and disconnect to it. Very important to run this
        function at the end of the use.
        This function is already done in Gantry_Interface.py
        """
        self.disconnect()

    def move_wrt_source(self, x_wrts: float, y_wrts: float, z: float = 0,               \
                        feed_rate: int = 400):
        """
        It works on source coordinate system

        Parameters
        ----------
        x : float, optional
             X position where the machine wants to be moved in mm.
             The default is 0. Max limit 344
        y : float, optional
            Y position where the machine wants to be moved in mm.
            The default is 0. Max limit 345
        z : float, optional
            Z position where the machine wants to be moved in mm.
            The default is 0.
        feed_rate: int, optional
            Feed rate. The default is 400

        Returns
        -------
        None.

        """
        x_wrtg, y_wrtg =                                                      \
            self.change_coordinates_source_to_gantry(x_wrts, y_wrts)

        self.position = self.get_position()
        self.position_wrts = self.position_source()
        self.check_limits(x_wrtg, y_wrtg)
        self.check_safety_distance(x_wrts, y_wrts)
        self.check_intersection(x_wrts, y_wrts, self.position_wrts[0], \
                                self.position_wrts[1])
        if self.inside_limits and self.safety_distance:
            if self.intersect:
                # Square movement
                quadrant = self.check_quadrant()
                # Check in what quadrant is the gantry
                self.move_cuadrado(quadrant, feed_rate)
                # Moves to the closest corner to the source of the quadrant
                self.move_next_corner(quadrant, feed_rate)
                # Moves to the next corner in clockwise
                self.move_wrt_source(x_wrts, y_wrts)
                # restart the function to get the wanted final position

            else:
                self._move_wrt_gantry(x_wrtg, y_wrtg, z, feed_rate)

        #Reinitialize safety measures
        self.inside_limits = self.safety_distance = False
        self.position = self.get_position()
        self.position_wrts = self.position_source()


    def move_cuadrado(self, quadrant: int,  feed_rate: float):
        """
        Moves to the closest corner to the source of the quadrant

        Parameters
        ----------
        quadrant : int
            The current quadrant of the gantry.
        feed_rate : float
            Feed rate. The default is 400.
        """

        if quadrant == 1:
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          self.distancia_minima, self.distancia_minima)
        if quadrant == 2:
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          -self.distancia_minima, self.distancia_minima)
        if quadrant == 3:
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          -self.distancia_minima, -self.distancia_minima)
        if quadrant == 4:
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          self.distancia_minima, -self.distancia_minima)
        self._move_wrt_gantry(distancia_x_wrtg, distancia_y_wrtg, 0, feed_rate)


    def move_next_corner(self, quadrant, feed_rate):
        """
        Moves to the next corner in clockwise

        Parameters
        ----------
        quadrant : int
            The current quadrant of the gantry.
        feed_rate : float
            Feed rate. The default is 400.
        """
        if quadrant == 1: # Si estas en la esquina 1 mueveta a la 4
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          self.distancia_minima, -self.distancia_minima)
        if quadrant == 2: # 2--> 1
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          self.distancia_minima, self.distancia_minima)
        if quadrant == 3: # 3 --> 2
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          -self.distancia_minima, self.distancia_minima)
        if quadrant == 4: # 4 --> 3
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                          -self.distancia_minima, -self.distancia_minima)

        self._move_wrt_gantry(distancia_x_wrtg, distancia_y_wrtg, 0, feed_rate)


    def change_coordinates_source_to_gantry(self, x_wrts: float,              \
                                            y_wrts: float):
        """
        Change between coordinate systems Source --> Gantry

        Parameters
        ----------
        x_wrts : float
            x value in source system.
        y_wrts : float
            y value in source system.

        Returns
        -------
        x_gantry : float
            x value in gantry system
        y_gantry : float
            y value in gantry system

        """
        x_gantry = self.x_source - x_wrts
        y_gantry = self.y_source - y_wrts
        return(x_gantry, y_gantry)

    def change_coordinates_gantry_to_source(self, x_wrtg: float,              \
                                            y_wrtg: float):
        """
        Change between coordinate systems Source --> Gantry

        Parameters
        ----------
        x_wrtg : float
            x value in gantry system.
        y_wrtg : float
            y value in gantry system.

        Returns
        -------
        x_source : float
            x value in source system
        y_source : float
            y value in source system

        """
        x_source = self.x_source - x_wrtg
        y_source = self.y_source - y_wrtg
        return(x_source, y_source)

    def position_source(self):
        """
        Set the positions respecto to the source in coordinates X and Y

        Returns
        -------
        List[2]
            Return the position in coordinates respect to the source.

        """
        self.position = self.get_position()
        position_x_s, position_y_s = self.change_coordinates_gantry_to_source(self.position[0],\
                                                                           self.position[1])
        self.position_wrts = [position_x_s, position_y_s]
        return self.position_wrts

    def check_limits(self, x_wrtg: float, y_wrtg: float):
        """
        Update inside_limits
        This function establishes the range limits of the movement in coordinates X
        and Y.
        Not considering safety distance towards the source

        Parameters
        ----------
        x : float
            X position in gantry coordinate.
        y : float
            Y position in gantry coordinate.

        Returns
        -------
        None.

        """
        self.position = self.get_position()
        if x_wrtg < self.x_min or x_wrtg > self.x_max:
            print('ERROR OUT OF MOMENT LIMIT IN COORDINATE X')
            self.inside_limits = False
        elif y_wrtg < self.y_min or y_wrtg > self.y_max:
            print('ERROR OUT OF MOMENT LIMIT IN COORDINATE Y')
            self.inside_limits = False
        else:
            self.inside_limits = True

    def check_safety_distance(self, x_wrts: float, y_wrts: float):
        """
        Update safety_distnace
        This function checks if the position x,y is outside source safety
        distance

        Parameters
        ----------
        x_wrts : float
            X position in source coordinate.
        y_wrts : float
            Y position in source coordinate.

        Returns
        -------
        None.

        """
        distance = np.sqrt(x_wrts**2+y_wrts**2)
        if distance >= self.distancia_minima:
            self.safety_distance = True
        else:
            self.safety_distance = False
            print('Too close to the source')

    def check_intersection(self, x_wrts, y_wrts, x_f, y_f):

        m = (y_wrts - y_f)/(x_wrts - x_f)
        n = -m*x_f + y_f
        b = 2*n*m
        a = m**2+1
        c = n**2-self.distancia_minima**2
        discriminante = b**2-4*a*c


        if discriminante <= 0:
            self.intersect = False
        else:
            self.intersect = True

    def check_quadrant(self):
        x_wrts, y_wrts = self.position_wrts
        if y_wrts >= 0:
            if x_wrts >= 0:
                return 1
            elif x_wrts < 0:
                return 2
        elif y_wrts < 0:
            if x_wrts < 0:
                return 3
            elif x_wrts >= 0:
                return 4

    def _move_wrt_gantry(self, x: float = 0, y: float = 0, z: float = 0,      \
                         feed_rate: int = 400):
        """
        It works on gantry coordinate system. It does NOT check safety
        measures


        Parameters
        ----------
        x : float, optional
             X position where the machine wants to be moved in mm.
             The default is 0. Max limit 344
        y : float, optional
            Y position where the machine wants to be moved in mm.
            The default is 0. Max limit 345
        feed_rate: int, optional
            Feed rate. The default is 400
        Returns
        -------
        None.

        """
        self.move(x, y, z, feed_rate)

    def _hard_homing(self):
        """
        Sends '$H' command to the machine so it starts a hard homing cycle
        where it looks for the limits and recolocates the (0,0) position.
        Important for calibration
        """
        self.hard_homing_cycle()

    def set_initial_position(self, radio : float):
        """
        Set the initial coordinates of the gantry with a specific radio

        Parameters
        ----------
        radio : float
            Distance y coordinate Y from the source.

        Returns
        -------
        None.

        """
        xinitial = 0
        yinitial = radio
        self.move_wrt_source(xinitial,yinitial)


    def arco_giro(self, angle : float):
        """

        Parameters

        ----------
        angle : float
            El angle se va a escribir en radianes

        Returns
        -------
        None.

        """
        if angle == 0:
            print('No me tengo que mover')
        else:

            self.position = self.get_position()
            self.position_wrts = self.position_source()
            x_final, y_final = self.final_positions(angle)

            self.check_limits(x_final, y_final)
            if self.inside_limits:
                distancia_x, distancia_y = self.calcular_distancia()
                self.circular_move(x_final, y_final, distancia_x, distancia_y)
                self.position = self.get_position()
                self.inside_limits = False
            else:
                print('OUT OF LIMITS')

    def radio_movement(self):
        """
        Calculate the radio of movement around the source

        Returns
        -------
        radio_movement : float
            Radio of movement.

        """
        radiox = abs(self.position[0] - self.x_source)
        radioy = abs(self.position[1] - self.y_source)
        radio_movement = np.sqrt(radiox**2 + radioy**2)
        return radio_movement


    def final_positions(self, angle : float):
        """
        Calculate the final positions in coordinate X and Y

        Parameters
        ----------
        angle : float
            Angle of rotation.

        Returns
        -------
        x_final : float
            Final position of the gantry in coordinate X.
        y_final : float
            Final position of the gantry in coordinate X.

        """
        self.position_wrts = self.position_source()
        position_x_s = self.position_wrts[0]
        position_y_s = self.position_wrts[1]
        x_final_wrts = -position_x_s*np.cos(angle) + position_y_s*np.sin(angle)
        y_final_wrts = -position_x_s*np.sin(angle) + position_y_s*np.cos(angle)
        x_final, y_final = self.change_coordinates_source_to_gantry(x_final_wrts, y_final_wrts)
        return x_final,y_final

    def calcular_distancia(self):
        """
        Calculate the positions of the gantry with respecto to the source

        Returns
        -------
        distancia_x : float
            Distance between the source and the initial position of the gantry
            in coordinate X.
        distancia_y : float
            Distance between the source and the initial position of the gantry
            in coordinate Y.

        """
        distancia_x =  self.x_source - self.position[0]
        distancia_y = self.y_source - self.position[1]
        return distancia_x, distancia_y

    def circulo_max(self):
        """
        Makes a maximum circle with radius from the source to the shortest end
        of the gantry movement area. The initial position is in the samen X
        coordinate as the source and the Y is the limit in the same line minus
        1 mm. This function makes a maximum circle in two time steps: both of
        180 degrees.

        """
        self.set_initial_position(self.y_source - self.y_min)
        self.arco_giro(2*np.pi)


    def circulo_min(self):
        """
        akes a maximum circle with radius from the source to the shortest end
        of the gantry movement area. The initial position is in the samen X
        coordinate as the source and the Y is the limit in the same line minus
        1 mm. This function makes a maximum circle in two time steps: both of
        180 degrees.

        """
        self.set_initial_position(np.sqrt(2)*self.distancia_minima)
        self.arco_giro(2*np.pi)
