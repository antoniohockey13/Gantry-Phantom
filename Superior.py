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
        Initializes the Gantry object with the given parameters.

        Parameters:
        - path (str): The serial port device node living under /dev. e.g.
                      /dev/ttyACM0 or /dev/ttyUSB0.
                      The default is 'COM3'.
        - baud (int): The baud rate. The default is 115200.
        - file (str): Name of the file where all parameter will be read.

        Attributes:
        - x_source (float): The x-coordinate of the source.
        - y_source (float): The y-coordinate of the source.
        - x_gantry (float): The initial x-coordinate of the gantry.
        - y_gantry (float): The initial y-coordinate of the gantry.
        - distancia_minima (float): The minimum distance between the source
                                    and the gantry.
        - x_min (float): The minimum x-coordinate that the gantry can move
                         within.
        - x_max (float): The maximum x-coordinate that the gantry can move
                         within.
        - y_min (float): The minimum y-coordinate that the gantry can move
                         within.
        - y_max (float): The maximum y-coordinate that the gantry can move
                         within.

        Returns:
        - None
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
        # Used to know if the movement is outside the source range,
        # updated in check_safety_distance()

    def disconnect(self):
        """
        Closes the device node and disconnects from it. It's very important to
        run this function at the end of the use.

        Parameters:
        - None

        Returns:
        - None
        """
        self.disconnect()

    def move_wrt_source(self, x_wrts: float, y_wrts: float, z: float = 0,     \
                        feed_rate: int = 400):
        """
        Moves the machine on the source coordinate system.

        Parameters
        ----------
        x_wrts : float
            The x-position where the machine should be moved in mm.
        y_wrts : float
            The y-position where the machine should be moved in mm.
        z : float, optional
            The z-position where the machine should be moved in mm.
        feed_rate: int, optional
            The feed rate of the machine. The default is 400.

        """

        # Convert the x and y position values from the source coordinate system
        # to the gantry coordinate system.
        x_wrtg, y_wrtg = self.change_coordinates_source_to_gantry(            \
                                                                x_wrts, y_wrts)

        # Get the current position of the machine in the gantry coordinate
        # system and the source coordinate system.
        self.position = self.get_position()
        self.position_wrts = self.position_source()

        # Check if the converted x and y position values are within the
        # maximum limit.
        self.check_limits(x_wrtg, y_wrtg)

        # Check if the converted x and y position values are within the safety
        # distance.
        self.check_safety_distance(x_wrts, y_wrts)

        # Check if there is an intersection between the current position and
        # the desired position of the machine.
        self.check_intersection(x_wrts, y_wrts, self.position_wrts[0],        \
                                self.position_wrts[1])

        # If the machine is within the limits and safety distance, move the
        # machine.
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

        # Reinitialize safety measures
        self.inside_limits = self.safety_distance = False
        self.position = self.get_position()
        self.position_wrts = self.position_source()


    def move_cuadrado(self, quadrant: int, feed_rate: float = 400):
        """
        Moves the machine to the closest corner of the specified quadrant.

        Parameters
        ----------
        quadrant : int
            The current quadrant of the gantry, ranging from 1 to 4.
        feed_rate : float, optional
            The feed rate of the machine in millimeters per minute (mm/min).
            The default value is 400 mm/min.
        """
        if quadrant == 1:
            # If the current quadrant is 1, move the machine to the closest
            # corner of quadrant 1.
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                                  self.distancia_minima, self.distancia_minima)
        elif quadrant == 2:
            # If the current quadrant is 2, move the machine to the closest
            # corner of quadrant 2.
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                                 -self.distancia_minima, self.distancia_minima)
        elif quadrant == 3:
            # If the current quadrant is 3, move the machine to the closest
            # corner of quadrant 3.
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                                -self.distancia_minima, -self.distancia_minima)
        elif quadrant == 4:
            # If the current quadrant is 4, move the machine to the closest
            # corner of quadrant 4.
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                                 self.distancia_minima, -self.distancia_minima)

        self._move_wrt_gantry(distancia_x_wrtg, distancia_y_wrtg, 0, feed_rate)


    def move_next_corner(self, quadrant: int, feed_rate: float = 400):
        """
        Moves the machine to the next corner in clockwise order.

        Parameters
        ----------
        quadrant : int
            The current quadrant of the gantry, ranging from 1 to 4.
        feed_rate : float, optional
            The feed rate of the machine in millimeters per minute (mm/min).
            The default value is 400 mm/min.
        """
        if quadrant == 1:
            # If the current quadrant is 1, move the machine to the next
            # corner in quadrant 1 (which is the bottom-right corner, i.e.,
            # quadrant 4).
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                             self.distancia_minima, -self.distancia_minima)
        elif quadrant == 2:
            # If the current quadrant is 2, move the machine to the next
            # corner in quadrant 2 (which is the top-right corner, i.e.,
            # quadrant 1).
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                                  self.distancia_minima, self.distancia_minima)
        elif quadrant == 3:
            # If the current quadrant is 3, move the machine to the next
            # corner in quadrant 3 (which is the top-left corner, i.e.,
            # quadrant 2).
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                                 -self.distancia_minima, self.distancia_minima)
        elif quadrant == 4:
            # If the current quadrant is 4, move the machine  to the next
            # corner in quadrant 3 (which is the bottom-left corner, i.e.,
            # quadrant 3).
            distancia_x_wrtg, distancia_y_wrtg =                              \
                self.change_coordinates_source_to_gantry(                     \
                                -self.distancia_minima, -self.distancia_minima)

        self._move_wrt_gantry(distancia_x_wrtg, distancia_y_wrtg, 0, feed_rate)


    def change_coordinates_source_to_gantry(self, x_wrts: float,              \
                                         y_wrts: float) -> tuple[float, float]:
        """
        Converts coordinates from the source coordinate system to the gantry
        coordinate system.

        Parameters
        ----------
        x_wrts : float
            The x-coordinate in the source coordinate system.
        y_wrts : float
            The y-coordinate in the source coordinate system.

        Returns
        -------
        tuple[float, float]
            The corresponding (x, y) coordinates in the gantry coordinate
            system.

        """
        x_gantry = self.x_source - x_wrts
        y_gantry = self.y_source - y_wrts
        return x_gantry, y_gantry

    def change_coordinates_gantry_to_source(self, x_wrtg: float,              \
                                         y_wrtg: float) -> tuple[float, float]:
        """
        Converts coordinates from the gantry coordinate system to the source
        coordinate system.

        Parameters
        ----------
        x_wrtg : float
            The x-coordinate in the gantry coordinate system.
        y_wrtg : float
            The y-coordinate in the gantry coordinate system.

        Returns
        -------
        tuple[float, float]
            The corresponding (x, y) coordinates in the source coordinate
            system.

        """
        x_source = self.x_source - x_wrtg
        y_source = self.y_source - y_wrtg
        return x_source, y_source

    def position_source(self):
        """
        Set the positions with respect to the source in coordinates X and Y

        Returns
        -------
        List[2]
            The position in coordinates with respect to the source.

        """
        self.position = self.get_position()
        position_x_s, position_y_s = self.change_coordinates_gantry_to_source(\
                                            self.position[0], self.position[1])
        self.position_wrts = [position_x_s, position_y_s]
        return self.position_wrts

    def check_limits(self, x_wrtg: float, y_wrtg: float):
        """
        Update inside_limits.
        This function establishes the range limits of the movement in
        coordinates X and Y.
        Does not consider the safety distance towards the source.

        Parameters
        ----------
        x_wrtg : float
            X position in gantry coordinate.
        y_wrtg : float
            Y position in gantry coordinate.

        Returns
        -------
        None.

        """
        self.position = self.get_position()
        if x_wrtg < self.x_min or x_wrtg > self.x_max:
            print('ERROR: OUT OF LIMITS IN COORDINATE X')
            self.inside_limits = False
        elif y_wrtg < self.y_min or y_wrtg > self.y_max:
            print('ERROR: OUT OF LIMITS IN COORDINATE Y')
            self.inside_limits = False
        else:
            self.inside_limits = True

    def check_safety_distance(self, x_wrts: float, y_wrts: float):
        """
        Update safety_distance.
        This function checks if the position (x,y) is outside the safety
        distance from the source.

        Parameters
        ----------
        x_wrts : float
            X position in source coordinates.
        y_wrts : float
            Y position in source coordinates.

        """
        distance = np.sqrt(x_wrts**2+y_wrts**2)
        if distance >= self.distancia_minima:
            self.safety_distance = True
        else:
            self.safety_distance = False
            print('Too close to the source')

    def check_intersection(self, x_wrts, y_wrts, x_f, y_f):
        """
        This function checks if the line that goes from the point
        (x_wrts, y_wrts) to (x_f, y_f) intersects the circle centered
        in the source.

        Parameters
        ----------
        x_wrts : float
            X position in source coordinates.
        y_wrts : float
            Y position in source coordinates.
        x_f : float
            X position of the final point in source coordinates.
        y_f : float
            Y position of the final point in source coordinates.

        Returns
        -------
        None.

        """
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
        """
        Determine the quadrant of the machine's current position with respect
        to the origin in the source coordinate system.

        Returns
        -------
        int
            An integer between 1 and 4, corresponding to the quadrant of the
            machine's current position.

        """
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
        Move the machine to a given position in the gantry coordinate system.
        It does NOT check safety measures

        Parameters
        ----------
        x : float, optional
             X position where the machine wants to be moved in mm.
             The default is 0. Max limit 344
        y : float, optional
            Y position where the machine wants to be moved in mm.
            The default is 0. Max limit 345
        feed_rate: int, optional
            Feed rate in mm/min. The default is 400
        Returns
        -------
        None.

        """
        self.move(x, y, z, feed_rate)

    def _hard_homing(self):
        """
        Sends '$H' command to the machine to start a hard homing cycle.
        During the homing cycle, the machine looks for its limits and
        repositions itself to (0, 0). This function is important for
        calibration.

        Returns
        -------
        None
        """

        self.hard_homing_cycle()


    def set_initial_position(self, radio: float):
        """
        Sets the initial coordinates of the gantry to a specific location
        with a given distance from the source.

        Parameters
        ----------
        radio : float
            Distance in the Y coordinate from the source.

        """
        x_initial = 0
        y_initial = radio
        self.move_wrt_source(x_initial, y_initial)

    def arco_giro(self, angle: float):
        """
        Executes a circular movement around the source by rotating the gantry
        by a certain angle in radians.

        Parameters
        ----------
        angle : float
            The angle of rotation in radians.

        Returns
        -------
        None.

        """
        if angle == 0:
            print('No movement required')
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
        Calculates the distance of the gantry from the source.

        Returns
        -------
        radio_movement : float
            The distance of the gantry from the source.

        """
        radiox = abs(self.position[0] - self.x_source)
        radioy = abs(self.position[1] - self.y_source)
        radio_movement = np.sqrt(radiox**2 + radioy**2)
        return radio_movement

    def final_positions(self, angle : float):
        """
        Calculate the final positions in the X and Y coordinates after a
        given angle of rotation.

        Parameters
        ----------
        angle : float
            The angle of rotation in radians.

        Returns
        -------
        x_final : float
            The final position of the gantry in the X coordinate.
        y_final : float
            The final position of the gantry in the Y coordinate.

        """
        self.position_wrts = self.position_source()
        position_x_s = self.position_wrts[0]
        position_y_s = self.position_wrts[1]
        x_final_wrts = -position_x_s*np.cos(angle) + position_y_s*np.sin(angle)
        y_final_wrts = -position_x_s*np.sin(angle) + position_y_s*np.cos(angle)
        x_final, y_final = self.change_coordinates_source_to_gantry(          \
                                                    x_final_wrts, y_final_wrts)
        return x_final, y_final

    def calcular_distancia(self):
        """
        Calculate the positions of the gantry with respect to the source

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
