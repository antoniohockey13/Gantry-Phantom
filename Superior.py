# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 10:53:04 2023

@author: antia
"""
import Gantry_Interface
import numpy as np
import time

#TO DO: Decidir cuantos cuadrantes queremos
#TO DO: queremos hacerlo para que gire en sentido horario y antihorario?


class Gantry:
    """
    This class is a superior module of Gantry_Interface
    """
    
    def __init__(self, path: str = 'COM4', baud: int =115200):
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

        """

        self.gantry = Gantry_Interface.Interface(path, baud)
        self.gantry.connect() #the laptop connects with the gantry
        self.position = self.gantry.get_position()
        # self.gantry.other_commands('G17, G21, G90, G55, G94') 
        self.x_source = 158 #centre of the phantom in coordinate x
        self.y_source = 127.5 #centre of the phantom in coordinate y
        self.x_gantry = 172.0 #centre of movement of the gantry in coordinate x
        self.y_gantry = 172.5 #centre of movement of the gantry in coordinate y
        self.distancia_minima =  2 #2 mm of radio 
        self.inside_limits = False
        self.move_first_X = True
        
    
    def disconnect(self):
        """
        Close the device node and disconnect to it. Very important to run this
        function at the end of the use. 
        This function is already done in Gantry_Interface.py
        """
        self.gantry.disconnect()
        
    
    def limits(self, x: float, y: float):
        """
        This function establishes the limits of the movement in coordinates X
        and Y.

        Parameters
        ----------
        x : float
            Limit in coordinate X.
        y : float
            Limit in coordinate Y.

        Returns
        -------
        None.

        """
        self.position = self.gantry.get_position()
        if x<1 or x>344.00: 
            print('ERROR OUT OF MOMENT LIMIT IN COORDINATE X')
            self.inside_limits = False
        elif y<1 or y>345.00:
            print('ERROR OUT OF MOMENT LIMIT IN COORDINATE Y')
            self.inside_limits = False
        elif self.x_source-self.distancia_minima<x and self.x_source+self.distancia_minima>x\
            and self.y_source-self.distancia_minima<y and self.y_source+self.distancia_minima>y:
            print('YOU ARE TRYING TO GO TOO CLOSE TO THE SOURCE')
            self.inside_limits = False
        else:
            self.inside_limits = True

    
    def move(self, x: float = 0, y: float = 0, z: float = 0,\
             feed_rate: int = 400):
        """
        
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
        self.limits(x, y)
        if self.inside_limits:
            self.position = self.gantry.get_position()
            self.move_cuadrantes(x, y)
            if self.move_first_X:
                self.gantry.move(x=x, y=self.position[1]) #x=cte and move only coordinate Y
                self.position = self.gantry.get_position() #actualize position
                self.gantry.move(x=self.position[0],y=y) # y=cte and move only coordinate X
                self.position = self.gantry.get_position() #actualize position
            else:
                self.gantry.move(x=self.position[0],y=y) # y=cte and move only coordinate X
                self.position = self.gantry.get_position() #actualize position
                self.gantry.move(x=x, y=self.position[1]) #x=cte and move only coordinate Y
                self.position = self.gantry.get_position() #actualize position
                
            self.inside_limits = False
        else:
            print('OUT OF LIMITS')
        
        
    
       
    def __hard_move(self, x: float = 0, y: float = 0, z: float = 0,\
             feed_rate: int = 400):
        """
        This function permits move the gantry without limits.

        Parameters
        ----------
        x : float, optional
            X position where the machine wants to be moved in mm.. The default is 0.
        y : float, optional
            Y position where the machine wants to be moved in mm.. The default is 0.
        z : float, optional
            Z position where the machine wants to be moved in mm.. The default is 0.
        feed_rate : int, optional
            Feed rate. The default is 400.

        Returns
        -------
        None.

        """
        self.gantry.move(x,y,z,feed_rate)
        
        
    def __hard_homing(self):
        """
        Sends '$H' command to the machine so it starts a hard homing cycle
        where it looks for the limits and recolocates the (0,0) position.
        Important for calibration
        """
        self.gantry.hard_homing_cycle()
        
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
        xinitial = self.x_source
        yinitial = self.y_source - radio
        self.move(xinitial,yinitial)
    

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
        self.position = self.gantry.get_position()
        x_final, y_final = self.final_positions(angle)
        self.limits(x_final, y_final)
        if self.inside_limits:
            distancia_x, distancia_y = self.calcular_distancia()
            self.gantry.circular_move(x_final, y_final, distancia_x, distancia_y)
            self.gantry.get_position()
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
        x_final = self.x_source + self.radio_movement() * np.sin(angle) #posiciones finales de giro
        y_final = self.y_source - self.radio_movement() * np.cos(angle)
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
        

        Returns
        -------
        None.

        """
        self.set_initial_position(self.y_source - 1)
        self.arco_giro(2*np.pi)
        # time.sleep(2)
        # self.arco_giro(np.pi)
    
    def circulo_min(self):
        self.set_initial_position(np.sqrt(2)*self.distancia_minima) 
        #we are trying to move the gantry in a circle with radious close to minimum distance 
        self.arco_giro(2*np.pi)
        
    def cuadrantes(self, x : float, y : float):
        if x>1 and x<=self.x_source-self.distancia_minima and y>1 and\
            y<=self.y_source-self.distancia_minima:
            return 1
        elif x>self.x_source-self.distancia_minima and x<self.x_source+self.distancia_minima\
            and y>1 and y<=self.y_source-self.distancia_minima:
            return 2
        elif x>=self.x_source+self.distancia_minima and x<344 and y>1 and\
            y<=self.y_source-self.distancia_minima:
            return 3
        elif x>1 and x<=self.x_source-self.distancia_minima and y>self.y_source-self.distancia_minima\
            and y<self.y_source+ self.distancia_minima:
            return 4
        elif x>=self.x_source+self.distancia_minima and x<344 and y>self.y_source-self.distancia_minima\
            and y<self.y_source+ self.distancia_minima:
            return 6
        elif x>1 and x<=self.x_source-self.distancia_minima and y>=self.y_source+self.distancia_minima\
            and y<345:
            return 7
        elif x>self.x_source-self.distancia_minima and x<self.x_source+self.distancia_minima\
            and y>=self.y_source+self.distancia_minima and y<345:
            return 8
        elif x>=self.x_source+self.distancia_minima and x<344 and \
            y>=self.y_source+self.distancia_minima and y<345:
            return 9
        else:
            print('OUT OF LIMITS')
            return -1
    
    def move_cuadrantes(self, x : float, y :float):
        cuadrante_final = self.cuadrantes(x, y)
        self.position = self.gantry.get_position()
        cuadrante_inicial = self.cuadrantes(self.position[0], self.position[1])
        if cuadrante_inicial == 1: 
            if cuadrante_final == 6:
                self.move_first_X = True
            if cuadrante_final == 8:
                self.move_first_X = False
        if cuadrante_inicial == 2:
            if cuadrante_final == 4 or cuadrante_final == 6 or cuadrante_final == 7\
                or cuadrante_final == 9:
                self.move_first_X = True
            if cuadrante_final == 8:
                self.move(x = self.x_source - self.distancia_minima,y=self.position[1])
                self.move(x,y)
        if cuadrante_inicial == 3:
            if cuadrante_final == 4:
                self.move_first_X = True
            if cuadrante_final == 8:
                self.move_first_X = False
        if cuadrante_inicial == 4:
            if cuadrante_final == 2 or cuadrante_final == 3 or cuadrante_final == 8\
                or cuadrante_final == 9:
                self.move_first_X = False
            if cuadrante_final == 6:
                self.move(x = self.position[0], y = self.y_source - self.distancia_minima)
                self.move(x,y)
        if cuadrante_inicial == 6:
            if cuadrante_final == 1 or cuadrante_final == 2 or cuadrante_final == 7\
                or cuadrante_final == 8:
                self.move_first_X = False
            if cuadrante_final == 4:
                self.move(x = self.position[0], y = self.y_source - self.distancia_minima)
                self.move(x,y)
        if cuadrante_inicial == 7:
            if cuadrante_final == 2:
                self.move_first_X = False
            if cuadrante_final == 6:
                self.move_first_X = False
        if cuadrante_inicial == 8:
            if cuadrante_final == 1 or cuadrante_final == 3 or cuadrante_final == 4\
                or cuadrante_final == 6:
                self.move_first_X = True
            if cuadrante_final == 2:
                self.move(x = self.x_source - self.distancia_minima,y=self.position[1])
                self.move(x,y)
        if cuadrante_inicial == 9:
            if cuadrante_final == 2:
                self.move_first_X = False
            if cuadrante_final == 4:
                self.move_first_X = True
                
            