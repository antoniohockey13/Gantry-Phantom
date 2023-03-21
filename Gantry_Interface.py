# -*- coding: utf-8 -*-
"""
Created on Wed Feb 22 10:26:08 2023

@author: Antonio
"""

import time
import serial

# TO DO: Chequear parity connect, inicializar G en connect,
# G28 despacito, como funciona logger, investigar G10 y G02, G03

class Interface:
    """
    Implements opening, closing, writing and reading from the serial port.

    G-Commands
    G20: inches, G21: mm
    G17: plano XY DO NOT CHANGE
    G90: absolut coordinates
    G91: relative coordinates
    G10 P0 L20 Xx Yy Zz para cambiar posición G54
    """

    def __init__(self, path: str = 'COM3', baud: int =115200,                 \
                 verbose: bool = False):
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
        verbose : bool, optional
            If true all prints will be done. Default False
        """

        self.path = path
        self.baud = baud
        self._do_receive = False
        self._position_counter = 0
        self._move_counter = 0
        self.verbose = verbose

    def connect(self):
        """
        Open the device node and connect to it. Without running this function
        you will NOT be able to communicate with the machine.
        """
        if self._do_receive is False:

            self.serialport = serial.Serial(self.path, self.baud,             \
                                            parity=serial.PARITY_NONE,        \
                                            stopbits=serial.STOPBITS_ONE,     \
                                            bytesize=serial.EIGHTBITS,        \
                                            timeout=1, writeTimeout=0)
            # TO DO: Chequear parity
            self.serialport.flushInput()
            self.serialport.flushOutput()
            self._do_receive = True
            self.position = self.get_position()
            print('Connection done')
        else:
            print('Ya se ha realizado la conexión')

    def disconnect(self):
        """
        Close the device node and disconnect to it. Very important to run this
        function at the end of the use
        """
        self.serialport.flushInput()
        self.serialport.flushOutput()
        self.serialport.close()
        self._do_receive = False
        print('Disconnected')

    def get_position(self):
        """
        It reads the machine's position

        Returns
        -------
        position : list[float] = [x,y,z]
            Position in cartesian coordinates.
        """
        write = '?'
        asci = self.handle_gcommand(write)
        position = self._extract_position_from_ascii(asci)
        return position

    def _extract_position_from_ascii(self, asci):
        """
        Extract the position from the read output

        Parameters
        ----------
        asci : str
            Output message.

        Returns
        -------
        position : list[float] = [x,y,z]
            Position in cartesian coordinates.
        """
        if self._position_counter >=20:
            print('A error has been found, reset machine')
            self._position_counter = 0
            return
        try:
            data = asci.split('|')[1].split(':')
            if data[0] == 'MPos':
                position = data[1].split(',')
                position = [float(position[0]), float(position[1]), \
                            float(position[2])]
                return position
            else:
                if self.verbose:
                    print('No he leido el MPos')
                self.get_position()
                self._position_counter += 1
        except IndexError:
            self.get_position()
            self._position_counter += 1

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
            The default is 0. Max limit
        z : float, optional
            Z position where the machine wants to be moved in mm.
            The default is 0.
        feed_rate: int, optional
            Feed rate. The default is 400
        Returns
        -------
        None.

        """
        line = f"G01 X{x} Y{y} Z{z} F{feed_rate}"
        self.write(line+"\n")

        self.it_moves = True
        while self.it_moves is True:
            data = self.serialport.read(1)
            waiting = self.serialport.inWaiting()
            data += self.serialport.read(waiting)
            self._handle_data(data)
            self.is_it_moving()

        print('He llegado')
        self.position = self.get_position()

    def circular_move(self, xfinal: float, yfinal: float, xcentro: float,     \
                      ycentro: float, move_to_right: bool = True):
        """
        https://howtomechatronics.com/tutorials/g-code-explained-list-of-most-important-g-code-commands/

        Parameters
        ----------
        xfinal : float
            Final position of x coordinate.
        yfinal : float
            Fianl position of y coordinate.
        xcentro : float
            x position of the center of the cicle, ALWAYS IN INCREMENTAL MODE
            from the starting position.
        ycentro : float
            y position of the center of the cicle, ALWAYS IN INCREMENTAL MODE
            from the starting position.
        move_to_right : bool, optional
            Move to the right if true or to the left if false.
            The default is True.

        """
        if move_to_right:
            line = f"G02 X{xfinal} Y{yfinal} I{xcentro} J{ycentro}"
        else:
            line = f"G03 X{xfinal} Y{yfinal} I{xcentro} J{ycentro}"

        self.write(line+"\n")

        self.it_moves = True
        while self.it_moves is True:
            data = self.serialport.read(1)
            waiting = self.serialport.inWaiting()
            data += self.serialport.read(waiting)
            self._handle_data(data)
            self.is_it_moving()

        print('He llegado')
        self.position = self.get_position()

    def _handle_data(self, data: str):
        """
        Decodes the output message of the machine. It transforms bytes into str

        Parameters
        ----------
        data : str
            Output message.

        Returns
        -------
        asci : str
            Readable output message.

        """
        try:
            asci = data.decode("ascii")
        except UnicodeDecodeError:
            if self.verbose:
                print(": Received a non-ascii byte. Probably junk.           "\
                      "Dropping it.")
            asci = ""
        return asci

    def write(self, data):
        """
        Parameters
        ----------
        data : str
            String which is written in the device node.
            If data is empty, no write is performed.

        Returns
        -------
        num_written : int
            The number of written characters is returned..

        """
        if len(data) > 0:
            num_written = self.serialport.write(bytes(data,"ascii"))
            return num_written
        else:
            if self.verbose:
                print(" nothing to write")

    def get_help(self):
        """
        Sends '$' command to the machine so it opens help panel

        Returns
        -------
        prints help panel
        """
        self.handle_gcommand('$')

    def view_write_grbl_settings(self):
        """
        Sends '$$' command to the machine so it opens GRBL settings panel

        Returns
        -------
        prints GRBL settings panel
        """
        self.handle_gcommand('$$')

    def view_gcode_parameters(self):
        """
        Sends '$#' command to the machine so it opens Gcode parameters panel

        Returns
        -------
        prints Gcode parameters panel
        """
        self.handle_gcommand('$#')

    def view_gcode_parser_state(self):
        """
        Sends '$G' command to the machine so it opens Gcode Parser state panel

        Returns
        -------
        prints Gcode Parser state panel
        """
        self.handle_gcommand('$G')

    def view_build_info(self):
        """
        Sends '$I' command to the machine so it opensthe build info panel

        Returns
        -------
        prints build info panel
        """

        self.handle_gcommand('$I')

    def view_startup_blocks(self):
        """
        Sends '$N' command to the machine so it opens startup blocks panel

        Returns
        -------
        prints startup blocks panel
        """
        self.handle_gcommand('$N')

    def check_gcode_mode(self):
        """
        Sends '$C' command to the machine so you can check the gcode mode
        Returns
        -------
        prints Gcode modes
        """
        self.handle_gcommand('$C')

    def kill_alarm_lock(self):
        """
        Sends '$X' to kill alarm lock
        """
        self.handle_gcommand('$X')

    def hard_homing_cycle(self):
        """
        Sends '$H' command to the machine so it starts a hard homing cycle
        where it looks for the limits and recolocates the (0,0) position.
        Important for calibration
        """
        self.handle_gcommand('$H')
        self.is_it_moving()
        while self.it_moves == True:
            self.is_it_moving()
        self.softreset()
        self.position = self.get_position()


    def is_it_moving(self):
        """
        Checks the machine's state of movement

        Returns
        -------
        BOOL
            self.it_moves.
        """
        write = '?'
        asci = self.handle_gcommand(write)
        self.position = self.get_position()
        self._extract_movement(asci)
        return self.it_moves

    def _extract_movement(self, asci):
        """
        Extract the actual state of the machine from the read output

        it_moves: True --> Is is moving
        it_moves: False --> It is stop
        """
        if self._move_counter >=20:
            print('A error has been found, reset machine')
            self._move_counter = 0
            return
        try:
            data = asci.split('|')
            if data[0] == '<Run':
                self.it_moves = True

            elif data[0] == '<Home':
                self.it_moves = True

            elif data[0] == '<Idle':
                self.it_moves = False

            elif data[0] == '<Alarm':
                print('Ha saltado una alarma')
                self.it_moves = True

            else:
                if self.verbose:
                    print(f'data = {data} \n asci = {asci}')
                self._move_counter += 1
                self.is_it_moving()
        except IndexError:
            self.is_it_moving()
            self._move_counter += 1



    def other_commands(self, command):
        """
        Parameters
        ----------
        command : str
            command wanted to read in the machine.
        """
        self.handle_gcommand(command)

    def handle_gcommand(self, gcommand_code: str):
        """
        Send the G-command to the machine and catch the output message

        Parameters
        ----------
        gcommand_code : str
            G-command send to the machine.

        Returns
        -------
        asci : str
            Readable output message. Processed by function hadle_data.

        """
        if self._do_receive == True:
            self.write(gcommand_code + '\n')
            time.sleep(0.5) # 0.5
            # Con time.sleep lee el mensaje entero, si no ponemos el time.sleep
            # no lee los mensajes enteros y no es consistente
            data = self.serialport.read(1)
            waiting = self.serialport.inWaiting()
            data += self.serialport.read(waiting)
            asci = self._handle_data(data)
            if self.verbose:
                print(asci)
            return asci
        else:
            print('You are not connected to any device')

    def go_zero(self):
        """
        Moves to (0,0,0) the machine

        Returns
        -------
        None.

        """
        self.handle_gcommand('G28')


    def absolut_coordinates(self, coordenadas_absolutas: bool):
        """
        Tells the machine which type of cordinate system must use.
        2 options are absolut (G90) or relative (G91)

        Parameters
        ----------
        coordenadas_absolutas : bool
            It answer the question, it the system using absolute coordinates.

        Returns
        -------
        None.
        """
        if coordenadas_absolutas:
            self.handle_gcommand('G90')
        else:
            self.handle_gcommand('G91')

    def softreset(self):
        """
        Immediately sends `Ctrl-X` to Grbl.
        """
        self.write("\x18")

    def change_verbose(self, verbose):
        """
        Update the value of verbose

        Parameters
        ----------
        verbose : bool
            If True all prints will be made.
        """
        self.verbose = verbose
        print('Done')

    def print_position(self):
        self.position = self.get_position()
        print(self.position)
