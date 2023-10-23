# -*- coding: utf-8 -*-
# Copyright (c) 2023 PHOXENE
# MIT License: 
# https://opensource.org/license/mit/
#
""" Python driver for Modbus RTU on serial port.

Usage:
    This driver has been developped as Modbus layer for
    Phoxene's devices that implements a serial Modbus communication.
    
    It is mainly used by Phoxene's applications or device drivers

Exceptions handling:
    Function involving serial communication can raises following exceptions:
        serial.SerialExeption(IOError): 
                Base class for serial port related exceptions
        serial.SerialTimeoutException(SerialException):
                Write timeouts give an exception
        Serial.PortNotOpenError(SerialException): 
                Port is not open

    ModbusError(IOError) is the base exception for:
        SlaveExceptionError(ModbusError):
                Exceptions that the Modbus device reports.
        IllegalRequestError(SlaveExceptionError):
                The slave has received an illegal request.
        
        MasterReportedError(ModbusError):
                Exceptions that are detected by the master.
        NoResponseError(MasterReportedError):
                No response from the slave.
        InvalidResponseError(MasterReportedError):
                The response does not fulfill the Modbus standad.
"""
__authors__ = ("Aurélien PLANTIN")
__contact__ = ("a.plantin@phoxene.com")
__copyright__ = "MIT"
__date__ = "2023-10-10"
#Style guide: refers to PEP 8
#Type Hints: refers to PEP 484
#Docstrings: refers to Spinx documentation 

import time
import serial

exceptions_dict = {
    1 : "Illegal function",
    2 : "Illegal address",
    3 : "Illegal data value",
    4 : "Device failure",
    5 : "Acknoledge",
    6 : "Busy",
    7 : "Negative ack",
    8 : "Memory parity error",
    9 : "Gateway path unavailable",
    10: "Gateway target device failed to respond", 
 }

parity_dict = {'odd' : serial.PARITY_ODD, 'even' : serial.PARITY_EVEN} 

def _word2bytes(value: int) -> list:
    """Split a word into a list of bytes, little endian.
    
    Args:
        value: 16bits word.

    Returns:
        list (int): list of two bytes, msb first.
    """
    return([value >> 8, value & 0xFF])

def _crc16(frame : list):
    """Compute the Modbus CRC16 of a frame (list of words)
    
    Args:
        frame: (list (int)): list of words

    Returns:
        int: crc16 value
    """
    POLYNOME = 0xA001
    crc = 0xFFFF
    for word in frame:
        crc ^= word
        for j in range(0, 8):
            parity = crc
            crc >>= 1
            if parity % 2:crc ^= POLYNOME
    return(crc)

class ModbusError(IOError):
    """Exception raised in case of Modbus communication error."""

class SlaveExceptionError(ModbusError):
    """Exceptions that the Modbus device reports."""

class IllegalRequestError(SlaveExceptionError):
    """The slave has received an illegal request.
    
    Can be "Illegal function", "Illegal address" or "Illegal data value"
    """

class MasterReportedError(ModbusError):
    """Exceptions that are detected by the master."""

class NoResponseError(MasterReportedError):
    """No response from the slave."""

class InvalidResponseError(MasterReportedError):
    """The response does not fulfill the Modbus standad."""

class Modbus:
    """Modbus class:
    Open a serial based modbus link.
    Give standard methods for Modbus functions.
    
    .. note::
        A feedback (sent and received frames...) is provided by registering
        a function using the register_feedback_handler method
        
        Feedback events are (CRC_error, Frame_OK, Empty_frame, 
        Frame_lenght_error, Unexpected_content, ...)   
    
    :param port: Serial port name or None (default).
    :param baudrate: Serial link baudrate. Default is 19200.
    :param timeout: Read timeout in seconds. Default is 0.1s.
    :param parity: Serial link parity. Possible valuers are 'even' (default) or 'odd'

    :returns: A modbus link object
    """
    def __init__(self, 
                 port: str = None, 
                 baudrate: int = 19200, 
                 timeout: int = 0.1, 
                 parity: str = 'even'
                 ):   
        self.init_baudrate = baudrate
        self.init_timeout = timeout
        self.serial_link = serial.Serial(port = port,
                                         baudrate = baudrate,
                                         stopbits = serial.STOPBITS_ONE,
                                         bytesize = serial.EIGHTBITS,
                                         timeout = timeout,
                                         write_timeout = 0)
        self.parity = parity
        # Assign to the feedback handler an empty function that accept kwargs
        self.feedback = lambda **kwargs: None 

    @property
    def parity(self):
        """Get or set the Modbus link parity. 
        Setting to a new value will reconfigure the serial port automatically.
        """
        return(self._parity)
    
    @parity.setter
    def parity(self, parity):
        if parity not in ['odd', 'even']:
            raise ValueError ('parity parameter shall be "odd" or "even"')
        self._parity = parity
        self.serial_link.parity = parity_dict[parity]

    @property
    def baudrate(self):
        """Get or set the Modbus link baudrate. 
        Setting to a new value will reconfigure the serial port automatically.
        """
        return(self.serial_link.baudrate)
    
    @baudrate.setter
    def baudrate(self, baudrate):
        self.serial_link.baudrate = baudrate

    @property
    def timeout(self):
        """Get or set the Modbus link receive timeout.
        Setting to a new value will reconfigure the serial port automatically.
        """
        return(self.serial_link.timeout)
    @timeout.setter
    def timeout(self, timeout):
        self.serial_link.timeout = timeout

    @property
    def is_open(self):
        """Get the modbus link state."""
        return(self.serial_link.is_open)

    def register_feedback_handler(self, handler) -> None:
        """Register a feedback output handler.

        :param handler: Function called each time there is
                        a feedback from the Modbus class
        """
        self.feedback = handler

    def open (self, port = None) -> None:
        """Open the Modbus link.

        :param port: Serial port name or None (default).      

        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """
        self.serial_link.port = port
        self.serial_link.open()
 
    def close (self) -> None:
        """Close the Modbus link."""
        self.serial_link.close()

    def read_registers(self, 
                       device_addr: int, 
                       reg_addr: int, 
                       nb_reg: int = 1
                       ) -> list:
        """Implements Modbus function 03: Read holding registers.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param reg_addr: Starting address (0x0000 to 0xFFFF).
        :param nb_reg: Quantity of registers to read (1 to 125).

        :returns list of int: Registers' content.

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                             or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """       
        return(self._request(device_addr = device_addr, instr = 3,
                             reg_addr = reg_addr, nb_reg = nb_reg))

    def read_register(self, 
                       device_addr: int, 
                       reg_addr: int, 
                       ) -> int:
        """Implements Modbus function 03: Read holding registers.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param reg_addr: Starting address (0x0000 to 0xFFFF).

        :returns int: Register' value.

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """            
        return(self._request(device_addr = device_addr, instr = 3,
                             reg_addr = reg_addr, nb_reg = 1)[0])

    def write_single_coil(self,
                          device_addr: int,
                          coil_addr: int,
                          state: str,
                          key = None
                          ) -> None:
        """Implements Modbus function 05 Write single coil.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param coil_addr: Coil's address (0x0000 to 0xFFFF).
        :param state: 'ON' or 'OFF'.
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        state_dict = {'ON': 0xFF00, 'OFF': 0x0000, 'KEY': key}
        if state not in state_dict:
            raise ValueError("Unexpected value for state parameter")
        if state == 'KEY' and not isinstance(key, int):
            raise ValueError("key parameter shall be an integer")
        self._request(device_addr = device_addr, instr = 5, 
                      reg_addr = coil_addr, value =  state_dict[state])

    def write_register(self,
                       device_addr: int,
                       reg_addr: int,
                       value: int,
                       **kwargs
                       ) -> None:       
        """Implements Modbus function 06: Preset single register.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param reg_addr: Starting address (0x0000 to 0xFFFF).
        :param value: Value to write (0x0000 to 0xFFFF).
        :param kwargs: optional key arguments
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                             or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """          
        self._request(device_addr = device_addr, instr = 6, 
                      reg_addr = reg_addr, value = value, **kwargs)

    def query_data(self, device_addr: int, value = int, **kwargs) -> int:
        """Implements Modbus function 08 > Sub-function 00: Query data.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param value: Value to query (0x0000 to 0xFFFF).

        returns int: The query data returned value.

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """                     
        return(self._request(device_addr = device_addr, instr = 8,
                              subfunction = 0, value = value, **kwargs))

    def diag_read(self, device_addr: int, subfunction: int, **kwargs) -> int:
        """Implements Modbus function 08 > 
                                    Sub-function 11 to 18: Return counters.
        
        :param device_addr: Modbus slave address of the device (0 to 247).
        :param subfunction: Sub-function (11 to 18).

        :returns int: The requested counter value.

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        return(self._request(device_addr = device_addr,
                             instr = 8, subfunction = subfunction, **kwargs))

    def write_registers(self,
                        device_addr: int, 
                        reg_addr: int, 
                        values: int, 
                        **kwargs
                        ) -> None:          
        """Implements Modbus function 16: Write multiple registers.
        
        :param device_addr: Modbus slave address of the device (0 to 247).
        :param reg_addr: First register's address (0x0000 to 0xFFFF).
        :param values: list of words (0x0000 to 0xFFFF) to write (max 123 words).

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                             or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """         
        return(self._request(device_addr = device_addr, instr = 16,
                              reg_addr = reg_addr, data = values, **kwargs))
    
    def read_device_id(self, device_addr: int, **kwargs) -> int:
        """Implements Modbus function 43 > MEI 14: Read device id.

        :param device_addr: Modbus slave address of the device (0 to 247).

        :returns int: device id.
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        return(self._request(device_addr = device_addr, instr = 43,
                             mei_type = 14, idcode = 1, object_id = 0,
                             **kwargs))

    def _request(
        self, device_addr: int, instr: int,
        mode: str = 'legacy', hack: str = None, **kwargs
        ) -> int or list:
        """Modbus general request function
        Supported Modbus functions:
            * Fonction 03 (read holding registers)
            * Fonction 04 (read input registers)
            * Fonction 05 (force single coil)
            * Fonction 06 (preset single resgister)
            * Fonction 08 (diagnostic)
            * Fonction 11 (get comm event counter)
            * Fonction 12 (get comm event log)
            * Fonction 16 (write_registers)
            * Fonction 43 (Read Device Identification)

        :param kwargs:
                reg_addr (int): first register address
                nb_reg (int): nb of registers requested
                coil_addr (int): address of the coil
                value (int): value to write
                data (list[int]): values to write
                subfunction (int):
                mei_type (int):
                id_code (int):
                object_id (int):

        :returns int or list[int]: value(s) returned by the modbus device

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """       
        # Modbus broadcast address
        BROADCAST_ADDR = 0
        # Allowed modbus address range
        MODBUS_ADDR_RANGE = range(0, 247 + 1)
        # Allowed values' range
        DATA_RANGE = range(0, 0xFFFF + 1)
        # Registers' address range
        REG_ADDR_RANGE = range(0, 0xFFFF + 1)
        # Maximum number of registers that can be write in a single operation
        MAX_WRITE_LENGTH = 123   
        # Allowed nb of registers to be read (function 03, 04 and 23)
        NB_REG_READ_RANGE = range(1, 125 + 1)


        if device_addr not in MODBUS_ADDR_RANGE:
            raise ValueError(f"Modbus address shall be in [0..247]")
        if device_addr == BROADCAST_ADDR:
            if instr not in [5, 6]:
                raise ValueError(f"Unsupported Modbus instruction {instr} "
                                 "in broadcast")
        if mode not in ['legacy','fast']:
            raise ValueError(f"Unsupported value {mode} for mode parameter")
        
        if hack not in [None, 'corrupt_crc', 'add_one_byte',
                        'add_two_bytes', 'miss_one_byte']:
            raise ValueError(f"Unsupported value {hack} for hack parameter")
        
        if 'reg_addr' in kwargs:
            if  kwargs['reg_addr'] not in REG_ADDR_RANGE:
                raise ValueError(f"Value {kwargs['value']} "
                                 f"is not in {REG_ADDR_RANGE}")
        if 'nb_reg' in kwargs:
            if  kwargs['nb_reg'] not in NB_REG_READ_RANGE:
                raise ValueError(f"quantity of registers {kwargs['nb_reg']} "
                                 "out of range (1..125)")
        
        ### ???? Merging value and data ????
        if 'value' in kwargs:
            if kwargs['value'] not in DATA_RANGE:
                raise ValueError(f"Value {kwargs['value']} "
                                 f"is not in {DATA_RANGE}")
        if 'data' in kwargs:
            if len(kwargs['data']) > MAX_WRITE_LENGTH:
                raise ValueError(f"Data lenght id > {MAX_WRITE_LENGTH}")
            for word in kwargs['data']:
                if word not in DATA_RANGE:
                    raise ValueError("Some data word is"
                                    f"is not in {DATA_RANGE}")
                
        # Modbus frame construction
        tx_data = [device_addr, instr]
        # 3: Read holding registers / 4: Read input registers
        if instr in [3,4]:
            tx_data.extend(_word2bytes(kwargs['reg_addr']))
            tx_data.extend(_word2bytes(kwargs['nb_reg']))
        # 5: Force single coil / 6: Preset single register
        elif instr in [5,6]:
            tx_data.extend(_word2bytes(kwargs['reg_addr']))
            tx_data.extend(_word2bytes(kwargs['value']))
        # 8: Diagnostics
        elif instr == 8:
            subfunction = kwargs['subfunction']
            tx_data.extend(_word2bytes(subfunction))
            # Subfunction 0: Query data
            if subfunction == 0:                  
                tx_data.extend(_word2bytes(kwargs['value']))
            # Subfunctions 11..18: Return a counter
            elif subfunction in range(11, 19):    
                tx_data.extend([0,0]) 
            else:
                raise ValueError(f"Unsuported subfunction {subfunction}")
        # 11: Get comm event count / 12: Get comm event log
        elif instr in [11,12]:  
            pass
        # 16: write registers
        elif instr == 16:       
            data = kwargs['data']   # Raise an exception if data key not in kwargs
            tx_data.extend(_word2bytes(kwargs['reg_addr']))
            nb_reg = len(data)
            if nb_reg > MAX_WRITE_LENGTH:
                raise ValueError(f"Data length is > {MAX_WRITE_LENGTH}")
            # Quantity of registers
            tx_data.append(0)                   # MSB is always 0
            tx_data.append(nb_reg)
            # Byte count
            tx_data.append(2 * nb_reg)
            for value in data:
                tx_data.extend(_word2bytes(value))
        elif instr == 43:       # 43: Read device identification
            tx_data.append(kwargs['mei_type'])
            tx_data.append(kwargs['idcode'])
            tx_data.append(kwargs['object_id'])

        else:
            raise ValueError(f"Unsuported instruction: {instr}")

        # Frame hacks:
        # Add one byte to the tx_data list
        if hack == 'add_one_byte':  tx_data.append(0x55)
        # Add two bytes to the tx_data list
        if hack == 'add_two_bytes': tx_data.extend([0x55, 0x56])
        # Remove the last byte of the tx_data list 
        if hack == 'miss_one_byte': del tx_data[-1]

        crc = _crc16(tx_data)  # Add CRC bytes to the Modbus frame to be send

        # CRC hack:
        if hack == 'corrupt_crc':   crc += 1

        tx_data.extend((crc & 0x00FF,crc >> 8))
        
        # Send the frame
        self.serial_link.reset_input_buffer()
        self.serial_link.write(tx_data)
        self.feedback(Sent = tx_data)
        if device_addr == BROADCAST_ADDR: return(0)      #No response expected

        # Receive the first bytes with a long timeout
        rx_data = list(self.serial_link.read(5))
        if len(rx_data) == 0:          # Received list is empty (timeout)
            self.feedback(Event = 'No_response', Received = None)
            raise NoResponseError("Empty frame")
        # Contrôle du nombre d'octets reçus
        if len(rx_data) < 5:           # Received list length is < 5 bytes
            self.feedback(Event = 'Frame_lenght_error', Received = rx_data)
            raise InvalidResponseError("Frame lenght error")

        # --- Modbus reception in fast mode
        # (computation of the expected receveid frame lenght instead of timeout)
        if mode == 'fast':
            # Exceptions and functions for which 5 bytes are expected:
            if rx_data[1] == 7 or rx_data[1] > 127:           
                pass
            # Functions for which rx_data[2] is a byte_count
            # -> expecting "byte_count" more bytes
            elif rx_data[1] in [3, 4, 12, 23]:
                rx_data.extend(list(self.serial_link.read(rx_data[2])))
            # Functions for which 8 bytes (3 more bytes) are expected
            elif rx_data[1] in [5, 6, 8, 11, 15, 16, 17]:
                rx_data.extend(list(self.serial_link.read(3)))
            # Functions for which 10 bytes (5 more bytes) are expected
            elif rx_data[1] == 22:
                rx_data.extend(list(self.serial_link.read(5)))
            # Function 43: Encapsulated Interface Transport
            elif rx_data[1] == 43:
                # MEI type = 14: Read device identification                     
                if rx_data[2] == 14:                             
                    ## Ici il faut dépioter toute la trame pour savoir le nombre d'octets attendus!!!
                    pass
            # Functions that are not supported in fast mode
            else:
                raise ValueError("Fast mode not available for instruction "
                                 f"{instr}, please try with legacy mode")
        # --- End of fast mode Modbus reception

        # --- Modbus reception in legacy mode (wait for a timeout)   
        else:
            # 1ms wait (OS does not allow 750µs timeout (T1.5))
            time.sleep(0.001)       
            while (self.serial_link.in_waiting != 0):
                rx_data.extend(list(self.serial_link.read(
                                            self.serial_link.in_waiting)))
                # 1ms wait (OS does not allow 750µs timeout (T1.5))
                time.sleep(0.001)
                # The modbus T3.5 is not verified
                # OS precision does not allow accurate verificatio of T3.5
        # --- End of legacy mode Modbus reception

        # CRC check
        # crc computation on the entire frame shall give 0
        if _crc16(rx_data) != 0:
            self.feedback(Event = "CRC_error", Received = rx_data)
            raise InvalidResponseError("CRC error")

        # Check that the received frame is from the requested device
        if rx_data[0] != device_addr:   # Not the expected device
            self.feedback(Event = "Unexpected_content", Received = rx_data)
            raise InvalidResponseError("Modbus device address does not match"
                            f"Expected: {device_addr}, Received: {rx_data[0]}")

        # Check that the returned instruction is consistent with the request
        if rx_data[1] & 0b1111111!= instr:
            self.feedback(Event = "Unexpected_content", Received = rx_data)
            raise InvalidResponseError("Returned instruction does not match"
                            f"Expected: {instr}, Received: {rx_data[1]}")

        # Check if the returned frame is an exception
        if (rx_data[1] & 0b10000000) != 0:
            # Illegal request Modbus exceptions (legal exception's codes)
            if rx_data[2] in [1, 2, 3]:     
                raise IllegalRequestError(f"{exceptions_dict[rx_data[2]]}")
            # Other legals Modbus exception's codes
            if rx_data[2] in [4, 5, 6, 7, 8, 9, 10]:
                exception_str = exceptions_dict[rx_data[2]]
            # Illegals Modbus exception's codes
            else:
                exception_str = "Illegal exception code"
            raise SlaveExceptionError(f"Receive Modbus Exception code "
                                      f"{rx_data[2]}: {exception_str}")

        # Instructions 5, 6 or 16
        if rx_data[1] in [5, 6, 16]:
            if len(rx_data)!= 8:
                self.feedback(Event = "Frame_lenght_error", Received = rx_data)
                raise InvalidResponseError("Frame lenght error :", len(rx_data))
            # Check the frame content conformity (shall be an echo)
            if rx_data[2:6] != tx_data[2:6]:
                self.feedback(Event = "Unexpected_content", Received = rx_data)
                raise InvalidResponseError("Returned content does not match")
            ### J'ai un doute pour 'Frame_OK', on pourrait trouver mieux ?
            self.feedback(Event = "Frame_OK", Received = rx_data)
            #### J'ai un doute pour les return(0), est-ce que l'on pourrait return() ??
            return(0)
        
        # Instructions 3 or 4
        if rx_data[1] in [3,4]:
            # Check the frame length conformity
            if len(rx_data)!= rx_data[2] + 5:     
                self.feedback(Event = 'Frame_lenght_error', Received = rx_data)
                raise InvalidResponseError("Frame lenght error")
            # Check the frame content conformity 
            if rx_data[2] != 2 * kwargs['nb_reg']:  # 3rd byte == nb bytes ?
                self.feedback(Event = 'Unexpected_content', Received = rx_data)
                raise InvalidResponseError("Returned data lenght error")
            # Returned frame is conform
            self.feedback(Event = "Frame_OK", Received = rx_data)
            data = []
            for i in range (kwargs['nb_reg']):
                data.append((rx_data[3 + 2 * i] << 8) + rx_data[4 + 2 * i])
            return(data)

        # Instruction 8
        if rx_data[1] == 8:
            # Check the frame length conformity
            if len(rx_data)!= 8:
                self.feedback(Event = "Frame_lenght_error", Received = rx_data)
                raise InvalidResponseError("Frame lenght error")
            # Check the diagnostic subfunction conformity
            if (rx_data[2] << 8 ) + rx_data[3] != kwargs['subfunction']:
                self.feedback(Event = "Unexpected_content", Received = rx_data)
                raise InvalidResponseError("Returned sub-function error")
            # Returned frame is conform
            self.feedback(Event = "Frame_OK", Received = rx_data)
            return((rx_data[4] << 8) + rx_data[5])
        # Requested instruction is not implemented at reception side 
        raise ValueError("Instruction not allowed")

if __name__ == "__main__":
    '''Modbus module test routine'''
    def terminal_output(**kwargs):
        '''This function output everything passed as
            key arguments to the terminal.
        '''
        for k, v in kwargs.items():
            print(f"{k}: {v}")

    # Create a modbus link  with defaut parameters (Modbus class instantiation)
    link = Modbus()
    # Open the modbus link on "COM3"
    link.open(port = 'COM3')        
   
    # Read the 1st four registers from the modbus device with slave address = 1
    try: 
        data = link.read_registers(device_addr = 1, reg_addr = 0, nb_reg = 4) 
    except ModbusError as exc:
        print(f"Modbus error: {exc}")
    else:
        for i, value in enumerate(data):
            print(f'Register {i} = {data[i]}')  # output values on the terminal
    
    # Register terminal_output function as handler for the feeback from modbus class
    # All feeback informations from modbus class will then be output to the terminal
    link.register_feedback_handler(terminal_output)

    # Read the first register from the modbus device with slave address = 1
    try: 
        data = link.read_registers(device_addr = 1, reg_addr = 268, nb_reg = 1)
    except ModbusError as exc:
        print(f'Modbus error: {exc}')
    else:
        print(f'Register 0 = {data[0]}')

    # Set energy level preset to 2 (in case of a SxIP device)
    link.write_register(device_addr = 1, reg_addr = 49, value = 2)

    # Change Modbus link parity
    #print (f'link.parity = {link.parity}')
    #link.parity = 'odd'
    #print (f'link.parity = {link.parity}')

    # Change Modbus link parity
    print (f'link.parity = {link.parity}')
    link.parity = 'odd'
    print (f'link.parity = {link.parity}')

    # Change Modbus link baudrate
    #print (f'link.baudrate = {link.baudrate}')
    #link.baudrate = 'B'
    #print (f'link.baudrate = {link.baudrate}')

    # Read the first register from the modbus device with slave address = 1
    # A ModbusError exception occurs du to the wrong parity
    try: 
        data = link.read_registers(device_addr = 1, reg_addr = 268, nb_reg = 1)
    except ModbusError as exc:
        print(f'Modbus error: {exc}')
    else:
        print(f'Register 0 = {data[0]}')
    
    link.close()                    # Close the modbus link



