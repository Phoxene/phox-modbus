# -*- coding: utf-8 -*-
# Copyright (c) 2025 PHOXENE
# MIT License: 
# https://opensource.org/license/mit/
#
""" Python driver for Modbus RTU on serial port.

Usage:
    This driver has been developped as Modbus layer for
    Phoxene's devices that implements a serial Modbus communication.
    
    It is mainly used by Phoxene's applications or device drivers

Modbus protocole violations:
    - By defaut, the driver use frame lenght prediction instead of T1.5 timeout.
        -> can be skipped by adding kwarg mode = "legacy" to the modbus requests.
    - If some extra data is received during the T3.5 timeout:
        - the timeout is restarted
        - a feedback event is generated
        - the frame is processed while Modbus protocole says it shall not be.


Exceptions handling:
    Function involving serial communication can raises following exceptions:
        serial.SerialExeption(IOError): 
                Base class for serial port related exceptions
        serial.SerialTimeoutException(SerialException):
                Write timeouts case an exception
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
__date__ = "2025-09-11"
__version__= "1.2.0"
# Changes 1.0.0 -> 1.0.1
# Correct some comments
# - Fix a bug where two consecutive request lead to reception error on a non-addressed device because the T3.5 modbus timing was not respected.
# - Add T1.5 and T3.5 modbus timeouts updated accordingly to baudrate (1.5 x 11 x bit duration; 3.5 x 11 x bit duration). 
#    For baudrate > 19200bps, minimum T1.5 = 750µs and T3.5 = 1750µs are not respected in order not to slow down the communication at high baudrates
#
# Changes 1.0.0 -> 1.0.1:
# - Fix a bug affecting query data diagnostic command
# - Add "bad_parity" and "serial_overflow" hacks
# - Some changes in the Factory f_key management
#
# Changes 1.1.0 -> 1.2.0:
# - Add a max_serial_latency parameter to the Modbus class. This parameter is intented to increase Modbus T1.5 and T3.5 parameters
#   when the client Modbus client serial latency can not comply with Modbus standard
# - Add restart communication support (Modbus function 08 sub-function 01)
# - Add return diag register support (Modbus function 08 sub-function 02)
# - Add force listen only support (Modbus function 08 sub-function 04)
# - Add clear overrun support (Modbus function 08 sub-function 20)
#
#Style guide: refers to PEP 8
#Type Hints: refers to PEP 484
#Docstrings: refers to Spinx documentation 

import time
import serial
from threading import Timer

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
    10: "Gateway target device failed to respond"
 }

parity_dict = {'odd' : serial.PARITY_ODD, 'even' : serial.PARITY_EVEN} 

objects_id_dict = {
    0 : "VendorName",
    1 : "ProductCode",
    2 : "MajorMinorRevision",
    3 : "VendorUrl",
    4 : "ProductName",
    5 : "ModelName",
    6 : "UserApplicationName"
}

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
    """The response does not fulfill the Modbus standard."""

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
    :param max_serial_latency: Shall be set to the maximum expected serial latency of the system
                               Default is 0.001s
                               Increase this parameter if peaces of frame are lost in legacy mode

    :returns: A modbus link object
    """
    def __init__(self, 
                 port: str = None, 
                 baudrate: int = 19200, 
                 timeout: int = 0.1, 
                 parity: str = 'even',
                 max_serial_latency = 0.001
                 ):   
        self.serial_link = serial.Serial(port = port,
                                         stopbits = serial.STOPBITS_ONE,
                                         bytesize = serial.EIGHTBITS,
                                         timeout = timeout,
                                         write_timeout = 0)
        self.max_serial_latency = max_serial_latency
        self.default_timeout = timeout
        self.parity = parity
        self.init_baudrate = baudrate   # Used to restor default baudrate
        self.baudrate = baudrate

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
        self.t15 = max(1.5 * (11 / baudrate), self.max_serial_latency)
        self.t35 = max(3.5 * (11 / baudrate), 2 * self.max_serial_latency)

    @property
    def timeout(self):
        """Get or set the Modbus link receive timeout.
        Setting to a new value will reconfigure the serial port automatically.
        """
        return(self.serial_link.timeout)
    @timeout.setter
    def timeout(self, timeout):
        if not isinstance (timeout, (int, float)):
            raise ValueError ("timeout parameter must be an integer")
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

    def open(self, port = None) -> None:
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
                       nb_reg: int = 1,
                       **kwargs 
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
                             reg_addr = reg_addr, nb_reg = nb_reg, **kwargs))

    def read_register(self, 
                       device_addr: int, 
                       reg_addr: int,
                       **kwargs
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
                             reg_addr = reg_addr, nb_reg = 1, **kwargs)[0])

    def write_single_coil(self,
                          device_addr: int,
                          coil_addr: int,
                          state: str,
                          f_key = None,
                          **kwargs
                          ) -> None:
        """Implements Modbus function 05 Write single coil.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param coil_addr: Coil's address (0x0000 to 0xFFFF).
        :param state: 'ON' or 'OFF'.
        :param kwargs: optional f_key argument
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        state_dict = {'ON': 0xFF00, 'OFF': 0x0000, 'KEY': f_key}
        if state not in state_dict:
            raise ValueError("Unexpected value for state parameter")
        if state == 'KEY' and f_key is None:
            raise ValueError("f_key parameter is missing")
        self._request(device_addr = device_addr, instr = 5, 
                      reg_addr = coil_addr, data =  state_dict[state], **kwargs)

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
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                             or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """          
        self._request(device_addr = device_addr, instr = 6, 
                      reg_addr = reg_addr, data = value, **kwargs)

    def query_data(self, device_addr: int, value = int, **kwargs) -> int:
        """Implements Modbus function 08 > Sub-function 00: Query data.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param data: Value to query (0x0000 to 0xFFFF).

        returns int: The query data returned value.

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """                     
        return(self._request(device_addr = device_addr, instr = 8,
                              subfunction = 0, value = value, **kwargs))

    def restart_communication(self, device_addr: int, **kwargs) -> None:
        """Implements Modbus function 08 > Sub-function 01: Restart communication
        
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """   
        self._request(device_addr = device_addr,
                             instr = 8, subfunction = 1, **kwargs)

    def return_diag_register(self, device_addr: int, **kwargs) -> int:
        """Implements Modbus function 08 > Sub-functions 02: Return diag register.

        :returns int: Status
        
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """   
        return self._request(device_addr = device_addr,
                             instr = 8, subfunction = 2, **kwargs)

    def force_listen_only(self, device_addr: int, **kwargs) -> None:
        """Implements Modbus function 08 > Sub-function 04: Force listen only
        
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """   
        self._request(device_addr = device_addr,
                             instr = 8, subfunction = 4, **kwargs)

    def diag_read(self, device_addr: int, subfunction: int, **kwargs) -> int:
        """Implements Modbus function 08 > Sub-functions 11 to 18: Return counters.
        
        :param device_addr: Modbus slave address of the device (0 to 247).
        :param subfunction: Sub-function (11 to 18).

        :returns int: The requested counter value.

        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        return self._request(device_addr = device_addr,
                             instr = 8, subfunction = subfunction, **kwargs)

    def clear_overrun(self, device_addr: int, **kwargs) -> None:
        """Implements Modbus function 08 > Sub-functions 20: Clear overrun
        
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """   
        self._request(device_addr = device_addr,
                             instr = 8, subfunction = 20, **kwargs)

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
    
    def read_device_id(self, device_addr: int, idcode = 1, object_id = 0, **kwargs) -> dict:
        """Implements Modbus function 43 > MEI 14: Read device id.

        :param device_addr: Modbus slave address of the device (0 to 247).
        :param idcode: Read device ID code 
                       (01: basic, 02: regular, 03: extended, 04: specific)
        :param object_id: Id of the first requested object

        :returns dict: dictionnay of id objects.
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        return(self._request(device_addr = device_addr, instr = 43,
                             mei_type = 14, idcode = idcode, object_id = object_id,
                             **kwargs))

    def get_comm_event_log(self, device_addr: int, **kwargs) -> dict:
        """Implements Modbus function 12 -> Get Comm Event Log

        :param device_addr: Modbus slave address of the device (0 to 247).

        :returns dict: Status (int), Event count (int), Events (list[64])
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        return(self._request(device_addr = device_addr, instr = 12, **kwargs))

    def get_comm_event_counter(self, device_addr: int, **kwargs) -> dict:
        """Implements Modbus function 11 -> Get Comm Event Counter

        :param device_addr: Modbus slave address of the device (0 to 247).

        :returns dict: Status (int), Event count (int)
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        return(self._request(device_addr = device_addr, instr = 11, **kwargs))


    def report_server_id(self, device_addr: int, **kwargs) -> dict:
        """Implements Modbus function 17 -> Report Server ID

        :param device_addr: Modbus slave address of the device (0 to 247).

        :returns dict: Server ID (str), Run indicator status ('ON' or 'OFF")
        
        :raises ValueError: Arguments are out of range.
        :raises ModbusError: Modbus protocol error
                            or the device answers with an exception.
        :raises serial.SerialException: Serial port is missing, busy,
                                        or can not be configured.
        """        
        return(self._request(device_addr = device_addr, instr = 17, **kwargs))

    def _request(self, device_addr: int, instr: int,
                 mode: str = 'fast', hack: str = None, **kwargs) -> list:
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
            * Fonction 17 (Report server ID)
            * Fonction 43 (Read Device Identification)

        :param kwargs:
                reg_addr (int): first register address or coil address
                nb_reg (int): nb of registers to read / write
                data (list[int]): values to write
                subfunction (int):Sub-function required for function 8
                mei_type (int): Mei type required for function 43
                id_code (int): id_code required for function 43
                object_id (int): object id required for function 43
                timeout (int): modbus timeout be used instead of default timeout

        :returns int or list[int] or dict: of value(s) returned by the modbus device

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
        if device_addr == BROADCAST_ADDR and instr not in [5, 6]:
            raise ValueError(f"Unsupported Modbus instruction {instr} "
                             "in broadcast")
        if mode not in ['legacy','fast']:
            raise ValueError(f"Unsupported value {mode} for mode parameter")
        if hack not in [None, 'bad_parity', 'bad_baudrate', 'serial_overflow', 'corrupt_crc', 'add_one_byte',
                        'add_two_bytes', 'miss_one_byte']:
            raise ValueError(f"Unsupported value {hack} for hack parameter")
        if 'reg_addr' in kwargs and kwargs['reg_addr'] not in REG_ADDR_RANGE:
            raise ValueError(f"reg_addr {kwargs['reg_addr']} "
                             f"is not in {REG_ADDR_RANGE}")
        if 'nb_reg' in kwargs and kwargs['nb_reg'] not in NB_REG_READ_RANGE:
            raise ValueError(f"quantity of registers {kwargs['nb_reg']} "
                             f"is not in  {NB_REG_READ_RANGE}")
        
        if 'value' in kwargs and kwargs['value'] not in DATA_RANGE:
            raise ValueError(f"Value {kwargs['value']} "
                             f"is not in {DATA_RANGE}")
        if 'data' in kwargs:
            if kwargs['data'] == None:
                raise ValueError("data = None")
            # If data is an int
            if isinstance (kwargs['data'], int):
                if kwargs['data'] not in DATA_RANGE:
                    raise ValueError(f"data = {kwargs['data']} "
                                f"is not in {DATA_RANGE}")
            # If data is a list of int
            else:
                if len(kwargs['data']) > MAX_WRITE_LENGTH:
                    raise ValueError(f"Data lenght > {MAX_WRITE_LENGTH}")
                for word in kwargs['data']:
                    if word not in DATA_RANGE:
                        raise ValueError(f"data[{i}] = {word} that"
                                        f"is not in {DATA_RANGE}")

        # Serial timeout management
        if 'timeout' in kwargs and kwargs['timeout'] != None: 
            self.timeout = kwargs['timeout']
        else:
            # Not fine but does not increase significantly the execution time
            self.timeout = self.default_timeout
        
        # Modbus frame construction
        tx_data = [device_addr, instr]
        # 3: Read holding registers / 4: Read input registers
        if instr in [3,4]:
            tx_data.extend(_word2bytes(kwargs['reg_addr']))
            tx_data.extend(_word2bytes(kwargs['nb_reg']))
        # 5: Force single coil / 6: Preset single register
        elif instr in [5,6]:
            tx_data.extend(_word2bytes(kwargs['reg_addr']))
            tx_data.extend(_word2bytes(kwargs['data']))
        # 8: Diagnostics
        elif instr == 8:
            subfunction = kwargs['subfunction']
            # Subfunction 0: Query data
            if subfunction == 0:                  
                value = kwargs['value']
            # Subfunction 1: Restart communication option
            # Subfunction 2: Return diagnostic register
            # Subfunction 4: Force listen only mode
            # subfunction 10: Clear counters and Diagnostic registers
            # Subfonctions 11 to 18: Return a message counter     
            # Subfunction 20: Clear overrun counter and flag            
            elif subfunction in [1, 2, 4, 10, 20, *range (11,19)]:
                value = 0
            else:
                raise ValueError(f"Unsuported subfunction {subfunction}")
            
            tx_data.extend(_word2bytes(subfunction))
            tx_data.extend(_word2bytes(value))

        # 11: Get comm event count / 12: Get comm event log / 17: Report slave ID
        elif instr in [11, 12, 17]:  
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
        
        # 43: Read device identification
        elif instr == 43:      
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

        # Add CRC bytes to the Modbus frame to be send
        crc = _crc16(tx_data)  

        # CRC hack:
        if hack == 'corrupt_crc': crc += 1
        tx_data.extend((crc & 0x00FF,crc >> 8))

        # Bad parity hack:
        if hack == 'bad_parity': 
            self.parity = 'odd'

        # Bad baudrate hack:
        if hack == 'bad_baudrate': 
            self.baudrate *= 2      # Double the baudrate

        # Serial overflow hack:
        if hack == 'serial_overflow': 
            for i in range(260):
                tx_data.append(0xFF)

        # Send the frame
        self._send_frame(tx_data)

        # Restore parity in case of Bad parity hack:
        if hack == 'bad_parity':
            time.sleep(0.1)     # Sans ce delay la paritée est parfois restaurée avant l'envoie de la trame
            self.parity = 'even'

        # Restore baudrate in case of Bad bauderate hack:
        if hack == 'bad_baudrate': 
            time.sleep(0.1)  
            self.baudrate /= 2  # Restore the baudrate

        if device_addr == BROADCAST_ADDR: return 0      #No response expected

        # Frame reception
        rx_data = self._receive_frame(mode)
        # frame integrity verification
        self._frame_check(device_addr, instr, rx_data)
        # Frame decoding according to instruction
        if rx_data[1] in [5, 6, 16]:
            return self._decode_instr_5_6_16(rx_data, tx_data)
        if rx_data[1] in [3,4]:
            return self._decode_instr_3_4(rx_data, kwargs['nb_reg'])
        if rx_data[1] == 8:
            return self._decode_instr_8(rx_data, kwargs['subfunction'])
        if rx_data[1] == 11:
            return self._decode_instr_11(rx_data)
        if rx_data[1] == 12:
           return self._decode_instr_12(rx_data)
        if rx_data[1] == 17:
            return self._decode_instr_17(rx_data)
        if rx_data[1] == 43:
            return self._decode_instr_43(rx_data, kwargs['idcode'])
        # Received instruction is not implemented at reception side 
        raise ValueError("Instruction not allowed")
    
    def _send_frame(self, data):
        self.serial_link.reset_input_buffer()
        self.serial_link.write(data)
        self.feedback(Sent = data)

    def _frame_check(self, device_addr, instr, rx_data):
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
            self.feedback(Event = "Modbus Exception", Received = rx_data)
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

    def _decode_instr_3_4(self, rx_data : list, nb_reg: int) -> list:
        # Check the frame length conformity
        if len(rx_data)!= rx_data[2] + 5:     
            self.feedback(Event = 'Frame_lenght_error', Received = rx_data)
            raise InvalidResponseError("Frame lenght error")
        # Check the frame content conformity 
        if rx_data[2] != 2 * nb_reg:  # 3rd byte == nb bytes ?
            self.feedback(Event = 'Unexpected_content', Received = rx_data)
            raise InvalidResponseError("Returned data lenght error")
        # Returned frame is conform
        self.feedback(Event = "Frame_OK", Received = rx_data)
        data = []
        for i in range (nb_reg):
            data.append((rx_data[3 + 2 * i] << 8) + rx_data[4 + 2 * i])
        return(data)
    
    def _decode_instr_5_6_16(self, rx_data : list, tx_data: list) -> None:
        if len(rx_data)!= 8:
            self.feedback(Event = "Frame_lenght_error", Received = rx_data)
            raise InvalidResponseError("Frame lenght error :", len(rx_data))
        # Check the frame content conformity (shall be an echo)
        if rx_data[2:6] != tx_data[2:6]:
            self.feedback(Event = "Unexpected_content", Received = rx_data)
            raise InvalidResponseError("Returned content does not match")
        ### J'ai un doute pour 'Frame_OK', on pourrait trouver mieux ?
        self.feedback(Event = "Frame_OK", Received = rx_data)
        return

    def _decode_instr_8(self, rx_data : list, subfunction: int) -> int:
        # Check the frame length conformity
        if len(rx_data)!= 8:
            self.feedback(Event = "Frame_lenght_error", Received = rx_data)
            raise InvalidResponseError("Frame lenght error")
        # Check the diagnostic subfunction conformity
        if (rx_data[2] << 8 ) + rx_data[3] != subfunction:
            self.feedback(Event = "Unexpected_content", Received = rx_data)
            raise InvalidResponseError("Returned sub-function error")
        # Returned frame is conform
        self.feedback(Event = "Frame_OK", Received = rx_data)
        return((rx_data[4] << 8) + rx_data[5])
    
    def _decode_instr_11(self, rx_data : list) -> dict:
        self.feedback(Event = "Frame_OK", Received = rx_data)
        dict = {}
        dict["Status"] = (rx_data[2] << 8) + rx_data[3]
        dict["Event count"] = (rx_data[4] << 8) + rx_data[5]
        return(dict)

    def _decode_instr_12(self, rx_data : list) -> dict:
        # According to the Modbus specification, Event count is not the number
        # of recorded event but the number of successful message completion
        self.feedback(Event = "Frame_OK", Received = rx_data)
        byte_count = rx_data[2]
        nb_events = byte_count - 6
        dict = {}
        dict["Status"] = (rx_data[3] << 8) + rx_data[4]
        dict["Event count"] = (rx_data[5] << 8) + rx_data[6]
        dict["Message count"] = (rx_data[7] << 8) + rx_data[8]
        dict["Events"] = rx_data[9 : 9 + nb_events]
        return(dict)
    
    def _decode_instr_17(self, rx_data : list) -> dict:
        byte_count = rx_data[2]
        id_length = byte_count - 1
        if rx_data[1 + byte_count] == 0:
            run_indicator_status = "OFF"
        elif rx_data[byte_count + 2] == 0xFF:
            run_indicator_status = "ON"
        else:
            self.feedback(Event = "Unexpected_content", Received = rx_data)
            raise InvalidResponseError("Unexpected value for run indicator status field")
        self.feedback(Event = "Frame_OK", Received = rx_data)
        dict = {}
        dict["Server ID"] = ''.join(chr(c) for c in rx_data[3 : 3 + id_length])
        dict["Run indicator status"] = run_indicator_status
        return(dict)

    def _decode_instr_43(self, rx_data : list, idcode : int) -> list:
        # MEI type = 14: Read device identification                     
        if rx_data[2] == 14:
            if rx_data[3] != idcode:
                self.feedback(Event = "Unexpected_content", Received = rx_data)
                raise InvalidResponseError("Returned idcode error")
            # rx_data[4] = conformity level
            if rx_data[4] not in [0x01, 0x02, 0x03, 0x81, 0x82, 0x83]:
                self.feedback(Event = "Unexpected_content", Received = rx_data)
                raise InvalidResponseError("Conformity level byte value is invalid")
            # rx_data[5] = more follows. 0x00 = nothing follows, 0xFF = more follows
            if rx_data[5] not in [0x00, 0xFF]:
                self.feedback(Event = "Unexpected_content", Received = rx_data)
                raise InvalidResponseError("More follow byte value is invalid")
            if rx_data[5] == 0xFF:
                raise InvalidResponseError("More follow option is not supported")
            if rx_data[6] != 00:
                self.feedback(Event = "Unexpected_content", Received = rx_data)
                raise InvalidResponseError("Next object id error")
            number_of_objects = rx_data[7]
            object_position = 8                     # First object ID position in rx_data is on byte 8
            object_dict = {}
            for i in range(0, number_of_objects):
                object_id = rx_data[object_position]
                object_length = rx_data[object_position + 1]
                object_value = rx_data[(object_position + 2):(object_position + 2 + object_length)]
                object_str = ''.join(chr(c) for c in object_value)
                object_position += (object_length + 2)
                if object_id in objects_id_dict:
                    key = objects_id_dict[object_id]
                    object_dict[key] = object_str
                else: 
                    key = object_id
                    object_dict[key] = object_value
            self.feedback(Event = "Frame_OK", Received = rx_data)
            return(object_dict)
    
    def _receive_frame(self, mode):
     # Receive the first bytes within timeout
        rx_data = list(self.serial_link.read(5))
        if len(rx_data) == 0:          # Received list is empty (timeout)
            self.feedback(Event = 'No_response', Received = None)
            raise NoResponseError("Empty frame")
        if len(rx_data) < 5:           # Received list length is < 5 bytes
            self.feedback(Event = 'Frame_lenght_error', Received = rx_data)
            raise InvalidResponseError("Frame lenght error")
        if mode == 'fast': 
            return self._fast_reception(rx_data)
        else: 
            return self._legacy_reception(rx_data)

    def _fast_reception(self, rx_data: list) -> list:
        # --- Modbus reception in fast mode
        # (computation of the expected receveid frame lenght instead of timeout)
        # Exceptions and functions for which 5 bytes are expected:
        if rx_data[1] == 7 or rx_data[1] > 127:           
            pass
        # Functions for which rx_data[2] is a byte_count
        # -> expecting "byte_count" more bytes
        elif rx_data[1] in [3, 4, 12, 17, 23]:
            rx_data.extend(list(self.serial_link.read(rx_data[2])))
        # Functions for which 8 bytes (3 more bytes) are expected
        elif rx_data[1] in [5, 6, 8, 11, 15, 16]:
            rx_data.extend(list(self.serial_link.read(3)))
        # Functions for which 10 bytes (5 more bytes) are expected
        elif rx_data[1] == 22:
            rx_data.extend(list(self.serial_link.read(5)))
        # Function 43: Encapsulated Interface Transport
        elif rx_data[1] == 43:
            # MEI type = 14: Read device identification                     
            if rx_data[2] == 14:
                # A minimum of 13 bytes (8 more bytes) are expected
                # 5 more byte are read
                rx_data.extend(list(self.serial_link.read(5)))
                number_of_objects = rx_data[7]
                index = 8                     # First object ID position in rx_data is on byte 8
                for i in range(0, rx_data[7]):          # rx_data[7] is the number of objects in the frame
                    object_length = rx_data[index + 1]
                    # Expected frame size is increased from object lenght + 2 bytes
                    rx_data.extend(list(self.serial_link.read(object_length + 2)))
                    index += object_length + 2
         # If the function is not supported in fast mode, turn to legacy mode
        else:
            self.feedback(Event = "Received frame can't be decoded in fast mode, turn to legacy mode")
            return self.legacy_reception(rx_data)
        time.sleep(self.t35)
        # If some data in the receive buffer
        while self.serial_link.in_waiting != 0:
            extra_data = list(self.serial_link.read(self.serial_link.in_waiting))
            self.feedback(Event = "Extra data received before T3.5 timeout", Received = extra_data)
            time.sleep(self.t35)
        return rx_data
    
    def _legacy_reception(self, rx_data: list) -> list:
        # --- Modbus reception in legacy mode (wait for a timeout)
        # According to the client system (computer, OS, RS485 transceiver...)
        # Legacy mode can catch additionnal bytes or lose pieces of frame
        # This is due to serial link latency that can be > T1.5 timeout 
        # Legacy mode is not recommanded
        # Legacy mode can be used in debug to catch a modbus string while ignoring the expected lenght
        time.sleep(self.t15)
        while self.serial_link.in_waiting != 0:
            rx_data.extend(list(self.serial_link.read(
                                        self.serial_link.in_waiting)))
            time.sleep(self.t15)
        # Sometimes, an extra 0xFF byte is received
        # The issue is not identified but patched by removing the last byte
        if rx_data[-1] == 0xFF and _crc16(rx_data) != 0:
            self.feedback(Event = "Extra 0xFF byte error", Received = rx_data)
            rx_data.pop()
        # T3.5 timeout
        time.sleep(self.t35 - self.t15)
        while self.serial_link.in_waiting != 0:
            # Extra data is reveived before the T3.5 timeout
            extra_data = list(self.serial_link.read(self.serial_link.in_waiting))
            self.feedback(Event = "Extra data received before T3.5 timeout", Received = extra_data)
            time.sleep(self.t35)
        return rx_data