# -*- coding: utf-8 -*-
# Copyright (c) 2025 PHOXENE
# MIT License: 
# https://opensource.org/license/mit/
#
""" Test for phox_modbus module

"""
__authors__ = ("Aurélien PLANTIN")
__contact__ = ("a.plantin@phoxene.com")
__copyright__ = "MIT"
__date__ = "2025-09-11"

import unittest                                         # The test framework
import phox_modbus.modbus as modbus                     # The module to be tested
from phox_modbus.modbus import ModbusError
from phox_modbus.modbus import IllegalRequestError
from serial import PortNotOpenError

COM_PORT = 'COM8'

def terminal_output(**kwargs):
    '''This function output everything passed as
        key arguments to the terminal.
    '''
    for k, v in kwargs.items():
        print(f"{k}: {v}")

class TestInternalFunctions(unittest.TestCase):
    def test_crc(self) -> None:
        # Simple crc computation result test
        self.assertEqual(modbus._crc16([1, 6, 0, 49, 0, 2]), 50265)
  
    def test_word2bytes(self) -> None:
        # Simple word2bytes result test
        self.assertEqual(modbus._word2bytes(0x4899), [0x48,0x99])

class TestWithNoCom(unittest.TestCase):
    def setUp(self):
        self.link = modbus.Modbus()

    def test_portnotopenerror(self) -> None:
        # Check that a read instruction without opening the port raises PortNotOpenError
        with self.assertRaises(PortNotOpenError):
            self.link.read_registers(device_addr = 1, reg_addr = 0, nb_reg = 1)

class TestWithCom(unittest.TestCase):
    def setUp(self):
        self.link = modbus.Modbus()
        self.link.open(port = COM_PORT)

    def tearDown(self):
        self.link.close()

    def test_single_write(self) -> None:
        # Read and write test reset_on_failure register
        self.link.write_register(device_addr = 1, reg_addr = 39, value = 879)
        self.assertEqual(self.link.read_register(device_addr = 1, reg_addr = 39), 879)

    def test_legacy_mode(self) -> None:
        # Query data in legacy mode
        self.assertEqual(self.link.query_data(device_addr = 1, value = 25639, mode = "legacy"), 25639)

    def test_not_allowed_broadcast(self) -> None:
        with self.assertRaises(ValueError): #Ajouter la vérification du text
            self.link.read_register(device_addr = 0, reg_addr = 0)

class TestIllegalRequests(unittest.TestCase):
    def setUp(self):
        self.link = modbus.Modbus()
        self.link.open(port = COM_PORT)
    
    def tearDown(self):
        self.link.close()
    
    def test_illegal_address(self) -> None:
        # Try to read a register at an non-afffected Modbus register's address
        with self.assertRaises(IllegalRequestError):
            self.link.read_register(device_addr = 1, reg_addr = 0xF100) 
        # Try to read out of read only register's range
        # Read only registers are 72 registers from 256
        with self.assertRaises(IllegalRequestError):
            self.link.read_registers(device_addr = 1, reg_addr = 256, nb_reg = 73)
        # Try to write a read only register's address (status)
        with self.assertRaises(IllegalRequestError):
            self.link.write_register(device_addr = 1, reg_addr = 256, value = 0)
    
    def test_illegal_value(self) -> None:
        # Try to write modbus_addr_preset with value > 247
        with self.assertRaises(IllegalRequestError):
            self.link.write_register(device_addr = 1, reg_addr = 48, value = 248)

'''
class Test_fast_mode(unittest.TestCase):
    def setUp(self):
        self.link = modbus.Modbus()
        self.link.open(port = COM_PORT)
        self.link.write_single_coil(device_addr = 1, coil_addr = 0, state = "ON", f_key = 0x5058)
        self.link.write_single_coil(device_addr = 1, coil_addr = 15, state = "ON")
        self.link.baudrate = 460800

    def tearDown(self):
        self.link.write_single_coil(device_addr = 1, coil_addr = 15, state = "OFF")
        self.link.write_single_coil(device_addr = 1, coil_addr = 0, state = "OFF")
        self.link.baudrate = self.link.init_baudrate
        self.link.close()

    def test_fast_mode(self) -> None:
        # Query data in fast mode
        self.assertEqual(self.link.query_data(device_addr = 1, value = 55), 55)
'''

if __name__ == '__main__':
    unittest.main()