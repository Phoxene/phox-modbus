# -*- coding: utf-8 -*-
# Copyright (c) 2023 PHOXENE
# MIT License: 
# https://opensource.org/license/mit/
#
""" Test for modbus module

"""
__authors__ = ("Aurélien PLANTIN")
__contact__ = ("a.plantin@phoxene.com")
__copyright__ = "MIT"
__date__ = "2023-10-10"

import unittest                     # The test framework
import modbus                       # The module to be tested
from modbus import ModbusError
from modbus import IllegalRequestError
from serial import PortNotOpenError

def terminal_output(**kwargs):
    '''This function output everything passed as
        key arguments to the terminal.
    '''
    for k, v in kwargs.items():
        print(f"{k}: {v}")

class Test_crc(unittest.TestCase):
    def test_crc(self) -> None:
        # Simple crc computation result test
        self.assertEqual(modbus._crc16([1, 6, 0, 49, 0, 2]), 50265)

class Test_with_port_not_open(unittest.TestCase):
    def setUp(self):
        self.link = modbus.Modbus()

    def test_port_not_open_error(self) -> None:
        with self.assertRaises(PortNotOpenError):
            self.link.read_registers(device_addr = 1, reg_addr = 0, nb_reg = 1)

class Test_with_port_open(unittest.TestCase):
    def setUp(self):
        self.link = modbus.Modbus()
        self.link.open(port = 'COM3')

    def tearDown(self):
        self.link.close()

    def test_single_write(self) -> None:
        # Read and write reset_on_failure register
        value = self.link.read_register(device_addr = 1, reg_addr = 39)
        self.link.write_register(device_addr = 1, reg_addr = 39, value = value + 879)
        self.assertEqual(self.link.read_register(device_addr = 1, reg_addr = 39), value + 879)

    def test_not_allowed_broadcast(self) -> None:
        with self.assertRaises(ValueError): #Ajouter la vérification du text

            self.link.read_register(device_addr = 0, reg_addr = 0)

    def test_illegal_requests(self) -> None:
        with self.assertRaises(IllegalRequestError):
            self.link.write_register(device_addr = 1, reg_addr = 0xFFFF, value = 2)
        with self.assertRaises(IllegalRequestError):
            # Write modbus_addr_preset with value > 256
            self.link.write_register(device_addr = 1, reg_addr = 48, value = 257)
        ### Miss a test with Illegal function
            ## The fact is illegal functions are detected at client level

class Test_fast_mode(unittest.TestCase):
    pass


if __name__ == '__main__':
    unittest.main()