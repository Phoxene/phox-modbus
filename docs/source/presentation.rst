===========
phox-modbus
===========

Overview
========

This module is a Python Modbus serial RTU driver

It has been developped as Modbus layer for PHOXENE's devices that
implements a serial Modbus communication.

It is realeased under a free software licence,
see the LICENSE file for more details

MIT License Copyright (c) 2023 PHOXENE


Features
========
* Implemented Modbus functions:
    * Function 03 - Read holding registers
    * Function 04 - Read input registers
    * Function 05 - Force single coil
    * Function 06 - Preset single register
    * Function 08 - Subfunctions 0 and 11 to 19 - Diagnostics
    * Function 11 - Get comm event counter
    * Function 12 - Get comm event log
    * Function 16 - Write multiple registers
    * Function 17 - Report server ID
    * Function 43 - MEI 14 - Read device identification
* Use fast reception mode that is not legacy (skip receive timeout
  by using frame lenght prediction). A legacy mode is available.
* Hack tools allows to test modbus server response to corrupted frames
* Optional feeeback of sent and received frames as well as Modbus events.
  Main usage is debbug.
* The files in this package are 100% pure Python.

Requirements
============
* Pyhton 3.7 or newer
* Windows 7 or newer
* Debian 10 or newer

Installation
============
phox-modbus can be installed from PyPI:

.. code-block:: console

    pip install phox-modbus

Developers also may be interested to get the source archive, because it contains examples, tests and this documentation.
