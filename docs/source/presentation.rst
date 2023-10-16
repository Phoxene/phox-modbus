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
    * Fonction 03 (read holding registers)
    * Fonction 04 (read input registers)
    * Fonction 05 (force single coil)
    * Fonction 06 (preset single resgister)
    * Fonction 08: subfunctions 0 and 11 to 19 (diagnostics)
    * Fonction 11 (get comm event counter)
    * Fonction 12 (get comm event log)
    * Fonction 16 (write_registers)
    * Fonction 43 (Read Device Identification)
* Optional "fast reception mode" that skip receive timeout
  by using frame lenght prediction
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

Developers also may be interested to get the source archive, because it contains examples, tests and the this documentation.
