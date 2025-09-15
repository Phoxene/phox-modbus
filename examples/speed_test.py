    
#!/usr/bin/env python3
"""
Test the Modbus link speed (mainly the client side serial communication)
"""
import time
import utils.select_com_port as select_com_port
from phox_modbus import modbus

MODBUS_ADDR = 1  # Modbus device address

def terminal_output(**kwargs):
    '''This function output everything passed as
        key arguments to the terminal.
    '''
    for k, v in kwargs.items():
        print(f"{k}: {v}")

def speed_test():
    NB_REQUEST = 100
    start_time = time.time()
    for i in range (NB_REQUEST):
        try:
            link.read_register(device_addr = 1, reg_addr = 1)
        except IOError as exc:
            print(f'Modbus error: {exc}')
    exec_time = time.time() - start_time
    # 2.4ms where lost while implementing timeout parameter
    print(f"Fast mode execution time: {(1000 * exec_time / NB_REQUEST):.2f}ms / request (reference is 15ms)")

    start_time = time.time()
    for i in range (NB_REQUEST):
        try:
            link.read_register(device_addr = 1, reg_addr = 1, mode = "legacy")
        except IOError as exc:
            print(f'Modbus error: {exc}')
            raise modbus.ModbusError
    exec_time = time.time() - start_time
    print(f"Legacy mode execution time: {(1000 * exec_time / NB_REQUEST):.2f}ms / request (reference is 15ms)")
 
if __name__ == "__main__":
    com_port = select_com_port.select_com_port()
    # Create a modbus link with defaut parameters (Modbus class instantiation)
    link = modbus.Modbus()
    # Open the modbus link on the selected port
    link.open(port = com_port)        
    # Register terminal_output function as handler for the feeback from modbus class
    # All feeback informations from modbus class will then be redirected to the terminal
    #link.register_feedback_handler(terminal_output)
    speed_test()
    link.close()                    # Close the modbus link