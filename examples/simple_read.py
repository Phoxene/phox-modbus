    
#!/usr/bin/env python3
"""
Example of using the phox_modbus package to read some registers in a Modbus device.
"""
import utils.select_com_port as select_com_port
from phox_modbus import modbus

MODBUS_ADDR = 1  # Modbus device address

def terminal_output(**kwargs):
    '''This function output everything passed as
        key arguments to the terminal.
    '''
    for k, v in kwargs.items():
        print(f"{k}: {v}")

def read_registers():
    # Read the 1st four registers from the modbus device with slave address = 1
    try: 
        data = link.read_registers(device_addr = MODBUS_ADDR, reg_addr = 0, nb_reg = 4) 
    except modbus.ModbusError as exc:
        print(f"Modbus error: {exc}")
    else:
        for i, value in enumerate(data):
            print(f'Register {i} = {data[i]}')  # output values on the terminal
 
if __name__ == "__main__":
    com_port = select_com_port.select_com_port()
    # Create a modbus link with defaut parameters (Modbus class instantiation)
    link = modbus.Modbus()
    # Open the modbus link on the selected port
    link.open(port = com_port)        
    # Register terminal_output function as handler for the feeback from modbus class
    # All feeback informations from modbus class will then be redirected to the terminal
    link.register_feedback_handler(terminal_output)
    read_registers()
    link.close()                    # Close the modbus link