## [1.2.0] - 2025-09-11
- Add a max_serial_latency parameter to the Modbus class. This parameter is intented to increase Modbus T1.5 and T3.5 parameters
-  when the client Modbus client serial latency can not comply with Modbus standard
- Add restart communication support (Modbus function 08 sub-function 01)
- Add return diag register support (Modbus function 08 sub-function 02)
- Add force listen only support (Modbus function 08 sub-function 04)
- Add clear overrun support (Modbus function 08 sub-function 20)

## [1.1.0]
- Fix a bug affecting query data diagnostic command
- Add "bad_parity" and "serial_overflow" hacks

## [1.0.1]
- Fix a bug where two consecutive request lead to reception error on a non-addressed device because the T3.5 modbus timing was not respected.
- Add T1.5 and T3.5 modbus timeouts updated accordingly to baudrate (1.5 x 11 x bit duration; 3.5 x 11 x bit duration). For baudrate > 19200bps, minimum T1.5 = 750µs and T3.5 = 1750µs are not respected in order not to slow down the communication at high baudrates


