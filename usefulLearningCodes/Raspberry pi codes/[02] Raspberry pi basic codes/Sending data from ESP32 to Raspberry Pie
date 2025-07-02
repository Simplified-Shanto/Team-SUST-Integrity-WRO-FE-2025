#!/usr/bin/env python 3
import serial
import time

ser = serial.Serial('/dev/ttyACM2', 115200, timeout = 1.0)
#Whenever the serial communication is established, the arduino resets, 
# so we are allowing arduino to have 3 seconds to be completely ready 
# for serial communication
time.sleep(3)
# At startup we have a fresh buffer with nothing in it. 
ser.reset_input_buffer()
print("Serial is okay:)")

try:
	while True:
		time.sleep(0.01)
		if ser.in_waiting > 0:
			line = ser.readline().decode('utf-8')
			print(line)
except KeyboardInterrupt:
	# After we are done with serial communication we'll close it. 
	# Because this might cause serial conflict, if another program
	# tries to access this serial port
	print("Serial communication is stopped"); 
	ser.close()
