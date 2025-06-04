#!/usr/bin/env python 3
import serial
import time

ser = serial.Serial('COM4', 115200)
# Whenever the serial communication is established, the arduino resets,
# so we are allowing arduino to have 3 seconds to be completely ready
# for serial communication
time.sleep(3)
# At startup we have a fresh buffer with nothing in it.
ser.reset_input_buffer()
print("Serial is okay:)")

try:
    while True:
        time.sleep(1)
        #message = "l:" + str(counter) + ";"
        message = "x;" #Stopping command for the car. 
        ser.write(message.encode('utf-8'))
        print(message)

except KeyboardInterrupt:
    # After we are done with serial communication we'll close it.
    # Because this might cause serial conflict, if another program
    # tries to access this serial port
    print("Serial communication is stopped");
    ser.close()
