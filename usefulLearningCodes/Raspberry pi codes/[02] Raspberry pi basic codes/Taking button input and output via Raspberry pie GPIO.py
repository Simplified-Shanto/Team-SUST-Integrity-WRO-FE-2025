import RPi.GPIO as gpio
import time

button_pin = 26
led_pin = 17

gpio.setmode(gpio.BCM)
gpio.setup(button_pin, gpio.IN)
gpio.setup(led_pin, gpio.OUT)

while True:
    time.sleep(0.01)
    if gpio.input(button_pin):
        gpio.output(led_pin, gpio.HIGH)
    else:
        gpio.output(led_pin, gpio.LOW)
    

gpio.cleanup()
