#!/usr/bin/env python
'''
RGB Class
Controls an RGB LED 
'''
import RPi.GPIO as GPIO
import time
from enum import Enum

class Color(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3
    WHITE = 4
    BLACK = 5
    PURPLE = 6
    YELLOW = 7

class RGB:
    def __init__(self, rpin=12, gpin=16, bpin=18):
        GPIO.setmode(GPIO.BOARD)  # Board pin-numbering scheme from Raspberry Pi
        # set pin as an output pin with optional initial state of HIGH
        self.rpin = rpin
        self.gpin = gpin
        self.bpin = bpin

        GPIO.setup(self.rpin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.gpin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.bpin, GPIO.OUT, initial=GPIO.LOW)
        self.current_color = Color.BLACK

    def color(self, color):
        # print("Color {}".format(color))
        if color == Color.RED.value:
            # print("red!")
            GPIO.output(self.rpin, GPIO.HIGH)
            GPIO.output(self.gpin, GPIO.LOW)
            GPIO.output(self.bpin, GPIO.LOW)
        if color == Color.BLACK.value:
            GPIO.output(self.rpin, GPIO.LOW)
            GPIO.output(self.gpin, GPIO.LOW)
            GPIO.output(self.bpin, GPIO.LOW)
        if color == Color.GREEN.value:
            GPIO.output(self.rpin, GPIO.LOW)
            GPIO.output(self.gpin, GPIO.HIGH)
            GPIO.output(self.bpin, GPIO.LOW)
        if color == Color.BLUE.value:
            GPIO.output(self.rpin, GPIO.LOW)
            GPIO.output(self.gpin, GPIO.LOW)
            GPIO.output(self.bpin, GPIO.HIGH)
        if color == Color.WHITE.value:
            GPIO.output(self.rpin, GPIO.HIGH)
            GPIO.output(self.gpin, GPIO.HIGH)
            GPIO.output(self.bpin, GPIO.HIGH)
        if color == Color.PURPLE.value:
            GPIO.output(self.rpin, GPIO.HIGH)
            GPIO.output(self.gpin, GPIO.LOW)
            GPIO.output(self.bpin, GPIO.HIGH)
        if color == Color.YELLOW.value:
            GPIO.output(self.rpin, GPIO.LOW)
            GPIO.output(self.gpin, GPIO.HIGH)
            GPIO.output(self.bpin, GPIO.HIGH)
        self.current_color = color

# Pin Definitions
output_pin = 18  # BCM pin 18, BOARD pin 12

def main():
    
    print("Starting RGB demo now! Press CTRL+C to exit")
    rgb = RGB()
    try:
        while True:
            user_option = int(input("Color (RED:1, GREEN:2, BLUE:3, WHITE:4, BLACK:5, PURPLE:6, YELLOW:7) or -1 to leave"))
            rgb.color(user_option)
            # Toggle the output every second
            print("Option {}".format(user_option))
            
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()