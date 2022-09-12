## IMPORTS ##
from doctest import OutputChecker
from unittest import skip
import pygame
from pygame.locals import *

## GLOBALS ##
previous_analog_keys = {0:0, 1:0, 2:0, 3:0} # Persistent data used to compare whether the controller input has changed state

## CONFIG ## 
tolerance = 0.01 # Used to set input tolerance in decimals. Smaller number equals smaller tolerance when removing jitter 

def sanitate_data(analog_keys): # Takes input data as args, validates and sanitates to forward to output
    c = 0 # Loop counter
    
    for i in analog_keys.values(): # Iterate through each axis
        if analog_keys[c] != previous_analog_keys[c]: # Check for change 
          #  if i < 0.1 > -0 : # Remove any jitter based on global 'tolerance'
               # analog_keys[c] = 0
            previous_analog_keys.update({c:translate(analog_keys[c])}) # Translate input data and save change to global

        else: skip # If no change is found
        c += 1 # Always increment each pass

    ard_send(previous_analog_keys) # End of loop - hand over data to output func

def translate(value): # Interpolates input values from 1 -> (-1) to 0 -> 1023
    leftMax = 1
    leftMin = -1
    rightMax = 1023
    rightMin = 0
    
     # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    output = rightMin + (valueScaled * rightSpan)
    return int(output)

def controller_handeler():
    armed = None
    pygame.init() # Initialize pygame lib
    joysticks = [] # List of joysticks
    for i in range(pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))

    for joystick in joysticks: # Check if all 4 joysticks are detected, otherwise do not arm
        joystick.init()
        if len(joysticks) <= 4:
            armed = True
        else:
            print("Input error. Are you connected?")
            exit()

    analog_keys = {0:0, 1:0, 2:0, 3:0}  
    
    while armed: # Read input data
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: # If quitting, unarm and quit 
                armed = False   
                exit()
            if event.type == pygame.KEYDOWN: # Detect pressed buttons. Not in use as of now
                pass

            if event.type == pygame.JOYAXISMOTION: # If pygame detects an input from the controller, forward to 'sanitate_data'
                analog_keys[event.axis] = event.value
                sanitate_data(analog_keys)

def ard_send (sanitated_data):

    # Setup Arduino
    # arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

    #arduino.write(bytes(sanitated_data, 'utf-8'))
    
    #data = arduino.readline()
    print(sanitated_data)


# Main stage
controller_handeler()