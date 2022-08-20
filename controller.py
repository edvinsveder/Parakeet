import pygame
import serial
import time

from pygame.locals import *
from pygame import event

# Removes jitter and rounds each axis up to an int
def sanitate_data(analog_keys):

    
    c = 0
    for i in analog_keys.values():
        # Remove jitter
        if i < 0.1 > -0.1:
            analog_keys[c] = 0

        # Keep values within bounds
        if i >= 1: 
            analog_keys[c] = 1
  
        if i <= -1:
            analog_keys[c] = -1

        # Default
        analog_keys[c] *= 1000
        format(analog_keys[c])   
        
        # Increment
        c += 1

    return analog_keys

def controller_handeler():
    # Initialize
    armed = None
    pygame.init()
    joysticks = []
    for i in range(pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))

    # Check if all joysticks are detected, otherwise do not arm

    for joystick in joysticks:
        joystick.init()
        if len(joysticks) <= 4:
            armed = True
        else:
            print("Controller not detected. Quitting!")
            exit()

    analog_keys = {0:0, 1:0, 2:0, 3:0}

    # Read input data
    while armed:
        armed = True

        for event in pygame.event.get(): 
            if event.type == pygame.QUIT:
                armed = False
            if event.type == pygame.KEYDOWN:
                pass

            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value
                
                # Important part! This is forwarding the data to the Arduino
                ard_send(sanitate_data(analog_keys))

def ard_send (sanitated_data):
    
    # Setup Arduino
    arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

    arduino.write(bytes(sanitated_data, 'utf-8'))
    time.sleep(0.05)
    
    data = arduino.readline()
    print(data)


# Main stage
controller_handeler()