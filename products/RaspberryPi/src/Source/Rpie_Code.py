# Import required libraries
import serial
import time
import pygame
import json
import os
import string

# Set global variable 'running' to True
running = True

# Function to initialize the controller
def Init_Controller():

    # Initialize pygame
    pygame.init()

    # Set global variables 'joysticks', 'button_keys' and 'analog_keys'
    global joysticks
    global button_keys
    global analog_keys

    # Initialize 'joysticks' list with connected joysticks
    joysticks = []
    for i in range(pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))

    # Initialize each joystick in the 'joysticks' list
    for joystick in joysticks:
        joystick.init()

    # Load button keys from 'ps4_keys.json' file
    with open(os.path.join("Parakeet\ps4_keys.json"), 'r+') as file:
        button_keys = json.load(file)

    # Initialize 'analog_keys' list with default values for analog keys
    # 0: Left analog horizonal, 1: Left Analog Vertical, 2: Right Analog Horizontal
    # 3: Right Analog Vertical 4: Left Trigger, 5: Right Trigger
    analog_keys = [0, 0, 0, 0, -1, -1]

# Function to initialize the Arduino
def Init_Arduino():

    # Set global variable 'arduinoData'
    global arduinoData

    # Initialize 'arduinoData' serial connection with specified port and baud rate
    arduinoData= serial.Serial('COM4', 115200, timeout= 2)

    # Wait for 'Start' signal from Arduino to start communication
    while True:
        line = arduinoData.readline().decode().strip()
        if line == "Start":
            break
        else:
            print("Waiting for 'Start'...")

# Function to map a value from one range to another
def map_range(fvalue, in_min, in_max, out_min, out_max):

    # Map 'fvalue' from input range ('in_min' to 'in_max') to output range ('out_min' to 'out_max')
    mapped_value = (fvalue - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # Limit mapped value to output range ('out_min' to 'out_max')
    mapped_value = max(mapped_value, out_min)
    mapped_value = min(mapped_value, out_max)

    # Return the mapped value as an integer
    return int(mapped_value)

# Function to read controller input
def read_controller():

    # Set global variable 'running' to False if 'down_arrow' button is pressed
    global running
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == button_keys['down_arrow']:
                running = False
        
        # Update 'analog_keys' list with new analog input values
        if event.type == pygame.JOYAXISMOTION:
            analog_keys[event.axis] = event.value

# Function to send a package of commands to the Arduino
def send_package(package):

    # Initialize 'answer' list to store responses from Arduino
    answer = []

    # Create a list of command codes from button and analog keys
    code = list(string.ascii_uppercase)[:len(button_keys)+len(analog_keys)]

    # Send each command in the package and append the response to the 'answer' list
    for i in range(len(package)):
        cmd = code[i] + str(package[i]) + '\r'
        arduinoData.write(cmd.encode())
        answer.append(arduinoData.readline().decode().rstrip())
    print(answer)
    


# initialize controller and Arduino
Init_Controller()
Init_Arduino()

# main loop
while running:
    # read input from controller
    read_controller()

    # map analog stick and trigger values to range 1000 - 2000 and send to Arduino
    analog_send = []
    for i in range(len(analog_keys)):
        analog_send.append(map_range(analog_keys[i], -1, 1, 1000, 2000))
        
    send_package(analog_send)