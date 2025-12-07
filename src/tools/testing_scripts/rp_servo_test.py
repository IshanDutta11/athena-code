import serial
import pyfirmata
import inspect
import pygame
import time
import numpy as np

if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

board = pyfirmata.Arduino('COM5')

# initialize servo
pin = 10
board.digital[pin].mode = pyfirmata.SERVO
zero_vel = 90
board.digital[pin].write(zero_vel)

possible_joysticks = ["sony computer entertainment wireless controller", "thrustmaster t.16000m"]

joysticks = 0
axis_data = None
controller = None
button_data = None
hat_data = None
activation = 0.08
joystick_index = None

# Pygame Controller
pygame.init()
pygame.joystick.init()            
joysticks = pygame.joystick.get_count()

for i in range(joysticks):
    js = pygame.joystick.Joystick(i)
    name = js.get_name().lower()
    if(name == possible_joysticks[joystick_type]):
        joystick_index = i
        break

# Only begin once a joystick is connected
while(joysticks == 0):
    print("No controllers are connected!")
    time.sleep(0.25)
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            print("Joystick connected.")
            pygame.joystick.init()            
            joysticks = pygame.joystick.get_count()
            break
    
controller = pygame.joystick.Joystick(joystick_index)
print(f"Using joystick: {controller.get_name()}")
controller.init()

if not axis_data:
    axis_data = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

if not button_data:
    button_data = {}
    for i in range(controller.get_numbuttons()):
        button_data[i] = False

if not hat_data:
    hat_data = {}
    for i in range(controller.get_numhats()):
        hat_data[i] = (0, 0)

previous_axes = np.zeros(6)
previous_buttons = np.zeros(13)   

def servo_test():
    """
        PS4 Controller:
        Joystick Velocities
        [0] = l/r left joystick
        [1] = u/d left joystick
        [2] = l/r right joystick
        [3] = u/d right joystick
        [4] = left trigger
        [5] = right trigger

        Button Activations
        [0] = x
        [1] = circle
        [2] = triangle
        [3] = square
        [4] = left bumper
        [5] = right bumper
        [6] = left trigger
        [7] = right trigger
        [8] = share
        [9] = options
        [10] = playstation button
        [11] = left joystick button
        [12] = right joystick button
        [13] = up arrow
        [14] = down arrow
        [15] = left arrow
        [16] = right arrow
    """
    while True:
        joystick_vels = previous_axes
        button_activations = previous_buttons
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                axis_data[event.axis] = round(event.value,2)
            elif event.type == pygame.JOYBUTTONDOWN:
                button_data[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                button_data[event.button] = False
            elif event.type == pygame.JOYHATMOTION:
                hat_data[event.hat] = event.value

            if(axis_data.get(0) < -activation):
                joystick_vels[0] = ((axis_data[0] + activation) / (1 - activation))  # normalizes the 0.25 to 1 range to a 0 - 1 range and then mult by max vel
            elif(axis_data.get(0) > activation):
                joystick_vels[0] = ((axis_data[0] - activation) / (1 - activation))
            else:
                joystick_vels[0] = 0

            # Left stick: up and down
            if(axis_data.get(1) < -activation):
                joystick_vels[1] = -((axis_data[1] + activation) / (1 -activation))  # normalizes the 0.25 to 1 range to a 0 - 1 range and then mult by max vel
            elif(axis_data.get(1) >activation):
                joystick_vels[1] = -((axis_data[1] - activation) / (1 -activation)) 
            else:
                joystick_vels[1] = 0

            # Right stick: left and right
            if(axis_data.get(3) < -activation):
                joystick_vels[2] = ((axis_data[3] + activation) / (1 -activation))  # normalizes the 0.25 to 1 range to a 0 - 1 range and then mult by max vel
            elif(axis_data.get(3) >activation):
                joystick_vels[2] = ((axis_data[3] - activation) / (1 -activation)) 
            else:
                joystick_vels[2] = 0

            # Right stick: up and down
            if(axis_data.get(4) < -activation):
                joystick_vels[3] = -((axis_data[4] + activation) / (1 -activation))  # normalizes the 0.25 to 1 range to a 0 - 1 range and then mult by max vel
            elif(axis_data.get(4) > activation):
                joystick_vels[3] = -((axis_data[4] - activation) / (1 -activation)) 
            else:
                joystick_vels[3] = 0

            # Left Trigger
            if(axis_data.get(2) > 0):
                joystick_vels[4] = axis_data[2] 
            else:
                joystick_vels[4] = 0

            # Right Trigger
            if(axis_data.get(5) > 0):
                joystick_vels[5] = axis_data[5]
            else:
                joystick_vels[5] = 0
            
            # Buttons
            for i in range(13):
                button_activations[i] = button_data[i]

            
        board.digital[pin].write(zero_vel + joystick_vels[3]*90)



if __name__ == "__main__":
    init_test()
    servo_test()
    pygame.joystick.quit()
    pygame.quit()