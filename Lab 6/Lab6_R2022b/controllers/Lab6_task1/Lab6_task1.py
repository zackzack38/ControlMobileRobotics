from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math

# Robot Deminsions in inch
wheel_radius = 1.6/2
axel_length = 2.28


#######################################################
# Creates Robot
#######################################################
robot = Robot()


#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())

#######################################################
# Gets Robots Distance Sensors
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

#######################################################
# Gets Robots Cameras 
# (4 cameras facing front, right, rear, and left)
#######################################################
cameras = []
cameras.append(robot.getDevice('front camera'))
# cameras.append(robot.getDevice('right camera'))
# cameras.append(robot.getDevice('rear camera'))
# cameras.append(robot.getDevice('left camera'))

# Enables all cameras
for camera in cameras:
    camera.enable(timestep)
    camera.recognitionEnable(timestep)

#######################################################
# cameras array map
#   [0] -> front
#   [1] -> right
#   [2] -> rear
#   [3] -> left
#######################################################
# default referance to front camera
camera = cameras[0]

#######################################################
# Gets Robots Motors
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)


#######################################################
# Gets Robot's the position sensors
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

map = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

def getCurrentCell(x, y):
    row = 0
    col = 0
    global map

    if(-20 <= x < -10):
        row = 1
    elif(-10 <= x < 0):
        row = 2
    elif(0 <= x < 10):
        row = 3
    elif(10 <= x < 20):
        row = 4
    if(-20 <= y < -10):
        col = 4
    elif(-10 <= y < 0):
        col = 3
    elif(0 <= y < 10):
        col = 2
    elif(10 <= y < 20):
        col = 1
    if(row == 1 and col == 1):
        map[0][0] = 1
        return 1
    elif(row == 1 and col == 2):
        map[1][0] = 1
        return 5
    elif(row == 1 and col == 3):
        map[2][0] = 1
        return 9
    elif(row == 1 and col == 4):
        map[3][0] = 1
        return 13
    elif(row == 2 and col == 1):
        map[0][1] = 1
        return 2
    elif(row == 2 and col == 2):
        map[1][1] = 1
        return 6
    elif(row == 2 and col == 3):
        map[2][1] = 1
        return 10
    elif(row == 2 and col == 4):
        map[3][1] = 1
        return 14
    elif(row == 3 and col == 1):
        map[0][2] = 1
        return 3
    elif(row == 3 and col == 2):
        map[1][2] = 1
        return 7
    elif(row == 3 and col == 3):
        map[2][2] = 1
        return 11
    elif(row == 3 and col == 4):
        map[3][2] = 1
        return 15
    elif(row == 4 and col == 1):
        map[0][3] = 1
        return 4
    elif(row == 4 and col == 2):
        map[1][3] = 1
        return 8
    elif(row == 4 and col == 3):
        map[2][3] = 1
        return 12
    elif(row == 4 and col == 4):
        map[3][3] = 1
        return 16
    #if it reaches here its out of bounds
    return 0
# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    startCell = [15,-15]
    print(getCurrentCell(startCell[0],startCell[1]))
    leftMotor.setVelocity(1)
    rightMotor.setVelocity(1)
    

    
    

# Enter here exit cleanup code.
