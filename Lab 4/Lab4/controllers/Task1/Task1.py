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
# Gets Robots Camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

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


def fsat(u):
    offset = 0.0031854 #error was thrown from speed exceeding the max.
    cmax = leftMotor.getMaxVelocity()
    cmin = -rightMotor.getMaxVelocity()
    if(u > cmax):
        ur = cmax
    elif(cmin <= u <= cmax):
        ur = u
    elif(u < cmin):
        ur = cmin
    return -ur

def FD():
    inches = frontDistanceSensor.getValue() * 39.3701
    return inches

def centerOnObject(obj):
    #print("Pos: " + str(obj.get_position_on_image()))
    #xPos = obj.get_position_on_image()[0]
    x = obj.get_position_on_image()
    camW = camera.getWidth()
    halfCamW = camW / 2
    print(x)
    if(x[0] == halfCamW):
        v = fsat(1.0 * (5 - FD())) #centered on object, go forward
        leftMotor.setVelocity(v)
        rightMotor.setVelocity(v)
    elif(x[0] < halfCamW):
        spinLeft()
    elif(x[0] > halfCamW):
        spinRight()
    return
def spinLeft():
    leftMotor.setVelocity(-1) #object to the left, turn left
    rightMotor.setVelocity(1)
    return
def spinRight():
    leftMotor.setVelocity(1) #object to the right, turn right
    rightMotor.setVelocity(-1)
    return
# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    leftMotor.setVelocity(1)
    rightMotor.setVelocity(-1)
    array = camera.getRecognitionObjects()
    if camera.getRecognitionNumberOfObjects() == 1:
        object = camera.getRecognitionObjects()[0]
        if(object.get_colors() == [1.0, 1.0, 0.0]):
            centerOnObject(object)
# Enter here exit cleanup code.
