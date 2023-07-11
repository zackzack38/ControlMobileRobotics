
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math


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

def FD():
    inches = frontDistanceSensor.getValue() * 39.37
    return inches
def LD():
    return leftDistanceSensor.getValue() * 39.37
def RD():
    return rightDistanceSensor.getValue() * 39.37


def fsat(u):
    offset = 0.0031854 #error was thrown from speed exceeding the max.
    cmax = 2 * math.pi - offset
    cmin = -2 * math.pi + offset
    if(u > cmax):
        ur = cmax
    elif(cmin <= u <= cmax):
        ur = u
    elif(u < cmin):
        ur = cmin
    return -ur
    

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    Kp = 1.0 #Kp(0.1, 0.5, 1.0, 2.0, 2.5, 5.0)
    r = 10 #desired distance to the wall
    y = FD() #frontDistanceSensor.getValue() #distance from robot to the wall
    #e = r - y #distance error
    #u = Kp * e #control signal corresponding to robot velocity
    ur = fsat(Kp * (r - y)) #fsat(u) # = fsat(Kp * e) = fsat(Kp * (r - y))
    
    if((3 <= LD() <= 7) and (3 <= RD() <= 7)):
        leftMotor.setVelocity(ur)
        rightMotor.setVelocity(ur)
    else:
        if (LD() < RD()):
            #Left wall close
            rightMotor.setVelocity(fsat(Kp * 3.5 - LD()))
            leftMotor.setVelocity(fsat(Kp * 3.5 - LD()))
        else:
            #Right wall close
            rightMotor.setVelocity(fsat(Kp * 3.5 - RD()))
            leftMotor.setVelocity(fsat(Kp * 3.5 - RD()))
    #Kp = #proportional gain or correction error gain
    
    
    
    #leftMotor.setVelocity(1)
    #rightMotor.setVelocity(1)
    print("IMU Reading: " + str(imu.getRollPitchYaw()[2]))
    #print("Front DS Reading: " + str(frontDistanceSensor.getValue()))
    print("Front DS Reading: " + str(FD()))
    print("Left DS Reading: " + str(LD()))
    print("Right DS Reading: " + str(RD()))
# Enter here exit cleanup code.
