
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math


#######################################################
# Creates Robot
#######################################################
robot = Robot()

#Dont read aloud
#I taped thier cabinets closed
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
wheel_radius = 1.6/2
axel_length = 2.28

def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)
    
def getIMUDegrees():
    degrees = imu.getRollPitchYaw()[2] / math.pi * 180
    if(degrees < 0):
        #if negative, converts to positive degrees
        degrees = degrees + 360
    #print("Degrees: " + str(degrees))
    return degrees
def IMU_Plus_L90():
    deg = getIMUDegrees()
    target = 0
    if 0 < deg < 45 or 315 < deg < 360:
        target = 90
    elif 45 < deg < 135:
        target = 180
    elif 135 < deg < 225:
        target = 270
    elif 255 < deg < 315:
        target = 360
    return target


def rotate(robot,degree):
    
    # Determines Rotation and sets proper speeds
    if degree < 0 :
        sign = -1
    else:
        sign = 1
    phi = sign*1

    start_position = leftposition_sensor.getValue()

    leftMotor.setVelocity(phi)
    rightMotor.setVelocity(-phi)

    # Arch length of wheel needs to travel
    D = math.radians(abs(degree))*(axel_length/2)
    
    while robot.step(timestep) != -1:
        print("IMU Reading: " + str(imu.getRollPitchYaw()[2]))
        print("Front DS Reading: " + str(FD()))
        print("Left DS Reading: " + str(LD()))
        print("Right DS Reading: " + str(RD()))
        # Checks to see if left wheel traveled past distance of arch lendth D
        if wheel_radius*(abs(leftposition_sensor.getValue() - start_position)) >= D -.01:
            
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
def driveD(robot,D,V):
    
    start_position = leftposition_sensor.getValue()
    
    # Calculates velocity of each motor and the robot
    phi = V / wheel_radius                # rad/sec

    # Calculates Time need to move a distance D
    T   = D/V               # sec

    # Sets motor speeds and sets start time
    t_start=robot.getTime()
    leftMotor.setVelocity(phi)
    rightMotor.setVelocity(phi)

    while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
        print("IMU Reading: " + str(imu.getRollPitchYaw()[2]))
        print("Front DS Reading: " + str(FD()))
        print("Left DS Reading: " + str(LD()))
        print("Right DS Reading: " + str(RD()))
        if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.01:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break

def FD():
    inches = frontDistanceSensor.getValue() * 39.3701
    return inches
def LD():
    return leftDistanceSensor.getValue() * 39.3701
def RD():
    return rightDistanceSensor.getValue() * 39.3701


def fsat(u):
    offset = 0.0031854 #error was thrown from speed exceeding the max.
    #cmax = leftMotor.getMaxVelocity() - 1
    #cmin = -rightMotor.getMaxVelocity() + 1
    cmax = 3
    cmin = -3
    if(u > cmax):
        ur = cmax
    elif(cmin <= u <= cmax):
        ur = u
    elif(u < cmin):
        ur = cmin
    return ur


# function for setting motor velocities to radians/sec (wheel radius is 0.8 inch)
def setSpeedsIPS(vl,vr):
    leftMotor.setVelocity(vl)#/.8)
    rightMotor.setVelocity(vr)#/.8)
# function for getting distance sensors in inches converted from meters
def getDistanceSensors():
    return [leftDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701]
# wall follow algorithm
def wallFollow(wall, targetDistance, Kp_side):
    vf = fsat(1.0 * (FD() - 2)) # speed, can be changed
    #errorL = targetDistance - LD()
    #errorR = targetDistance - RD()
    #deltaVL = Kp_side * errorL * vf
    #deltaVR = Kp_side * errorR * vf
    #print(vf)
    if(wall == 'l'):
        Dmin = 2.5
        Dmax = 4
        EL = 0
        #Vf = fsat(1.0 * (targetDistance - FD()))
        if(Dmin < LD() < Dmax):
            EL = 0
        elif(LD() > Dmax):
            EL = LD() - Dmax #too far
            close = 1
        elif(LD() < Dmin):
            EL = LD() - Dmin #too close
            close = 0
        deltaVL = fsat(Kp_side * (targetDistance - EL) * vf)
        print("DeltaV: " + str(deltaVL))
        #edge cases
        print("vf: " +str(vf))
        print("ER: " +str(EL))
        if(EL == 0):
            print("Forward")
            leftMotor.setVelocity(vf)
            rightMotor.setVelocity(vf)
            return
        elif(close == 1):
            print("too far")
            vl = fsat(vf - abs(EL*Kp_side))
            print("vl: " + str(vl))
            leftMotor.setVelocity(vl)
            rightMotor.setVelocity(vf)
            return
        elif(close == 0):
            print("too close")
            vl = fsat(vf - abs(EL*Kp_side))
            print("vl: " + str(vl))
            rightMotor.setVelocity(vl)
            leftMotor.setVelocity(vf)
            return
    elif(wall == 'r'):
        Dmin = 2.5
        Dmax = 4
        ER = 0
        #Vf = fsat(1.0 * (targetDistance - FD()))
        if(Dmin < RD() < Dmax):
            ER = 0
        elif(RD() > Dmax):
            ER = RD() - Dmax #too far
            close = 1
        elif(RD() < Dmin):
            ER = RD() - Dmin #too close
            close = 0
        deltaVR = fsat(Kp_side * (targetDistance - ER) * vf)
        print("DeltaV: " + str(deltaVR))
        #edge cases
        print("vf: " +str(vf))
        print("ER: " +str(ER))
        if(vf < 0.1):
            rotate(robot, -90)
            return
        if(RD() > 10):
            speedD = 3.86
            driveD(robot, 4, 3.86)
            rotate(robot, 90)
            driveD(robot, 6, 3.86)
            return
        if(ER == 0):
            print("Forward")
            leftMotor.setVelocity(vf)
            rightMotor.setVelocity(vf)
            return
        elif(close == 1):
            print("too far")
            vr = fsat(vf - abs(ER*Kp_side))
            print("vr: " + str(vr))
            leftMotor.setVelocity(vf)
            rightMotor.setVelocity(vr)
            return
        elif(close == 0):
            print("too close")
            vl = fsat(vf - abs(ER*Kp_side))
            print("vl: " + str(vl))
            rightMotor.setVelocity(vf)
            leftMotor.setVelocity(vl)
            return            
        
        cornerError = 0.5
        
        #if((targetDistance - cornerError < RD() < targetDistance + cornerError) and (targetDistance - cornerError < FD() < targetDistance + cornerError)):
            #corner turn right
            #rotate(robot, -90)
            #print("Corner")
            
                   
    
    
    """
        leftMotor.setVelocity
        if(-0.5 < (targetDistance - LD()) < 0.5):
            #setSpeedsIPS(v - abs(error)*Kp_side, v) # turn away from right wall
            leftMotor.setVelocity(deltaVL)
            rightMotor.setVelocity(deltaVR)
        else:
            #setSpeedsIPS(v,v - abs(error)*Kp_side) # turn towards right wall 
            leftMotor.setVelocity(vf)
            leftMotor.setVelocity(vf)
    elif(wall == 'r'):
        rightMotor.setVelocity(0)
       #if(errorR < 0):
           #setSpeedsIPS(v,v - abs(error)*Kp_side) # turn away from left wall
       #else:
           #setSpeedsIPS(v - abs(error)*Kp_side, vf) # turn towards left wall
    """
    
    
    """
    if(round(v) == 0):
        rotate(robot, getIMUDegrees() - IMU_Plus_L90())
    
    if(RD() > 10):
        speedD = 3.86
        driveD(robot, 4, 3.86)
        rotate(robot, (getIMUDegrees() + IMU_Plus_L90())%360)
        driveD(robot, 20, 3.86)
        rotate(robot, 90)
        driveD(robot, 8, 3.86)
        rotate(robot, (getIMUDegrees() + IMU_Plus_L90())%360)
        driveD(robot, 7, 3.86)
        """

        
        
        #rotate(robot, (getIMUDegrees() + IMU_Plus_L90())%360)
        #imu = getIMUDegrees()
        #target = getIMUDegrees() - 90
    return

# main loop
#while robot.step(timestep) != -1:
"""
 if(leftDist<rightDist):
 wallFollow('l',7,.1)
 else:
 wallFollow('r',7,.1)
 """
# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    Kp = 1.0 #Kp(0.1, 0.5, 1.0, 2.0, 2.5, 5.0)
    r = 3.86 #desired distance to the wall
    y = FD() #frontDistanceSensor.getValue() #distance from robot to the wall
    #e = r - y #distance error
    #u = Kp * e #control signal corresponding to robot velocity
    """
    ur = fsat(Kp * (r - y)) #fsat(u) # = fsat(Kp * e) = fsat(Kp * (r - y))
    if(LD() < RD()):
        wallFollow('l',4,.01)
    else:
        wallFollow('r',4,.01)
    """
    wallFollow('r', r, 0.01)
    
    print("IMU Reading: " + str(imu.getRollPitchYaw()[2]))
    #print("Front DS Reading: " + str(frontDistanceSensor.getValue()))
    #print("UR Speed: " + str(ur))
    print("Front DS Reading: " + str(FD()))
    #print("Left DS Reading: " + str(LD()))
    print("Right DS Reading: " + str(RD()))
# Enter here exit cleanup code.
