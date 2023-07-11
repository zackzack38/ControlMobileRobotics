
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

def FD():
    inches = frontDistanceSensor.getValue() * 39.3701
    return inches
def LD():
    return leftDistanceSensor.getValue() * 39.3701
def RD():
    return rightDistanceSensor.getValue() * 39.3701

def getPosIn(obj):
    inches = obj.get_position()
    
    inches[0] = inches[0] * 39.3701
    inches[1] = inches[1] * 39.3701
    inches[2] = inches[2] * 39.3701
    #print(inches)
    return inches


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
    print("Driving")
    while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
        #print("IMU Reading: " + str(imu.getRollPitchYaw()[2]))
        #print("Front DS Reading: " + str(FD()))
        #print("Left DS Reading: " + str(LD()))
        #print("Right DS Reading: " + str(RD()))
        if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.01:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break

def rotate(robot=robot,degree=90):
    
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
    print("rotating " + str(degree))

    while robot.step(timestep) != -1:
        #print("IMU Reading: " + str(imu.getRollPitchYaw()[2]))
        #print("Front DS Reading: " + str(FD()))
        #print("Left DS Reading: " + str(LD()))
        #print("Right DS Reading: " + str(RD()))
        # Checks to see if left wheel traveled past distance of arch lendth D
        if wheel_radius*(abs(leftposition_sensor.getValue() - start_position)) >= D -.01:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break

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

def FD():
    inches = frontDistanceSensor.getValue() * 39.3701
    return inches

def centerOnObject(obj):
    #print("Pos: " + str(obj.get_position_on_image()))
    #xPos = obj.get_position_on_image()[0]
    x = obj.get_position_on_image()
    camW = camera.getWidth()
    halfCamW = round(camW / 2)
    #print(halfCamW)
    #print(x[0])
    if(halfCamW - 1< x[0] < halfCamW + 1):
        stopMotion()
        return 1
    elif(x[0] < halfCamW):
        spinLeft()
        return 0
    elif(x[0] > halfCamW):
        spinRight()
        return 0
    return
    
def setVelocityLR(v):
    leftMotor.setVelocity(v)
    rightMotor.setVelocity(v)
    return v

def stopMotion():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
def spinLeft():
    leftMotor.setVelocity(-0.7) #object to the left, turn left
    rightMotor.setVelocity(0.7)
    
def spinRight():
    leftMotor.setVelocity(0.7) #object to the right, turn right
    rightMotor.setVelocity(-0.7)
    
def wallFollow(wall, targetDistance, Kp_side):
    vf = fsat(3.0 * (FD() - 2)) # speed, can be changed
    if(wall == 'l'):
        Dmin = 2.5
        Dmax = 4
        EL = 0
        if(Dmin < LD() < Dmax):
            EL = 0
        elif(LD() > Dmax):
            EL = LD() - Dmax #too far
            close = 1
        elif(LD() < Dmin):
            EL = LD() - Dmin #too close
            close = 0
        deltaVL = fsat(Kp_side * (targetDistance - EL) * vf)
        #print("DeltaV: " + str(deltaVL))
        #edge cases
        #print("wall follow vf: " +str(vf))
        #print("ER: " +str(EL))
        if(EL == 0):
            print("Forward")
            leftMotor.setVelocity(vf)
            rightMotor.setVelocity(vf)
            return
        elif(close == 1):
            print("too far")
            vl = fsat(vf - abs(EL*Kp_side))
            #print("vl: " + str(vl))
            leftMotor.setVelocity(vl)
            rightMotor.setVelocity(vf)
            return
        elif(close == 0):
            print("too close")
            vl = fsat(vf - abs(EL*Kp_side))
            #print("vl: " + str(vl))
            rightMotor.setVelocity(vl)
            leftMotor.setVelocity(vf)
            return
    elif(wall == 'r'):
        Dmin = 3.5
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
        if(vf < 0.1):
            rotate(robot, -90)
            return
        if(RD() > 15):
            speedD = 3.86
            driveD(robot, 2, 3.86)
            rotate(robot, 90)
            driveD(robot, 3, 3.86)
        if((round(FD()) == targetDistance) and (round(RD()) == targetDistance)):
            rotate(robot, -90)
            return
        if(ER == 0):
            print("Forward")
            leftMotor.setVelocity(vf)
            rightMotor.setVelocity(vf)
            return 1
        elif(close == 1):
            print("too far")
            vr = fsat(vf - abs(ER*Kp_side))
            #print("vr: " + str(vr))
            leftMotor.setVelocity(vf)
            rightMotor.setVelocity(vr)
            return 1
        elif(close == 0):
            print("too close")
            vl = fsat(vf - abs(ER*Kp_side))
            #print("vl: " + str(vl))
            rightMotor.setVelocity(vf)
            leftMotor.setVelocity(vl)
            return 1

def obstacleBlockingGoal(goal):
    if(FD() < 10):
        return 1
    else:
        return 0


def direction_to_goal():
    array = camera.getRecognitionObjects()
    if camera.getRecognitionNumberOfObjects() == 1:
        object = camera.getRecognitionObjects()[0]
        

def bugZero(obj, wall = 'r'):
    getPosIn(obj)
    print(centerOnObject(obj))
    print("FD: " + str(FD()))
    following = centerOnObject(obj) + obstacleBlockingGoal(obj)
    print("f: " +str(following))
    while(following >= 1):
        wallFollow(wall, 6, 0.01)
        #if(getIMUDegrees() == 1):
        #    following = 0
    #1 Head towards goal and stop 5 inches from it
    if(centerOnObject(obj) == 0):
        #object centered drive forward
        centerOnObject(obj)
        return
    else:
        vf = fsat(1.0 * (FD() - 5))
        leftMotor.setVelocity(vf)
        rightMotor.setVelocity(vf)

def obstacle_check(obstacle):
    pos = getPosIn(object)
    if(pos[0] > 10 and FD() < 5):
        #obstacle present return 1
        return 1
    else:
        #no obstacle detected return 0
        return 0

#FLAGS        
flag_follow = 0 #not following
flag_motion_to_goal = 0

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    array = camera.getRecognitionObjects()
    if camera.getRecognitionNumberOfObjects() == 1:
        object = camera.getRecognitionObjects()[0]
        
        
    if(flag_follow == 0 and flag_motion_to_goal == 0):
        #case 0
        print("case 0")
        #not following and not motion to goal, look for goal
        spinRight()
        if camera.getRecognitionNumberOfObjects() == 1:
            flag_motion_to_goal = 1
            stopMotion()
        #centerOnObject returns 0 if not centered, 1 if centered
    elif(flag_follow == 0 and flag_motion_to_goal == 1):
        #case 1 rotate towards goal and center it
        print("case 1")
        if centerOnObject(object) == 1:
            #object is centered go to case 2
            flag_follow = 1
            flag_motion_to_goal = 0
            stopMotion()
    elif(flag_follow == 1 and flag_motion_to_goal == 0):
        #case 2 motion to goal
        print("case 2")
        if (round(fsat(1.0 * (FD() - 4))) == 0 and obstacle_check(object)):
            #robot is stopped from the FD and is not close to goal go to case 3
            flag_follow = 1
            flag_motion_to_goal = 1
            stopMotion()
        if(centerOnObject(object) == 1):
            setVelocityLR(fsat(1.0 * (FD() - 4)))
        if camera.getRecognitionNumberOfObjects() == 0:
            #object lost while going towards it find it
            flag_follow = 0
            flag_motion_to_goal = 0
            stopMotion()
    elif(flag_follow == 1 and flag_motion_to_goal == 1):
        #case 3 wall follow
        print("case 3")
        if camera.getRecognitionNumberOfObjects() == 1 and (obstacle_check(object) == 0) and (RD() > 5):
            print("GO TO CASE 1")
            flag_follow = 0
            flag_motion_to_goal = 1
            stopMotion()
            continue
        wallFollow('r', 6, 0.02)
        
        
# Enter here exit cleanup code.