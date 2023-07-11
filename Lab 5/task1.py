
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
wheel_radius = 1.6/2
axel_length = 2.28
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

x0 = 15
y0 = -15
theta0 = 0
edelt = 0
enew = 0
leftSensorReadingold = 0
startCell = 0
xnew = x0
ynew = y0
leftSensorReadingnew = 0
map = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
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
    if 0 <= deg < 45 or 315 < deg < 360:
        target = 90
    elif 45 < deg < 135:
        target = 180
    elif 135 < deg < 225:
        target = 270
    elif 255 < deg < 315:
        target = 360
    return target


def rotate(robot,degree):
    global leftSensorReadingold
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
        #print("IMU Reading: " + str(imu.getRollPitchYaw()[2]))
        #print("Front DS Reading: " + str(FD()))
        #print("Left DS Reading: " + str(LD()))
        #print("Right DS Reading: " + str(RD()))
        # Checks to see if left wheel traveled past distance of arch lendth D
        if wheel_radius*(abs(leftposition_sensor.getValue() - start_position)) >= D -.01:
            
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            leftSensorReadingold = leftposition_sensor.getValue()

            break

def driveD(robot = robot,D = 10,V = 3.14):
    global x0
    global y0
    global leftSensorReadingold
    global xnew
    global ynew
    global theta0
    leftSensorReadingnew = 0
    #print("Robot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
    start_position = leftposition_sensor.getValue()
    
    # Calculates velocity of each motor and the robot
    phi = V / wheel_radius                # rad/sec

    # Calculates Time need to move a distance D
    T   = D/V               # sec

    # Sets motor speeds and sets start time
    t_start=robot.getTime()
    leftMotor.setVelocity(phi)
    rightMotor.setVelocity(phi)
    
    
    #print("CRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))

    while robot.step(timestep) != -1:
        """
        leftSensorReadingnew = leftposition_sensor.getValue()
        
        Dist = wheel_radius * (leftSensorReadingnew - leftSensorReadingold) 
        leftSensorReadingold = leftSensorReadingnew    
        #update position
        print("DRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
        xnew = x0 + Dist*math.cos(imu_cleaner(imu.getRollPitchYaw()[2]))
        ynew = y0 + Dist*math.sin(imu_cleaner(imu.getRollPitchYaw()[2]))
        x0 = xnew
        y0 = ynew
        print("ERobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
        getCurrentCell(x0, y0)
        """
        cords = updateMap()
        print(np.matrix(map))
        print("Cords: (" + str(cords[0]) + ", " + str(cords[1]) +")" + " Cell: " + str(getCurrentCell(cords[0],cords[1])))
        if (wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.01):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break


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
    
def forward():
    driveD(robot,10,5)
def turnLeft():
    rotate(robot,-90)    
def turnRight():
    rotate(robot,90)
    
def updateMap():
    global x0
    global y0
    global leftSensorReadingold
    global xnew
    global ynew
    global theta0
    global leftSensorReadingnew
    
    leftSensorReadingnew = leftposition_sensor.getValue()
    D = wheel_radius * (leftSensorReadingnew - leftSensorReadingold) 
    leftSensorReadingold = leftSensorReadingnew    
    #update position
    #print("Robot at (" + str(x0) + ", " + str(y0) + ")" + " D: " + str(D))
    xnew = x0 + D*math.cos(imu.getRollPitchYaw()[2])
    ynew = y0 + D*math.sin(imu.getRollPitchYaw()[2])
    x0 = xnew
    y0 = ynew
    #print("Robot at (" + str(x0) + ", " + str(y0) + ")")
    
    return [x0, y0]
    
def goPath(startCell):
    global x0
    global y0
    global leftSensorReadingold
    global xnew
    global ynew
    global theta0
    """
    leftSensorReadingnew = leftposition_sensor.getValue()
    leftSensorReadingold = leftSensorReadingnew    

    rightSensorReading = rightposition_sensor.getValue()
    D = wheel_radius * (leftSensorReadingnew - leftSensorReadingold) 
    leftSensorReadingold = leftSensorReadingnew    
    #update position
    print("Robot at (" + str(x0) + ", " + str(y0) + ")" + " D: " + str(D))
    xnew = x0 + D*math.cos(imu.getRollPitchYaw()[2])
    ynew = y0 + D*math.sin(imu.getRollPitchYaw()[2])
    x0 = xnew
    y0 = ynew
    print("Robot at (" + str(x0) + ", " + str(y0) + ")")
    """
    leftSensorReadingold = leftposition_sensor.getValue()
    updateMap()
    getCurrentCell(x0, y0)
    if startCell == 16:
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
    elif startCell == 15:
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
    elif startCell == 14:
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        forward()
        turnLeft()
        forward()
        return
    elif startCell == 13:
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        return
    elif startCell == 12:
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        return
    elif startCell == 11:
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        return
    elif startCell == 10:
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        return
    elif startCell == 9:
        forward()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        return
    elif startCell == 8:
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        return
    elif startCell == 7:
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        turnRight()
        forward()
        return
    elif startCell == 6:
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        forward()
        turnRight()
        forward()
        return
    elif startCell == 5:
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        return
    elif startCell == 4:
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        return
    elif startCell == 3:
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        forward()
        forward()
        turnRight()
        forward()
        turnRight()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        return
    elif startCell == 2:
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        turnRight()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        return
    elif startCell == 1:
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        turnLeft()
        forward()
        forward()
        turnLeft()
        forward()
        turnLeft()
        forward()
        return
    return
# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    #Get sensor readings
    """
    imuReading = getIMUDegrees()
    leftSensorReadingnew = leftposition_sensor.getValue()
    rightSensorReading = rightposition_sensor.getValue()
    
    D = wheel_radius * (leftSensorReadingnew - leftSensorReadingold) 
    leftSensorReadingold = leftSensorReadingnew    
    #update position
    print("Robot at (" + str(x0) + ", " + str(y0) + ")")
    xnew = x0 + D*math.cos(imu.getRollPitchYaw()[2])
    ynew = y0 + D*math.sin(imu.getRollPitchYaw()[2])
    x0 = xnew
    y0 = ynew
    theta0 = imuReading
    getCurrentCell(x0, y0)
    
    """
    
    if(startCell == 0):
        startCell = getCurrentCell(x0,y0)
    elif(startCell == 16):
        leftMotor.setVelocity(0.7)
        rightMotor.setVelocity(-0.7)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 15):
        leftMotor.setVelocity(0.7)
        rightMotor.setVelocity(-0.7)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 0:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 14):
        leftMotor.setVelocity(0.7)
        rightMotor.setVelocity(-0.7)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 0:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 13):
        leftMotor.setVelocity(0.7)
        rightMotor.setVelocity(-0.7)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 0:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 12):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 11):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 10):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 9):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 8):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 7):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 0:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 6):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        turnRight()
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 5):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 4):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 3):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 90:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 2):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 180:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 1):
        leftMotor.setVelocity(0.5)
        rightMotor.setVelocity(-0.5)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 180:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
        turnLeft()
        leftSensorOld = leftposition_sensor.getValue()
        goPath(startCell)
        break
    elif(startCell == 0):
        print("Robot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
        #check rotation
        leftMotor.setVelocity(0.7)
        rightMotor.setVelocity(-0.7)
        while robot.step(timestep) != -1:
            print("BRobot at (" + str(x0) + ", " + str(y0) + ") Cell: " + str(getCurrentCell(x0,y0)))
            if round(getIMUDegrees()) == 0 or round(getIMUDegrees()) == 360:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                break
    if camera.hasRecognition():
        for landmark in camera.getRecognitionObjects():
            print("Landmark color: " + str(landmark.get_colors()))
    

# Enter here exit cleanup code.
