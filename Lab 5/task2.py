
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
cameras.append(robot.getDevice('right camera'))
cameras.append(robot.getDevice('rear camera'))
cameras.append(robot.getDevice('left camera'))

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
#fcam = cameras[0]
#rcam = cameras[1]
#bcam = cameras[2]
#lcam = cameras[3]
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

def getPosObject(color):
    if(color == [1.0, 1.0, 0.0]):
        #print("YELLOW")
        return [-20, 20]
    elif(color == [1.0, 0.0, 0.0]):
        #print("RED")
        return [20, 20]
    elif(color == [0.0, 0.0, 1.0]):
        #print("BLUE")
        return [20, -20]
    elif(color == [0.0, 1.0, 0.0]):
        #print("GREEN")
        return [-20, -20]
    return
    
def findPos():
    """
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    x3 = 0
    y3 = 0
    """
    seenY = 0
    seenR = 0
    seenB = 0
    seenG = 0
    r = [0, 0, 0, 0]
    xPos = [0, 0, 0, 0]
    yPos = [0, 0, 0, 0]
    i = 0
    j = 0
    for camera in cameras:
        if camera.getRecognitionNumberOfObjects() >= 1:
            j = 0
            for landmark in camera.getRecognitionObjects():
                colorCheck = landmark.get_colors()
                if seenY == 0 and getPosObject(colorCheck) == [-20, 20]:
                    seenY = 1
                    r[i] = landmark.get_position()[j]
                    xPos[i] = getPosObject(colorCheck)[0]
                    yPos[i] = getPosObject(colorCheck)[1]
                    i = i+1
                    print("Landmark colorY: " + str(landmark.get_colors()))

                elif seenR == 0 and getPosObject(colorCheck) == [20, 20]:
                    seenR = 1
                    r[i] = landmark.get_position()[j]
                    xPos[i] = getPosObject(colorCheck)[0]
                    yPos[i] = getPosObject(colorCheck)[1]
                    i = i+1
                    print("Landmark colorR: " + str(landmark.get_colors()))
                    
                elif seenB == 0 and getPosObject(colorCheck) == [20, -20]:
                    seenB = 1
                    r[i] = landmark.get_position()[j]
                    xPos[i] = getPosObject(colorCheck)[0]
                    yPos[i] = getPosObject(colorCheck)[1]
                    i = i+1
                    print("Landmark colorB: " + str(landmark.get_colors()))

                if seenG == 0 and getPosObject(colorCheck) == [-20, -20]:
                    seenG = 1
                    r[i] = landmark.get_position()[j]
                    xPos[i] = getPosObject(colorCheck)[0]
                    yPos[i] = getPosObject(colorCheck)[1]
                    i = i+1
                    print("Landmark colorG: " + str(landmark.get_colors()))
                j = j + 1
            #xCords[i]  = camera.get_postition[0]
            #r getPos[0]
            #print("Landmark color: " + str(landmark.get_colors()))
    x1 = xPos[0]
    y1 = yPos[0]
    x2 = xPos[1] 
    y2 = yPos[1]
    x3 = xPos[2]
    y3 = yPos[2]
    x4 = xPos[3]
    y4 = yPos[3]
    r1 = r[0] * 39.31 + 3.14
    r2 = r[1] * 39.31 + 3.14
    r3 = r[2] * 39.31 + 3.14
    r4 = r[3] * 39.31 + 3.14
    print("(" + str(x1) + ", " + str(y1) + ", " + str(r1) + ") - (" + str(x2) + ", " + str(y2) + ", " + str(r2) + ") - (" + str(x3) + ", " + str(y3) + ", " + str(r3) + ")")
    A = (-2*x1 + 2*x2)
    B = (-2*y1 + 2*y2)
    C = (r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2)
    D = (-2*x2 + 2*x3)
    E = (-2*y2 + 2*y3)
    F = (r2*r2 - r3*r3 - x2*x2 + x3*x3 - y2*y2 + y3*y3)
    
    x = (C*E - F*B) / (E*A - B*D)
    y = (C*D - A*F) / (B*D - A*E)
    #x = (r1*r1 - r2*r2 + x2*x2) / (2*x2)
    #y = (r1*r1-r3*r3+x3*x3+y3*y3-(2*x3*x))/(2*y3)
    print("(" + str(x) + ", " + str(y) + ")")
    return [x,y]
def cellUp():
    global map
    currentCell = getCurrentCell(findPos()[0],findPos()[1])
    if(currentCell == 1):
        if(map[0][1] == 0):
            return 2
        elif(map[1][0] == 0):
            return 3
        else:
            return 0
    elif(currentCell == 5):
        if(map[0][0] == 0):
            return 1
        elif(map[1][1] == 0):
            return 2
        elif(map[2][0] == 0):
            return 3
        else:
            return 0
    elif(currentCell == 9):
        if(map[1][0] == 0):
            return 1
        elif(map[2][1] == 0):
            return 2
        elif(map[3][0] == 0):
            return 3
        else:
            return 0
    elif(currentCell ==13):
        if(map[2][0] == 0):
            return 1
        elif(map[3][1] == 0):
            return 2
        else:
            return 0
    elif(currentCell ==2):
        if(map[0][2] == 0):
            return 2
        elif(map[1][1] == 0):
            return 3
        elif(map[0][0] == 0):
            return 4
        else:
            return 0
    elif(currentCell ==6):
        if(map[0][1] == 0):
            return 1
        elif(map[1][2] == 0):
            return 2
        elif(map[2][1] == 0):
            return 3
        elif(map[1][0] == 0):
            return 4
        else:
            return 0
    elif(currentCell == 10):
        if(map[1][1] == 0):
            return 1
        elif(map[2][2] == 0):
            return 2
        elif(map[3][1] == 0):
            return 3
        elif(map[2][0] == 0):
            return 4
        else:
            return 0
    elif(currentCell ==14):
        if(map[2][1] == 0):
            return 1
        elif(map[3][2] == 0):
            return 2
        elif(map[3][0] == 0):
            return 4
        else:
            return 0
    elif(currentCell ==3):
        if(map[0][3] == 0):
            return 2
        elif(map[1][2] == 0):
            return 3
        elif(map[0][1] == 0):
            return 4
        else:
            return 0
    elif(currentCell ==7):
        if(map[0][2] == 0):
            return 1
        elif(map[1][3] == 0):
            return 2
        elif(map[2][2] == 0):
            return 3
        elif(map[1][1] == 0):
            return 4
        else:
            return 0
    elif(currentCell ==11):
        if(map[1][2] == 0):
            return 1
        elif(map[2][3] == 0):
            return 2
        elif(map[3][2] == 0):
            return 3
        elif(map[2][1] == 0):
            return 4
        else:
            return 0
    elif(currentCell ==15):
        if(map[2][2] == 0):
            return 1
        elif(map[3][3] == 0):
            return 2
        elif(map[3][1] == 0):
            return 4
        else:
            return 0
        map[3][2] = 1
        return 15
    elif(currentCell ==4):
        if(map[1][3] == 0):
            return 3
        elif(map[0][2] == 0):
            return 4
        else:
            return 0
        map[0][3] = 1
        return 4
    elif(currentCell ==8):
        if(map[0][3] == 0):
            return 1
        elif(map[2][3] == 0):
            return 3
        elif(map[1][2] == 0):
            return 4
        else:
            return 0
        map[1][3] = 1
        return 8
    elif(currentCell ==12):
        if(map[1][3] == 0):
            return 1
        elif(map[3][3] == 0):
            return 3
        elif(map[2][2] == 0):
            return 4
        else:
            return 0
    elif(currentCell ==16):
        if(map[2][3] == 0):
            return 1
        elif(map[3][2] == 0):
            return 4
        else:
            return 0
    return 1

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    for camera in cameras:
        if camera.getRecognitionNumberOfObjects() >= 1:
            for landmark in camera.getRecognitionObjects():
                #print("Landmark color: " + str(landmark.get_colors()))
                #getPosObject(landmark.get_colors())
                temp = 0
    findPos()
    print(np.matrix(map))
    if(cellUp() == 1):
        forward()
    elif(cellUp() == 2):
        turnRight()
        forward()
        turnLeft()
        #turnRight()
    elif(cellUp() == 3):
        turnRight()
        turnRight()
        forward()
        turnLeft()
        turnLeft()
        #turnLeft()
    elif(cellUp() == 4):
        turnLeft()
        forward()
        turnRight()
    else:
        if(np.matrix(map) == [[1,1,1,1],[1,1,1,1],[1,1,1,1],[1,1,1,1]]).all():
            break
        else:
            break
            print("???")
            turnRight()

# Enter here exit cleanup code.
