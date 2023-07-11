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
x0 = 15
y0 = -15
startCell = [x0,y0]
theta0 = 0
edelt = 0
enew = 0
leftSensorReadingold = 0
startCell = 0
xnew = x0
ynew = y0
leftSensorReadingnew = 0
map = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
leftSensorReadingold = 0

def getIMUDegrees():
    degrees = imu.getRollPitchYaw()[2] / math.pi * 180
    if(degrees < 0):
        #if negative, converts to positive degrees
        degrees = degrees + 360
    #print("Degrees: " + str(degrees))
    return degrees

def getCurrentCell(x, y):
    row = 0
    col = 0
    cellWidth = 10
    cellHeight = 10
    global map

    if(-2*cellWidth <= x < -1*cellWidth):
        row = 1
    elif(-1*cellWidth <= x < 0*cellWidth):
        row = 2
    elif(0*cellWidth <= x < 1*cellWidth):
        row = 3
    elif(1*cellWidth <= x < 2*cellWidth):
        row = 4
    if(-2*cellHeight <= y < -1*cellHeight):
        col = 4
    elif(-1*cellHeight <= y < 0*cellHeight):
        col = 3
    elif(0*cellHeight <= y < 1*cellHeight):
        col = 2
    elif(1*cellHeight <= y < 2*cellHeight):
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
    n = getCurrentCell(x0,y0)
    return x0, y0, n, getIMUDegrees()

def FD():
    inches = frontDistanceSensor.getValue() * 39.3701
    return inches
def LD():
    return leftDistanceSensor.getValue() * 39.3701
def RD():
    return rightDistanceSensor.getValue() * 39.3701


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
    
#[Visited, North, East, South, West]
grid = [[0, 1, 0, 0, 1], [0, 1, 0, 0, 0], [0, 1, 0, 0, 0],[0, 1, 1, 0, 0], 
#starting grid
maze = [[0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0], [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 1, 0, 0], [0, 0, 0, 1, 1], [0, 0, 0, 1, 0], [0, 0, 0, 1, 0],[0, 0, 1, 1, 0]]

def printMaze(maze):
    print("________________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x+1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x+2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x+3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
        print("|  "+ str(maze[x][1]) +"\t  " +str(maze[x+1][1])+"\t  " 
+str(maze[x+2][1])
              +"\t  " +str(maze[x+3][1])+ "    |")
        print("|" +str(maze[x][4]) + " " +v1+" " + str(maze[x][2])+"\t" 
+str(maze[x+1][4])+ " " +v2+" " + str(maze[x+1][2])
              +"\t" +str(maze[x+2][4])+ " " +v3+" " + str(maze[x+2][2])
              +"\t" +str(maze[x+3][4]) + " " +v4+" " + str(maze[x+3][2]) +"  |")
        print("|  "+str(maze[x][3]) +"\t  " +str(maze[x+1][3])+"\t  " 
+str(maze[x+2][3])
              +"\t  " +str(maze[x+3][3])+"    |")
        if(i==3):
            print("|_______________________________________|\n")
        else:
            print("|                                       |")    
    
    
# function for setting motor velocities to radians/sec (wheel radius is 0.8 inch)
def setSpeedsIPS(vl,vr):
    leftMotor.setVelocity(vl)#/.8)
    rightMotor.setVelocity(vr)#/.8)
# function for getting distance sensors in inches converted from meters
def getDistanceSensors():
    return [leftDistanceSensor.getValue()*39.3701, rightDistanceSensor.getValue()*39.3701]
# wall follow algorithm
def wallFollow(wall, targetDistance, Kp_side):
    v = fsat(1.0 * (3.86 - FD())) # speed, can be changed
    if(wall == 'l'):
        error = LD() - targetDistance
        if(error < 0):
            setSpeedsIPS(v - abs(error)*Kp_side, v) # turn away from right wall
        else:
            setSpeedsIPS(v,v - abs(error)*Kp_side) # turn towards right wall 
    elif(wall == 'r'):
       error = RD() - targetDistance
       if(error < 0):
           setSpeedsIPS(v,v - abs(error)*Kp_side) # turn away from left wall
       else:
           setSpeedsIPS(v - abs(error)*Kp_side, v) # turn towards left wall
    if(round(v) == 0):
        rotate(robot, getIMUDegrees() - IMU_Plus_L90())
    if(RD() > 10):
        start_position = leftposition_sensor.getValue()
    
        # Calculates velocity of each motor and the robot
        phi = 3 / wheel_radius                # rad/sec
    
        # Calculates Time need to move a distance D
        T   = 4/3               # sec
    
        # Sets motor speeds and sets start time
        t_start=robot.getTime()
        leftMotor.setVelocity(phi)
        rightMotor.setVelocity(phi)
        while robot.step(timestep) != -1:
        # Checks if wheel distance is larger than D
            updateMap()
            if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= 4-0.01:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
            break
            
        """
        speedD = 3.86
        driveD(robot, 4, 3.86)
        rotate(robot, (getIMUDegrees() + IMU_Plus_L90())%360)
        driveD(robot, 20, 3.86)
        rotate(robot, 90)
        driveD(robot, 8, 3.86)
        rotate(robot, (getIMUDegrees() + IMU_Plus_L90())%360)
        driveD(robot, 7, 3.86)
        """
    return
    
def IMU_Plus_L90():
    deg = getIMUDegrees()
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
        if wheel_radius*abs(leftposition_sensor.getValue() - start_position) >= D-0.01:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break

# Main loop:
# perform simulation steps until Webots is stopping the controller
cords = [0,0,0,0]
while robot.step(timestep) != -1:
    startCell = [15,-15]
    #print(getCurrentCell(startCell[0],startCell[1]))
    cordstemp = cords[2]
    cords = updateMap()
    if cords[2] != cordstemp:
        print(np.matrix(map))
        print("s=" + str(cords))
    #cords = updateMap()
    #cordsx, cordsy, n, theta = updateMap()
    cordstemp = cords[2]
    printmaze(map)
    #print("s=(" + str(cords[0]) + ", " + str(cords[1]) + ", " + str(cords[2]) + ", " + str(getIMUDegrees()) + ")")
    #print(np.matrix(map))
    #print("s=" + str(cords))
    #leftMotor.setVelocity(3)
    #rightMotor.setVelocity(3)
    wallFollow('r',4,.01)

    
    

# Enter here exit cleanup code.
