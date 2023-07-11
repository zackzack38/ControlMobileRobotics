"""lab2_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#getting the position sensors

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

wheelRadius = 0.8
RNum = 1
DNum = 1
PNum = 1
C = np.matrix([[0], [0], [math.pi/2]])
np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
def stopMotionT(T):
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    startTime = robot.getTime()
    while(robot.getTime() - startTime < T):
        robot.step(timestep)

#IMU Functions
def getIMUDegrees():
    degrees = imu.getRollPitchYaw()[2] / math.pi * 180
    if(degrees < 0):
        #if negative, converts to positive degrees
        degrees = degrees + 360
    #print("Degrees: " + str(degrees))
    return degrees

def forwardKinematics(P, VL, VR, T):
    startTime = robot.getTime()
    leftVel = VL
    rightVel = VR
    radiusTotal = 0
    distanceTotal = 0
    global DNum
    global RNum
    global PNum
    global C
    if(leftVel > 6.28 or rightVel > 6.28 or leftVel < -6.28 or rightVel < -6.28):
        print("Velocity entered is to fast")
        quit()
        return
    leftMotor.setVelocity(leftVel)
    rightMotor.setVelocity(rightVel)
    if(leftVel == rightVel):
        #straight line so D
        distanceTotal = leftVel * wheelRadius * T
        leftMotor.setVelocity(VL)
        rightMotor.setVelocity(VR)
    else:
        #curved line so R
        if(VL == -VR):
            radiusTotal = 0
            leftMotor.setVelocity(VL)
            rightMotor.setVelocity(VR)
        else:
            L = 2.28 #from lab 1
            radiusTotal = L/2*(VR+VL)/(VL-VR)
    thetaOne = getIMUDegrees()
    while(not(T - 0.032 < robot.getTime() - startTime < T + 0.032)):
        robot.step(timestep)
    thetaTwo = getIMUDegrees()
    theta = thetaTwo - thetaOne
    #print("T: " + str(theta) + " 1: " + str(thetaOne) + " 2: " + str(thetaTwo))
    #print("theta: " + str(theta))
    if(not(-0.5 < theta < 0.5)):
        #curve
        VelLeft = VL * 0.8
        VelRight = VR * 0.8
        W = (VelLeft - VelRight)/2.28
        R = np.abs(1.14*(VelRight+VelLeft)/(VelLeft-VelRight))
        V = (VelLeft+VelRight)/2
        S = V*T
        AngleTheta = S / R
        X = C[0][0]
        Y = C[1][0]
        ICCX = X - R * math.sin(AngleTheta)
        ICCY = Y + R * math.cos(AngleTheta)
        CF = np.matrix([[math.cos(W*T), -math.sin(W*T), 0], [math.sin(W*T), math.cos(W*T), 0], [0, 0 , 1]]) * np.matrix([[X-ICCX], [Y-ICCY], [AngleTheta]]) + np.matrix([[ICCX], [ICCY], [W*T]])
        
        
        """
        ICCX = C[0][0] - R * math.sin(Theta)
        ICCX = C[0][0]-RC*math.sin(imu.getRollPitchYaw()[2])
        ICCY = C[1][0]-RC*math.cos(imu.getRollPitchYaw()[2])
        M1 = np.matrix([[math.cos(WT), -math.sin(WT), 0], [math.sin(WT), math.cos(WT), 0], [0, 0, 1]])
        M2 = np.matrix([[-RC*math.sin(WT)],[-RC*math.cos(WT)],[math.radians(C[2][0])]])
        M3 = np.matrix([[ICCX],[ICCY],[WT]])
        M4 = M1 @ M2 + M3
        C[0][0] = M4[0][0]
        C[1][0] = M4[1][0]
        C[2][0] = M4[2][0]
        
        
        #this is dumb but it works
        xCheck = 0
        yCheck = 0
        if(-math.pi/2 < imu.getRollPitchYaw()[2] < math.pi/2):
            yCheck = 1
        else:
            yCheck = -1
        if(0 < imu.getRollPitchYaw()[2] < -math.pi):
            xCheck = 1
        else:
            xCheck = -1
        C[1] += radiusTotal*(1-math.cos(math.radians(theta)))*xCheck
        C[0] += radiusTotal*(math.sin(math.radians(theta)))*yCheck
        print("R" + str(RNum) + ": " + str(abs(radiusTotal)))
        RNum = RNum + 1
        """
        print("R" + str(RNum) + ": " + str(abs(radiusTotal)))
        RNum = RNum + 1
    else:
        X = C[0][0]
        Y = C[1][0]
        AngleTheta = C[2][0]
        V = VR * 0.8
        CF = [[X+V*math.cos(AngleTheta)*T], [Y+V*math.sin(AngleTheta)*T], [AngleTheta]]
        """
        xCheck = 0
        yCheck = 0
        if(-math.pi/2 < imu.getRollPitchYaw()[2] +math.pi/2 < math.pi/2):
            yCheck = 1
        else:
            yCheck = -1
        if(0 < imu.getRollPitchYaw()[2] +math.pi/2 < math.pi):
            xCheck = 1
        else:
            xCheck = -1
        C[0] += distanceTotal*(math.cos(math.radians(getIMUDegrees()+90)))*xCheck
        C[1] += distanceTotal*(math.sin(math.radians(getIMUDegrees()+90)))*yCheck
        """
        print("D" + str(DNum) + ": " + str(distanceTotal))
        DNum = DNum + 1

    #C[2] = getIMUDegrees()
    print("P" + str(PNum) + ": (" + str(CF[0][0]) + ", " + str(CF[1][0]) + ", " + str(CF[2][0]) + ")")
    PNum = PNum + 1
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    return

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # print simulation time in sec (timesteps are in msec)
    print("Time: " +str(robot.getTime()))
    
    # Process sensor data here.   
    #print("Left position sensor: " +str(leftposition_sensor.getValue()))
    #print("Right position sensor: " +str(rightposition_sensor.getValue()))

    # Process sensor data here.
    #print("IMU: "+str(imu.getRollPitchYaw()[2]))
    #getIMUDegrees()
    
    #forwardKinematics(P, VL, VR, T)
    #C = (0,0,0)
    #VL = [5, 4, 5, 4]
    #VR = [3.143, 4, 3.143, 4]
    #T = [1.929, 3, 3.857, 3]
    forwardKinematics(C, 5, 3.143, 1.929)
    #print("P1: (" + str(C[0]) + ", " + str(C[1]) + ", " + str(C[2]) + ")")
    forwardKinematics(C, 4, 4, 3)
    #print("P2: (" + str(C[0]) + ", " + str(C[1]) + ", " + str(C[2]) + ")")
    forwardKinematics(C, 5, 3.143, 3.857)
    #print("P3: (" + str(C[0]) + ", " + str(C[1]) + ", " + str(C[2]) + ")")
    forwardKinematics(C, 4, 4, 3)
    break
    #stopMotionT(250)
    
    #print("P4: (" + str(C[0]) + ", " + str(C[1]) + ", " + str(C[2]) + ")")
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #leftMotor.setVelocity(-1)
    #rightMotor.setVelocity(1)
    #rightMotor.setVelocity(3.143)
    
    pass

# Enter here exit cleanup code.
