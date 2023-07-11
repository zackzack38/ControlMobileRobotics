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

def stopMotionT(T):
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    startTime = robot.getTime()
    while(robot.getTime() - startTime < T):
        robot.step(timestep)
        
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

def inverseKinematics(Cx, Cy, Ct, Fx, Fy, Ft, V):
    if(V > 6.28 or V > 6.28 or V < -6.28 or V < -6.28):
        print("Velocity entered is to fast")
        quit()
        return
    #force theta 1 to be 90
    thetas = math.degrees(Ct) #starting angle
    theta1 = 90
    thetaf = math.degrees(Ft) #ending angle
    print("tf" + str(thetaf))
    #maths - see paper notes for solving
    theta2 = thetaf - theta1 - thetas
    D2 = (math.cos(math.radians(thetas + theta1)) * (Fy - Cx) / math.sin(math.radians(thetas + theta1)) - Fx + Cx) / (math.sin(math.radians(thetas + theta1 + theta2)) * math.cos(math.radians(thetas + theta1)) / math.sin(math.radians(thetas + theta1)) - math.cos(math.radians(theta1 + thetas + theta2)))
    D1 = -(Fx - Cx - D2 * math.cos(math.radians(thetas + theta1 + theta2))) / math.cos(math.radians(thetas + theta1))
    if(theta2 <= 0):
        theta2 = theta2 + 360
    
    
    print("D1: " + str(D1) + " D2: " + str(D2) + " Ts: " + str(thetas) + " Tf: " + str(thetaf))
    print("T1: " + str(theta1) + " T2: " + str(theta2))
    
    #R1
    leftMotor.setVelocity(1)
    rightMotor.setVelocity(-1)
    while robot.step(timestep) != 1:
        if imu_cleaner(imu.getRollPitchYaw()[2]) >= theta1-0.02:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    print("R1: 0")
    print("P1: (0, 0, " + str(imu_cleaner(imu.getRollPitchYaw()[2])) + ")")
    
    #D1
    leftMotor.setVelocity(V)
    rightMotor.setVelocity(V)
    start_position = leftposition_sensor.getValue()
    while robot.step(timestep) != 1:
        if 0.8 * abs(leftposition_sensor.getValue() - start_position) >= D1-0.01:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    print("D1: " + str(D1))
    x_pos_d1 = D1
    y_pos_d1 = 0 #because this is hardcoded to start at a 90 deg turn and goes straight on the x axis
    print("P2: (" + str(x_pos_d1) + ", " + str(y_pos_d1) + ", " + str(imu_cleaner(imu.getRollPitchYaw()[2])) + ")")
    
    #R2
    if(thetaf <= 0):
        thetaf = thetaf + 360
    leftMotor.setVelocity(0.5)
    rightMotor.setVelocity(-0.5)
    """
    while robot.step(timestep) != 1:
        if 0 <= imu_cleaner(imu.getRollPitchYaw()[2]) <= 3:
            break
    """
    while robot.step(timestep) != 1:
        if imu_cleaner(imu.getRollPitchYaw()[2]) >= thetaf-0.12 or imu_cleaner(imu.getRollPitchYaw()[2])  <= thetaf+0.12:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    print("R2: 0")
    print("P3: (" + str(x_pos_d1) + ", " + str(y_pos_d1) + ", " + str(imu_cleaner(imu.getRollPitchYaw()[2])) + ")")
    
    #D2
    if (V != 0):
        S = D2 / V
    else:
        # velocity of 0 can never move to X distance
        return
    T1 = robot.getTime()
    while (robot.getTime() - T1) <= S:
        robot.step(timestep)
        leftMotor.setVelocity(V)
        rightMotor.setVelocity(V)
    
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    """
    start_pos = leftposition_sensor.getValue()
    while robot.step(timestep) != 1:
        if abs(leftposition_sensor.getValue() - start_position) >= D2-0.01:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    """
    print("D2: " + str(D2))
    x_pos_d2 = D2*(math.cos(math.radians(imu_cleaner(imu.getRollPitchYaw()[2])))) + x_pos_d1
    y_pos_d2 = D2*math.sin(math.radians(imu_cleaner(imu.getRollPitchYaw()[2]))) + y_pos_d1
    print("P4: (" + str(x_pos_d2) + ", " + str(y_pos_d2) + ", " + str(imu_cleaner(imu.getRollPitchYaw()[2])) + ")")
    
    """
    if(V > 6.28 or V > 6.28 or V < -6.28 or V < -6.28):
        print("Velocity entered is to fast")
        quit()
        return
    #cx cy ct and fx fy ft are done because I don't know python and can't ask the TA or teacher from the hurricane Ian
    X = Fx - Cx
    Y = Fy - Cy
    theta = np.arctan(Y/X) * 180 / math.pi
    leftMotor.setVelocity(-0.4)
    rightMotor.setVelocity(0.4)
    #print("theta: " + str(theta) + " IMU: " + str(getIMUDegrees()))
    check = getIMUDegrees()
    while(not(theta - 0.4 < check < theta + 0.5)):
        if(getIMUDegrees() > 180):
            check = getIMUDegrees() - 360
        else:
            check = getIMUDegrees()
        robot.step(timestep)
    print("R1: 0")
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
        #print("theta: " + str(theta) + " IMU: " + str(check))
    a = X
    b = Y
    C = math.sqrt(a*a + b*b)
    T = C / (V*0.8)
    leftMotor.setVelocity(V)
    rightMotor.setVelocity(V)
    #print("T: " + str(T) + " C: " + str(C))
    startTime = robot.getTime()
    while(not(T - 0.32 < robot.getTime() - startTime < T + 0.32)):
        robot.step(timestep)
        #print("T: " + str(T) + " C: " + str(C))
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    print("D1: " + str(C/2))
    print("D2: " + str(C/2))
    leftMotor.setVelocity(-0.4)
    rightMotor.setVelocity(0.4)
    check = getIMUDegrees()
    FD = Ft * 180/math.pi
    #print("theta: " + str(theta) + " IMU: " + str(check) + "FD: " + str(FD))
    while(not(FD - 0.4 < check < FD + 0.5)):
        #print("theta: " + str(theta) + " IMU: " + str(check) + "FT: " + str(FD))
        if(getIMUDegrees() > 180):
            check = getIMUDegrees() - 360
        else:
            check = getIMUDegrees()
        robot.step(timestep)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    print("R2: 0")
    #print("theta: " + str(theta) + " IMU: " + str(check) + "FT: " + str(Ft))
    """

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # print simulation time in sec (timesteps are in msec)
    #print("Time: " +str(robot.getTime()))
    
    # Process sensor data here.   
    #print("Left position sensor: " +str(leftposition_sensor.getValue()))
    #print("Right position sensor: " +str(rightposition_sensor.getValue()))

    # Process sensor data here.
    #print("IMU: "+str(imu.getRollPitchYaw()[2]))

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    #leftMotor.setVelocity(1)
    #rightMotor.setVelocity(-1)
    #print(imu_cleaner(imu.getRollPitchYaw()[2]))

    inverseKinematics(0,0,math.pi/2, 15,-2.072, -3*math.pi/4,3)
    #inverseKinematics(0,0,math.pi/2, 5, 5,-math.pi,3)    
    break
    #stopMotionT(250)
    pass

# Enter here exit cleanup code.
