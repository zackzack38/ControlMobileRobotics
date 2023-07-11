"""lab1_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math
# create the Robot instance.
robot = Robot()

# get the time step of the current world in msec.
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

# getting the position sensors

leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)

# math
# d = 2.6 in r = 1.3 in
# c = pi * d = 2*pi*r found using calculator
# c = 8.1681408993334624200028727965267 inches


# motor functions

# robot stops moving, sets velocity to 0
def stopMotion():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    return
    
# robot stops moving for a set time T
def stopMotionT(T):
    temp = robot.getTime()
    while (robot.getTime() - temp < T):
        stopMotion();
        robot.step(timestep)
    #resetCounts()
    return
    
# straightlineMotionV(V) - robot moves in a straight line at a linear velocity of “V” inches per second
def straightlineMotionV(V):
    # V is in inches per second, setVelocity takes rad/s, radians don't exist. Max speed of 8.164 in/s
    #V = S/T | omega = theta/T | V = R * omega
    radius = 0.8
    vel = V / radius
    if(vel >= 6.28 or vel <= -6.28):
        print("Velocity entered is to fast")
        return
    leftMotor.setVelocity(vel)
    rightMotor.setVelocity(vel)
    return

# straightlineMotionVX(V, X) - robot moves in a straight line at a linear velocity of “V” inches per second for a distance of “X” inches. 
def straightlineMotionVX(V, X):
    #Seconds = inches / (inches/seconds)
    if (V != 0):
        S = X / V
    else:
        # velocity of 0 can never move to X distance
        return
    T1 = robot.getTime()
    while (robot.getTime() - T1) <= S:
        robot.step(timestep)
        straightlineMotionV(V)
    return

# straightlineMotionVT(V, T) - moves in a straight line at a linear velocity of “V” inches per second for “T” seconds.
def straightlineMotionVT(V, T):
    if (T <= 0 and (V > 6.28 and V < -6.28)):
        #time <= 0 is instant so nothing happens
        return
    if(vel > 6.28 or vel < -6.28):
        print("Velocity entered is to fast")
        return
    temp = robot.getTime()
    while (robot.getTime() - temp) <= T:
        straightlineMotionV(V)
        robot.step(timestep)
    return

# circleMotionRV(R, V) - robot moves in a circle of radius “R” at a constant linear velocity of “V” inches per second.
def circleMotionRV(R, V):
    # V is in inches per second, setVelocity takes rad/s, radians don't exist. Max speed of 8.164 in/s
    #V = S/T | omega = theta/T | V = R * omega
    if (R < 0 and (V < 6.28 and V > -6.28)):
        print("Radius entered is too small: Min radius of 0")
        return
    if (R == 0):
        #if R is 0 then the linear velocity is 0 but the robot is expected to turn so I am treating it as the the angular velocity
        velZero = 1.14 * V / 0.8
        if(velZero > 6.28 or velZero < -6.28):
            print("Velocity entered is to fast")
            return
        leftMotor.setVelocity(velZero)
        rightMotor.setVelocity(-velZero)
        return
    wheelDistance = 2.28 / 2
    rightV = V * (R - wheelDistance) / R
    leftV = 2 * V - rightV
    if((leftV / 0.8) > 6.28 or (rightV / 0.8) > 6.28):
        print("Velocity entered is to fast")
        return
    leftMotor.setVelocity(leftV / 0.8)
    rightMotor.setVelocity(rightV / 0.8)
    return
    
# circleMotionRVX(R, V, X) - robot moves in a circle of radius “R” at a constant linear velocity of “V” inches per second for a distance of “X” inches.
def circleMotionRVX(R, V, X):
    #v = wR = (vr + vl)/2
    motor 
    omega = V/R
    T = 90 / omega
#    S = V T
    return
    
    if (R < 0 and (V < 6.28 and V > -6.28)):
        print("Radius entered is too small: Min radius of 0")
        return
    omega = V/R
    vl = omega * (R + 1.14) / 0.8
    vr = omega * (R - 1.14) / 0.8
    #s = v*t = theta*R
    theta = X / R
    wheelDistance = 2.28 / 2
    rightV = V * (R - wheelDistance) / R
    leftV = 2 * V - rightV
    
    
    if (R < 0 and (V < 6.28 and V > -6.28)):
        print("Radius entered is too small: Min radius of 0")
        return
    omega = V/R
    vl = omega * (R + 1.14) / 0.8
    vr = omega * (R - 1.14) / 0.8
    #s = v*t = theta*R
    
    wheelDistance = 2.28 / 2
    rightV = V * (R - wheelDistance) / R
    leftV = 2 * V - rightV
    return

# circleMotionRVT(R, V, T) - robot moves in a circle of radius “R” at a constant linear velocity of “V” inches per second for “T” seconds.
def circleMotionRVT(R, V, T):
    if (R < 0):
        print("Radius entered is too small: Min radius of 0")
        return
    if (R == 0):
        #if R is 0 then the linear velocity is 0 but the robot is expected to turn so I am treating it as the the angular velocity
        velZero = 1.14 * V / 0.8
        if(velZero > 6.28 or velZero < -6.28):
            print("Velocity entered is to fast")
            #exit program
            return
        leftV = velZero
        rightV = -velZero
    else:
        omega = V / R
        rightV = (V * (R - 1.14) / R)/0.8
        leftV = (V * (R + 1.14) / R)/0.8
        if((leftV) > 6.28 or (rightV) > 6.28):
            print("Velocity entered is to fast")
            return
        temp = robot.getTime()
        leftMotor.setVelocity(leftV)
        rightMotor.setVelocity(rightV)
        while (robot.getTime() < T + temp):
            robot.step(timestep)
            return
    return
# Task 1 - motor Control - Rectangle
# rectangleMotion(W, H, V) - main task function making the robot follow the rectangle with width “W”, height “H”, at a constant linear velocity of “V” inches per second.
def rectangleMotion(W, H, V):
    L = (math.pi * 1.14 / 2)
    S = (1.14 * math.pi / 2)
    #L = (math.pi * 1.14 / 2)
    #cicleMotionRVX(0, V, L)
    
    straightlineMotionVX(V, H/2)
    #turn right 90 degrees
    temp = robot.getTime()
    lPos = leftposition_sensor.getValue()
    lNewPos = 0
    dumb = 0.345
    things = 2
    while ((lNewPos - lPos) < S+dumb):
        print("Left position sensor: " +str(leftposition_sensor.getValue()))
        print("Right position sensor: " +str(rightposition_sensor.getValue()))
        lNewPos = leftposition_sensor.getValue()
        leftMotor.setVelocity(things)
        rightMotor.setVelocity(-things)
        robot.step(timestep)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    straightlineMotionVX(V, W)
    
    #turn right 90 degrees
    temp = robot.getTime()
    lPos = leftposition_sensor.getValue()
    lNewPos = 0
    while ((lNewPos - lPos) < S+dumb):
        print("Left position sensor: " +str(leftposition_sensor.getValue()))
        print("Right position sensor: " +str(rightposition_sensor.getValue()))
        lNewPos = leftposition_sensor.getValue()
        leftMotor.setVelocity(things)
        rightMotor.setVelocity(-things)
        robot.step(timestep)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    straightlineMotionVX(V, H)
    
    #turn right 90 degrees
    temp = robot.getTime()
    lPos = leftposition_sensor.getValue()
    lNewPos = 0
    while ((lNewPos - lPos) < S+dumb):
        print("Left position sensor: " +str(leftposition_sensor.getValue()))
        print("Right position sensor: " +str(rightposition_sensor.getValue()))
        lNewPos = leftposition_sensor.getValue()
        leftMotor.setVelocity(things)
        rightMotor.setVelocity(-things)
        robot.step(timestep)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    straightlineMotionVX(V, W)
    
    temp = robot.getTime()
    lPos = leftposition_sensor.getValue()
    lNewPos = 0
    while ((lNewPos - lPos) < S+dumb):
        print("Left position sensor: " +str(leftposition_sensor.getValue()))
        print("Right position sensor: " +str(rightposition_sensor.getValue()))
        lNewPos = leftposition_sensor.getValue()
        leftMotor.setVelocity(things)
        rightMotor.setVelocity(-things)
        robot.step(timestep)
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    straightlineMotionVX(V, H/2)
    return    

# Main loop:
# perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    print("Time: " +str(robot.getTime()))
    rectangleMotion(20, 10, 4) #W H V
    stopMotionT(100)
# Enter here exit cleanup code.