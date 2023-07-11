"""lab1_task2 controller."""

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
        while (robot.getTime() - temp) < T:
            leftMotor.setVelocity(leftV)
            rightMotor.setVelocity(rightV)
            robot.step(timestep)
    return
def circleMotionRVTC(R, V, T):
    if (R < 0):
        print("Radius entered is too small: Min radius of 0")
        return
    if (R == 0):
        #if R is 0 then the linear velocity is 0 but the robot is expected to turn so I am treating it as the the angular velocity
        velZero = 1.14 * V / 0.8
        if(velZero > 6.28 or velZero < -6.28):
            print("Velocity entered is to fast")
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
        while (robot.getTime() - temp) < T:
            leftMotor.setVelocity(rightV)
            rightMotor.setVelocity(leftV)
            robot.step(timestep)
    return


def turnAround():
    temp = robot.getTime()
    T = 4.5
    while (robot.getTime() < T + temp):
        robot.step(timestep)
        leftMotor.setVelocity(1)
        rightMotor.setVelocity(-1)
    return

def circlesMotion(R1, R2, V):
    turnAround()
    #Omega = V/R1
    S = 2 * math.pi * R1 / V
    circleMotionRVT(R1, V, S)
    S = 2 * math.pi * R2 / V
    circleMotionRVTC(R2, V, S)
    return



# Main loop:
# perform simulation steps until Webots is stopping the controller

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    # val = ds.getValue()

    # print simulation time in sec (timesteps are in msec)
    print("Time: " +str(robot.getTime()))
    
    # Process sensor data here.   
    print("Left position sensor: " +str(leftposition_sensor.getValue()))
    print("Right position sensor: " +str(rightposition_sensor.getValue()))

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    circlesMotion(5, 10, 3)
    print("Time: " +str(robot.getTime()))
    stopMotionT(50)

# Enter here exit cleanup code.