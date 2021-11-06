"""lab3task1 controller."""

from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

### CONSTANTS ###
MAX_ANGULAR_SPEED = 116.921     # Maximum number of degrees robot can turn in 1 sec.
MAX_MOTOR_SPEED_ANGULAR = 2.92  # Motor setting that results in the fastest possible in-place rotation.
MAX_MOTOR_SPEED_LINEAR = 6.28 # max speed of motors for forward motion = 2 pi rad/s
MAX_LINEAR_VELOCITY = 5.0265 # maximum speed of robot in inches / sec
STOPPING_DISTANCE = 4        # Distance, in inches, the robot should keep from walls

### UNIT CONVERSIONS ###

# Converts input meters to inches.
def mToIn(m):
    return m * 39.3701
    
# Converts input radians to inches.
def inToRad(inch):
    return inch * 1.25
    
# Converts input radians to inches.
def radToIn(rad):
    return 0.8 * rad
  
# Converts inputs radians to degrees  
def radToDeg(rad):
    return 180 * rad / math.pi
    
### RETRIEVING INFORMATION FROM SENSORS ###

# Get the yaw value from robot's imu, in degrees, normalized to be in the range [0, 360)
def getIMU():
    angle = imu.getRollPitchYaw()[2]
    return radToDeg(angle) + 180
    
# Get the average of the two position sensor values, in inches
def getPS():
    leftValue = lps.getValue()
    rightValue = rps.getValue()
    avgValue = (leftValue + rightValue) / 2
    return radToIn(avgValue)
    
# Get the front sensor value, in inches
def getFS():
    value = fs.getValue()
    return mToIn(value)
    
# Get the left sensor value, in inches
def getLS():
    value = ls.getValue()
    return mToIn(value)
    
# Get the right sensor value, in inches
def getRS():
    value = rs.getValue()
    return mToIn(value)
    
### TURNING FUNCTIONS ###

# Controls how fast the robot turns
turnSpeedPercent = 0.3

# Turns left 90 +/- offset deg if turnType == left, else turn right 90 +/- offset deg
def turn(turnType, offset):
    initialIMU = getIMU()
    if turnType == "left":
        targetIMU = initialIMU + 90
        if targetIMU >= 360:
            targetIMU = targetIMU - 360
        while getIMU() < targetIMU - offset or getIMU() > targetIMU + offset:
            lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)          
            robot.step(timestep)
    else:
        targetIMU = initialIMU - 90
        if targetIMU < 0:
            targetIMU = targetIMU + 360
        while getIMU() < targetIMU - offset or getIMU() > targetIMU + offset:
            lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)          
            robot.step(timestep)

# Turn robot until IMU is within offset of target. Target should be 180, 90, or 270
def turnUntil(target, offset):
    if target == 180:
        while getIMU() < target - offset or getIMU() > target + offset:
            # Turning clockwise
            lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)          
            robot.step(timestep)
        return
    else:
        while getIMU() < target - offset or getIMU() > target + offset:
            lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            robot.step(timestep)
        return

# Returns "west" or "east" or "north" (should only be used if robot is facing approximately west or east or north)
def getFacing():
    imuReading = getIMU()
    if imuReading >= 87.5 and imuReading <= 92.5:
        return "west"
    elif imuReading <= 2.5 or imuReading >= 357.5:
        return "north"
    else:
        return "east"

### FORWARD MOTION FUNCTIONS ###

# Controls how fast the robot moves forward
forwardSpeedPercent = 1

# Robot moves forward until it front ds reports a value <= wallDist
def moveUntilWall(wallDist):
    while getFS() > wallDist:
        lm.setVelocity(MAX_MOTOR_SPEED_LINEAR * forwardSpeedPercent)
        rm.setVelocity(MAX_MOTOR_SPEED_LINEAR * forwardSpeedPercent)
        robot.step(timestep)
    lm.setVelocity(0)
    rm.setVelocity(0)

# Robot moves forward 10 inches if moveType == "short", 30 inches if moveType == "long"
def move(moveType):
    if moveType == "short":
        inchesToMove = 10
    else:
        inchesToMove = 30
        
    initialPS = getPS()
    targetPS = inchesToMove + initialPS
    stepPS = initialPS + 10
    
    # Move forward until robot has moved 10 or 30 inches
    currentPS = initialPS
    while currentPS < targetPS:
       currentPS = getPS()
       if currentPS >= stepPS:
           printGrid()
           printPose()
           stepPS = currentPS + 10
       lm.setVelocity(MAX_MOTOR_SPEED_LINEAR * forwardSpeedPercent)
       rm.setVelocity(MAX_MOTOR_SPEED_LINEAR * forwardSpeedPercent)
       robot.step(timestep)

### INFORMATION PRINTING FUNCTIONS ###

# Matrix indicating not yet visited cells with "." and visited cells with "X"
cellsVisited = [[".", ".", ".", "."],
                [".", ".", ".", "."],
                [".", ".", ".", "."],
                [".", ".", ".", "."]]

# Index variables for cellsVisited
i = 3
j = -1

# Returns False unless all grid cells have been visited
def allCellsVisited():
    for row in cellsVisited:
        for col in row:
            if col == ".":
                return False
    return True

# Prints the grid, indicating which cells have been visited so far, and updates index variables
def printGrid():

    global i, j

    if getFacing() == "east":
        j = j + 1
    elif getFacing() == "west":
        j = j - 1
    else:
        i = i - 1
    
    cellsVisited[i][j] = "X"

    for row in cellsVisited:
        for col in row:
            print(col, end = " ")
        print()
 
x = 0
y = 0 
        
# Prints the current robot pose information 
def printPose():

    global x, y

    fs = getFS()
    ls = getLS()
    rs = getRS()
    if getFacing() == "east":
        x = 20 - fs
        y = 20 - ls
    elif getFacing() == "west":
        x = fs - 20
        y = 20 - rs
    else:
        y = 20 - fs
        
    n = (4 * i) + j + 1
    theta = getIMU()
    
    print("(" + str(x) + ", " + str(y) + ", " + str(n) + ", " + str(theta) + ")")

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Setup necessary robot devices
fs = robot.getDevice('front_ds')
ls = robot.getDevice('left_ds')
rs = robot.getDevice('right_ds')
fs.enable(timestep)
ls.enable(timestep)
rs.enable(timestep)

lps = robot.getDevice('left wheel sensor')
lps.enable(timestep)
rps = robot.getDevice('right wheel sensor')
rps.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0)
rm.setVelocity(0)

# Set to False if robot starts in an unknown state
knownState = False

moveType = "long"
turnType = "left"

while robot.step(timestep) != -1:
    # If robot's state is not known be at grid cell 13 facing east, reposition robot there first
    if not knownState:
        print("Robot is repositioning itself in grid cell 13")
        turnUntil(180, 1.5)
        moveUntilWall(STOPPING_DISTANCE)
        turnUntil(90, 1.5)
        moveUntilWall(STOPPING_DISTANCE)
        turnUntil(270, 1.5)
        printGrid()
        printPose()
    knownState = True
    
    # Move forward and alternate move type
    move("long")
    printGrid()
    printPose()
    
    # Finish task if all cells are visited
    if(allCellsVisited()):
        lm.setVelocity(0)
        rm.setVelocity(0)
        break
    
    # Turn approximately 90 degrees
    turn(turnType, 1.5)
    
    # Move forward and alternate move type
    move("short")    
    
    # Turn approximately 90 degrees and alternate turn type
    turn(turnType, 1.5)
    if turnType == "left":
        turnType = "right"
    else:
        turnType = "left"
             
             