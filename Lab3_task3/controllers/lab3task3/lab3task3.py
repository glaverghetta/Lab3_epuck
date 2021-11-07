# lab3task3 controller.

from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math

### CONSTANTS ###
MAX_ANGULAR_SPEED = 116.921     # Maximum number of degrees robot can turn in 1 sec.
MAX_MOTOR_SPEED_ANGULAR = 2.92  # Motor setting that results in the fastest possible in-place rotation.
MAX_MOTOR_SPEED_LINEAR = 6.28 # max speed of motors for forward motion = 2 pi rad/s
MAX_LINEAR_VELOCITY = 5.0265 # maximum speed of robot in inches / sec
STOPPING_DISTANCE = 4        # Distance, in inches, the robot should keep from walls FIXME  remove this?

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
    
# Returns "west" or "east" or "north" or "south" (should only be used if robot is facing approximately west or east or north or south)
def getFacing():
    imuReading = getIMU()
    if imuReading <= 2.5 or imuReading >= 357.5:
        return "north"
    elif imuReading >= 87.5 and imuReading <= 92.5:
        return "west"
    elif imuReading >= 177.5 and imuReading <= 182.5:
        return "south"
    else:
        return "east"
        
# Turn robot to face north, detect nearby walls, then turn back to original orientation.
def detectWalls():
    imuReading = getIMU()
    if getFacing() != "north":
        turnUntil(0, 1.5)
        
    currentWalls = ["O","O","O","O"]
    if getLS() <= STOPPING_DISTANCE:
        currentWalls[0] = "W"
    if getFS() <= STOPPING_DISTANCE:
        currentWalls[1] = "W"
    if getRS() <= STOPPING_DISTANCE:
        currentWalls[2] = "W"
    
    turnUntil(imuReading, 1.5)
        
    #print("current wall config is " + str(currentWalls))
    
    return currentWalls

### TURNING FUNCTIONS ###

# Controls how fast the robot turns
turnSpeedPercent = 0.5

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

### FORWARD MOTION FUNCTIONS ###

# controls hwo fast the robot moves forward
forwardSpeedPercent = 1

# Robot moves forward inchesToMove inches FIXME: review information printing
def move(inchesToMove):
    """if moveType == "short":
        inchesToMove = 10
    else:
        inchesToMove = 30"""
        
    initialPS = getPS()
    targetPS = inchesToMove + initialPS
    stepPS = initialPS + 10
    
    # Move forward until robot has moved 10 or 30 inches
    currentPS = initialPS
    while currentPS < targetPS:
        currentPS = getPS()
        if currentPS >= stepPS:
           #printGrid()
           #printPose()
            stepPS = currentPS + 10
        lm.setVelocity(MAX_MOTOR_SPEED_LINEAR * forwardSpeedPercent)
        rm.setVelocity(MAX_MOTOR_SPEED_LINEAR * forwardSpeedPercent)
        robot.step(timestep)        

### PARTICLE FILTER ###

# Matrix containing information about the walls adjacent to each grid cell.
wallConfiguration = [["W","W","O","W"],
                     ["O","W","O","W"],
                     ["O","W","O","O"],
                     ["O","W","W","O"],
                     ["W","W","O","O"],
                     ["O","W","W","O"],
                     ["W","O","W","O"],
                     ["W","O","W","O"],
                     ["W","O","W","O"],
                     ["W","O","O","W"],
                     ["O","O","W","W"],
                     ["W","O","W","O"],
                     ["W","O","O","W"],
                     ["O","W","O","W"],
                     ["O","W","O","W"],
                     ["O","O","W","W"]]

# Probabilities for the sensors
# row = s, the actual robot state
# column = z, the sensor reading
# Example: p_ls[0][0] is equivalent to p(z = 0 | s = 0)
#          p_ls[1][0] is equivalent to p(z = 1 | s = 0)
#
# Note: Left and right sensors have the same probability values. Can use the same matrix for both.

p_ls = [[0.6, 0.2],
        [0.4, 0.8]]
        
p_fs = [[0.7, 0.1],
        [0.3, 0.9]]

NUM_CELLS = 16          # Number of cells in the maze
TOTAL_PARTICLES = 80    # Total number of particles to use in particle filter
FINISHING_PARTICLE_NUMBER = 60  # Robot is confident enough to estimate its location when a cell has at least this many particles

# Matrix containing the number of particles currently alloted to each cell. 
# Initialized with all cells having 5 particles.
particles = [[5, 5, 5, 5],
             [5, 5, 5, 5],
             [5, 5, 5, 5],
             [5, 5, 5, 5]]
             
# Matrix containing the current probability estimates for each cell.
currentProbabilities = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
             
# Run the particle filter to determine robot's location
def particleFilter():
    readyToEstimate = False
    iterationNumber = -1
    while(not readyToEstimate):
        iterationNumber = iterationNumber + 1
        probabilitySum = 0

        # Detect the walls, if any, that are adjacent to this grid cell.
        adjacentWalls = detectWalls()
        
        # Compare sensor readings with known wall configuration to create probability estimates
        # Loop through each grid cell
        for cell in range(NUM_CELLS):
            currentCellProbability = 0
            # Loop through all walls in the cell, except the south wall
            for wall in range(3):
                # Get s and z values
                if wallConfiguration[cell][wall] == "W":
                    s = 1
                else:
                    s = 0
                if adjacentWalls[wall] == "W":
                    z = 1
                else:
                    z = 0
                # Use p_ls for left and right walls
                if wall == 0 or wall == 2:
                    currentCellProbability = currentCellProbability + p_ls[z][s]
                else:
                    currentCellProbability = currentCellProbability + p_fs[z][s]
                    
            currentProbabilities[cell] = currentCellProbability
            probabilitySum = probabilitySum + currentCellProbability
        
        print("base probs " + str(currentProbabilities))
        #print(str(probabilitySum))
        
        # Normalize probabilities and resample
        for cell in range(NUM_CELLS):
            #currentProbabilities[cell] = (currentProbabilities[cell]) * TOTAL_PARTICLES
            # Importance factor
            """if iterationNumber > 0:
                row = math.floor(cell / 4)
                col = cell - (4 * row)
                particlesInCell = particles[row][col]
                currentProbabilities[cell] = currentProbabilities[cell] * particlesInCell"""
             
            currentProbabilities[cell] = currentProbabilities[cell] / probabilitySum * TOTAL_PARTICLES
                
            #probabilitySum = probabilitySum + currentProbabilities[cell]
             
        #for cell in range(NUM_CELLS):
        #    currentProbabilities[cell] = currentProbabilities[cell] / probabilitySum * TOTAL_PARTICLES
        
        print("changed probs " + str(currentProbabilities))
        
        # Assign paticles
        remainingParticles = TOTAL_PARTICLES
        while(remainingParticles > 0):
            # Find the cell with the highest probability and give it particles
            highestProbability = currentProbabilities[0]
            index = 0
            for cell in range(1, NUM_CELLS):
                if currentProbabilities[cell] > highestProbability:
                    highestProbability = currentProbabilities[cell]
                    index = cell
            #print(highestProbability)
            
            particlesToAssign = math.ceil(highestProbability)
            if particlesToAssign >= FINISHING_PARTICLE_NUMBER:
                readyToEstimate = True
            
            # Determine row and column values from index
            row = math.floor(index  / 4)
            col = index - (row * 4)
            #print("row is " + str(row) + " col is " + str(col))
            
            # Give particles to the cell
            particles[row][col] = particlesToAssign
            currentProbabilities[index] = 0  
            
            remainingParticles = remainingParticles - particlesToAssign
              
        print(particles)
        
        if readyToEstimate:
            break
            
        # Motion update
        
        # Rotate robot away from walls
        while(getFS() <= STOPPING_DISTANCE):
            turn("left", 1.5)
            
        # Shift particles
        directionOfMotion = getFacing() 
        print(directionOfMotion)
        cellNum = 0
        if directionOfMotion == "north":
            for row in range(3, 0, -1):
                for col in range(4):
                    cellNum = (4 * row) + col
                    # Don't shift particles if there is a wall in the way
                    if wallConfiguration[cellNum][1] == "O":
                        particlesLeftInCell = math.floor(particles[row][col] * p_fs[0][0])
                        particles[row - 1][col] = particles[row - 1][col] + (particles[row][col] - particlesLeftInCell)
                        particles[row][col] = particlesLeftInCell
                        
                    #cellNum = cellNum + 1
        
                    
        elif directionOfMotion == "west":
            for row in range(4):
                for col in range(3, 0, -1):
                    #print("row " + str(row) + " col " + str(col))
                    cellNum = (4 * row) + col
                    # Don't shift particles if there is a wall in the way
                    if wallConfiguration[cellNum][0] == "O":
                        particlesLeftInCell = math.floor(particles[row][col] * p_ls[0][0])
                        particles[row][col - 1] = particles[row][col - 1] + (particles[row][col] - particlesLeftInCell)
                        particles[row][col] = particlesLeftInCell
                        
                    #cellNum = cellNum + 1
        
        elif directionOfMotion == "east":
            for row in range(4):
                for col in range(3):
                    #print("row " + str(row) + " col " + str(col))
                    cellNum = (4 * row) + col
                    # Don't shift particles if there is a wall in the way
                    if wallConfiguration[cellNum][2] == "O":
                        particlesLeftInCell = math.floor(particles[row][col] * p_ls[0][0])
                        particles[row][col + 1] = particles[row][col + 1] + (particles[row][col] - particlesLeftInCell)
                        particles[row][col] = particlesLeftInCell
                        
        elif directionOfMotion == "south":
            for row in range(3):
                for col in range(4):
                    #print("row " + str(row) + " col " + str(col))
                    cellNum = (4 * row) + col
                    # Don't shift particles if there is a wall in the way
                    if wallConfiguration[cellNum][3] == "O":
                        particlesLeftInCell = math.floor(particles[row][col] * p_ls[0][0])
                        particles[row + 1][col] = particles[row + 1][col] + (particles[row][col] - particlesLeftInCell)
                        particles[row][col] = particlesLeftInCell
           
        print(particles)
                    
        # Move the robot
        move(10)
        
        print("motion done")
                        
   
                         
                
                
        

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
    
    print("i is " + str(i) + " and j is " + str(j))
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
        
### MAIN ROBOT CONTROL CODE ###

robot = Robot()

# get the time step of the current world.
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

while robot.step(timestep) != -1:
    # Step 1: Use particle filter to figure out where the robot is
    particleFilter()
    break
    # Step 2: Navigate to all cells in environment
    

