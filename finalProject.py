import modern_robotics as mr
import math
import numpy as np

#############
# Globals
allStates = np.array([])

############

# Input: 12 vector of current state 
# 3 variables for the chassis configuration, 5 variables for the arm 
# configuration, and 4 variables for the wheel angles
# Output: 12 vector of the next state
def nextState(current_state, controls):

    dt = 0.01
    
    new_state = current_state.copy()

    # new arm joint angles = (old arm joint angles) + (joint speeds) * delta t
    
    # new wheel angles = (old wheel angles) + (wheel speeds) * delta 2
    new_state[8] = current_state[8] + controls[5] * dt   
    new_state[9] = current_state[9] + controls[6] * dt
    new_state[10] = current_state[10] + controls[7] * dt
    new_state[11] = current_state[11] + controls[8] * dt
    
    # new chassis configuration is obtained from odometry, as described in Chapter 13.4 

    
    return new_state
    


num_seconds = 3
numEntries = num_seconds * 100 
length = 12

# Set up the current state
allStates = np.zeros( (numEntries, length) )

current_state = np.zeros(12) 


for i in range(0, 100 * num_seconds):
    
    controls = np.zeros(9)
    current_state = nextState(current_state, controls)
    
    print(current_state)

    # Add the state to the allStates vector
    allStates[i] = current_state

# Save the csv file 
np.savetxt("milestone1.csv", allStates, delimiter=",")



