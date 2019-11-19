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


# Take the 3-D array returned from mr library and 
# turn it into a 2-D array
# gripper is either 0 or 1. 0 means gripper closed. 1 means open/opening
def process_array(myArray, gripper):
    
    length = len(myArray)
    finalArray = np.zeros( (length, 13) ) 
    
    
    for i in range( length ):
        # r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz
        finalArray[i][0] = myArray[i][0][0]
        finalArray[i][1] = myArray[i][0][1]
        finalArray[i][2] = myArray[i][0][2]


        finalArray[i][3] = myArray[i][1][0]
        finalArray[i][4] = myArray[i][1][1]
        finalArray[i][5] = myArray[i][1][2]
         
        finalArray[i][6] = myArray[i][2][0]
        finalArray[i][7] = myArray[i][2][1]
        finalArray[i][8] = myArray[i][2][2]
    
                
        finalArray[i][9] = myArray[i][0][3]
        finalArray[i][10] = myArray[i][1][3]
        finalArray[i][11] = myArray[i][2][3]

        # Add the gripper open/close variable
        finalArray[i][12] = gripper

    
    return finalArray


# This method will generate the trajectory
# The initial configuration of the end-effector in the reference trajectory: Tse,initial.
# The cube's initial configuration: Tsc,initial.
# The cube's desired final configuration: Tsc,final.
# The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp.
# The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: Tce,standoff.
# The number of trajectory reference configurations per 0.01 seconds: k. The value k is an integer 
# with a value of 1 or greater.
def TrajectoryGenerator( T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k):
    
    # This method should call CartesianCoordinates 8 times

    # List of order of the path entries 
    # r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz

    totalSeconds = 10
    N = float(totalSeconds) / float(k) 
    
    # path_states = np.zeros( (N, 13) )
    path_states = np.array([])

    # Use ScrewTrajectory or CartesianTrajectory
    X_start = T_se_initial   
    X_end = T_sc_initial
    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
    
    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)
    

    return path_states



# Milestone 1

num_seconds = 3
numEntries = num_seconds * 100 
length = 12

# Set up the current state
allStates = np.zeros( (numEntries, length) )

current_state = np.zeros(12) 


#for i in range(0, 100 * num_seconds):
 
#    controls = np.zeros(9)
#    controls[5] = 10
#    controls[6] = 10
#    controls[7] = 10
#    controls[8] = 10

#    current_state = nextState(current_state, controls)   
#    print(current_state)

    # Add the state to the allStates vector
#    allStates[i] = current_state



# Milestone 2
# Call TrajectoryGenerator 8 times and concatenate each segment

# Set up the initial conditions
T_se_initial = np.array( [ [0, 0, 1, 0],
                           [0, 1, 0, 0],
                           [-1, 0, 0, 0.5],
                           [0, 0, 0, 1] ] )

T_sc_initial = np.array( [ [0, 0, 1, 1],
                           [-1, 0, 0, 0],
                           [0, 0, 1, 0.3],
                           [0, 0, 0, 1] ] )

# T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k
allStates = TrajectoryGenerator( T_se_initial, T_sc_initial, 0, 0, 0, 1)


# Save the csv file 
np.savetxt("milestone1.csv", allStates, delimiter=",")



