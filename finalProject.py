import modern_robotics as mr
import math
import numpy as np
import matplotlib.pyplot as plt
import logging


#############
# Global Variables
# This is the array that will store each point on the 
# desired trajectory
allStates = np.array([])

# There is an array for each part of the error vector
# X_err is a 6-vector
allError_1 = np.array( []  )
allError_2 = np.array( []  )
allError_3 = np.array( []  )
allError_4 = np.array( []  )
allError_5 = np.array( []  )
allError_6 = np.array( []  )

# This records how often to update the 
# error data strucutres. Every kth point, record the error
updateCount = 2

# This is the integral error term
runningError = 0

############

# This is just a friendly reminder of the order in which I put the arrays
# Input: 12 vector of current state 
# 3 variables for the chassis configuration, 5 variables for the arm 
# configuration, and 4 variables for the wheel angles
# Output: 12 vector of the next state 

# Input data 
# currentState
# [0, 2] = (theta, x, y)
# [3, 7] = arm configuration
# [8, 11] = wheel configuration

# controls 
# [0, 4] = arm torques 
# [5, 8] = wheel velocities


##############

# This method takes the current state on the actual trajectory
# and has the controls vector execute. We return the state after 
# dt has passed as we apply the given controls vector
# Input
# currentState is current state on the actual trajectory 
# controls is controls vector the controls wants to execute
# speedLimit is an optional parameter. It enforces a speed 
# limit on each joint. If the controller requests 
# speed above it, then the real speed is the limited one
# Return: the given state of the system after dt 
def nextState(currentState, controls, speedLimit = 12.5):

    # Check for illegal control values
    # If they are ilegal, enforce the speed limit
    for i in range(len(controls) ):
        
        negative = True
        if ( controls[i] > 0 ):
            negative = False

        if ( abs(controls[i] ) > speedLimit):
            controls[i] = speedLimit
            
            if ( negative == True ):
                controls[i] = -1 * controls[i]

    dt = 0.01
    
    new_state = currentState.copy()
    
    # Euler Integration 
    # new arm joint angles = (old arm joint angles) + (joint speeds) * delta t 
    new_state[3] = currentState[3] + ( controls[0] * dt )
    new_state[4] = currentState[4] + ( controls[1] * dt )
    new_state[5] = currentState[5] + ( controls[2] * dt )
    new_state[6] = currentState[6] + ( controls[3] * dt )
    new_state[7] =  currentState[7] + ( controls[4] * dt )

    # new wheel angles = (old wheel angles) + (wheel speeds) * delta 2 
    new_state[8] = currentState[8] + (controls[5] * dt)   
    new_state[9] = currentState[9] + (controls[6] * dt)
    new_state[10] = currentState[10] + (controls[7] * dt)
    new_state[11] = currentState[11] + (controls[8] * dt)
     
    # new chassis configuration is obtained from odometry, as described in Chapter 13.4 
    r = 0.0475 
    l = 0.47 / 2.0
    w = 0.30 / 2.0
    
    # This matrix is described in 13.4
    # It relates how a control vector moves the mecanum wheel base
    H_p_i = (r / 4.0) * np.array( [ [ -1 / (l + w), 1, -1 ], [ 1 / (l + w), 1, 1 ], [ 1 / (l + w), 1, -1], [ -1 / (l + w), 1, 1] ] ).T 
        
    # wheel velocities are controls[5, 8]
    wheel_velocities = np.array( [controls[5] , controls[6], controls[7], controls[8] ] ).T

    delta_theta = ( wheel_velocities ) * dt
    
    twist_b = ( ( np.matmul( H_p_i, delta_theta) ) )
    
    w_b_z = twist_b[0]
    v_b_x = twist_b[1]
    v_b_y = twist_b[2]

    delta_q_b = None

    if ( abs(w_b_z) < 0.01 ):
        delta_q_b = np.array( [ 0, v_b_x, v_b_y ]   )
    else:
        
        value1 = ( (v_b_x * np.sin(w_b_z) ) + (v_b_y * (np.cos(w_b_z - 1) ) ) ) / w_b_z  
        
        value2 = ( (v_b_y * np.sin(w_b_z) ) + (v_b_x * ( 1 - np.cos(w_b_z)  ) ) ) / w_b_z 
    
        delta_q_b = np.array( [ w_b_z, value1, value2 ]  ).T
    

    # Transoform delta_q_b to the space frame 
    # Rotate by the chassis's angle
    angle = currentState[0]
    rotationMatrix = np.array( [ [1, 0, 0], [0, np.cos(angle), np.sin(angle)  ], [ 0, -1 * np.sin(angle), np.cos(angle) ] ] ).T 

    q_s = np.matmul( rotationMatrix, delta_q_b)

    # Add the delta q to the old values
    new_state[0] = new_state[0] + q_s[0] 
    new_state[1] = new_state[1] + q_s[1]
    new_state[2] = new_state[2] + q_s[2]
    
    return new_state


# Take the 3-D array returned from mr library and 
# turn it into a 2-D array
# gripper is either 0 or 1. 0 means gripper closed. 1 means open/opening
# This allows us to process the data for V-REP to use
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

# This method creates the first segment that takes the end-effector 
# from the start state to the stand-off state
# T_se_initial is the configuration of the end effector in the space frame
# T_sc_final is the cube's initial configuration in the space frame 
# T_ce_standoff is the end effector's orientation relative to the 
# cube at standoff 
# Return the segment of the trajectory in array ready for csv writing
def createSegment1( T_se_initial, T_sc_initial, T_ce_standoff, k ):
    
    # Need to compute the desired orientation R
    T_se_standoff = np.matmul(T_sc_initial.copy(), T_ce_standoff.copy() )
    
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = T_se_initial.copy()

    X_end = T_se_standoff.copy()
    
    X_end = X_end.astype(float)

    # The total time in seconds
    Tf = totalSeconds
    
    # Method describes cubic or quintic scaling
    method = 5
   
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
    
    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)
    
    return path_states

# This segment will create second segment that takes 
# the end-effector from the standoff state to the grapsing state 
# T_se_initial is the configuration of the end effector in the space frame
# T_sc_final is the cube's initial configuration in the space frame 
# T_ce_grasp is the end-effector's orientation relative to the 
# cube when it is being grasped
# Return the segment of the trajectory in array ready for csv writing
def createSegment2(T_se_initial, T_sc_initial, T_ce_grasp, k ):

    totalSeconds = 100
    N = float(totalSeconds) / float(k)
    
    T_goal = np.matmul( T_sc_initial.copy(), T_ce_grasp.copy() )

    X_start = T_se_initial.copy()

    X_end = T_goal.copy()

    Tf = totalSeconds

    # Method describes cubic or quintic scaling 
    method = 5
    
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
    
    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)
    
    return path_states

# Take the linear array and construct the SE3 representation
# Input: the data in the format that V-REP desires
# Return: the same data but in SE(3) form
def convertLinearToSE3(linearList):
    
    finalList = np.zeros( (4, 4) ) 
    
    # r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz
    finalList[0][0] = linearList[0]  
    finalList[0][1] = linearList[1]
    finalList[0][2] = linearList[2]


    finalList[1][0] = linearList[3]
    finalList[1][1] = linearList[4]
    finalList[1][2] = linearList[5]
    
    finalList[2][0] = linearList[6]
    finalList[2][1] = linearList[7]
    finalList[2][2] = linearList[8]

    finalList[0][3] = linearList[9]
    finalList[1][3] = linearList[10]
    finalList[2][3] = linearList[11]


    finalList[3][0] = 0
    finalList[3][1] = 0
    finalList[3][2] = 0
    finalList[3][3] = 1

    return finalList

# Take the T_se and convert it to the format that 
# the csv file wants
# Input: currentState from nextState function
# Returns: chassis phi, chassis x, chassis y, J1, J2, J3, 
# J4, J5, W1, W2, W3, W4, gripper state
# This reformatting lets us input the data to V-REP
def convertSE3_List( array ):

    myList = np.zeros(13)
    
    myList[0] = array[2] 
    myList[1] = array[0]
    myList[2] = array[1]
    myList[3] = array[3]
    myList[4] = array[4]
    myList[5] = array[5]
    myList[6] = array[6]
    myList[7] = array[7]
    myList[8] = array[8]
    myList[9] = array[9]
    myList[10] = array[10]
    myList[11] = array[11]
    myList[12] = 0

    return myList


# This method creates a path which only closes the gripper
# current_state is the end effector's configuration in the space frame
# while we are closing the gripper
def createSegment3( current_state, k ):
    
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = current_state.copy()

    # The total time in seconds
    Tf = totalSeconds

    # Method describes cubic or quintic scaling
    method = 5
    
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
   
    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)

    return path_states

# Moves the robot from grasping the block back to the INITIAL standoff state 
#  current state is the T_se as the block is being grasped
# stanoff state is the end effector's state in the space frame at standoff
# Return the trajectory segment in the desired format so we can write it to VREP
def createSegment4( current_state, standoff_state, k ):
        
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = standoff_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    
    # Method describes cubic or quintic scaling
    method = 5
    
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)
    
    return path_states

# This generates a path from the initial standoff state 
# to the final standoff state.
# current state is the T_se as the block is being grasped
# stanoff state is the end effector's state in the space frame at standoff
# T_sc_final is the cube's final, desired location in the space frame
# Return the trajectory segment in the desired format so we can write it to VREP
def createSegment5(  current_state, standoff_state, T_sc_final, k):

    #  current_gripper_state, T_ce_standoff, T_sc_final, k )
    totalSeconds = 200
    N = float(totalSeconds) / float(k)

    goal_state = np.matmul( standoff_state, T_sc_final )
    
    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds

    # Method describes cubic or quintic scaling
    method = 5
    
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)

    return path_states

# This method creates a path from the final standoff state to the 
# the grasping state
# current state is the T_se as the block is being grasped
# T_sc_final is the cube's final, desired location in the space frame
# T_ce_grasp is the end effector's configuration as we grasp it
# Return the trajectory segment in the desired format so we can write it to VREP
def createSegment6( current_state, T_sc_final, T_ce_grasp, k ):

    goal_state = np.matmul( T_sc_final, T_ce_grasp )

    totalSeconds = 200
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds

    # Method describes cubic or quintic scaling
    method = 5
    
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)

    return path_states

# This segment simply releases the gripper
# currentState is the end effector's configuration in the space frame as 
# we release the cube. goal_state is the same configuration as currentState
# Return the trajectory segment in the desired format so we can write it to VREP
def createSegment7( current_state, goal_state, k ):
    
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds

    # method describes cubic or quintic scaling
    method = 5
    
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)

    return path_states

# Move the end-effector from putting the block down
# back to the standoff state
# current_state is end effector's configuration in the space frame
# goal state is the final standoff state 
def createSegment8( current_state, goal_state, k  ):
    
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    
    # Method describes cubic or quintic scaling
    method = 5
    
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)

    return path_states


# This method will generate the trajectory
# The initial configuration of the end-effector in the reference trajectory: Tse,initial.
# The cube's initial configuration: T_sc_initial.
# The cube's desired final configuration: T_sc_final.
# The end-effector's configuration relative to the cube when it is grasping the cube: T_ce_grasp.
# The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: T_ce_standoff.
# The number of trajectory reference configurations per 0.01 seconds: k. The value k is an integer 
# with a value of 1 or greater.
def TrajectoryGenerator( T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k):

    # List of order of the path entries 
    # r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz
    
    # This is the overall list of all the states on the trajectory
    path_states = np.array([])
    
    # Create the first segment of the path
    segment_1 = createSegment1(T_se_initial, T_sc_initial, T_ce_standoff, k)
    
    current_gripper_state = segment_1[len(segment_1) - 1]
    current_gripper_state =  convertLinearToSE3(current_gripper_state)
     
    standoff_state_initial = current_gripper_state.copy()  

    # Create the second segment
    segment_2 = createSegment2(current_gripper_state, T_sc_initial, T_ce_grasp, k) 
    
    current_gripper_state = segment_2[len(segment_2) - 1]
    current_gripper_state =  convertLinearToSE3(current_gripper_state)
   
    # This just fully closes the gripper
    segment_3 = createSegment3(current_gripper_state, k)
     
    # current_gripper_state is the same after the prior step
    # Move the end-effector back to the standoff state
    segment_4 = createSegment4( current_gripper_state, standoff_state_initial, k )
    
    current_gripper_state = segment_4[len(segment_4) - 1]
    current_gripper_state =  convertLinearToSE3(current_gripper_state)

    standoff_final = T_sc_final.copy()
    
    segment_5 = createSegment5( current_gripper_state,  T_sc_final, T_ce_standoff, k )
    
    current_gripper_state = segment_5[len(segment_5) - 1]
    current_gripper_state =  convertLinearToSE3(current_gripper_state)
    
    # Want to keep the orientation the same  
    # segment_6 = createSegment6( current_gripper_state, T_sc_final, k )
    standoff_state_final = current_gripper_state.copy()

    segment_6 = createSegment6( current_gripper_state, T_sc_final, T_ce_grasp, k )
    
    current_gripper_state = segment_6[len(segment_6) - 1]
    current_gripper_state =  convertLinearToSE3(current_gripper_state)

    segment_7 = createSegment7( current_gripper_state, current_gripper_state, k )

    current_gripper_state = segment_7[len(segment_7) - 1]
    current_gripper_state =  convertLinearToSE3(current_gripper_state)
    segment_8 = createSegment8( current_gripper_state, standoff_state_final, k )

    # Combine each segment so that we have the desired format for VREP
    # path_states = segment_1
    path_states = np.concatenate( (segment_1, segment_2) )
    path_states = np.concatenate( (path_states, segment_3) )
    path_states = np.concatenate( (path_states, segment_4) )
    path_states = np.concatenate( (path_states, segment_5) )
    path_states = np.concatenate( (path_states, segment_6) )
    path_states = np.concatenate( (path_states, segment_7) )
    path_states = np.concatenate( (path_states, segment_8) )

    return path_states

# This method updates data structures that we need 
# in order to plot the error at the end of the program
# x_err is the error vector at the current time
def updateError(x_err):

    # Copy the error vector 
    x_err_copy = x_err.copy()
    
    global allError_1
    global allError_2
    global allError_3
    global allError_4
    global allError_5
    global allError_6    
    
    # Store each element of the current error into the right array
    # We store it this way in order to plotting easier
    allError_1 = np.append(allError_1, x_err_copy[0] )
    allError_2 = np.append(allError_2, x_err_copy[1] )
    allError_3 = np.append(allError_3, x_err_copy[2] )
    allError_4 = np.append(allError_4, x_err_copy[3] )
    allError_5 = np.append(allError_5, x_err_copy[4] )
    allError_6 = np.append(allError_6, x_err_copy[5] )
    
   


# X is the current, ACTUAL state on the trajectory 
# X_d is the current DESIRED state on the trajectory
# X_d_next is the desired next state on the trajectory
# K_p is is the proportional gains matrix  
# K_i is the intergral gains matrix 
# dt is the timestep
# The return value is the twist we should execute 
# in order to guide the end-effector towards the desired path
def FeedbackControl( X, X_d, X_d_next, K_p, K_i, dt ):
    
    global runningError  # This is the integral error over time (0, now)
    global updateCount   # Describes if we should store the 
                         # Current point on trajectory's error
   

    # Compute the desired twist using Modern Robotics 13.4 and 13.5
    AdjointResult = mr.Adjoint( np.matmul( mr.TransInv( X ), X_d ) ) 
    
    v_d = (1.0 / dt) * ( mr.MatrixLog6(  np.matmul( mr.TransInv( X_d ), X_d_next )  )  )
    
    v_d = mr.se3ToVec( v_d )
    
    X_err =  mr.MatrixLog6( np.matmul( mr.TransInv( X ), X_d ) ) 
    X_err = mr.se3ToVec( X_err )
    
    # Store every kth error term
    if ( updateCount == 2):
        updateError(X_err)
        updateCount = 0
    else:
        updateCount = updateCount + 1
    
    # Increment our integral error term
    runningError = runningError + (X_err * dt) 
    
    return np.matmul(AdjointResult, v_d) + np.matmul( K_p, X_err ) + np.matmul( K_i, runningError ) 
        


# Testing setup 
X = np.array( [ [0.170, 0.0, -1 * 0.985, 0.0], [0.0, 1.0, 0.0, 0.0], [0.985, 0.0, 0.170, 0.0], [0.387, 0.0, 0.570, 1.0] ]   ).T

X_d = np.array( [ [0.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.5, 0.0, 0.5, 1.0] ] ).T 

X_d_next = np.array(  [ [0.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, 0.0],  [1.0, 0.0, 0.0, 0.0], [ 0.6, 0.0, 0.3, 1.0 ]  ]    ).T

dt = 0.01

# 65
K_p = 65 * np.identity(6)

# K_i = np.zeros( (6 , 6) ) 
K_i = 1.0 *  np.identity(6)


twist = FeedbackControl( X, X_d, X_d_next, K_p, K_i, dt )

J = np.array( [  [0.030, 0.0, -0.005, 0.002, -0.024, 0.012],  [ -0.030, 0.0, 0.005, 0.002, 0.024, 0.012 ], [-0.030, 0.0, 0.005, 0.002, 0.0, 0.012 ], 
    
    [ 0.030, 0.0, -0.005, 0.002, 0.0, 0.012], [-0.985, 0.0, 0.170, 0.0, 0.221, 0.0],  [0.0, -1.0, 0.0, -0.240, 0.0, -0.288 ],
    
    [0.0, -1.0, 0.0, -0.214, 0.0, -0.135 ], [ 0.0, -1.0, 0.0, -0.218, 0.0, 0.0 ], [ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 ]  ] ).T



J_pi = np.linalg.pinv( J, rcond = 0.001 ) 

result = np.matmul( J_pi, twist )


### End of testing setup 

# Set up the initial conditions
T_se_initial = np.array( [ [0.0, 0.0, 1.0, 0.2038],
                           [0.0, 1.0, 0.0, -0.0348],
                           [-1.0, 0.0, 0.0, 0.5],
                           [0.0, 0.0, 0.0, 1.0] ] )

# Construct a rotation matrix to inject error into the intial system
angle = np.pi / 6.0
rot_z = np.array( [        [ np.cos(angle),          np.sin(angle),               0.0,     0.115],

                           [-1 * np.sin(angle),      np.cos(angle),               0.0,        0.1],

                           [ 0.0,                     0.0,                        1.0,        0.2],

                           [0.0,                      0.0,                        0.0,        1.0] ] ) 

T_se_initial = np.matmul(T_se_initial, rot_z)


T_sc_initial = np.array( [ [1, 0, 0, 1.0],
                           [0, 1, 0, 0.0],
                           [0, 0, 1, 0.0],
                           [0, 0, 0, 1.0] ] )


T_sc_final = np.array( [ [0, 1, 0, 0.0],
                         [-1, 0, 0, -1.0],
                         [0, 0, 1, 0.0],
                         [0, 0, 0, 1.0] ] )



angle = np.pi / 2.0  
T_ce_standoff = np.array( [ [np.cos(angle),          0.0,               np.sin(angle),                     0.015],
                           
                           [0.0,                     1.0,               0.0,                              0.0],
                           
                           [ -1 * np.sin(angle),     0.0,               np.cos(angle),                    0.2],
                           
                           [0.0,                      0.0,              0.0,                             1.0] ] ) 

T_ce_grasp = T_ce_standoff.copy()
T_ce_grasp[2][3] = 0.03

k = 1

allStates = TrajectoryGenerator(  T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k )

N = 100

# Take a trajectory and construct the T_end_effector
# input is r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz
# Output is the SE(3) representation of the end-effector
def convertToMatrix( state ):

    myMatrix = np.zeros( (4, 4) )

    x = state[9]
    y = state[10]
    z = state[11]

    myMatrix[0][0] = state[0] 
    myMatrix[0][1] = state[1]
    myMatrix[0][2] = state[2]
    myMatrix[0][3] = x
   
    myMatrix[1][0] = state[3] 
    myMatrix[1][1] = state[4]
    myMatrix[1][2] = state[5]
    myMatrix[1][3] = y
    
    myMatrix[2][0] = state[6]
    myMatrix[2][1] = state[7]
    myMatrix[2][2] = state[8]
    myMatrix[2][3] = z
    
    myMatrix[3][0] = 0.0
    myMatrix[3][1] = 0.0
    myMatrix[3][2] = 0.0
    myMatrix[3][3] = 1.0

    return myMatrix


# # currentState
# [0, 2] = (x, y, theta)
# [3, 7] = arm configuration
# [7, 11] = wheel configuration

# This method takes in the current state of the system
# and returns the SE(3) matrix representation of the 
# current end effector in the space frame
def construct_T_Sb(currentState):
        
    angle = currentState[0]
    x = currentState[1]
    y = currentState[2]
    z = 0.0963

    myArray = np.array( [  [np.cos(angle), np.sin(angle), 0.0, 0.0 ], [-1 * np.sin(angle), np.cos(angle), 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0 ], [x, y, z, 1.0]     ]  ).T
    
    return myArray

# This method takes a jacobian matrix and a column index 
# and makes all the entries in the column zero
def createZeroColumn(jacobian, column):
    
    myArray = jacobian.copy()

    for i in range( len(jacobian)  ):
        myArray[i][column] = 0.0

    return myArray


# This method implements joint limits on the 
# arm in orde to help prevent many collisions
def checkCollisions( J1, currentState  ):
    
    # Joints are 3, 4, 5, 6, 7 of currentState
    
    newJ = J1.copy()
    
    returnValue = False

    
    if ( (currentState[3] < -2.3 ) or ( currentState[3] > 2.3 ) ):
        returnValue = True
        newJ = createZeroColumn(newJ, 0)

    # if( (currentState[4] < -1.0 ) or ( currentState[4] > 1.5 ) ):
    #    returnValue = True
    #    newJ = createZeroColumn(newJ, 1)        

    if( (currentState[5] < -1.5 ) or ( currentState[5] > -0.2 ) ):
        returnValue = True
        newJ = createZeroColumn(newJ, 2)        

    relativeAngle = currentState[6] - currentState[5]
    
    if ( ( relativeAngle < -1.117 ) or ( relativeAngle > 1 ) ):
        returnValue = True
        newJ = createZeroColumn(newJ, 3)

    #if ( (currentState[6] < -1.117 ) or ( currentState[6] > -0.2 ) ):   
    #    returnValue = True
    #    newJ = createZeroColumn(newJ, 3)     

    if(  (currentState[7] < -2.89 ) or ( currentState[7] > 2.89 )  ):   
        returnValue = True
        newJ = createZeroColumn(newJ, 4)


    return newJ, returnValue


############### Put it all together #####################

logging.basicConfig(filename = 'myLogFile2.log', filemode = 'w', level=logging.DEBUG, format = '%(name)s - %(levelname)s - %(message)s')
logging.info('Setting up Initial Conditions')

# Define the home configuration
M_home = np.array( [  [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0 ], [ 0.033, 0.0, 0.6546, 1.0] ] ).T

# Define the screw axes at the home configuration

b1 = np.array( [ 0.0, 0.0, 1.0, 0.0, 0.033, 0.0 ] )
    
b2 = np.array( [0.0, -1.0, 0.0, -0.5076, 0, 0 ] )
    
b3 = np.array( [ 0.0, -1.0, 0.0, -0.3526, 0.0, 0.0 ] )
    
b4 = np.array( [ 0.0, -1.0, 0.0, -0.2176, 0.0, 0.0 ] )
    
b5 = np.array( [ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 ] )

blist = np.array( [b1, b2, b3, b4, b5]  ).T

# This describes where the base of the arm is in the chassis's frame
# This is fixed as dthe chassis and the arm are rigidly attatched
T_b0 = np.array( [  [ 1.0, 0.0, 0.0, 0.0 ],  [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0],  [0.1662, 0.0, 0.0026, 1.0] ] ).T

# This is the H matrix for the robot with mecanmum wheels
r = 0.0475 # / * 1.5
l = 0.47 / 2.0
w = 0.30 / 2.0

# Do I need to use the r/4 for H(0)?
H_p_i = (r / 4.0) * np.array( [ [ -1 / (l + w), 1, -1 ], [ 1 / (l + w), 1, 1 ], [ 1 / (l + w), 1, -1], [ -1 / (l + w), 1, 1] ] ).T

F = H_p_i

# Generate the refrence trajectory
logging.info('Generating the Trajectory')
trajectory = TrajectoryGenerator( T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k )

X = T_se_initial

N = len(trajectory)

dt = 0.01

# Initialize the system
# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
# X = convertToMatrix( trajectory[0] )  

length = 13
allStates = np.zeros( (N, length) )

# Initial conditions
# J3 = 0.2
# J4 = -1.6
current_state = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   ]  )

logging.info('Running the Feedback controller')

# Run the controller 
for i in range( N - 1 ):
    
    if ( trajectory[i][12] == 1):
        grasp = True
    else:
        grasp = False 

    X_d = convertToMatrix( trajectory[i] )
    X_d_next = convertToMatrix( trajectory[i + 1] )

    twist = FeedbackControl( X,  X_d, X_d_next, K_p, K_i, dt )
    
    F6 = F
    F6 = np.concatenate( ( (np.zeros( (2, 4) ) ), F6), axis = 0 )    
    F6 = np.concatenate( ( F6, (np.zeros( (1, 4) ) ) ), axis = 0 )
    
    arm_theta = np.array( [ current_state[3], current_state[4], current_state[5], current_state[6], current_state[7] ]  )
    
    T_0e = mr.FKinBody(M_home, blist, arm_theta )
         
    intermediate1 = np.matmul( mr.TransInv( T_0e ) , mr.TransInv( T_b0  ) )  

    J_base = np.matmul( ( mr.Adjoint( intermediate1 )  ) , F6 )
    
    J_arm = mr.JacobianBody( blist, arm_theta )
    
    # Combine the two Jacobians into one 
    J_total = np.concatenate( (J_arm, J_base ), axis = 1)
     
    controls = np.matmul( np.linalg.pinv( J_total, rcond = 0.01  ) , twist)
    
    priorState = current_state.copy()
    
    # Get the state of the system after following the given controls vector
    # over a time period dt
    current_state = nextState(current_state, controls) 
    
    newJacobian, collision = checkCollisions( J_total, current_state  )  
    
    # For testing, set to false
    # collision = False
        
    # If the computed controls vector will result in a collisison,
    # then enforce joint limits and recompute the controls vector
    if (collision == True):
        # Use the new Jacobian to calculate the new state 
        
        controls = np.matmul( np.linalg.pinv( newJacobian, rcond = 0.01  ) , twist)

        current_state = nextState( priorState, controls)

    
    T_sb = construct_T_Sb( current_state ) 
    
    X = np.matmul(  T_sb  , np.matmul(T_b0 , T_0e) )
    
    
    # Add the actual point in trajectory space we end up at 
    # so that we can send it to VREP later
    allStates[i] = current_state   
   

    # Append the grasp variable to the array in the csv file      
    if ( grasp == True ):
        allStates[i][12] = 1.0
    else:
        allStates[i][12] = 0.0


# Save the csv file
np.savetxt("milestone3.csv", allStates, delimiter=",")


# Plot the error

logging.info('Plotting the Error')
plt.figure(1)

plt.subplot(1, 1, 1)
plt.title('Error Plot Over States of End-Effector')
plt.xlabel('state')

time = np.linspace(0, len(allError_1), len(allError_1) )


# Plot each error vector
plt.plot( time, allError_1, '.')
plt.plot( time, allError_2, '*')
plt.plot( time, allError_3, '.')
plt.plot( time, allError_4, '.')
plt.plot( time, allError_5, '.')
plt.plot( time, allError_6, '.')

plt.plot(time, allError_1, "-b", label = "Err_W_1")
plt.plot(time, allError_2, "-r", label = "Err_W_2")
plt.plot(time, allError_3, "-p", label = "Err_W_3")
plt.plot(time, allError_4, "-y", label = "Err_V_x")
plt.plot(time, allError_5, "-o", label = "Err_V_y")
plt.plot(time, allError_6, "-w", label = "Err_V_z")

plt.legend(loc="upper left")


plt.show()


#########################################################






