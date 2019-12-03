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

# Input data 
# currentState
# [0, 2] = (x, y, theta)
# [3, 7] = arm configuration
# [7, 11] = wheel configuration

# controls 
# [0, 4] = arm torques 
# [5, 8] = wheel velocities
def nextState(currentState, controls):

    dt = 0.01
    
    new_state = currentState.copy()
        
    # Euler Integration 
    # new arm joint angles = (old arm joint angles) + (joint speeds) * delta t 
    new_state[3] = currentState[3] + ( controls[0] * dt )
    new_state[4] = currentState[4] + ( controls[1] * dt )
    new_state[5] = currentState[5] + ( controls[2] * dt )
    new_state[6] = currentState[6] + ( controls[3] * dt )
 
    # new wheel angles = (old wheel angles) + (wheel speeds) * delta 2 
    new_state[8] = currentState[8] + (controls[5] * dt)   
    new_state[9] = currentState[9] + (controls[6] * dt)
    new_state[10] = currentState[10] + (controls[7] * dt)
    new_state[11] = currentState[11] + (controls[8] * dt)
     
    # new chassis configuration is obtained from odometry, as described in Chapter 13.4 
    r = 0.0475 
    l = 0.47 / 2.0
    w = 0.30 / 2.0
    
    H_p_i = np.array( [ [ -1 / (l + w), 1, -1 ], [ 1 / (l + w), 1, 1 ], [ 1 / (l + w), 1, -1], [ -1 / (l + w), 1, 1] ] ).T 
        
    # wheel velocities are controls[5, 8]
    # No transpose?
    wheel_velocities = np.array( [controls[5] , controls[6], controls[7], controls[8] ] ).T

    delta_theta = ( wheel_velocities ) * dt

    twist_b = ( (r / 4.0) * ( np.matmul( H_p_i, delta_theta) ) )
    
    # print("The twist is ")
    # print( twist_b )
    

    # The twist is not 1 x 6
    w_b_z = twist_b[0]
    v_b_x = twist_b[1]
    v_b_y = twist_b[2]

    # print("")
    # print("v_b_x is ")
    # print(v_b_x)
    
    delta_q_b = None

    if ( abs(w_b_z) < 0.01 ):
        delta_q_b = np.array( [ 0, v_b_x, v_b_y ]   )
    else:
        
        value1 = (v_b_x * np.sin(w_b_z) + v_b_y * (np.cos(w_b_z - 1) ) ) / w_b_z  
        
        value2 = (v_b_y * np.sin(w_b_z) + v_b_x * ( 1 - np.cos(w_b_z)  )  ) / w_b_z 
    
        delta_q_b = np.array( [ w_b_z, value1, value2 ]  )


    # print("")
    # print("delta_q_b is ")
    # print(delta_q_b)

    
    # Transoform delta_q_b to the space frame 
    # Rotate by the chassis's angle
    angle = currentState[2]
    rotationMatrix = np.array( [ [1, 0, 0], [0, np.cos(angle), np.sin(angle)  ], [ 0, -1 * np.sin(angle), np.cos(angle) ] ] ).T 

    q_s = np.matmul( rotationMatrix, delta_q_b)

    # print(" ")
    # print("q_s is ")
    # print(q_s)

    # Add the delta q to the old values
    new_state[0] = new_state[0] + q_s[0] 
    new_state[1] = new_state[1] + q_s[1]
    new_state[2] = new_state[2] + q_s[2]
    

    # print("")
    # print(q_s[1])

    # print("The new state is ")
    # print(new_state)
    
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


def createSegment1( T_se_initial, T_sc_initial, T_ce_standoff, k ):
    

    # Need to compute the desired orientation R
    # R_se_at standoff = R_sc * R_ce
    # Can I multiply the SE(3) matrices too?
    # T_se_standoff = np.matmul(  T_ce_standoff.copy(), T_sc_initial.copy()) 
    T_se_standoff = np.matmul(T_sc_initial.copy(), T_ce_standoff.copy() )
    
    #T_se_standoff = T_ce_standoff
    # replace the offset 
    #T_se_standoff[0][3] = T_sc_initial[0][3]
    #T_se_standoff[1][3] = T_sc_initial[1][3]
    #T_se_standoff[2][3] = T_sc_initial[1][3] + 0.5

    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = T_se_initial.copy()

    X_end = T_se_standoff.copy()
    # print(X_end)
    # X_end = T_sc_initial.copy()
    # Add a few cms to the z coordinate 
    
    X_end = X_end.astype(float)
    # X_end[2][3] = X_end[2][3] + 0.5
    

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)
    
    # print(path_states)
    # print(X_end)

    return path_states

# This segment will 
def createSegment2(T_se_initial, T_sc_initial, T_ce_grasp, k ):

    totalSeconds = 100
    N = float(totalSeconds) / float(k)
    
    # T_ce_grasp_new = np.matmul(  T_ce_grasp.copy(), T_sc_initial.copy())
    T_goal = np.matmul( T_sc_initial.copy(), T_ce_grasp.copy() )

    X_start = T_se_initial.copy()

    X_end = T_goal.copy()

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)
    
    return path_states

# Take the linear array and construct the SE3 representation 
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

# This method closes the gripper
def createSegment3( current_state, k ):
    
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = current_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
   
   # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)

    return path_states

# Moves the robot from grasping the block to 
def createSegment4( current_state, standoff_state, k ):
        
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = standoff_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)
    
    return path_states

def createSegment5(  current_state, standoff_state, T_sc_final, k):

    #  current_gripper_state, T_ce_standoff, T_sc_final, k )
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    goal_state = np.matmul( standoff_state, T_sc_final )
    
    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)

    return path_states

def createSegment6( current_state, T_sc_final, T_ce_grasp, k ):

    # createSegment6( current_gripper_state, T_sc_final, T_ce_grasp, k )
    goal_state = np.matmul( T_sc_final, T_ce_grasp )


    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 1)

    return path_states

def createSegment7( current_state, goal_state, k ):
    
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)

    return path_states

# Move the end-effector from putting the block down
# back to the standoff state
def createSegment8( current_state, goal_state, k  ):
    
    totalSeconds = 100
    N = float(totalSeconds) / float(k)

    X_start = current_state.copy()

    X_end = goal_state.copy()

    # The total time in seconds
    Tf = totalSeconds
    # method describes cubic or quintic scaling
    method = 5
    # For testing, just compute the first segment
    path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)

    # Take the 3-D array and put it into a 2-D form
    path_states = process_array(path_states, 0)

    return path_states




# This method will generate the trajectory
# The initial configuration of the end-effector in the reference trajectory: Tse,initial.
# The cube's initial configuration: Tsc,initial.
# The cube's desired final configuration: Tsc,final.
# The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp.
# The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: Tce,standoff.
# The number of trajectory reference configurations per 0.01 seconds: k. The value k is an integer 
# with a value of 1 or greater.
def TrajectoryGenerator( T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k):

    # List of order of the path entries 
    # r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz
    
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

    # standoff_final = T_sc_final.copy()
    # standoff_final = standoff_final.astype(float)
    # standoff_final[2][3] = standoff_final[2][3] + 0.5
    segment_5 = createSegment5( current_gripper_state, T_ce_standoff, T_sc_final, k )
    

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


    # Combine each segment
    # path_states = segment_1
    path_states = np.concatenate( (segment_1, segment_2) )
    path_states = np.concatenate( (path_states, segment_3) )
    path_states = np.concatenate( (path_states, segment_4) )
    path_states = np.concatenate( (path_states, segment_5) )
    path_states = np.concatenate( (path_states, segment_6) )
    path_states = np.concatenate( (path_states, segment_7) )
    path_states = np.concatenate( (path_states, segment_8) )


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


T_sc_initial = np.array( [ [1, 0, 0, 1],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1] ] )

T_sc_final = np.array( [ [0, 1, 0, 0],
                         [-1, 0, 0, -1],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1] ] )




T_ce_grasp = np.array( [ [-1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, -1, 0],
                         [0, 0, 0, 1] ] )
   


T_ce_standoff = np.array( [ [-1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, -1, 0.5],
                           [0, 0, 0, 1] ] )


k = 1

# T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k
allStates = TrajectoryGenerator(  T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k )


# Save the csv file 
np.savetxt("milestone1.csv", allStates, delimiter=",")


# u = (10,10,10,10)
N = 100
allStates = np.zeros( (N, 12)  )  
currentState = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   )
controls = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, -1 * 10.0, 10.0, -1 * 10.0,  10.0]  )

for i in range(N):
    
    # allStates = np.concatenate( (allStates, currentState) )
    allStates[i] = currentState
    currentState = nextState(currentState, controls)
    

# Save the file for the second milestone
np.savetxt("milestone2.csv", allStates, delimiter=",")





