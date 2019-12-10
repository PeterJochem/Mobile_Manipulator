import modern_robotics as mr
import math
import numpy as np
import matplotlib.pyplot as plt

#############
# Globals
allStates = np.array([])

allError_1 = np.array( []  )
allError_2 = np.array( []  )
allError_3 = np.array( []  )
allError_4 = np.array( []  )
allError_5 = np.array( []  )
allError_6 = np.array( []  )

updateCount = 0

############

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
def nextState(currentState, controls, speedLimit = 12.5):

    # Check for illegal control values
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
    
    # Do I need to use (r / 4.0) ?
    H_p_i = (r / 4.0) * np.array( [ [ -1 / (l + w), 1, -1 ], [ 1 / (l + w), 1, 1 ], [ 1 / (l + w), 1, -1], [ -1 / (l + w), 1, 1] ] ).T 
        
    # wheel velocities are controls[5, 8]
    # No transpose?
    wheel_velocities = np.array( [controls[5] , controls[6], controls[7], controls[8] ] ).T

    delta_theta = ( wheel_velocities ) * dt
    
    # ?
    twist_b = ( ( np.matmul( H_p_i, delta_theta) ) )
    
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
        
        value1 = ( (v_b_x * np.sin(w_b_z) ) + (v_b_y * (np.cos(w_b_z - 1) ) ) ) / w_b_z  
        
        value2 = ( (v_b_y * np.sin(w_b_z) ) + (v_b_x * ( 1 - np.cos(w_b_z)  ) ) ) / w_b_z 
    
        delta_q_b = np.array( [ w_b_z, value1, value2 ]  ).T


    # print("")
    # print("delta_q_b is ")
    # print(delta_q_b)

    
    # Transoform delta_q_b to the space frame 
    # Rotate by the chassis's angle
    angle = currentState[0]
    rotationMatrix = np.array( [ [1, 0, 0], [0, np.cos(angle), np.sin(angle)  ], [ 0, -1 * np.sin(angle), np.cos(angle) ] ] ).T 

    q_s = np.matmul( rotationMatrix, delta_q_b)

    # print(" ")
    # print("q_s is ")
    # print(q_s)

    # Add the delta q to the old values
    new_state[0] = new_state[0] + q_s[0] 
    new_state[1] = new_state[1] + q_s[1]
    # new_state is [theta, x, y ... ]
    # q_s is [theta, x, y]
    new_state[2] = new_state[2] + q_s[2]
    
    # print("The angle value is " + str(new_state[0] ) )
    # print("The x value is " + str(new_state[1] ) )
    # print("The y value is " + str(new_state[2] ) )

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

# Take the T_se and convert it to the format that 
# the csv file wants
# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
# Input: currentState from nextState function
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
    totalSeconds = 200
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


    totalSeconds = 200
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

    standoff_final = T_sc_final.copy()
    # standoff_final = standoff_final.astype(float)
    # standoff_final[2][3] = standoff_final[2][3] + 0.5
    
    #T_ce_standoff = T_sc_final.copy()
    #T_ce_standoff[1][3] = 0.3

    # 2 x 3 = T_se
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


def updateError(x_err):
    x_err_copy = x_err.copy()
    
    global allError_1
    global allError_2
    global allError_3
    global allError_4
    global allError_5
    global allError_6    

    allError_1 = np.append(allError_1, x_err_copy[0] )
    allError_2 = np.append(allError_2, x_err_copy[1] )
    allError_3 = np.append(allError_3, x_err_copy[2] )
    allError_4 = np.append(allError_4, x_err_copy[3] )
    allError_5 = np.append(allError_5, x_err_copy[4] )
    allError_6 = np.append(allError_6, x_err_copy[5] )
    
   

runningError = 0

# Milestone 3 
def FeedbackControl( X, X_d, X_d_next, K_p, K_i, dt ):
    
    global runningError
    global updateCount
    
    # Use equation to calculate the twist
    result = np.matmul( mr.TransInv( X ), X_d )
    # print(result)
    
    # Is this the right adjoint? 
    # Now apply the bracket operation? 
    AdjointResult = mr.Adjoint( result ) 
    

    v_d = (1.0 / dt) * ( mr.MatrixLog6(  np.matmul( mr.TransInv( X_d ), X_d_next )  )  )
    
    v_d = mr.se3ToVec( v_d )
    
    X_err =  mr.MatrixLog6( np.matmul( mr.TransInv( X ), X_d ) ) 
    X_err = mr.se3ToVec( X_err )
    
    if ( updateCount == 20):
        updateError(X_err)
        updateCount = 0
    else:
        updateCount = updateCount + 1

    runningError = runningError + (X_err * dt) 
    
    result = np.matmul(AdjointResult, v_d) + np.matmul( K_p, X_err ) + np.matmul( K_i, runningError ) 
        
    # print("V is " + str(result) )
    return result


# Testing setup 
X = np.array( [ [0.170, 0.0, -1 * 0.985, 0.0], [0.0, 1.0, 0.0, 0.0], [0.985, 0.0, 0.170, 0.0], [0.387, 0.0, 0.570, 1.0] ]   ).T

X_d = np.array( [ [0.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.5, 0.0, 0.5, 1.0] ] ).T 

X_d_next = np.array(  [ [0.0, 0.0, -1.0, 0.0], [0.0, 1.0, 0.0, 0.0],  [1.0, 0.0, 0.0, 0.0], [ 0.6, 0.0, 0.3, 1.0 ]  ]    ).T

dt = 0.01


# 80, 50

# K_p = np.zeros( (6 , 6) )

K_p = 65 * np.identity(6)

# K_i = np.zeros( (6 , 6) ) 
# K_i = 5 *  np.identity(6)
K_i = 1 *  np.identity(6)


twist = FeedbackControl( X, X_d, X_d_next, K_p, K_i, dt )

J = np.array(  [   [0.030, 0.0, -0.005, 0.002, -0.024, 0.012],  [ -0.030, 0.0, 0.005, 0.002, 0.024, 0.012 ], [-0.030, 0.0, 0.005, 0.002, 0.0, 0.012 ], 
    
    [ 0.030, 0.0, -0.005, 0.002, 0.0, 0.012], [-0.985, 0.0, 0.170, 0.0, 0.221, 0.0],  [0.0, -1.0, 0.0, -0.240, 0.0, -0.288 ],
    
    [0.0, -1.0, 0.0, -0.214, 0.0, -0.135 ], [ 0.0, -1.0, 0.0, -0.218, 0.0, 0.0 ], [ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 ]  ] ).T



J_pi = np.linalg.pinv( J, rcond = 0.001 ) 

result = np.matmul( J_pi, twist )

print("The twist is ")
print(result)
print("")

print("")
print("The controls vector is " + str(result) )


### End of testing setup 

# Milestone 2
# Call TrajectoryGenerator 8 times and concatenate each segment

# Set up the initial conditions
T_se_initial = np.array( [ [1.0, 0, 0.0, 0.0],
                           [0, 1.0, 0, 0.0],
                           [0.0, 0, 1.0, 0.5],
                           [0, 0, 0, 1.0] ] )



T_sc_initial = np.array( [ [1, 0, 0, 1.0],
                           [0, 1, 0, 0.0],
                           [0, 0, 1, 0.0],
                           [0, 0, 0, 1.0] ] )


T_sc_final = np.array( [ [0, 1, 0, 0.0],
                         [-1, 0, 0, -1.0],
                         [0, 0, 1, 0.0],
                         [0, 0, 0, 1.0] ] )



angle = np.pi / 2.0  
T_ce_standoff = np.array( [ [np.cos(angle),          0.0,               np.sin(angle),                     0.01],
                           
                           [0.0,                     1.0,               0.0,                              0.0],
                           
                           [ -1 * np.sin(angle),     0.0,               np.cos(angle),                    0.2],
                           
                           [0.0,                      0.0,              0.0,                             1.0] ] ) 

T_ce_grasp = T_ce_standoff.copy()
T_ce_grasp[2][3] = 0.03

k = 1

# T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k
allStates = TrajectoryGenerator(  T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k )


# Save the csv file 
np.savetxt("milestone1.csv", allStates, delimiter=",")


N = 100

# Should be 13? one for gripper? 
allStates = np.zeros( (N, 12)  )  
currentState = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   )
controls = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, -1 * 10.0, 10.0, 10.0, -1 * 10.0]  )

for i in range(N):
        
    
    # allStates = np.concatenate( (allStates, currentState) )
    # CSV wants phi angle, x, y, joints, wheels
    allStates[i] = currentState
    
    currentState = nextState(currentState, controls)
    
# Save the file for the second milestone
np.savetxt("milestone2.csv", allStates, delimiter=",")


# Take a trajectory and construct the T_end_effector
# input is r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz
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

def construct_T_Sb(currentState):
        
    angle = currentState[0]
    x = currentState[1]
    y = currentState[2]
    z = 0.0963

    myArray = np.array( [  [np.cos(angle), np.sin(angle), 0.0, 0.0 ], [-1 * np.sin(angle), np.cos(angle), 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0 ], [x, y, z, 1.0]     ]  ).T


    
    return myArray

def createZeroColumn(jacobian, column):
    
    myArray = jacobian.copy()

    for i in range( len(jacobian)  ):
        myArray[i][column] = 0.0


    return myArray
    

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

# Define the home configuration
M_home = np.array( [  [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0 ], [ 0.033, 0.0, 0.6546, 1.0] ] ).T

# Define the screw axes at the home configuration

b1 = np.array( [ 0.0, 0.0, 1.0, 0.0, 0.033, 0.0 ] )
    
b2 = np.array( [0.0, -1.0, 0.0, -0.5076, 0, 0 ] )
    
b3 = np.array( [ 0.0, -1.0, 0.0, -0.3526, 0.0, 0.0 ] )
    
b4 = np.array( [ 0.0, -1.0, 0.0, -0.2176, 0.0, 0.0 ] )
    
b5 = np.array( [ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 ] )

# REMEBER TO TRANSPOSE THIS
blist = np.array( [b1, b2, b3, b4, b5]  ).T
# REMEMBER TO TRANSPOSE THIS

# This describes where the base of the arm is in the chassis's frame
# This is fixed as dthe chassis and the arm are rigidly attatched
T_b0 = np.array( [  [ 1.0, 0.0, 0.0, 0.0 ],  [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0],  [0.1662, 0.0, 0.0026, 1.0] ] ).T


# This is the H matrix for the robot with mecanmum wheels
r = 0.0475  # / 1.5
l = 0.47 / 2.0
w = 0.30 / 2.0

# Do I need to use the r/4 for H(0)?
H_p_i = (r / 4.0) * np.array( [ [ -1 / (l + w), 1, -1 ], [ 1 / (l + w), 1, 1 ], [ 1 / (l + w), 1, -1], [ -1 / (l + w), 1, 1] ] ).T

print("")
print(H_p_i)
print("")
print("")

F = H_p_i

# Generate the refrence trajectory
# T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k 
trajectory = TrajectoryGenerator( T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k )


# T_se_initial = np.array( [ [1.0, 0, 0.0, 0],
#                           [0, 1.0, 0, 0],
#                           [0.0, 0, 1.0, 0.5],
#                           [0, 0, 0, 1] ] )

X = T_se_initial


#X_start = T_se_initial
#X_end = X_start.copy()
#X_end[0][3] = X_end[0][3] + 2.0

# The total time in seconds
#Tf = 10
#N = 100
# method describes cubic or quintic scaling
#method = 5
# For testing, just compute the first segment
#path_states = mr.CartesianTrajectory(X_start, X_end, Tf, N, method)
# Take the 3-D array and put it into a 2-D form
#path_states = process_array(path_states, 0)


# trajectory = path_states 

# Test it by giving it a short and simple trajectory to follow

N = len(trajectory)

# Define K_p
# Define K_i

dt = 0.01

# Initialize the system
# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
# Is this how I initialize this?
# X = convertToMatrix( trajectory[0] )  




#print("")
#print("X is ")
#print(X)
# print("")

N = len(trajectory)
# N = 4

length = 13
allStates = np.zeros( (N, length) )

# Initial conditions
J3 = 0.2
J4 = -1.6
current_state = np.array( [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   ]  )

#print("")
#print("F is ")
#print(F)
#print("")

print("")
print("The trajectory is ")
print(trajectory[0] )
print("")
print( convertToMatrix( trajectory[0] )  )
print("")
print("")

all_error = []

# X = np.array( [  [], [], [], []    ]    )

# N = 1

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
    
    # print("")
    # print("F is ")
    # print(F)
    # print("")

    arm_theta = np.array( [ current_state[3], current_state[4], current_state[5], current_state[6], current_state[7] ]  )
    
    T_0e = mr.FKinBody(M_home, blist, arm_theta )
    
     
    intermediate1 = np.matmul( mr.TransInv( T_0e ) , mr.TransInv( T_b0  ) )  

    J_base = np.matmul( ( mr.Adjoint( intermediate1 )  ) , F6 )
    
    # JacobianBody(Blist, thetalist)
    # thetaList would be just the thetalist of the arm
    J_arm = mr.JacobianBody( blist, arm_theta )
    
    
    J_total = np.concatenate( (J_arm, J_base ), axis = 1)
    
    controls = np.matmul( np.linalg.pinv( J_total, rcond = 0.01  ) , twist)
    
    priorState = current_state.copy()
    current_state = nextState(current_state, controls) 
    
    newJacobian, collision = checkCollisions( J_total, current_state  )  

    # collision = False

    if (collision == True):
    #    print("Computing new")
        # Use the new Jacobian to calculate the new state 
        
        controls = np.matmul( np.linalg.pinv( newJacobian, rcond = 0.01  ) , twist)

        current_state = nextState( priorState, controls)

    
    T_sb = construct_T_Sb( current_state ) 
    
    X = np.matmul(  T_sb  , np.matmul(T_b0 , T_0e) )
    
    # Convert X to the desired csv list 
    # Add the gripper state to this??
    allStates[i] = current_state   
   

    
    if ( grasp == True ):
        allStates[i][12] = 1.0
    else:
        allStates[i][12] = 0.0


# Save the csv file
np.savetxt("milestone3.csv", allStates, delimiter=",")



# Store every kth X_err 
# Plot the error

plt.figure(1)

plt.subplot(5,1,1)
plt.title('Error Plot over states of end-effector')
plt.xlabel('state')

time = np.linspace(0, len(allError_1), len(allError_1) )

# print(allError )


# Plot each error vector
plt.plot( time, allError_1, '.')
plt.plot( time, allError_2, '*')
plt.plot( time, allError_3, '.')
plt.plot( time, allError_4, '.')
plt.plot( time, allError_5, '.')
plt.plot( time, allError_6, '.')



plt.show()


#########################################################






