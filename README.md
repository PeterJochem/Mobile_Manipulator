# How To Run My Code 
Install VREP, the Modern Robotics library, and clone my repo

Run ```python3 MobileManipulator.py``` and this will generate a csv file that can be loaded in VREP

# Description

## Path Planning
Given eight configurations indicating the relationship between end-effector, cube and world frame under different conditions, generate a reference trajectory for the gripper on frame {e}. The output is written to a cvs file containing 13 attributes: r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state

## Kinematics Simulation
Given the current configuration of youBot (Chassis phi, Chassis x, Chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, Gripper), joints speed and wheel speed, return the next configuration of the robot after a short time dt(default as 0.01s).

## Forward Control
Given the current, next and actual end-effector configurations, PI controller gains, return the commanded end-effector twist V and the error list of each joint. Given the joint angles, Body Jacobians and several other configurations, return the Jacobian of robot arm and base.


# Results
![Robot in Action]( https://github.com/PeterJochem/Mobile_Manipulator/blob/jointLimits/Kuka_In_Action.png ).


The full video of the robot in action 

[![](http://img.youtube.com/vi/oar5Ui4zqd0/0.jpg)](http://www.youtube.com/watch?v=oar5Ui4zqd0 "Mobile Manipulator")

