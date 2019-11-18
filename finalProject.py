import modern_robotics as mr
import math
import numpy as np

# List of states of robot's joints
overallThetaList = []

# Robot's parameters
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]


# ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes):

# 300 because we evaluate every 0.01 second for 3 seconds
taumat = np.zeros( (300, 6) )
dt = 0.01
g = np.array([0, 0, -9.8])
Ftipmat = np.ones((np.array(taumat).shape[0], 6))

# Initial joint positions
thetaList = np.array([0, 0, 0, 0, 0, 0])
# Initial joint velocities 
dthetaList = np.array([0, 0, 0, 0, 0, 0])

outList = mr.ForwardDynamicsTrajectory(thetaList, dthetaList, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, 8)

# Save the csv file 
np.savetxt("partA.csv", outList[0], delimiter=",")

# Do part B

# 500 because we evaluate every 0.01 second for 5 seconds
taumat = np.zeros( (500, 6) )
dt = 0.01
g = np.array([0, 0, -9.8])
Ftipmat = np.ones((np.array(taumat).shape[0], 6))

# Initial joint positions
thetaList = np.array([0, -1, 0, 0, 0, 0])
# Initial joint velocities
dthetaList = np.array([0, 0, 0, 0, 0, 0])

outList = mr.ForwardDynamicsTrajectory(thetaList, dthetaList, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, 8)

# Save the csv file 
np.savetxt("partB.csv", outList[0], delimiter=",")



