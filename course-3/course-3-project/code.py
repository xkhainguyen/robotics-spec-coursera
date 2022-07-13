import numpy as np
import modern_robotics as mr
# np.set_printoptions(precision=2, suppress=True)

print("\n------ Course 3 Project ------")

# ====================
# Setup simulation
# ====================

TASK = 1        # select first and second tasks of the project

pi = np.pi
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

g = np.array([0,0,-9.81])   # gravity acc

dt = 1./1000     # delta_t: integration interval
tf = 3 if TASK == 0 else 5          # total simulation time

N = int(tf/dt) + 1   # no of state index, no of steps is N-1   

theta = np.zeros((N, 6))
dtheta = np.zeros_like(theta)
if TASK == 1:
    theta[0] = [0, -1, 0, 0, 0, 0]
ddtheta = np.zeros_like(theta)
tau = np.zeros(6)   # zero torques 
Ftip = np.zeros(6)  # zero ee wrench

# ====================
# Main loop
# ====================

for i in range(N-1):
    ddtheta[i] = mr.ForwardDynamics(theta[i], dtheta[i], tau, g, Ftip, 
                                                Mlist, Glist, Slist)
    theta[i+1], dtheta[i+1]  = mr.EulerStep(theta[i], dtheta[i], 
                                                        ddtheta[i], dt)

# save to csv
if TASK == 0:
    np.savetxt('course-3/course-3-project/simulation1.csv', theta, 
                                                delimiter=",", fmt = '%.3f')
else:
    np.savetxt('course-3/course-3-project/simulation2.csv', theta, 
                                                delimiter=",", fmt = '%.3f')


print(f"Task {TASK+1} finished!")                                                