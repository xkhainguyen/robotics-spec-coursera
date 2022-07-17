import numpy as np
import modern_robotics as mr
np.set_printoptions(precision=2, suppress=True)

print("Q5:\n",mr.QuinticTimeScaling(5, 3))

N = 10
Xstart = np.identity(4)
Xend = np.array([[0,0,1,1],[1,0,0,2],[0,1,0,3],[0,0,0,1]])
Tf = 10
X = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method=3)
print("Q6:\n",np.array2string(X[8], separator=','))

N = 10
Xstart = np.identity(4)
Xend = np.array([[0,0,1,1],[1,0,0,2],[0,1,0,3],[0,0,0,1]])
Tf = 10
X = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method=5)
print("Q7:\n",np.array2string(X[8], separator=','))