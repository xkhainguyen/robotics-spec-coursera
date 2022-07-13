import numpy as np
import modern_robotics as mr

W1 = 109e-3
W2 = 82e-3
L1 = 425e-3
L2 = 392e-3
H1 = 89e-3
H2 = 95e-3

Tsd = np.array([[0, 1, 0, -0.5],
                [0, 0, -1, 0.1],
                [-1, 0, 0, 0.1],
                [0, 0, 0, 1]])
thetalist0 = np.array([-0.1, -1.5, -1.5, -2, 2, -1])
eomg = 0.001
ev = 0.0001
Blist = np.c_[np.array([0,1,0,W1+W2,0,L1+L2]),np.array([0,0,1,H2,-L1-L2,0]), \
              np.array([0,0,1,H2,-L2,0]),np.array([0,0,1,H2,0,0]),\
              np.array([0,-1,0,-W2,0,0]),np.array([0,0,1,0,0,0])]
M = np.array([[-1,0,0,L1+L2], [0,0,1,W1+W2], [0,1,0,H1-H2], [0, 0, 0, 1]])

ans = mr.IKinBodyIterates(Blist, M, Tsd, thetalist0, eomg, ev)