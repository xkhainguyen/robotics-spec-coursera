import numpy as np
import modern_robotics as mr

# X0 = np.array([1, 1])
# # print(np.linalg.inv([[1/(2*X0[0]), 0], [0, 1/(2*X0[1])]]))

# X1 = X0 - np.linalg.inv(np.array([[2*X0[0], 0], [0, 2*X0[1]]])).dot(np.array([X0[0]**2-9, X0[1]**2-4]))
# X2 = X1 - np.linalg.inv(np.array([[2*X1[0], 0], [0, 2*X1[1]]])).dot(np.array([X1[0]**2-9, X1[1]**2-4]))
# X3 = X2 - np.linalg.inv(np.array([[2*X2[0], 0], [0, 2*X2[1]]])).dot(np.array([X2[0]**2-9, X2[1]**2-4]))
# X4 = X3 - np.linalg.inv(np.array([[2*X3[0], 0], [0, 2*X3[1]]])).dot(np.array([X3[0]**2-9, X3[1]**2-4]))
# print("Q1\n", X1, X2, X3, X4)

Tsd = np.array([[-0.585, -0.811, 0, 0.076],
                [0.811, -0.585, 0, 2.608],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
thetalist0 = np.array([0.7875, 0.7875, 0.7875])
eomg = 0.001
ev = 0.0001

Blist = np.c_[np.array([0,1,0,3,0,0]),np.array([-1,0,0,0,3,0]), \
              np.array([0,0,0,0,0,1])]
Slist = np.c_[np.array([0, 0, 1, 0, 0, 0]), np.array([0, 0, 1, 0, -1, 0]), \
                np.array([0, 0, 1, 0, -2, 0])]
# print(Slist)
M = np.array([[1, 0, 0, 3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

print("Q2\n", mr.IKinSpace(Slist, M, Tsd, thetalist0, eomg, ev))