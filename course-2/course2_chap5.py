import modern_robotics as mr
import numpy as np

Jb = np.array([[1, 1, 1, 1],
               [-1, -1, -1, 0],
               [1+1+1, 1+1, 1, 1]])
Fb = np.array([10, 10, 10])
print("Q1", Jb.T.dot(Fb))

Slist = np.c_[np.array([0,0,1,0,0,0]),np.array([1,0,0,0,2,0]), \
              np.array([0,0,0,0,1,0])]
thetalist = [np.pi/2, np.pi/2, 1]
Js = mr.JacobianSpace(Slist, thetalist)
print("Q3", np.array_str(Js, precision=2, suppress_small=True))

Blist = np.c_[np.array([0,1,0,3,0,0]),np.array([-1,0,0,0,3,0]), \
              np.array([0,0,0,0,0,1])]
thetalist = [np.pi/2, np.pi/2, 1]
Jb = mr.JacobianBody(Blist, thetalist)
print("Q4", np.array_str(Jb, precision=2, suppress_small=True))

Jv = np.array([[-0.105, 0, 0.006, -0.045, 0, 0.006, 0],
               [-0.889, 0.006, 0, -0.844, 0.006, 0, 0],
               [0, -0.105, 0.889, 0, 0, 0, 0]])
A = Jv.dot(Jv.T)       
print(np.linalg.eig(A))       


