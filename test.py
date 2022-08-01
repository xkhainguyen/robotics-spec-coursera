import numpy as np
import modern_robotics as mr

T0e = np.array([[0,-1,0,0],
                [1,0,0,3],
                [0,0,1,0],
                [0,0,0,1]])
Tb0 = np.array([[1,0,0,2],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]])
F6 = np.array([[0,0],[0,0],[-0.5,0.5],[0.5,0.5],[0,0],[0,0]])*0.5
ans = mr.Adjoint(np.linalg.inv(T0e)@np.linalg.inv(Tb0))@F6
Tbs = np.array([[0,1,0,-1],
                [-1,0,0,0],
                [0,0,1,0],
                [0,0,0,1]])
ans = mr.Adjoint(Tbs)@np.array([0,0,1,-3,0,0])
print(ans)
ans = mr.JacobianBody(Blist=np.array([0,0,1,0,3,0]), thetalist=[np.pi/2])
print(ans)