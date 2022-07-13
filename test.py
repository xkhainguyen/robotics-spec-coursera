import modern_robotics as mr
import numpy as np
R = np.array([[1,0,0],[0, 0.707,0.707],[0, -0.707, 0.707]])
R1 = np.array([[0,0,1],[0,1,1],[1,0,0]])
print(mr.TestIfSO3(R))
print(R.T@R)