import modern_robotics as mr
import numpy as np

# omg3 = np.array([0, 1, 0])
# o3 = np.array([(np.sqrt(3)+1), 0, -1])
# v3 = np.cross(-omg3, o3)
# print(v3)

# omg4 = np.array([0, 1, 0])
# o4 = np.array([np.sqrt(3)+2, 0, np.sqrt(3)-1])
# v4 = np.cross(-omg4, o4)
# print(v4)

# S5 = np.array([0, 0, 0, 0, 0, 1])

# omg6 = np.array([0, 0, 1])
# o6 = np.array([np.sqrt(3)+2, 0, np.sqrt(3)+1])
# v6 = np.cross(-omg6, o6)
# print(v6)

# omg1_ = np.array([0, 0, 1])
# o1_ = np.array([-np.sqrt(3)-1, 0, -np.sqrt(3)-1])
# v1_ = np.cross(-omg1_, o1_)
# print(v1_)

# omg2_ = np.array([0, 1, 0])
# o2_ = np.array([-np.sqrt(3)-1, 0, -np.sqrt(3)-1])
# v2_ = np.cross(-omg2_, o2_)
# print(v2_)

# omg3_ = np.array([0, 1, 0])
# o3_ = np.array([-1, 0, -np.sqrt(3)-2])
# v3_ = np.cross(-omg3_, o3_)
# print(v3_)

omg4_ = np.array([0, 1, 0])
o4_ = np.array([0, 0, -2])
v4_ = np.cross(-omg4_, o4_)
print(v4_)

# B5 = np.array([0, 0, 0, 0, 0, 1])

M = np.array([[1,0,0,3.732],[0,1,0,0],[0,0,1,2.732],[0,0,0,1]])
S = np.array([[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,0,1,-0.732,0,0],[-1,0,0,0,0,-3.732],[0,1,2.732,3.732,1,0]])
B = np.array([[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,2.732,3.732,2,0,0],[2.732,0,0,0,0,0],[0,-2.732,-1,0,1,0]])
Ts = mr.FKinSpace(M, S, np.array([-np.pi/2, np.pi/2, np.pi/3, -np.pi/4, 1, np.pi/6]))
Tb = mr.FKinBody(M, B, np.array([-np.pi/2, np.pi/2, np.pi/3, -np.pi/4, 1, np.pi/6]))
print(np.array_str(Ts,precision=2, suppress_small=True))
print(np.array_str(Tb,precision=2, suppress_small=True))