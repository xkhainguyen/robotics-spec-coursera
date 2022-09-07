import numpy as np
np.set_printoptions(precision=2, suppress=True)
pi = np.pi
ri = 0.25
gamma_i = 0
H = np.zeros((4,3))
w = np.array([[2,2],[-2,2],[-2,-2],[2,-2]])
beta = np.array([-pi/4, pi/4, 3*pi/4, -3*pi/4])
for i in range(len(w)):
    A = np.array([1/ri, np.tan(gamma_i)])
    B = np.array([[np.cos(beta[i]), np.sin(beta[i])],
                    [-np.sin(beta[i]), np.cos(beta[i])]])
    C = np.array([[-w[i,1], 1, 0], [w[i,0], 0, 1]])
    H[i,:] = A@B@C

print(H)

# print(H@[1,2,3])

V = np.array([10,10,-10,-10])
print(np.linalg.pinv(H)@V)
