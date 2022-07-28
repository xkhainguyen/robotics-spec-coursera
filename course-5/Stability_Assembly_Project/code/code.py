import numpy as np
from scipy import optimize
linprog = optimize.linprog
pi = np.pi
np.set_printoptions(precision=2, suppress=True)

#==============================================================================

def check_stability(mass, contacts):
    """
    Determines if an assembly of planar rigid bodies, in frictional contact 
    with each other, can remain standing in gravity (where gravity acts in 
    the - y direction), or if the assembly must collapse.
    Args:
      mass (list, (N, 3))       : static mass properties of N bodies, the (x,y)
                                location and total mass [kg]
      contacts (list, (n, 6))   : description of n contacts. Each contact 
                                consists of a list of the two bodies involved
                                in the contact (0 means stationary ground, 
                                1 means body 1, 2 means body 2, etc.); the (x,y) 
                                location of the contact; the contact normal 
                                direction into the first body involved in the 
                                contact; and the friction coefficient μ at the 
                                contact. 
    Returns:
      (Boolean)                 : True if the assembly is stable; 
                                False otherwise
    """  
    queue = []    
    M = len(mass)                           # number of bodies
    N = len(contacts)                       # number of contacts
    F = np.zeros((M*3, N*2))
    m = 0                                   # column index
    n = 0                                   # row index
    for body_i in range(1, M+1):            # for each body, do
        for contact in contacts:            # for each contact, do
            if contact[0] == body_i:        # if contact of body i, do
                if (contact[1] == 0) or (contact[1] > body_i - 1):
                    # if not in contact with previous body, do
                    muy = contact[-1]
                    alpha = np.arctan(muy)
                    ni1 = [np.cos(contact[4] + alpha), np.sin(contact[4] + alpha)] 
                    ni2 = [np.cos(contact[4] - alpha), np.sin(contact[4] - alpha)] 
                    mi1 = np.cross(contact[2:4], ni1)
                    mi2 = np.cross(contact[2:4], ni2)
                    Fi = np.c_[np.r_[mi1, ni1], np.r_[mi2, ni2]]

                    if body_i < N and contact[1] == body_i+1:
                        # save the index of two-body contact
                        queue.append((m, n))    

            elif contact[0] == body_i - 1 and body_i > 1 and contact[1] == body_i:
                # these wrenches are equal to minus corresponding wrenches
                index = queue.pop(0)
                Fi = -F[index[0]:index[0]+3,index[1]:index[1]+2]
            else:   # not contact of body i
                Fi = np.zeros((3,2))
            
            F[m:m+3, n:n+2] = Fi
            n = n + 2
        m = m + 3
        n = 0
        # print((F))                
    # After F is found, set up linear programming problem
    f = np.ones((N*2, 1))
    A = -np.identity(N*2)
    b = np.zeros((N*2, 1))     # b = zeros not ones
    Aeq = F
    beq = np.empty((M*3,1))
    for i in range(M):
        ni = [0,-mass[i][2]]
        mi = np.cross(mass[i][0:2], ni)
        beq[3*i:3*i+3,0] = -np.r_[mi,ni]
    k = linprog(f, A, b, Aeq, beq, method='interior-point')                    

    return k.success

#==============================================================================

if __name__ == '__main__':
    mass1 = [[25, 35, 2],
            [66, 42, 5]]
    contacts1 = [[1, 0, 0, 0, pi/2, 0.1],
                [1, 2, 60, 60, pi, 0.5],
                [2, 0, 60, 0, pi/2, 0.5],
                [2, 0, 72, 0, pi/2, 0.5]]
    mass2 = [[25, 35, 2],
            [66, 42, 10]]
    contacts2 = [[1, 0, 0, 0, pi/2, 0.5],
                [1, 2, 60, 60, pi, 0.5],
                [2, 0, 60, 0, pi/2, 0.5],
                [2, 0, 72, 0, pi/2, 0.5]]
    mass3 = [[-2.1,-1.2,5],
            [2.1,-1.2,5],
            [5,-4,5]]
    contacts3 = [[1,0,-3.7,-2.4,pi/2,0.5],
                [1,0,-2,-2.4,pi/2,0.5],
                [1,0,-1,-0.8,3.6818,0.5],
                [1,0,-1.8,0.8,3.6818,0.5],
                [1,0,1.8,0.8,2.6014,0.5],
                [1,0,1,-0.8,2.6014,0.5],
                [1,0,2,-2.4,pi/2,0.5],
                [1,0,3.7,-2.4,pi/2,0.5]]
  
    mass4 = [[-2.1,-1.2,10],
            [2.1,-1.2,10],
            [5,-4,10]]
    contacts4 = [[1,0,-3.7,-2.4,pi/2,0.9],
                [1,0,-2,-2.4,pi/2,0.9],
                [1,2,-1,-0.8,3.6818,0.9],
                [1,2,-1.8,0.8,3.6818,0.9],
                [2,3,1.8,0.8,2.6014,0.9],
                [2,3,1,-0.8,2.6014,0.9],
                [3,0,2,-2.4,pi/2,0.9],
                [3,0,3.7,-2.4,pi/2,0.9]]
    mass = mass3                # CHANGE MASS TO ONE OF FOUR CASES
    contacts = contacts3        # CHANGE CONTACTS TO ONE OF FOUR CASES
    print("With mass:\n", mass)
    print("and contacts:\n", contacts)
    print(">> The assembly is stable: ", check_stability(mass, contacts))