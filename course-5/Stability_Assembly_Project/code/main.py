import numpy as np
from scipy import optimize
linprog = optimize.linprog
pi = np.pi

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
    

    return False

#==============================================================================

if __name__ == '__main__':
    mass = [[25, 35, 2],
            [66, 42, 5]]
    contacts = [[2, 0, 72, 0, pi/2, 0.5],
                [2, 0, 60, 0, pi/2, 0.5],
                [1, 0, 0, 0, pi/2, 0.1],
                [1, 2, 60, 60, -pi, 0.5]]
    print("The assembly is stable: ", check_stability(mass, contacts))