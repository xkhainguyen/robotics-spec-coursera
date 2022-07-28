import numpy as np
from scipy import optimize
linprog = optimize.linprog
pi = np.pi

#==============================================================================

def test_form_closure(contacts):
    """
    Determines if a planar rigid body, subject to a specified set of stationary
    point contacts, is in form closure.
    Args:
      contacts (list, (m,n))        : list of m point contacts (n = 3) each 
                                    specified by the (x,y) contact location and
                                    the direction of the contact normal (in 
                                    radians, relative to the positive x-axis)
    Returns:
      (Boolean)                     : True if in form closure, False otherwise
    """
    # form_closure = False
    F = np.empty((3,0))
    for contact in contacts:
        ni = [np.cos(contact[2]), np.sin(contact[2])]
        mi = np.cross(contact[0:2], ni)
        Fi = np.r_[mi, ni]
        F = np.append(F, Fi.reshape(3,1), axis=1)
    f = np.ones((len(contacts), 1))
    A = -np.identity(len(contacts))
    b = -np.ones((len(contacts), 1))
    Aeq = F
    beq = np.zeros((3,1))
    k = linprog(f, A, b, Aeq, beq, method='interior-point')
    return k.success
#==============================================================================

if __name__ == '__main__':
    contacts1 = [[0, 0, pi], [0,0,-pi/2], [2,1,0], [2,1,pi/2]]
    contacts2 = [[0, 0, pi], [0,0,-pi/2], [2,0,0], [2,0,pi/2]]
    print(">> Form Closure: ",test_form_closure(contacts2))

