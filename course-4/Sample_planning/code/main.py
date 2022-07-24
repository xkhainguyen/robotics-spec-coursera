"""
Sampling-Based Planning Project
Modern Robotics, Course 4: Robot Motion Planning and Control
"""
###############################################################################
# Imports
###############################################################################
import numpy as np
from numpy import Inf, isclose
norm = np.linalg.norm
import heapq            # heap queue

SPACE_LIM = 0.5
BOT_RADIUS = 0.01 
MAX_SIZE = 10000

def check_collision(q1, q2, obs):
    """
    Checks if the path q1->q2 collides with obstacles.
    Args:
      q1 (list [x, y])      : first node, cordinates x, y
      q2 (list [x, y])      : second node, cordinates x, y
      obs (ndarray (No, 3)) : circular obstacle information, No obstacles, 
                              cordinates x, y and radius
    Returns:
      isFree (Boolean)      : True if free, False otherwise  
    """
    isFree = True
    for circle in obs:
        xr, yr, r = circle[0], circle[1], circle[2]
        a = (q2[1]-q1[1])**2 + (q2[0]-q1[0])**2
        b = 2*(q1[0]-xr)*(q2[0]-q1[0]) + 2*(q1[1]-yr)*(q2[1]-q1[1])
        c = (q1[1]-yr)**2 + (q1[0]-xr)**2 - r**2
        delta = b**2 - 4*a*c
        if delta < 0:
            isFree = isFree and True
        elif delta == 0:
            t0 = -b/(2*a)
            x0 = q1[0] + (q2[0] - q1[0])*t0
            y0 = q1[1] + (q2[1] - q1[1])*t0
            if (q2[0] > x0 > q1[0]) or (q2[0] < x0 < q1[0]):
                if (q2[1] > y0 > q1[1]) or (q2[1] < y0 < q1[1]):
                    return False
        else:
            t1 = (-b + np.sqrt(delta))/(2*a)
            t2=  (-b - np.sqrt(delta))/(2*a)
            x1= q1[0] + (q2[0] - q1[0])*t1
            y1 = q1[1] + (q2[1] - q1[1])*t1
            x2= q1[0] + (q2[0] - q1[0])*t2
            y2 = q1[1] + (q2[1] - q1[1])*t2
            if (q2[0] > x1 > q1[0]) or (q2[0] < x1 < q1[0]):
                if (q2[1] > y1 > q1[1]) or (q2[1] < y1 < q1[1]):
                    return False
            if (q2[0] > x2 > q1[0]) or (q2[0] < x2 < q1[0]):
                if (q2[1] > y2 > q1[1]) or (q2[1] < y2 < q1[1]):
                    return False
    return isFree

def write_output():
    pass

def get_path(parent, root, goal):
    current = tuple(goal)
    path = [current]
    while not np.all(np.isclose(current, root, atol=1e-5)):
        current = parent[current]
        path.insert(0, current)
    return path

def rrt_plan(obstacle_file, root, goal, goal_rate=0.1):
    # Initialize search tree T
    T = [root]
    parent = {}
    # Load obstacle csv file
    obs_reader = np.loadtxt(obstacle_file, delimiter=',')

    while len(T) < MAX_SIZE:
        # Sampling with a rate of choosing goal config
        goal_sampling = np.random.choice([0, 1], p=[0.9, 0.1])
        if (goal_sampling == 1):
            q_sam = goal
        else:
            q_sam = np.random.uniform([-SPACE_LIM, -SPACE_LIM], 
                                        [SPACE_LIM, SPACE_LIM]) 
            # q_sam = [0.5, 0.5]      
        # Finding the nearest node to a sampled config
        distance = norm(np.subtract(T, q_sam), axis=1)
        nearest_idx = np.argmin(distance)
        q_nearest = T[nearest_idx]
        # Local straight-line planner
        isFree = check_collision(np.array(q_nearest), np.array(q_sam),
                                                            obs_reader)
        if isFree:
            T.append(q_sam)
            parent[tuple(q_sam)] = tuple(q_nearest)
            if np.all(np.isclose(q_sam, goal, atol=0.1)):
                return True, get_path(parent, root, goal)
    print(T)
    return False, root

def prm_plan(obstacle_file, root, goal):
    # Load obstacle csv file
    obs_reader = np.loadtxt(obstacle_file, delimiter=',')
    # Sampling

    # Creating edges

    # Searching the graph using A* search
       
    pass



if __name__=='__main__':
    root = [-0.5, -0.5]
    goal = [0.5, 0.5]
    obstacle_file = "course-4\Sample_planning\\results\obstacles.csv"

    _, path = rrt_plan(obstacle_file, root, goal, goal_rate=0.1)
    print("Path:\n",path)
    path = np.array(path)
    # np.savetxt("course-4\A_star_search\\results\path.csv", 
    #     path.reshape(1, len(path)), fmt='%s', delimiter=",") 
