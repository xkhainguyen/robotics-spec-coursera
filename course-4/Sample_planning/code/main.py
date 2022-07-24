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
np.set_printoptions(precision=2, suppress=True)
np.random.seed(0) 

SPACE_LIM = 0.5
BOT_RADIUS = 0.03

# RRT planner parameters
RRT_MAX_SIZE = 50

# PRM planner parameters
PRM_MAX_SIZE = 50
PRM_MAX_NBR  = 10

def check_collision_node(q, obs):
    """
    Checks if node q within obstacles
    Args:
      q (list [x, y])      : first node, cordinates x, y
      obs (ndarray (No, 3)) : circular obstacle information, No obstacles, 
                              cordinates x, y and radius
    Returns:
      isFree (Boolean)      : True if free, False otherwise      
    """
    distance = norm(np.subtract(obs[:, 0:2], q), axis=1)
    isFree = np.all(distance > (obs[:,2] + BOT_RADIUS))
    return isFree

def check_collision_path(q1, q2, obs):
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
        xr, yr, r = circle[0], circle[1], circle[2]/2
        # d2r = norm([xr-q1[0], yr-q1[1]) - BOT_RADIUS
        a = (q2[1]-q1[1])**2 + (q2[0]-q1[0])**2
        b = 2*(q1[0]-xr)*(q2[0]-q1[0]) + 2*(q1[1]-yr)*(q2[1]-q1[1])
        c = (q1[1]-yr)**2 + (q1[0]-xr)**2 - (r+BOT_RADIUS)**2
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

def write_output(nodes, edges, path):
    np.savetxt("course-4\Sample_planning\\results\\nodes.csv", 
        nodes, fmt='%s', delimiter=",") 
    np.savetxt("course-4\Sample_planning\\results\edges.csv", 
        edges, fmt='%s', delimiter=",") 
    path = np.array(path)
    np.savetxt("course-4\Sample_planning\\results\path.csv", 
        path.reshape(1, len(path)), fmt='%s', delimiter=",") 

def get_path(parent, root, goal):
    """
    Trace back the parent dictionary to get the solution
    Args:
      parent (dict)     : parent dictionary
      root (int)        : id of root node
      goal (int)        : id of goal node
    Returns:
      path (list)       : list of points to form a path from root to goal
    """
    current = goal
    path = [current]
    while current != root:
        current = parent[current]
        path.insert(0, current)
    return path

def rrt_plan(obstacle_file, root, goal, goal_rate=0.1):
    # Initialize search tree 
    i = 1   # node id
    nodes = np.array([[int(i), root[0], root[1], 0.]])
    edges = np.empty((0,3))
    parent = {}
    # Load obstacle csv file
    obs_reader = np.loadtxt(obstacle_file, delimiter=',')

    while len(nodes) < RRT_MAX_SIZE:
        # Sampling with a rate of choosing goal config
        goal_sampling = np.random.choice([0, 1], p=[1-goal_rate, goal_rate])
        if (goal_sampling == 1):
            q_sam = goal
        else:
            q_sam = np.random.uniform([-SPACE_LIM, -SPACE_LIM], 
                                        [SPACE_LIM, SPACE_LIM]) 
  
        # Finding the nearest node to a sampled config
        distance = norm(np.subtract(nodes[:, 1:3], q_sam), axis=1)
        nearest_idx = np.argmin(distance)
        q_nearest = nodes[nearest_idx][1:3]
        # Local straight-line planner
        isFree = check_collision_path(np.array(q_nearest), np.array(q_sam),
                                                            obs_reader)                                                    
        if isFree:
            i += 1
            # add new node to the search tree
            nodes = np.append(nodes, [[i, q_sam[0], q_sam[1], 0.]], axis=0)
            edges = np.append(edges, [[nearest_idx+1, i, np.min(distance)]], axis=0)
            parent[i] = nearest_idx+1
            if np.all(np.isclose(q_sam, goal, atol=0.01)):
                path = get_path(parent, 1, i)
                write_output(nodes, edges, path)
                return True, path
    return False, root

def prm_plan(obstacle_file, root, goal):
    # Load obstacle csv file
    obs_reader = np.loadtxt(obstacle_file, delimiter=',')
    # Sampling
    R = np.empty((0,4))
    id = 0
    while len(R) > PRM_MAX_SIZE:
        q = np.random.uniform([-SPACE_LIM, -SPACE_LIM], 
                              [SPACE_LIM, SPACE_LIM]) 
        if check_collision_node(q, obs_reader):
            id +=1
            cost = norm(np.subtract(q, goal))
            R = np.append(R, [[id, q[0], q[1], cost]], axis=0)
    # Creating edges

    # Searching the graph using A* search
       
    return False



if __name__=='__main__':
    root = [-0.5, -0.5]
    goal = [0.5, 0.5]
    obstacle_file = "course-4\Sample_planning\\results\obstacles.csv"

    # _, path = rrt_plan(obstacle_file, root, goal, goal_rate=0.1)
    # print("Path:\n",path)

    prm_plan(obstacle_file, root, goal)