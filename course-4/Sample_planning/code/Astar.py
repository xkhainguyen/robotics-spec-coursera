"""
A* Graph Search Project
Modern Robotics, Course 4: Robot Motion Planning and Control
"""
###############################################################################
# Imports
###############################################################################
import numpy as np
from numpy import Inf
import heapq            # heap queue

###############################################################################
# A* algorithm
###############################################################################

def a_star_search(root, goal):
    node_reader = np.loadtxt("course-4\Sample_planning\\results\\nodes.csv", 
                                                                delimiter=",")
    edge_reader = np.loadtxt("course-4\Sample_planning\\results\edges.csv", 
                                                                delimiter=",")
    # heuristic cost-to-go dictionary
    heuristic = {}  
    for node in node_reader:
        heuristic[int(node[0])] = node[3]
    # print("Heuristic:\n",heuristic)

    # create our graph using an adjacency list representation;
    # each key (node) maps to a value (a list), each list element includes 
    # a neighbor node and the corresponding cost
    graph = {k: [] for k in list(heuristic.keys())}
    for edge in edge_reader:
        id1, id2, cost = edge[0], edge[1], edge[2]
        graph[int(id1)].append((int(id2), cost))
        graph[int(id2)].append((int(id1), cost))

    # print("Graph:\n",graph)
    # number of nodes
    N = len(graph) + 1
    # set up priority queue with (est_total_cost, node)
    open = [(heuristic[root], root)]
    # dictionary for past_cost at each node
    past_cost = [Inf for _ in range(N)]
    past_cost[root] = 0
    # closed list to store visited nodes
    closed = []
    # dictionary to store parent of a node
    parent = {}

    # while there are nodes to process
    while len(open) > 0:
        # get the root, discard current distance
        _, current = heapq.heappop(open)
        # set the node to closed
        closed.append(current)
        # if the node is our goal, return SUCCESS and the path to current
        if current == goal:
            path = get_path(parent, root, goal)
            print("Path:\n",path)
            path = np.array(path)
            np.savetxt("course-4\Sample_planning\\results\path.csv", 
                    path.reshape(1, len(path)), fmt='%s', delimiter=",") 
            return True, path
        # check the distance and node and distance
        for nbr, cost in graph[current]:
            if nbr not in closed:
                tentative_past_cost = past_cost[current] + cost
                if tentative_past_cost < past_cost[nbr]:
                    past_cost[nbr] = tentative_past_cost
                    parent[nbr] = current
                    est_total_cost = past_cost[nbr] + heuristic[nbr]
                    heapq.heappush(open, (est_total_cost, nbr))
    return False, [root]

# Get the path from root to goal based on parent dictionary
def get_path(parent, root, goal):
    current = goal
    path = [current]
    while current != root:
        current = parent[current]
        path.insert(0, current)
    return path