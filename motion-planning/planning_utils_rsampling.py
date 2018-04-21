import numpy as np
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue
from sklearn.neighbors import KDTree
import networkx as nx

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def extract_polygons(data):
    """
    Returns polygons and their heights from obstacle data
    """
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        north_coord_min = north - d_north
        north_coord_max = north + d_north
        east_coord_min = east - d_east
        east_coord_max = east + d_east
        
        
        corners = [(np.int(north_coord_min), np.int(east_coord_min)),
                   (np.int(north_coord_max), np.int(east_coord_min)),
                   (np.int(north_coord_max), np.int(east_coord_max)),
                   (np.int(north_coord_min), np.int(east_coord_max))]
        
        height = alt+d_alt

        p = Polygon(corners)
        
        polygons.append([p, height])
        
    return polygons

def collides(polygon, point):   
    """
    Returns true if point is inside polygon (obstacle) or if the altitude of
    the point is smaller than polygon height
    """    
    p = Point(point[0], point[1])
    p_height = point[2]
    collision = False
    
    if polygon[0].contains(p) and polygon[1] >= p_height:
        collision = True       
    
    return collision

def random_sampling (num_samples, data, TARGET_ALTITUDE):
    """
    Returns `num_samples` random points outside obstacles and between certain
    range in x, y (relative to grid boundaries) and z (lowest and highest
    admissible altitude).
    """    
    polygons = extract_polygons(data)

    xmin = np.min(data[:, 0] - data[:, 3])
    xmax = np.max(data[:, 0] + data[:, 3])

    ymin = np.min(data[:, 1] - data[:, 4])
    ymax = np.max(data[:, 1] + data[:, 4])

    zmin = TARGET_ALTITUDE
    zmax = 10 # Limit the z axis for the visualization'

    poly_tree = KDTree(data[:,0:2], leaf_size = 2)
        
    to_keep = []
    
    while len(to_keep) != num_samples:
    
        remaining_num_samples = num_samples - len(to_keep)
        xvals = np.random.uniform(xmin, xmax, remaining_num_samples)
        yvals = np.random.uniform(ymin, ymax, remaining_num_samples)
        zvals = np.random.uniform(zmin, zmax, remaining_num_samples)

        samples = list(zip(xvals, yvals, zvals))

        for point in samples:

            query_point = np.array([point[0], point[1]]).reshape(1, -1)

            _, idx = poly_tree.query(query_point)

            nearest_polygon = polygons[int(idx)]

            if not collides(nearest_polygon, point):
                to_keep.append(point)

        print("Generated {0} / {1} samples so far".format(len(to_keep), num_samples))
    
    return to_keep

def can_connect(p1, p2, polygons):
    """
    Returns true if an edge can be drawn from p1 to p2 without crossing any
    polygon (obstacle)
    """    
    line = LineString([p1, p2])
    
    for p, height in polygons:
        
        if p.crosses(line) and height >= min(p1[2], p2[2]):
            return False

    return True

def create_graph(nodes, k, polygons):
    """
    Creates graph from nodes trying to create edges between each node and its k
    nearest neighbors
    """
    graph = nx.Graph()
    
    tree = KDTree(nodes)
    
    for p1 in nodes:
        
        idxs = tree.query([p1], k, return_distance=False)[0]  
    
        for idx in idxs:
            
            p2 = nodes[int(idx)]
            
            if p2 == p1:
                continue            

            if can_connect(p1, p2, polygons):
                dist = np.linalg.norm(np.subtract(p1, p2))
                #print(dist)
                graph.add_edge(p1, p2, weight=int(dist))
    
    return graph

def find_start_goal(graph, start, goal):
    """
    Returns closest graph nodes to start and goal positions
    """        
    start = np.array(start)
    goal = np.array(goal)
    
    dist_to_start = []
    dist_to_end = []
        
    for node in graph.nodes:
        dist_to_start.append(np.linalg.norm(np.subtract(node, start)))
        dist_to_end.append(np.linalg.norm(np.subtract(node, goal)))
    
#    print(np.argmin(dist_to_start))
#    print(np.argmin(dist_to_end))
    near_start = list(graph.nodes)[np.argmin(dist_to_start)]
    near_goal = list(graph.nodes)[np.argmin(dist_to_end)]

    return near_start, near_goal

def heuristic(n1, n2):
    """
    Returns Euclidean distance between two points
    """
    return np.linalg.norm(np.array(n1) - np.array(n2))
    #return np.sqrt((n2[1] - n1[1]) ** 2 + (n2[0] - n1[0]) ** 2)

def a_star(graph, heuristic, start, goal):
    """
    Modified A* to work with NetworkX graphs.
    """    
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost