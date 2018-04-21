#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 14 19:21:54 2018

@author: bernardo
"""
from queue import PriorityQueue
import numpy as np
import numpy.linalg as LA
from scipy.spatial import Voronoi
from bresenham import bresenham

#import sys
#!{sys.executable} -m pip install -I networkx==2.1
#import pkg_resources
#pkg_resources.require("networkx==2.1")
import networkx as nx

def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
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
    # Initialize an empty list for Voronoi points
    points = []
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

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    graph = Voronoi(points)
    
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))
    return grid, edges, int(north_min), int(east_min)

def create_voronoi_graph(edges):
    """
    Creates Voronoi graphs from edges
    """    
    graph = nx.Graph()
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        dist = LA.norm(np.array(p2) - np.array(p1))
        graph.add_edge(p1, p2, weight=dist)
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
        p = np.array(node)
        dist_to_start.append(np.linalg.norm(np.subtract(p, start)))
        dist_to_end.append(np.linalg.norm(np.subtract(p, goal)))
    
#    nodes = np.array(graph.nodes)
#    near_start = nodes[np.argmin(dist_to_start)]
#    near_goal = nodes[np.argmin(dist_to_end)]
     
    near_start = list(graph.nodes)[np.argmin(dist_to_start)]
    near_goal = list(graph.nodes)[np.argmin(dist_to_end)]
    
    near_start = tuple(near_start)
    near_goal = tuple(near_goal)    
    
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
                # get the tuple representation
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

def bresenham_prune_path(grid, path):
    """
    Prune path using Bresenham
    """
    # Start variables 
    pruned_path = [p for p in path]
    i = 0
    # Checks list of waypoints
    while i < np.shape(pruned_path)[0] - 2: # Length of pruned path changes every loop
        p1 = pruned_path[i]
        p3 = pruned_path[i+2]
        # If all points returned by Bresenham between points i and i+2 are not obstacles
        if all((grid[pp] == 0) for pp in bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1]))):
            # Remove i+1 point
            pruned_path.remove(pruned_path[i+1])
        else:
            # Go to next waypoint
            i += 1
    return pruned_path
