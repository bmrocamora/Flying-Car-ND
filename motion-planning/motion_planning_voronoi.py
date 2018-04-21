import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils_voronoi import create_grid_and_edges, create_voronoi_graph, find_start_goal, a_star, heuristic, bresenham_prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import matplotlib.pyplot as plt


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 2.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        with open("colliders.csv") as f:
            lat_str, lon_str = f.readline().split(',')
            lat0, lon0 = float(lat_str.split(' ')[-1]), float(lon_str.split(' ')[-1])
            print(lat0, lon0)
        
        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # DONE: retrieve current global position
        global_position = (self._longitude, self._latitude, self._altitude)

        # DONE: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
        
        print(global_position)
        print(local_position)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        graph = create_voronoi_graph(edges)
       
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # DONE: convert start position to current position rather than map center
        start = local_position
        grid_start = (int(start[0] - north_offset), int(start[1] - east_offset))

        # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # DONE: adapt to set goal as latitude / longitude position and convert
        goal_lat = 37.794760
        goal_lon = -122.401120

        goal = global_to_local((goal_lon, goal_lat, 0), self.global_home)
        grid_goal = (int(goal[0] - north_offset), int(goal[1] - east_offset))

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        
        graph_start, graph_goal = find_start_goal(graph, grid_start, grid_goal)
        print('Local Graph Start and Goal: ', graph_start, graph_goal)
        
        path, _ = a_star(graph, heuristic, graph_start, graph_goal)
        
        print(path)
        
        path.insert(0, grid_start)
        path.append(grid_goal)
        
#        plt.figure()
#        plt.imshow(grid, origin='lower', cmap='Greys') 
#
#        for e in edges:
#            p1 = e[0]
#            p2 = e[1]
#            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
#            
#        pp = np.array(path)
#        plt.plot(pp[:, 1], pp[:, 0], 'b')
#        plt.scatter(pp[:, 1], pp[:, 0])
#        
#        plt.xlabel('EAST')
#        plt.ylabel('NORTH')
#        plt.show()
        
        pruned_path = bresenham_prune_path(grid, path)
        
#        plt.figure()
#        plt.imshow(grid, origin='lower', cmap='Greys') 
#
#        for e in edges:
#            p1 = e[0]
#            p2 = e[1]
#            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
#            
#        pp = np.array(pruned_path)
#        plt.plot(pp[:, 1], pp[:, 0], 'b')
#        plt.scatter(pp[:, 1], pp[:, 0])     
#        
#        plt.xlabel('EAST')
#        plt.ylabel('NORTH')
#        plt.show()
        
        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), int(TARGET_ALTITUDE), int(0)] for p in pruned_path]
        
        for i in range(len(waypoints)-1):
            wp1 = waypoints[i]
            wp2 = waypoints[i+1]
            waypoints[i+1][3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
        
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO_LATER: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
