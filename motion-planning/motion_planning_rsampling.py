import argparse
import time
import msgpack
from enum import Enum, auto
import matplotlib.pyplot as plt

import numpy as np

from planning_utils_rsampling import create_grid, extract_polygons, create_graph, find_start_goal, a_star, random_sampling, heuristic
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

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
        
        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # DONE: retrieve current global position
        global_position = (self._longitude, self._latitude, self._altitude)

        # DONE: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
        
        print('Global home: {0}.\nGlobal position: {1}.\nLocal position: {2}.'.format(tuple(self.global_home), tuple(self.global_position),
                                                                         tuple(self.local_position)))
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        polygons = extract_polygons(data)
        
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}.\nEast offset = {1}.".format(north_offset, east_offset))


        start = (local_position[0], local_position[1], TARGET_ALTITUDE)
        grid_start = (int(start[0] - north_offset), int(start[1] - east_offset), TARGET_ALTITUDE)

        goal_lat = 37.794760
        goal_lon = -122.401120

        goal = global_to_local((goal_lon, goal_lat, 0), self.global_home)
        goal = (goal[0], goal[1], TARGET_ALTITUDE)
        
        grid_goal = (int(goal[0] - north_offset), int(goal[1] - east_offset), TARGET_ALTITUDE)

        polygons = extract_polygons(data)
        
        print('Grid Start: {0}.\nGrid Goal: {1}.'.format(grid_start, grid_goal))

        n_nodes = 200
        nodes = random_sampling(n_nodes, data, TARGET_ALTITUDE)
        
        n_conn_near = 10
    
        print('Number of nodes: {0}.\nMax number of edges per node: {1}.'.format(n_nodes, n_conn_near))

        graph = create_graph(nodes, n_conn_near, polygons)
        
        graph_start, graph_goal = find_start_goal(graph, start, goal)
        
        graph_start_ongrid = (int(graph_start[0] - north_offset), int(graph_start[1] - east_offset), graph_start[2])
        graph_goal_ongrid = (int(graph_goal[0] - north_offset), int(graph_goal[1] - east_offset), graph_goal[2])
        
        print('Graph Start: {0}.\nGraph Goal: {1}.'.format(graph_start_ongrid, graph_goal_ongrid))
        
        path, _ = a_star(graph, heuristic, graph_start, graph_goal)
        
        path.insert(0, start)
        path.append(goal)
        
        print('Path:', path)
                
        plt.imshow(grid, cmap='Greys', origin='lower')
        
        for n1 in graph.nodes:
            plt.scatter(n1[1] - east_offset, n1[0] - north_offset, c='red')
            
        for (n1, n2) in graph.edges:
            plt.plot([n1[1] - east_offset, n2[1] - east_offset], [n1[0] - north_offset, n2[0] - north_offset], 'black')
            
        path_pairs = zip(path[:-1], path[1:])
        for (n1, n2) in path_pairs:
            plt.plot([n1[1] - east_offset, n2[1] - east_offset], [n1[0] - north_offset, n2[0] - north_offset], 'green')
        
        plt.plot(grid_start[1], grid_start[0], 'rx')
        plt.plot(grid_goal[1], grid_goal[0], 'rx')

        plt.plot(graph_start_ongrid[1], graph_start_ongrid[0], 'go')
        plt.plot(graph_goal_ongrid[1], graph_goal_ongrid[0], 'go')
        
        plt.xlabel('NORTH')
        plt.ylabel('EAST')
        
        plt.show()
        
        # Convert path to waypoints
        waypoints = [[int(p[0]), int(p[1]), int(p[2]), int(0)] for p in path]
        
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

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=120)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
