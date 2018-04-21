# **Project: 3D Motion Planning**
___
## **Task 1:** Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.
This file contains four sections:
1. Task 1: Provide a Writeup / README.
2. Task 2: Explain the Starter Code. 
3. Task 3: Implementing Your Path Planning Algorithm.
4. BONUS: 
___

## **Task 2:** Explain the Starter Code.

The goal here is to explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`. I will divide the task in parts A and B, one for each file.

### **Part A:** Motion Planning Script.

The `motion_planning.py` script is divided in four main parts:

1. Imports.
2. States Class.
3. Motion Planning Class.
4. Code Evaluation.

#### A.1. Imports.
This part is self-explanatory. It imports all packages necessary for sound running of the code. 

#### A.2. States Class.
This part defines a class that inherits from `enum` class and enumerates possible states the drone can be in,  automatically associating constant values to them, and making possible to use them as symbolical variables.

#### A.3. Motion Planning Class.
This part defines a class that inherits from `Drone` class. It is subdivided in the following blocks:
1. Constructor Function.
2. Callback Functions.
3. Transition Functions.
4. Path Planning Function.
5. Start Function.

##### Constructor Function.
This block overwrites some of the superclass properties and register callbacks for each of the MsgIDs.

##### Callback Functions.
This block defines through `local_position_callback`, `velocity_callback` and `state_callback` how to react to different events using transition functions. One should note that the deadband, which permits transition between target positions, is defined here in `local_position_callback`. Other similar parameters that permits transition between states are also in this block.

##### Transition Functions.
This block defines each of the transition functions (`arming_transition`, `takeoff_transition`, `waypoint_transition`, `landing_transition`, `disarming_transition` and `manual_transition`) which command actions and change the vehicle's flight state.

##### Path Planning Function.
This block defines the path (sequence of waypoints) that the vehicle should follow. It receives target altitude, local position and goal position and, by processing obstacles data and safety distance from them, defines a safe path to reach goal position. It also send these waypoint coordinates to the simulator, in order to visualize them.

##### Start Function.
This block starts a log file, starts the API connection and, after mission completion, stops the log file.

### A.4. Code Evaluation.
This part of the code just runs everything.

___
### **Part B:** Planning Utilities Script
The `planning_utils.py` script is divided in four main parts:
1. Imports.
2. Create Grid Function.
3. Action Class and Valid Actions Function.
4. A* Functions.

#### B.1. Imports.
This part is self-explanatory. It imports all packages necessary for sound running of the code. 

#### B.2. Create Grid Function.
This function, `create_grid` creates a 2D grid with ones and zeros, where one are obstacles. The function asks for `data`, `drone_altitude` and `safety_distance`. Obstacle position is provided by `data`. The altitude in which drone is going to operate is given by `drone_altitude` and will help set obstacles in the grid only if their height is greater than `drone_altitude`. Finally, the obstacles in the grid are set greater than they actually are, to provide extra safety.  This offset is given by `safety_distance` value.

#### B.3. Action Class and Valid Actions Function.
The planning algorithm (A*) needs information on how the vehicle will move in the grid and this is provided by `Action` class. This class inherits from `Enum` class and sets possible movements: west, east, north, south (and diagonals). The class also sets properties `cost` and `delta`, which access the values of the movement. The function `valid_actions` checks if the movement that will be used by A* will not leave the grid boundaries or hit obstacles. If any of this consequences happens, the action is removed from `valid_actions` list.

#### B.4. A* Function.

The planning algorithm (A*) writes a list `path` of waypoints from `start` position to `goal` position. For each point in the path, beginning from `start` position, the algorithm adds valid motions and moves to the direction of lesser `queue_cost`. The `queue_cost` is composed by two costs: the first is `actions_cost` (that can be one to lateral motions and square root of 2 for diagonal motions)  and the other is the `heuristic_cost` that is the distance from `next_node`, i.e. node after motion, to the `goal`. This makes the algorithm to prefer paths that try to reach the goal as fast as possible.

___
## **Task 3:** Implementing Your Path Planning Algorithm.

### 1. Set your global home position
read the first line of the csv file, extract lat0 and lon0 as floating point values.
```
with open("colliders.csv") as f:
    lat_str, lon_str = f.readline().split(',')
    lat0, lon0 = float(lat_str.split(' ')[-1]), float(lon_str.split(' ')[-1])
    print(lat0, lon0)
```

use the self.set_home_position() method to set `global_home`
```
self.set_home_position(lon0, lat0, 0)
```

### 2. Set your current local position
Global position can be obtained by the GPS and barometer readings of the drone. A local variable `global_position` is then set to:
```
global_position = (self._longitude, self._latitude, self._altitude)
```
Now, `local_position` is obtained using `global_to_local()` that calculates NED coordinates by feeding `global_position` relative to `global_home`.
```
local_position = global_to_local(global_position, self.global_home)
```

### 3. Set grid start position from local position
Defining `start` position as `local_position`. To run the start point in A*, we need to change coordiantes to account for the grid offset. Thus, `start_goal` is simply obtained by taking first two elements of the goal array, subtracting offset values from the grid and transforming into integers and putting in a tuple.
```
start = local_position
grid_start = (int(start[0] - north_offset), int(start[1] - east_offset))
```
### 4. Set grid goal position from geodetic coords
Any latitude and longitude within the map can be chosen as goal, for this task I chose:
```
goal_lat = 37.794760
goal_lon = -122.401120
```
Now `goal` is obtained through `global_to_local()` as an array containing NED coordinates from a tuple containing EFEC coordinates, using `global_home` position as reference.
```
goal = global_to_local((goal_lon, goal_lat, 0), self.global_home)
```
Again, to run the goal in A*, we need to change coordiantes to account for the grid offset. Therefore, `grid_goal` is simply obtained by:
```
grid_goal = (int(goal[0] - north_offset), int(goal[1] - east_offset))
```
### 5. Modify A* to include diagonal motion (or replace A* altogether)
Before running A*, we need to modify `Action(Enum)` class and `valid_actions()` function. Regarding to `Action(Enum)` class, it is necessary to add new types of motion. Diagonal motion can be added using the following tuples:
```
NORTH_WEST = (-1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
```
These tuples give combined motion and add cost relative to diagonal length of a square of side equal to one. Adding constraints in `valid_actions()` function, as shown in code lines below, include checking if the combined motion do not hit obstacles or leave the grid boundaries.
```
if (x - 1 < 0 and y - 1 < 0) or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if (x + 1 > n and y + 1 > m) or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
if (x - 1 < 0 and y + 1 > m) or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if (x + 1 > n and y - 1 < 0) or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
```
The planned path is generated using the A* for grids (given). It returns a list of tuple waypoints `path`.
```
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
```
### 6. Cull waypoints 

The A* algorithm returns `path`, but the list of waypoints that come from the A* algorithm will often have unnecessary waypoints, either too close to each other, or collinear. To cull the waypoints, I have developed two functions, one for each method: collinearity test and ray tracing (Bresenham). The `path` can be fed to `prune_path()` for collinearity test or `bres_pruned_path`.
```
pruned_path = prune_path(path)
```
#### Collinearity pruning
The `prune_path()` function access two consecutive points to the i-th point in the path and check their collinearity (using `collinearity_check()` function shown below). If they are collinear, the algorithm removes the middle one, it keeps checking the two consecutive points to the i-th point until no collinearity is found and the reference point is iterated.

```
def prune_path(path):
    pruned_path = [p for p in path]
    i = 0
    while i < np.shape(pruned_path)[0] - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        if collinearity_check(p1 ,p2 , p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
pruned_path = prune_path(path)
```
The function `collinearity_check` checks the area of the triangle formed by three points. If the area is sufficiently close to zero, i.e. below a threshold `epsilon`, the function returns that the points are collinear. Function is given by:
```
def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((point(p1), point(p2), point(p3)), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon
```
where `point` is:
```
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)
```
#### Ray-tracing method: Bresenham pruning

The `bresenham_prune_path()` function access two consecutive points to the i-th point in the path. In this, Bresenham between the i-th and the (i+2)-th point is run and, if all squares in the grid given by the algorithm, are free from collision, the (i+1)-th waypoint can be removed. This is iterated through waypoints list.

```
def bresenham_prune_path(grid, path):
   pruned_path = [p for p in path]
   i = 0
   while i < np.shape(pruned_path)[0] - 2:
       p1 = pruned_path[i]
       p3 = pruned_path[i+2]
       if all((grid[pp] == 0) for pp in bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1]))):
           pruned_path.remove(pruned_path[i+1])
       else:
           i += 1
   return pruned_path
```

### 7. Execute the flight

Test configuration:
Global home latitude and longitude = (37.79248, -122.39745)
Global position = (-122.3974501, 37.7924788, 0.089)
Local position = (-0.17767039,  0.27920842, -0.08998074)
North offset = -316, east offset = -445
Local Start and Goal:  (315, 445) (566, 120)

Here's a picture of my planned path before pruning:
![Planned path](./planned_path.png)
Number of waypoints:  469

After pruning using collinearity:
![Pruned planned path](./pruned_planned_path.png)
Number of waypoints (collinear pruning):  27

After pruning using ray tracing (Bresenham):
![Bresenham pruned planned path](./bres_pruned_planned_path.png)
Number of waypoints (Bresenham pruning):  13

Second starting position:
![Second path: From Embarcadero](./embarcadero_route.png)

Third starting position:
![Third path: From South Neighborhood](./south_route.png)


## **BONUS IMPLEMENTATIONS** 

### Playing with Deadzones
Due!

### Playing with Vehicle Heading
Due!

### Representation: Medial Axis Graph
Due!

### Representation: Voronoi Graph
Due!

### Representation: Probabilistic Roadmap Graph
Due!

