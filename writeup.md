## Project: 3D Motion Planning
### Marko Sarkanj
![Quad Image](./misc/enroute.png)

---


### Writeup / README

This writeup contains explanation of the implementation of the 3D Motion Planning project. This project has been completed as part of Self Flying Car Nanodegree program from Udacity.

### Explanation of the Starter Code

#### 1. Explanation of the `motion_planning.py` script
`motion_planning.py` script is advanced version of the `backyard_flyer_solution.py` script. That can be seen by drone behaving the same way in the simulator while running both scripts in their basic implementation. Drone moves from one waypoint to another in square on the map.

The first difference between the scripts is that the script `motion_planning.py` contains one additional state "PLANNING" and all the states are automatically enumerated with `auto()`. This additional state is necessary because of the additional step that calculates waypoints required to reach the desired goal on the map. The `backyard_flyer_solution.py` script does not require the planning step as the only functionality provided is drone moving in square, without path planning step.

`motion_planning.py`, line 16:
```python
class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

```

The second significant difference is the `path_plan()` function that the `motion_planning.py` scrip contains(line 114). This function gets executed before the `waypoint_transition()` step to calculate waypoints that are required to reach the goal. In the `path_plan()` function all the required positions are defined(home, current position, start and goal) and transferred from global coordinate system(latitude and longitude) to local coordinate system if needed. In the `path_plan()` function obstacle data is loaded, planning grid is created and the A* algorithm is executed. After executing the A* algorithm resulting waypoints are sent for processing.

#### 2. Explanation of the `planning_utils.py` script
`planning_utils.py` script contains `create_grid()` function that, as previously described, creates planning grid based on the obstacle data(2.5D map), safety distance and drone altitude.

Part of the `planning_utils.py` script is the `Action` class as well, which contains all the allowed moves on the planning grid as well as the costs related to them. `valid_actions()` function removes all the actions from the `Action` class that cannot be executed from the specific position on the planning grid, because of obstacles or end of grid.

The main part of the `planning_utils.py` script is the `a_star()` function that executes the A* algorithm on the previously created planning grid, start and goal locations. This function uses the `heuristic()` function as well, to assign cost to a specific A* queue based on the distance to the goal, in addition to branch cost. 

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


