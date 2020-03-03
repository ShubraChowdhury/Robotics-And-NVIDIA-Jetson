## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

There are two main files that I have modified 
###### 1.`motion_planning.py` this file differs from  backyard_flyer_solution.py  where the sates [MANUAL ,ARMING ,TAKEOFF ,WAYPOINT ,LANDING ,DISARMING ] has been set to manual where as in motion planning [MANUAL,ARMING,TAKEOFF,WAYPOINT,LANDING,DISARMING,PLANNING] have been set to auto.  backyard_flyer_solution.py has a defines path /waypoint[[10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]] where is in motion planning we are setting up the start point , goal are set and the path is derived by A* and further is pruned.

path_prune(self,path): is used to prune path it checks for collinearity, where If the 3 points are in a line remove the 2nd point. The 3rd point now becomes and 2nd point and the check is redone with a new third point on the next iteration.

Prune details :
    def point(self,p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)
    
    def collinearity_check(self,p1, p2, p3, epsilon=1e-6):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def path_prune(self,path):
        pruned_path = [p for p in path]
        # TODO: prune the path!
        
        i = 0
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i+1])
            p3 = self.point(pruned_path[i+2])
            
            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if self.collinearity_check(p1, p2, p3):
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        return pruned_path


###### 2.  `planning_utils.py` creates grid, and takes multiple action those are  valid actions,  a_star, heuristic (using np.linalg.), heuristic_func(np.sqrt ..). In A* method i have implemented  diagonal motions with a cost of sqrt(2)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

planning utils also validated obstruction for diagonals and removes it in addition to  removing the obstructed NORTH, SOUTH, EAST , WEST

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
   


Following are the heuristics implemented in planning utils
def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))



def heuristic_func(position, goal_position):
    return np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)


And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

Lat and Long are defined in the file as first line of the file `colliders.csv' values are reflected as lat0 37.792480, lon0 -122.397450. I have parsed the file to read the first line and used split to row by coma and replace the text value. Once the value has been retrived the same has been applied to home position , below is the code snippet.
    
    with open('colliders.csv') as lat_long:
            latLon = lat_long.readline().rstrip().replace('lat0','').replace('lon0 ','').split(',')
            lat0 = float(latLon[0])
            lon0 = float(latLon[1])
    
    self.set_home_position(lon0, lat0, 0)



And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

In order to get the local position i have to get the global position  which has Long , Lat and alt and this is obtained by self._longitude, self._latitude, self._altitude. Next you will have to get the global home (self.global_home).Once you have both you can use global_to_local method of frame_utils.py () https://github.com/udacity/udacidrone/blob/master/udacidrone/frame_utils.py 
global_to_local(global_position, global_home), Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position. Returns  numpy array of the local position [north, east, down]

    global_position = [self._longitude, self._latitude, self._altitude]
    current_local_position = global_to_local(global_position,self.global_home)
    print('global home {0}, global position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
                                                                    
    print('current_local_position {0}'.format(current_local_position))
    
    Following are the values I have received:
    #global home [-122.3957515   37.7932817    0.       ]
    #global position [-1.22397449e+02  3.77924796e+01 -2.80000000e-02]
    # local position [-0.03823025  0.05000484  0.02808646]
    #ccurrent_local_position [-0.04403942  0.05311325  0.028     ]



Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!
Starting position in this case is not the map center rather the current position adjusted for North and East offset.
In order to get the starting position first we read the data from the colliders.csv then get the offsets by passing the data tru create_grid method ( from planning utils file).
Below is the code for grid_start.


        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # North offset = -316, east offset = -445
        
        
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        
        print(" Grid start --initial that is map center :",grid_start)
        #Grid start --initial that is map center : (315, 445)
        
        # TODO: convert start position to current position rather than map center
        
        grid_start = (int(current_local_position[0]-north_offset), int(current_local_position[1]-east_offset))
        
        print("Grid start after adding map center ", grid_start)
        #Grid start after adding map center  (315, 445)
        
#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.
For grid goal as given a fixed number 10 has been added to grid goal , this i have used just to test the system if the car reacheas a desired position. 
Once satisfied i have intruduced an arbitary Lon, Lat and Alt in this case i have considered a space on the map with no obstruction and close to Davis Street these are -122.3957445   ,37.792657    ,0.00 .  Using the global_to_local method i have converted the same to goal local positionand subsequently converting to grid goal. Below is the code snippet.

        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
        print("Grid Goal plus 10 from mao center ",grid_goal)
        #Grid Goal plus 10 from mao center  (326, 455)
        
              
        # TODO: adapt to set goal as latitude / longitude position and convert
        # Arbitary goal position
        goal_local_position = global_to_local([-122.3957445   ,37.792657    ,0.00   ],self.global_home)
        # Converting Lat Long
        grid_goal = (int(goal_local_position[0]-north_offset), int(goal_local_position[1]-east_offset))
        

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

This is the declaration i have added for diagonal motion 
    #add diagonal motions with a cost of sqrt(2) to your A* implementation
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

following has been added to def valid_actions(grid, current_node): method to check 

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

 If the 3 points are in a line remove the 2nd point. The 3rd point now becomes and 2nd point  and the check is redone with a new third point on the next iteration. Something subtle here but we can mutate   `pruned_path` freely because the length of the list is check on every iteration.  See the below mentioned code snippet
            
 def point(self,p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)
        
 def collinearity_check(self,p1, p2, p3, epsilon=1e-6):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

def path_prune(self,path):
        pruned_path = [p for p in path]
        # TODO: prune the path!
        
        i = 0
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i+1])
            p3 = self.point(pruned_path[i+2])
            
            
            if self.collinearity_check(p1, p2, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        return pruned_path

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


