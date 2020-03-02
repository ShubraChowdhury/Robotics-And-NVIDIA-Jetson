import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid,heuristic_func
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
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
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

################################CODE FROM A-Star-City-Solution.ipynb Planning Lesson 3 #############################
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
#########################################################################
    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 2

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        
        #lat0 37.792480, lon0 -122.397450
        
        with open('colliders.csv') as lat_long:
            latLon = lat_long.readline().rstrip().replace('lat0','').replace('lon0 ','').split(',')
            lat0 = float(latLon[0])
            lon0 = float(latLon[1])
        print("LAT LONG ",latLon, "LAT  =",lat0, "LON =", lon0)
            
        #LAT LONG  [' 37.792480', ' -122.397450'] LAT  = 37.79248 LON = -122.39745


        # TODO: set home position to (lon0, lat0, 0)
        
        self.set_home_position(lon0, lat0, 0)
        
        
        # TODO: retrieve current global position
        
# ==============================TEST for GLOBAL POS IN LAT LONG ALT ===============================================
#         current_global_position = self.global_position
#         global_position = current_global_position
#         global_position_lat_long = [self._longitude, self._latitude, self._altitude]
#         
#         print("CURRENT GLOBAL POSITION ",current_global_position)
#         
#         #CURRENT GLOBAL POSITION  [-1.22395752e+02  3.77932813e+01  8.10000000e-02]
# 
#         
#         print(" GLOBAL POSITION ",global_position)
#         #  GLOBAL POSITION  [-1.22395752e+02  3.77932813e+01  8.10000000e-02]
# 
#         
#         print(" GPS LAT LONG ",global_position_lat_long)
#         #  GPS LAT LONG  [-122.3957515, 37.7932813, 0.081]
# =============================================================================

        global_position = [self._longitude, self._latitude, self._altitude]
 
        # TODO: convert to current local position using global_to_local()
        
# ========================================= TEST  ====================================
#         #global_position==>[longitude (degree), latitude (degree), altitude (meter)]==>	the current GPS position of the drone
#         # https://udacity.github.io/udacidrone/docs/drone-attributes.html
#         #Global positions are defined as [longitude, latitude, altitude (positive up)]. 
#         #Local reference frames are defined [North, East, Down (positive down)] 
#         
#         #https://github.com/udacity/udacidrone/blob/master/udacidrone/frame_utils.py
#         #frame_utils.py ==> 
#             # global_to_local(global_position, global_home): 
#             # Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position.
#             #Returns:         numpy array of the local position [north, east, down]
#             
#             
#             #local_to_global(local_position, global_home):
#             # Convert a local position (north, east, down) relative to the home position to a global position (lon, lat, up)
#             # Returns:         numpy array of the global position [longitude, latitude, altitude]
#     
# =============================================================================
        
        
        
        
        current_local_position = global_to_local(global_position,self.global_home)
        print('global home {0}, global position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        
        #global home [-122.3957515   37.7932817    0.       ]
        #global position [-1.22397449e+02  3.77924796e+01 -2.80000000e-02]
        # local position [-0.03823025  0.05000484  0.02808646]

                
        print('current_local_position {0}'.format(current_local_position))
        
        #ccurrent_local_position [-0.04403942  0.05311325  0.028     ]


        
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
        
        



        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
        
        print("Grid Goal plus 10 from mao center ",grid_goal)
        #Grid Goal plus 10 from mao center  (326, 455)
        
        
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        
        
        # Arbitary goal position
        goal_local_position = global_to_local([-122.3957445   ,37.792657    ,0.00   ],self.global_home)
        
        # Converting Lat Long
        grid_goal = (int(goal_local_position[0]-north_offset), int(goal_local_position[1]-east_offset))
        
        print("COLLISION OR NOT :",grid[grid_goal[0]][grid_goal[1]])

 
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        
        
        #path = [grid_start]
        if grid_goal != grid_start:
            path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)
            path_2, path_cost_2 = a_star(grid, heuristic_func, grid_start, grid_goal)
        
        print('Local Start and Goal: ', grid_start, grid_goal)
        #Local Start and Goal:  (315, 445) (336, 595)
        print("PATH COST=",path_cost, ' COST 2', path_cost_2)
        #PATH COST= 205.8355697996828  COST 2 205.8355697996828
        print("LENGTH OF PATH =",len(path), len(path_2))
        #LENGTH OF PATH = 165 165
        print("\n PATH :",path)
        print("\n PATH 2 :",path_2)
        
        # From above got both path and path_2 of same lenght and same waypoints 
        
        

        
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        
 #       path = path_prune(path,collinear_points)
        path = self.path_prune(path)
        print("\n Length of pruned path =",len(path))
        print ("\n revised path: ", path)
        

        path_2 = self.path_prune(path_2)
        print("\n Length of pruned path_2 =",len(path_2))
        print ("\n revised path_2: ", path_2)
        
        
        # Length of pruned path = 11
        #
        # revised path:  [(315, 445), (315, 470), (383, 538), (383, 552), (363, 572), (363, 582), (352, 593), (340, 593), (339, 594), (337, 594), (336, 595)]
        #
        # Length of pruned path_2 = 11
        #
        # revised path_2:  [(315, 445), (315, 470), (383, 538), (383, 552), (363, 572), (363, 582), (352, 593), (340, 593), (339, 594), (337, 594), (336, 595)]

        
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        

        
        
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
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
