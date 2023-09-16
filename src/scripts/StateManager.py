import rospy
import numpy as np
from Robot import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from states.Beacon import Beacon
from states.ObstacleAvoidance import ObstacleAvoidance
from states.RandomWalk import RandomWalk
from states.State import *
from states.Frontier import *


RANDOM_WALK_STATE = "Random Walk"
OBSTACLE_AVOIDANCE_STATE = "Obstacle Avoidance"
BEACON_STATE = "Beacon"
FRONTIER_STATE = "Frontier"

# State manager to interface with the different states
# This is the class that should be referenced by the robot
class StateManager:
    def __init__(self, robot_instance):
        self.current_state = None
        self.robot = robot_instance
        self.states = {
            RANDOM_WALK_STATE: RandomWalk(),
            OBSTACLE_AVOIDANCE_STATE: ObstacleAvoidance(),
            BEACON_STATE: Beacon(),
            FRONTIER_STATE: Frontier(),
            }
        
    def set_state(self, state_str):

        self.current_state = self.states[state_str]
        rospy.loginfo("scripts.StateManager: " + state_str)
        self.current_state.execute_state(self.robot)
        
