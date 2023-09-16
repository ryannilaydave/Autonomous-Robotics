import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from State import State
from geometry_msgs.msg import Twist

import rospy

# Obstacle avoidance state
class ObstacleAvoidance(State):
    def __init__(self):
        State.__init__(self, "Obstacle Avoidance", 1)
        
    # Makes the robot rotate until its path is no longer blocked
    def execute_state(self, robot):
        msg = Twist()
        msg.angular.z = robot.base_ang_vel \
            if not robot.is_blocked_left else -(robot.base_ang_vel)
        while robot.is_blocked:
            robot.twist_pub.publish(msg)  

        msg.angular.z = 0
        robot.twist_pub.publish(msg)
