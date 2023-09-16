from State import *
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
import rospy
import OccupancyGridMethods as og

# Beacon state used to beacon towards an object that is detected by the 
# turtlebot
class Beacon(State):
    def __init__(self):
        State.__init__(self, "Beacon", 2)
        
    # Method to generate MoveBaseGoal objects
    def goal_pose(self, pose):  
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0]
        goal_pose.target_pose.pose.position.y = pose[1]
        goal_pose.target_pose.pose.position.z = 0.0
        goal_pose.target_pose.pose.orientation.x = 0.0
        goal_pose.target_pose.pose.orientation.y = 0.0
        goal_pose.target_pose.pose.orientation.z = 0.0
        goal_pose.target_pose.pose.orientation.w = 1.0

        return goal_pose
    
    # If the robot sees green it will beacon toward that green object
    def execute_state(self, robot):
        while robot.current_pose is None:
            continue

        robot.goal_manager.change_to_closest_goal(robot.current_pose)
        goal = robot.goal_manager.active_goal

        while goal is not None:
            goal_pose = self.goal_pose(goal.movebase_pose)

            while True:
                robot.move_base.send_goal(goal_pose)
            
                robot.move_base.wait_for_result(rospy.Duration(15))
                # If has reached the goal, then we complete the goal!!!
                if(robot.move_base.get_state() == 3):
                    grid_point = og.to_grid(
                        goal.centre_pose[0], 
                        goal.centre_pose[1], 
                        robot.occupancy_grid.info.origin, 
                        robot.map.grid_size, 
                        robot.occupancy_grid.info.resolution)
                        
                    robot.invalid_frontiers.append(grid_point)
                    robot.goal_manager.complete_active_goal()
                    return
                # If goal is in an unreachable state, then remove goal from 
                # active/complete list
                elif(robot.move_base.get_state() == 4):
                    robot.goal_manager.remove_goal(goal.label, goal.count)
                    return
