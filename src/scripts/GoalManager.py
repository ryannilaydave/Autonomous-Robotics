import sys

from numpy.lib.arraysetops import isin
import rospy
import math
from Goal import *

from Pose import Pose

GREEN_BOX_LABEL = "green box"
BLUE_MAILBOX_LABEL = "mail box"
FIRE_HYDRANT_LABEL = "fire hydrant"

# GoalManager manages all the goals to return the closest goal and complete 
# goals correctly
class GoalManager:
    def __init__(self):
        self.labels = [GREEN_BOX_LABEL, BLUE_MAILBOX_LABEL, FIRE_HYDRANT_LABEL]
        self.active_goal = None
        self.active_goal_list = []
        self.completed_goal_list = []  
        self.error_thresh = 0.5 #0.2  
  
    
    #Adds a defined goal to the active goal list
    def add_goal(self, goal):
        if(goal != None):
            
            #Increment label count by 1
            goal.count =  self.get_label_count(goal.label) + 1

            #Ensure is Goal instance class sent in
            self.active_goal_list.append(goal)
    
    #Marks a goal as completed
    def complete_goal(self, goal):
        if(goal != None):
            goal.completed = True
            
            #Remove from active goal list
            self.active_goal_list.remove(goal)
            
            #Add to completed goal list
            self.completed_goal_list.append(goal)
            
            rospy.loginfo("states.GoalManager: Completed " + goal.label + " " +\
                 str(goal.count))
        else:
            rospy.loginfo(
                "states.GoalManager: Could not complete goal as was None")

    #Completes the currently active goal and sets it to None to enable reset
    def complete_active_goal(self):
        self.complete_goal(self.active_goal)
        self.active_goal = None
    
    # Removes goal from the list of goals
    def remove_goal(self, label, goal_id):
        (found, list_to_remove_from) = \
            self.find_goal_by_label_and_id(label, goal_id)

        if(found != None and list_to_remove_from != None):
            if(list_to_remove_from == self.active_goal_list):
                self.active_goal_list.remove(found)
                rospy.loginfo("scripts.GoalManager: Removed " + found.label + \
                    " " + str(found.count) + " from active goal list")
            elif(list_to_remove_from == self.completed_goal_list):
                self.completed_goal_list.remove(found)
                rospy.loginfo("scripts.GoalManager: Removed " + found.label + \
                    " " + str(found.count) + " from completed goal list")     
        
    # Finds and return the closest goal relative to the position of the robot 
    # (euclidean)
    def find_closest_goal(self, pose):
        if(len(self.active_goal_list) > 0):
            # Assume first goal is closed for initialisation
            closest_goal = None
            closest_dist = sys.float_info.max

            # Iterate through all active goals to find the closest one
            for goal in self.active_goal_list:
                dist = self.calculate_distance(
                    pose.x, 
                    pose.y, 
                    goal.movebase_pose[0],
                    goal.movebase_pose[1])

                if(dist < closest_dist):
                    closest_goal = goal
                    closest_dist = dist

            return closest_goal
        
        return None
    
    # Changes the active goal to the goal which is closest (euclidean) the robot
    def change_to_closest_goal(self, pose):
        closest = self.find_closest_goal(pose)

        # Swap if not already active   
        if(closest and closest != self.active_goal):
            self.active_goal = closest
            rospy.loginfo("scripts.GoalManager: Changed to closest goal - " + \
                self.active_goal.get_formatted_name())
    
    def check_existing_goals(self, object_radius, new_centre_pose): 
        # Check if goal is in the active and completed list
        for goal in (self.active_goal_list + self.completed_goal_list):
            # Get difference between all previous goal poses
            x_diff = abs(goal.centre_pose[0] - new_centre_pose[0])
            y_diff = abs(goal.centre_pose[1] - new_centre_pose[1])

            # Allow for error within object with obj radius
            if x_diff <= (object_radius+self.error_thresh) and \
                y_diff <= (object_radius+self.error_thresh): 
                # Object is already a goal
                return True 
        # Object has not been used yet
        return False
    
    # Calculates distance between (x1,y1) and (x2,y2)
    def calculate_distance(self, x1, y1, x2, y2):
        x_val = math.pow(x2-x1, 2)
        y_val = math.pow(y2-y1, 2)
        return math.sqrt(x_val + y_val)
    
    # Finds a goal by label and id and returns a tuple, containing the goal and 
    # the list it is in
    def find_goal_by_label_and_id(self, label, goal_id):
        # Check in the active goal
        for g in self.active_goal_list:
            if(g.label == label and g.count == goal_id):
                return (g, self.active_goal_list)

        # Check in the completed list
        for g in self.completed_goal_list:
            if(g.label == label and g.count == goal_id):
                return (g, self.completed_goal_list)  
              
        return None
    
    def find_active_by_label(self, label):
        label_id = self.get_label_id(label)

        return \
            [g for g in self.active_goal_list \
                if g.label == self.labels[label_id]]
    
    def find_completed_by_label(self, label):
        label_id = self.get_label_id(label)
        return \
            [g for g in self.completed_goal_list \
                if g.label == self.labels[label_id]]
    
    # Returns amount of goals with provided label
    def get_label_count(self, label):
        id = self.get_label_id(label)
        
        number_in_active = \
            len([g for g in self.active_goal_list \
                if g.label == self.labels[id]])

        number_in_complete = \
            len([g for g in self.completed_goal_list \
                if g.label == self.labels[id]])

        return number_in_active + number_in_complete
    
    # Returns the id of the id (index of labels) associated with the provided 
    # goal
    def get_label_id(self, label):
        
        for i in range(0, len(self.labels)):
            # If found label in labels list is same as provided then found 
            # correct id
            if(self.labels[i] == label):
                return i
            
        return None

    def find_goals_by_label(self, label):
        total = 0

        for goal in self.completed_goal_list:
            if goal.label == label:
                total += 1

        return total
            
    def get_active_green_box_goals(self):
        return self.find_goals_by_label(GREEN_BOX_LABEL)
    
    def get_active_blue_mailbox_goals(self):
        return self.find_goals_by_label(BLUE_MAILBOX_LABEL)
    
    def get_active_fire_hydrant_goals(self):
        return self.find_goals_by_label(FIRE_HYDRANT_LABEL)

    def get_completed_green_box_goals(self):
        return self.find_completed_by_label(GREEN_BOX_LABEL)
    
    def get_completed_blue_mailbox_goals(self):
        return self.find_completed_by_label(BLUE_MAILBOX_LABEL)
    
    def get_completed_fire_hydrant_goals(self):
        return self.find_completed_by_label(FIRE_HYDRANT_LABEL)
    
    def print_all_label_count(self):
        total_str = "\n"
        for label in self.labels:
            count = self.find_goals_by_label(label)
            total_str += (label + " total = " + str(count) + "\n")
            
        rospy.loginfo("scripts.GoalManager: " + total_str)
