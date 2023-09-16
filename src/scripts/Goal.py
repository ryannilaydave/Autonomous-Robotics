# Stores information about the object that has been detected
class Goal:
    def __init__(self, label, centre_pose, movebase_pose):
        self.label = label
        self.centre_pose = centre_pose
        self.movebase_pose = movebase_pose
        self.count = 0
        self.completed = False
    
    # Returns a string representing information about the Goal
    def get_formatted_name(self):
        return str(self.label) + " " \
        + str(self.count) \
        + ", Movebase Position (Goal): " + \
        str((self.movebase_pose[0], self.movebase_pose[1]))
    
    # Checks to see if another goal that has been detected is actually this goal
    def is_equivalent(self, other_goal):
        if(self.label == other_goal.label
           and self.centre_pose == other_goal.centre_pose
           and self.movebase_pose == other_goal.movebase_pose
           and self.count == other_goal.count
           and self.completed == other_goal.completed):
            return True
        
        return False
        