import sys
import os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from State import State
from geometry_msgs.msg import Twist
import random
import rospy

# Random walk state to randomly walk if no frontiers can be found, meaning the 
# map is fully explored
class RandomWalk(State):
    def __init__(self):
        State.__init__(self, "Random Walk", 3)
        
    # Makes the robot move forward a set distance, and then rotate a random 
    # amount
    def execute_state(self, robot):
        r = rospy.Rate(10)
        msg = Twist()
        current_distance = 0
        msg.linear.x = robot.x_vel

        while not robot.current_pose:
            continue

        robot.original_pose = robot.current_pose

        # Moves the turtle bot a required distance
        while(current_distance < robot.distance):
            if robot.is_blocked or robot.sees_green:
                msg.linear.x = 0
                robot.twist_pub.publish(msg)

                return
            robot.twist_pub.publish(msg)

            current_distance = \
                robot.get_distance(robot.original_pose, robot.current_pose)


        msg.linear.x = 0
        robot.twist_pub.publish(msg)

        left = random.getrandbits(1)
        ang_vel = robot.base_ang_vel if left else -(robot.base_ang_vel)

        angle_time = random.uniform(0,robot.limit)
        msg.angular.z = ang_vel
        time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - time < \
            rospy.Duration(angle_time).to_sec():

            if robot.sees_green:
                msg.linear.z = 0
                robot.twist_pub.publish(msg)
                return
                
            robot.twist_pub.publish(msg)

        msg.angular.z = 0
        robot.twist_pub.publish(msg)
