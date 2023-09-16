#!/usr/bin/env python
from Robot import *
import rospy
import sys

def main():
    turtle_bot = Robot()
    turtle_bot.robot_control()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("scripts.Runner: RosInterrupException")