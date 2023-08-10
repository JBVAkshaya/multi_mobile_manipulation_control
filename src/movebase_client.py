#!/usr/bin/env python3

import rospy
import actionlib

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point
def move_base_client():
    client = actionlib.SimpleActionClient('/TBwOM_1/movebase', MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_position = Point(2.0,0.8,0.0)  # Target position of the joint in radians
    goal.vel = Twist(linear=Point(-0.03,0.0,0.0),angular=Point(0.0,0.0,0.0))  # Velocity of the joint in radians per second
    goal.duration = 5.0
    print(goal)
    client.send_goal(goal)
    client.wait_for_result()
    
    result = client.get_result()
    print('result:', result)
    if result.success:
        rospy.loginfo("Action completed successfully.")
    else:
        rospy.loginfo("Action failed.")

if __name__ == '__main__':
    rospy.init_node('movebase_client')
    move_base_client()
