#!/usr/bin/env python3

import rospy
import actionlib

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point

def move_base_client():

    # Instantiating a SimpleActionClient object and passing the namespace and the ActionSpec
    client = actionlib.SimpleActionClient('/r1/movebase', MoveBaseAction)
    # Wait for the server to connect to the client
    client.wait_for_server()
    
    # Creating a goal object 
    goal = MoveBaseGoal()
    goal.target_position = Point(0.4,0.0,0.0)  # Target position of the joint in radians
    goal.vel = Twist(linear = Point(0.03,0.0,0.0),angular=Point(0.0,0.0,0.0))  # Velocity of the joint in radians per second (only care about x, y, and yaw)
    print(goal)

    # Send the goal to the server and wait to recieve result (true if goal finished, false if goal not finished in allocated time)
    client.send_goal(goal)
    client.wait_for_result()

    # Get and print the result
    result = client.get_result()
    print('result:', result)
    if result.success:
        rospy.loginfo("Action completed successfully.")
    else:
        rospy.loginfo("Action failed.")

if __name__ == '__main__':
    rospy.init_node('movebase_client')
    move_base_client()
