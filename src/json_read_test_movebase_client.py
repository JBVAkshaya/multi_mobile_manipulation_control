#!/usr/bin/env python3

import rospy
import actionlib
import json

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point

# Initial goal: I want to try sending three trajectories one right after the other to the base

def move_base_client():

    # Step 1 here is to read in the test json file and from there extract the first trajectory (comprised of six subtrajectories)
    # Note that once the file is loaded in, the json is treated as Python
    with open("test_trajectory.json", "r") as f:
        data = json.load(f)

    # print(data['results']['path'][0])
    test_traj = data['results']['path'][0]['trajectory'] # Accessing the first trajectory of the path
    # print(test_traj)

    lin_vel_list = []
    ang_vel_list = []
    duration_list = []

    # Now need to do something like...for dict in trajectory: extract first item
    for dict in test_traj: # This gets us all of the "robot 0" base and arm trajectories

        lin_vel_list.append(dict['0']['base'][0])
        ang_vel_list.append(dict['0']['base'][1])
        duration_list.append(dict['0']['time_step'])

    print("list of linear velocities for robot 0: ", lin_vel_list)
    print("list of angular velocities for robot 0: ", ang_vel_list)
    print("list of goal durations: ", duration_list)


    # LOOPING OVER EACH OF PATH OBJECTS WOULD BE SOMETHING LIKE "for path in data['results']['path'][0]"

    # testGoals = []
    # current_x = 1.0
    # current_y = 2.0

    # # Loop over the trajectories, and for each loop, add the attributes found in each dictionary item
    # for dict in testPath:

    #     new_goal = MoveBaseGoal()
    #     new_goal.vel = Twist(linear=Point(dict["vel_lin"],0.0,0.0),angular=Point(0.0,0.0,dict["vel_ang"]))
    #     new_goal.duration = 0.5
    #     new_goal.target_position = Point(current_x + (dict["vel_lin"]*new_goal.duration), current_y, 0.0) # x, y, z - next step is figure out how this works
    #     testGoals.append(new_goal)

    # print(testGoals)

    # # Instantiating a SimpleActionClient object and passing the namespace and the ActionSpec
    # client = actionlib.SimpleActionClient('/TBwOM_2/movebase', MoveBaseAction)
    # # Wait for the server to connect to the client
    # client.wait_for_server()
    
    # ### OLD ###
    # # Creating a goal object 
    # # goal = MoveBaseGoal()
    # # goal.target_position = Point(2.0,0.8,0.0)  # Target position of the joint in radians
    # # goal.vel = Twist(linear=Point(-0.03,0.0,0.0),angular=Point(0.0,0.0,0.0))  # Velocity of the joint in radians per second
    # # goal.duration = 5.0
    # # print(goal)


    # for goal in testGoals:
    #     client.send_goal(goal)
    #     client.wait_for_result()
    #     result = client.get_result()
    #     print('result:', result)

    # Send the goal to the server and wait to recieve result (true if goal finished, false if goal not finished in allocated time)
    # client.send_goal(goal)
    # client.wait_for_result()
    
    # Get and print the result
    # result = client.get_result()
    # print('result:', result)

    # Next steps if goal is success or not
    # if result.success:
    #     rospy.loginfo("Action completed successfully.")
    # else:
    #     rospy.loginfo("Action failed.")

# def find_next_position():


if __name__ == '__main__':
    rospy.init_node('movebase_client')
    move_base_client()
