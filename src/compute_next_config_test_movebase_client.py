#!/usr/bin/env python3

import rospy
import actionlib
import json
import numpy as np

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point

# Initial goal: I want to try sending three trajectories one right after the other to the base

def move_base_client():
    print("this is the client method!")

    # Step 1 here is to read in the test json file and from there extract the first trajectory (comprised of six subtrajectories)
    # Note that once the file is loaded in, the json is treated as Python
    # with open("test_trajectory.json", "r") as f:
    #     data = json.load(f)

    # # print(data['results']['path'][0])
    # test_traj = data['results']['path'][0]['trajectory'] # Accessing the first trajectory of the path
    # # print(test_traj)

    # lin_vel_list = []
    # ang_vel_list = []
    # duration_list = []

    # Now need to do something like...for dict in trajectory: extract first item
    # for dict in test_traj: # This gets us all of the "robot 0" base and arm trajectories

    #     lin_vel_list.append(dict['0']['base'][0])
    #     ang_vel_list.append(dict['0']['base'][1])
    #     duration_list.append(dict['0']['time_step'])

    # print("list of linear velocities for robot 0: ", lin_vel_list)
    # print("list of angular velocities for robot 0: ", ang_vel_list)
    # print("list of goal durations: ", duration_list)

    # We on

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

# This function takes a current x, y, and yaw, as well as assigned linear and angular velocities,
# and will use kinematics of a differential drive robot to compute what the new x, y, and yaw will be.
# This is useful for computing the target_position of the robot.
def compute_next_config(x, y, yaw, v_lin, v_ang, time_step):

    current_x = x
    current_y = y
    current_yaw = yaw

    next_x = current_x + ((v_lin*np.cos(yaw)) * time_step)
    next_y = current_y + ((v_lin*np.sin(yaw)) * time_step)
    next_yaw = current_yaw + (v_ang * time_step)

    return next_x, next_y, next_yaw

    # print("next x: ", next_x)
    # print("next y: ", next_y)
    # print("next yaw: ", next_yaw)


    # # Step 1 - set up the designed and measured parameters
    # wheel_base = 0.287 # meters
    # wheel_radius = 0.033 # meters
    # lin_vel = 0.1 # m/s - ultimately we will read this in from json
    # ang_vel = 0.0 # rad/s - ultimately we will read this in from json

    # # Step 2 - compute the left and right wheel velocities
    # right_wheel_turn_rate = ((2*lin_vel) + (ang_vel*wheel_base))/(2*wheel_radius) # rad/s
    # left_wheel_turn_rate = ((2*lin_vel) - (ang_vel*wheel_base))/(2*wheel_radius) # rad/s

    # print("v_r: ", right_wheel_turn_rate)
    # print("v_l: ", left_wheel_turn_rate)

if __name__ == '__main__':
    rospy.init_node('movebase_client')
    # move_base_client()
