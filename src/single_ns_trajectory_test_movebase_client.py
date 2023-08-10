#!/usr/bin/env python3

import rospy
import numpy as np
import actionlib
import json

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point

# This function creates a list of goals, extracted from the json, to send to the robot
def create_goal_list():

    goal_list = []

    trajectories = load_json("test_trajectory.json")

    # These need to change to the hard-coded values
    current_x = 0.0 # m
    current_y = 0.0 # m
    current_yaw = 0.0 # rad
    new_config = []
    
    # Iterate over all of the trajectory dictionaries in "path"
    for dict in trajectories['results']['path']:
        conf_space = dict['conf_space'] # Grabbing conf_space data

        # Iterate over the sub-trajectories within each trajectory
        for sub_traj in dict['trajectory']:
            # print("Robot 0's base velocities: ", sub_traj['0']['base'])

            # Grab the velocities and time step for robot 1
            v_lin = sub_traj['0']['base'][0]
            v_ang = sub_traj['0']['base'][1]
            time_step = sub_traj['0']['time_step']

            # Set up a goal and give it its linear and angular velocities, and duration
            new_goal = MoveBaseGoal()
            new_goal.vel = Twist(linear=Point(v_lin, 0.0, 0.0),angular=Point(0.0, 0.0, v_ang))
            new_goal.duration = time_step

            # Next, we can compute the next x, y, yaw
            new_config = compute_next_config(current_x, current_y, current_yaw, v_lin, v_ang, time_step)
            new_x = new_config[0]
            new_y = new_config[1]
            new_yaw = new_config[2]
            # Set the target position & add the goal to the list
            new_goal.target_position = Point(new_x, new_y, 0.0) # x, y, z
            goal_list.append(new_goal)

            # Finally, we update our current positions
            current_x = new_x
            current_y = new_y
            current_yaw = new_yaw

        # Finally, after iterating over the six sub-trajectories, we want to update the positions to the conf_space given
        # at the start of their trajectory
        current_x = conf_space[0][0]
        current_y = conf_space[0][1]
        current_yaw = conf_space[0][2]

    # Return the goal list
    print(goal_list[:6])
    return goal_list
    
# This is the function for loading in the json file
def load_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    return data
    
# This is the function to compute the next x, y, yaw, from current x, y, yaw, and lin/ang velocities
def compute_next_config(x, y, yaw, v_lin, v_ang, time_step):

    current_x = x
    current_y = y
    current_yaw = yaw

    next_x = current_x + ((v_lin*np.cos(yaw)) * time_step) # Horizontal component of linear velocity times goal duration plus current x position
    next_y = current_y + ((v_lin*np.sin(yaw)) * time_step) # Vertical component of linear velocity times goal duration plus current y position
    next_yaw = current_yaw + (v_ang * time_step) # Angular velocity times goal duration plus current yaw angle 

    return next_x, next_y, next_yaw

def move_base_client():
    # Instantiating a SimpleActionClient object and passing the namespace and the ActionSpec
    client = actionlib.SimpleActionClient('/TBwOM_2/movebase', MoveBaseAction)
    # Wait for the server to connect to the client
    client.wait_for_server()

    move_base_goals = create_goal_list()

    # Consecutively sending each goal in the trajectory list
    for goal in move_base_goals:
        print(goal)
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        print('result:', result)



if __name__ == '__main__':
    rospy.init_node('movebase_client')
    move_base_client()
    # create_goal_list()
