#!/usr/bin/env python3

import json
import rospy
import actionlib

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal, MoveArmAction, MoveArmGoal
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray

# Each goal is a list of timestamp followed by joint angles. The goal_list would be a list of lists,
# where each trajectory (set of six sub-trajectories) is its own list
def create_serialized_goal_list():
    goal_list = []

    trajectories = load_json("test_trajectory.json")

    current_q1 = 0.0 # rad
    current_q2 = 0.0 # rad
    current_q3 = 0.0 # rad
    current_q4 = 0.0 # rad

    # Iterate over all of the trajectory dictionaries in "path"
    for dict in trajectories['results']['path']: # This iterates 18 times, which makes sense...
        conf_space = dict['conf_space'] # Grabbing conf_space data

        new_goal_data = []
        time_stamp = 0.0

        # Iterate over the sub-trajectories within each trajectory
        for sub_traj in dict['trajectory']: # This iterates six times, 
            # print("Robot 0's arm velocities: ", sub_traj['0']['arm'])

            # Grab the velocities and time step for robot 1
            # 8/9 The velocities appear to be correct
            vel_q1 = sub_traj['0']['arm'][0]
            vel_q2 = sub_traj['0']['arm'][1]
            vel_q3 = sub_traj['0']['arm'][2]
            vel_q4 = sub_traj['0']['arm'][3]
            time_step = sub_traj['0']['time_step']
            time_stamp = time_stamp + time_step
            new_goal_data.append(time_stamp)

            # The next step here would be to grab the next angles of the arms
            next_joint_states = compute_next_joint_state(current_q1, current_q2, current_q3, current_q4, vel_q1, vel_q2, vel_q3, vel_q4, time_step)
            # print("next joint positions: ", next_joint_states)
            new_q1 = next_joint_states[0]
            new_q2 = next_joint_states[1]
            new_q3 = next_joint_states[2]
            new_q4 = next_joint_states[3]

            # Next, need to append the arm positions to the goal data
            new_goal_data.append(new_q1)
            new_goal_data.append(new_q2)
            new_goal_data.append(new_q3)
            new_goal_data.append(new_q4)

            # Then update the current positions
            current_q1 = new_q1
            current_q2 = new_q2
            current_q3 = new_q3
            current_q4 = new_q4

        goal_list.append(new_goal_data)

        # Finally, after iterating over the six sub-trajectories, we want to update the joint angles to the conf_space given
        # at the start of their trajectory
        current_q1 = conf_space[0][3]
        current_q2 = conf_space[0][4]
        current_q3 = conf_space[0][5]
        current_q4 = conf_space[0][6]
        # print("conf q1: ", current_q1, " conf q2: ", current_q2, " conf q3: ", current_q3, " conf q4: ", current_q4)


    # print(goal_list[0]) # This is the first trajectory to send to the robot
    # Return the goal list
    return goal_list

# This method is simply for reading in the json file
def load_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    return data

# From current joint angles, we'd need to compute the next set of joint angles...
def compute_next_joint_state(c_q1, c_q2, c_q3, c_q4, v_q1, v_q2, v_q3, v_q4, ts):
    n_q1 = c_q1 +(v_q1*ts)
    n_q2 = c_q2 +(v_q2*ts)
    n_q3 = c_q3 +(v_q3*ts)
    n_q4 = c_q4 +(v_q4*ts)

    # Check that computations are correct [they are as of 8/09]
    # print("current q2: ", c_q2, " velocity q2: ", v_q2, " time step: ", ts )
    # print("\nnew q2 = current + velocity * time: ", n_q2, "\n")

    return n_q1, n_q2, n_q3, n_q4

# The arm initialization function simply sends the robot to its "home" position of {q1=q2=q3=q4=0.0 radians}
def arm_initialization_goal(action_name):
    client = actionlib.SimpleActionClient(action_name, MoveArmAction)
    client.wait_for_server()

    init_goal = MoveArmGoal()
    init_goal.duration = 3.5

    # This homing trajectory was generated simply by deciding the total time and time step, and then breaking
    # down the trajectory into tiny bite-sized steps
    # Serialized data being sent to the arm - [time_stamp q1 q2 q3 q4]
    # This is its hard-coded "home" position

    data = [0.0, 0.002, -1.575, 1.218, 0.603, \
            0.2, 0.00186666666667, -1.47, 1.1368, 0.5628,\
            0.4, 0.00173333333333, -1.365, 1.0556, 0.5226,\
            0.6, 0.0016, -1.26, 0.9744, 0.4824,\
            0.8, 0.00146666666667, -1.155, 0.8932, 0.4422,\
            1.0, 0.00133333333333, -1.05, 0.812, 0.402,\
            1.2, 0.0012, -0.945, 0.7308, 0.3618,\
            1.4, 0.00106666666667, -0.84, 0.6496, 0.3216,\
            1.6, 0.000933333333336, -0.735, 0.5684, 0.2814,\
            1.8, 0.000800000000003, -0.63, 0.4872, 0.2412,\
            2.0, 0.00066666666667, -0.525, 0.406, 0.201,\
            2.2, 0.000533333333337, -0.42, 0.3248, 0.1608,\
            2.4, 0.000400000000004, -0.315, 0.2436, 0.1206,\
            2.6, 0.000266666666671, -0.21, 0.1624, 0.0804,\
            2.8, 0.000133333333338, -0.105, 0.0812, 0.0402,\
            3.0, 0.000, 0.000, 0.000, 0.000]
    

    init_goal.target_joint_trajectory = Float64MultiArray(data=data)

    now = rospy.get_rostime()
    rospy.loginfo('arm goal sent timestamp %s: %i secs %i nsecs', action_name, now.secs, now.nsecs) 
    
    print(init_goal)
    client.send_goal(init_goal)
    client.wait_for_result()
    result = client.get_result()
    print('result:', result)

def move_arm_client(action_name):

    client = actionlib.SimpleActionClient(action_name, MoveArmAction)
    client.wait_for_server()

    trajectory_list = create_serialized_goal_list()
    # test_trajectory_data = trajectory_list[0]

    for traj in trajectory_list[:25]:
        goal = MoveArmGoal()
        goal.duration = 3.0
        goal.target_joint_trajectory = Float64MultiArray(data=traj)

        print(goal)
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        print('result:', result)



if __name__ == '__main__':
    rospy.init_node('move_mobile_manipulator_client')
    arm_initialization_goal('/TBwOM_2/movearm')
    # move_arm_client('/TBwOM_2/movearm')


