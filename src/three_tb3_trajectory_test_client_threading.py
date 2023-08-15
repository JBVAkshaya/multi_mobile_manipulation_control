#!/usr/bin/env python3

import rospy
import actionlib
import threading
import json
import numpy as np

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal, MoveArmAction, MoveArmGoal
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray

"""Using Threading for concurency test."""

# Action Client Executor class for multithreading
class ActionClientExecutor(threading.Thread):
    def __init__(self, client, goal, ns):
        super(ActionClientExecutor, self).__init__()
        self.ns = ns
        self.client = client
        self.goal = goal
        self.result = None

    def run(self):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        now = rospy.get_rostime()
        rospy.loginfo('%s result timestamp: %i secs %i nsecs', self.ns, now.secs, now.nsecs)
        self.result = self.client.get_result()
        if self.result.success:
            rospy.loginfo("%s: Action completed successfully.", self.ns)
        else:
            rospy.loginfo("%s: Action failed.", self.ns)

# This is the function for loading in the json file
def load_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    return data

# This function creates a list of goals, extracted from the json, to send to the robot
def create_base_goals(robot_id, start_x, start_y, start_yaw):

    base_goal_list = []

    trajectories = load_json("test_trajectory.json")
    # trajectories = load_json("test_trajectory_3.json")


    # These need to change to the hard-coded values
    current_x = start_x # m
    current_y = start_y # m
    current_yaw = start_yaw # rad
    new_config = []
    
    # Iterate over all of the trajectory dictionaries in "path"
    for dict in trajectories['results']['path']:
        conf_space = dict['conf_space'] # Grabbing conf_space data

        index = 0
        # Iterate over the sub-trajectories within each trajectory
        for sub_traj in dict['trajectory']:
            # print("Robot 0's base velocities: ", sub_traj['0']['base'])

            # Grab the velocities and time step for robot 0
            v_lin = sub_traj[robot_id]['base'][0]
            v_ang = sub_traj[robot_id]['base'][1]
            time_step = sub_traj[robot_id]['time_step']

            # Set up a goal and give it its linear and angular velocities, and duration
            new_goal = MoveBaseGoal()
            new_goal.vel = Twist(linear=Point(v_lin, 0.0, 0.0),angular=Point(0.0, 0.0, v_ang))
            new_goal.duration = time_step

            # Next, we can compute the next x, y, yaw
            new_config = compute_next_base_config(current_x, current_y, current_yaw, v_lin, v_ang, time_step)
            new_x = new_config[0]
            new_y = new_config[1]
            new_yaw = new_config[2]
            # Set the target position & add the goal to the list
            new_goal.target_position = Point(new_x, new_y, 0.0) # x, y, z
            base_goal_list.append(new_goal)

            # print("aiming for yaw ", index, ": ", new_yaw)
            # index=index+1

            # Finally, we update our current positions
            current_x = new_x
            current_y = new_y
            current_yaw = new_yaw

        # Finally, after iterating over the six sub-trajectories, we want to update the positions to the conf_space given
        # at the start of their trajectory

        current_x = conf_space[int(robot_id)][0]
        current_y = conf_space[int(robot_id)][1]
        current_yaw = conf_space[int(robot_id)][2]

        # print("current x: ", current_x)

    # Return the goal list
    print(base_goal_list[:6])
    return base_goal_list

# This is the function to compute the next x, y, yaw, from current x, y, yaw, and lin/ang velocities
def compute_next_base_config(x, y, yaw, v_lin, v_ang, time_step):

    current_x = x
    current_y = y
    current_yaw = yaw

    next_x = current_x + ((v_lin*np.cos(yaw)) * time_step) # Horizontal component of linear velocity times goal duration plus current x position
    next_y = current_y + ((v_lin*np.sin(yaw)) * time_step) # Vertical component of linear velocity times goal duration plus current y position
    next_yaw = current_yaw + (v_ang * time_step) # Angular velocity times goal duration plus current yaw angle 

    return next_x, next_y, next_yaw

# Each goal is a list of timestamp followed by joint angles. The goal_list would be a list of lists,
# where each sub-trajectory is its own list. This is a slight re-design from "single_ns_trajectory_test_movearm_client.py"
# because I wanted to send just one 0.5s sub-trajectory to arm + base concurrently
def create_arm_goals(robot_id, start_q1, start_q2, start_q3, start_q4):
    arm_goal_list = []

    trajectories = load_json("test_trajectory.json")

    current_q1 = start_q1 # rad
    current_q2 = start_q2 # rad
    current_q3 = start_q3 # rad
    current_q4 = start_q4 # rad

    # Iterate over all of the trajectory dictionaries in "path"
    for dict in trajectories['results']['path']: # This iterates 18 times, which makes sense...
        conf_space = dict['conf_space'] # Grabbing conf_space data

        # new_goal_data = []
        # time_stamp = 0.0

        # Iterate over the sub-trajectories within each trajectory
        for sub_traj in dict['trajectory']: # This iterates six times, 
            # print("Robot 0's arm velocities: ", sub_traj['0']['arm'])

            # Grab the velocities and time step for robot 1
            # 8/9 The velocities appear to be correct
            vel_q1 = sub_traj[robot_id]['arm'][0]
            vel_q2 = sub_traj[robot_id]['arm'][1]
            vel_q3 = sub_traj[robot_id]['arm'][2]
            vel_q4 = sub_traj[robot_id]['arm'][3]
            time_final = sub_traj[robot_id]['time_step']

            # time_stamp = time_stamp + time_step
            # new_goal_data.append(time_stamp)

            current_joint_states = [current_q1, current_q2, current_q3, current_q4]

            # The next step here would be to grab the next angles of the arms
            next_joint_states = compute_next_arm_config(current_q1, current_q2, current_q3, current_q4, vel_q1, vel_q2, vel_q3, vel_q4, time_final)
            # print("next joint positions: ", next_joint_states)
            new_q1 = next_joint_states[0]
            new_q2 = next_joint_states[1]
            new_q3 = next_joint_states[2]
            new_q4 = next_joint_states[3]

            # This breaks down each 0.5 second sub-trajectory into three sets of joint positions
            # at 0s 0.25s and 0.5s to smoothe the movement
            goal_data = []
            goal_data.append(0.0)
            for i in range(0, 4):
                goal_data.append(current_joint_states[i])
                # print("current: ", current_joint_states)
            goal_data.append(time_final/2)
            for i in range(0, 4):
                mid_point = (current_joint_states[i] + next_joint_states[i])/2
                goal_data.append(mid_point)
                # print("mid: ", mid_point)
            goal_data.append(time_final)
            for i in range(0, 4):
                goal_data.append(next_joint_states[i])
                # print("final: ", next_joint_states)

            single_goal = MoveArmGoal()
            single_goal.duration = time_final
            single_goal.target_joint_trajectory = Float64MultiArray(data=goal_data)

            # Then update the current positions
            current_q1 = new_q1
            current_q2 = new_q2
            current_q3 = new_q3
            current_q4 = new_q4

            arm_goal_list.append(single_goal)

        # Finally, after iterating over the six sub-trajectories, we want to update the joint angles to the conf_space given
        # at the start of their trajectory
        current_q1 = conf_space[int(robot_id)][3]
        current_q2 = conf_space[int(robot_id)][4]
        current_q3 = conf_space[int(robot_id)][5]
        current_q4 = conf_space[int(robot_id)][6]
        # print("conf q1: ", current_q1, " conf q2: ", current_q2, " conf q3: ", current_q3, " conf q4: ", current_q4)

    # print(arm_goal_list[0]) # This is the first trajectory to send to the robot
    # Return the goal list
    return arm_goal_list

# From current joint angles, we'd need to compute the next set of joint angles...
def compute_next_arm_config(c_q1, c_q2, c_q3, c_q4, v_q1, v_q2, v_q3, v_q4, ts):
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

def move_base_client(action_name):
    # Instantiating a SimpleActionClient object and passing the namespace and the ActionSpec
    client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
    # Wait for the server to connect to the client
    client.wait_for_server()
    return client

def move_arm_client(action_name):
    # Instantiating a SimpleActionClient object and passing the namespace and the ActionSpec
    client = actionlib.SimpleActionClient(action_name, MoveArmAction)
    # Wait for the server to connect to the client
    client.wait_for_server()
    return client

# Comment/uncomment to test different robots
def execute():
    base_goals_robot_0 = create_base_goals('0', 0.25, 0.25, 0.0)
    arm_goals_robot_0 = create_arm_goals('0', 0.0, 0.0, 0.0, 0.0)

    base_goals_robot_1 = create_base_goals('1', 0.25, 0.75, 0.0)
    arm_goals_robot_1 = create_arm_goals('1', 0.0, 0.0, 0.0, 0.0)

    base_goals_robot_2 = create_base_goals('2', 0.25, 1.25, 0.0)
    arm_goals_robot_2 = create_arm_goals('2', 0.0, 0.0, 0.0, 0.0)


    client_base_1 = move_base_client('/TBwOM_1/movebase')
    client_arm_1 = move_arm_client('/TBwOM_1/movearm')
    client_base_2 = move_base_client('/TBwOM_2/movebase')
    client_arm_2 = move_arm_client('/TBwOM_2/movearm')
    client_base_3 = move_base_client('/TBwOM_3/movebase')
    client_arm_3 = move_arm_client('/TBwOM_3/movearm')

    for i in range(0, len(base_goals_robot_0[:36])):
        goal_base_1 = base_goals_robot_0[i]
        goal_arm_1 = arm_goals_robot_0[i]

        goal_base_2 = base_goals_robot_1[i]
        goal_arm_2 = arm_goals_robot_1[i]

        # print(base_goals_robot_1[i])

        goal_base_3 = base_goals_robot_2[i]
        goal_arm_3 = arm_goals_robot_2[i]

        # print("goal robot 1: ", goal_base_1 )
        # print("goal robot 2: ", goal_base_2 )
        # print("goal robot 3: ", goal_base_3 )

        executor_base_1 = ActionClientExecutor(client_base_1, goal_base_1, 'base_1')
        executor_arm_1 = ActionClientExecutor(client_arm_1, goal_arm_1, 'arm_1')
        executor_base_2 = ActionClientExecutor(client_base_2, goal_base_2, 'base_2')
        executor_arm_2 = ActionClientExecutor(client_arm_2, goal_arm_2, 'arm_2')
        executor_base_3 = ActionClientExecutor(client_base_3, goal_base_3, 'base_3')
        executor_arm_3 = ActionClientExecutor(client_arm_3, goal_arm_3, 'arm_3')

        executor_base_1.start()
        executor_arm_1.start()
        executor_base_2.start()
        executor_arm_2.start()
        executor_base_3.start()
        executor_arm_3.start()

        # Wait for both action clients to finish
        executor_base_1.join()
        executor_arm_1.join()
        executor_base_2.join()
        executor_arm_2.join()
        executor_base_3.join()
        executor_arm_3.join()

        # Process the results - action 1
        result_base_1 = executor_base_1.result
        result_arm_1 = executor_arm_1.result

        # Process the results - action 2
        result_base_2 = executor_base_2.result
        result_arm_2 = executor_arm_2.result

        # # Process the results - action 3
        result_base_3 = executor_base_3.result
        result_arm_3 = executor_arm_3.result

if __name__ == '__main__':
    rospy.init_node('move_mobile_manipulator_client')
    # arm_initialization_goal('/TBwOM_1/movearm')
    # arm_initialization_goal('/TBwOM_2/movearm')
    # arm_initialization_goal('/TBwOM_3/movearm')
    execute() # Uncomment to run the goals




