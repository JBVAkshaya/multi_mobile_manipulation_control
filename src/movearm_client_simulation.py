#!/usr/bin/env python3

'''
    This action client script sends a goal to the action topic /arm_controller/follow_joint_trajectory/goal
    in order to send a goal for the OpenMANIPULATOR-X in Gazebo

'''
import rospy
import actionlib
import rosbag
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal # The message type used by the /arm_controller/follow_joint_trajectory/goal topic

def move_arm_client():

    # Instantiating a SimpleActionClient object and passing the namespace and the ActionSpec
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    # Wait for the server to connect to the client
    client.wait_for_server()
    print("server connected")

    # Import a sample bag that contains one message (a sample FollowJointTrajectoryActionGoal)
    bag = rosbag.Bag('test.bag')

    # Grab the message from the rosbag (note there is only one message in this bag so for loop is unnecessary)
    for topic, msg, t in bag.read_messages(topics=['/arm_controller/follow_joint_trajectory/goal']):
        header_data = msg.header
        goal_id_data = msg.goal_id
        goal_data = msg.goal

    # Insantiating the goal and passing the three args
    new_goal = FollowJointTrajectoryActionGoal(header_data, goal_id_data, goal_data)

    # Send the goal to the server and wait to recieve result (true if goal finished, false if goal not finished in allocated time)
    client.send_goal(new_goal)
    print("goal sent")
    client.wait_for_result()
    
    # Get the result
    result = client.get_result()

if __name__ == '__main__':
    rospy.init_node('movearm_client')
    move_arm_client()
