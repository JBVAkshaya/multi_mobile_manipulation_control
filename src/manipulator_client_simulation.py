#!/usr/bin/env python3

import sys
import rosbag

from copy import copy
import rospy

import actionlib
# from multi_mobile_manipulation_control.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


class Trajectory(object):
    def __init__(self):
        ns = 'TBwOM_2/arm_controller/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()


import rosbag
def main():
    """Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.
    """

    print("Initializing node... ")
    rospy.init_node("tbwom1_joint_trajectory_client")
    print("Getting robot state... ")
    
    traj = Trajectory()
    print(traj._goal)
    rospy.on_shutdown(traj.stop)
    bag = rosbag.Bag('/home/akshaya/Downloads/test.bag')
    goal = FollowJointTrajectoryActionGoal()
    for topic, msg, t in bag.read_messages(topics=['/arm_controller/follow_joint_trajectory/goal']):
        for i in range (0, len(msg.goal.trajectory.points)):
            msg.goal.trajectory.points[i].velocities = []
            msg.goal.trajectory.points[i].accelerations = []
        goal = msg
    bag.close()    
    traj._goal.trajectory = goal.goal.trajectory
    print(traj._goal)

    traj.start()
    traj.wait(15.0)
    # print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()