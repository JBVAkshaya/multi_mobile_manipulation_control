#!/usr/bin/env python3

import rosbag
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal

rospy.init_node('move_arm_server')
goal_pub = rospy.Publisher('/TBwOM_1/arm_controller/follow_joint_trajectory/goal',
                            FollowJointTrajectoryActionGoal,
                            queue_size=10)

bag = rosbag.Bag('/home/akshaya/Downloads/test.bag')
for topic, msg, t in bag.read_messages(topics=['/arm_controller/follow_joint_trajectory/goal']):
    for i in range (0,2000):
        print(msg)
        rospy.sleep(1)
        goal_pub.publish(msg)
    # for i in range (0, len(msg.goal.trajectory.points)):
    #     msg.goal.trajectory.points[i].velocities = []
    #     msg.goal.trajectory.points[i].accelerations = []
    # print(msg.goal.trajectory.points)
bag.close()
rospy.spin()