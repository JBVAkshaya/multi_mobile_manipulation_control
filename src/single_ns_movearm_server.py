#!/usr/bin/env python3

'''
/joint_states: float64[] position
/joint_trajectory_point: Float64[]
'''
import rospy
import actionlib

from multi_mobile_manipulation_control.msg import MoveArmAction, MoveArmResult
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class MoveArmServer:
    def __init__(self, ns, pub_topic_name, sub_topic_name, threshold = 0.05): # Change back the threshold to 0.05 once testing is done
        
        self.rate = rospy.Rate(1)
        self.server = actionlib.SimpleActionServer(ns, MoveArmAction, self.execute, False)
        self.server.start()
        self.joint_trajectory_pub = rospy.Publisher(pub_topic_name, Float64MultiArray, queue_size=10)

        # Might be incorrect syntax
        self.true_arm_state = Float64MultiArray(data =[0.0,0.0,0.0,0.0])
        rospy.Subscriber(sub_topic_name, JointState, self.get_true_arm_state)  
        self.threshold = threshold
        

    def target_reached(self, goal):
        status = False
        print("target: %s curr: %s",np.array(goal.target_joint_trajectory.data[-4:]), np.array(self.true_arm_state))
        print('max diff: ', np.max(np.abs(np.array(goal.target_joint_trajectory.data[-4:]) - np.array(self.true_arm_state))))
        if np.max(np.abs(goal.target_joint_trajectory.data[-4:] - np.array(self.true_arm_state)))<self.threshold:
            status = True
        print("target reached", status)
        return status

    
    def get_true_arm_state(self, data):
        # TODO: Get joint state from jointState
        self.true_arm_state = data.position[2:6]


    def execute(self, goal):
        time_start = rospy.get_rostime()
        rospy.loginfo("Execute time movearm: %i  secs and %i nsecs", time_start.secs, time_start.nsecs)
        self.joint_trajectory_pub.publish(goal.target_joint_trajectory)

        #TODO: Check for sleep not working
        # print('sleep time: ', np.round(goal.target_joint_trajectory.data[-5],2))
        # self.rate.sleep()
        # d = rospy.Duration(1, 260000000)
        # Update the result
        print('i m up')
        result = MoveArmResult()
        result.success = False
        time_now = rospy.get_rostime()
        while time_now.to_sec() - time_start.to_sec() < goal.duration:
            if self.target_reached(goal):
                result.success = True
                result.time_taken = time_now.to_sec() - time_start.to_sec()
                break
            time_now = rospy.get_rostime()
        
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_arm_server')

    # server = MoveArmServer('TBwOM_1/movearm', 'TBwOM_1/joint_trajectory_point', 'TBwOM_1/joint_states')
    # now = rospy.get_rostime()
    # rospy.loginfo("Current time movearm 1: %i  secs and %i nsecs", now.secs, now.nsecs)

    server = MoveArmServer('TBwOM_2/movearm', 'TBwOM_2/joint_trajectory_point', 'TBwOM_2/joint_states')
    now = rospy.get_rostime()
    rospy.loginfo("Current time movearm 2: %i  secs and %i nsecs", now.secs, now.nsecs)
    rospy.spin()
