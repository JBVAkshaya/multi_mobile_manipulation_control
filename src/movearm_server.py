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

class MoveBaseServer:
    def __init__(self, ns, pub_topic_name, sub_topic_name, threshold = 0.2): # Change back the threshold to 0.05 once testing is done
        
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
        print(np.max(np.abs(goal.target_joint_trajectory.data[-4:] - np.array(self.true_arm_state.data))))
        if np.max(np.abs(goal.target_joint_trajectory.data[-4:] - np.array(self.true_arm_state.data)))<self.threshold:
            status = True
        print(status)
        return status

    
    def get_true_arm_state(self, data):
        # TODO: Get joint state from jointState
        self.true_arm_state = data.position
        print(self.true_arm_state)


    def execute(self, goal):
        self.joint_trajectory_pub.publish(goal.target_joint_trajectory)

        #TODO: Check for sleep not working
        # print('sleep time: ', np.round(goal.target_joint_trajectory.data[-5],2))
        # self.rate.sleep()
        # Update the result
        result = MoveArmResult()
        if self.target_reached(goal):
            result.success = True
        else:
            result.success = False
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_arm_server')
    server = MoveBaseServer('r1/movearm', '/joint_trajectory_point', '/joint_state')
    print('success')
    rospy.spin()
