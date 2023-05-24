#!/usr/bin/env python3

import rospy
import actionlib

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseResult
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point

class MoveBaseServer:
    def __init__(self, ns, pub_topic_name, sub_topic_name, threshold = 0.2, pub_freq = 10): # Change back the threshold to 0.05 once testing is done
        """Velocity control for robot base with amcl pose feedback

        Args:
            ns (_type_): _description_
            pub_topic_name (_type_): _description_
            sub_topic_name (_type_): _description_
            threshold (float, optional): _description_. Defaults to 0.05.
            pub_freq (int, optional): _description_. Defaults to 10.
        """
        self.rate = rospy.Rate(pub_freq)
        self.server = actionlib.SimpleActionServer(ns, MoveBaseAction, self.execute, False)
        self.server.start()
        self.vel_pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
        self.amcl_pose = Point(0.0,0.0,0.0) # Initialize the pose
        rospy.Subscriber(sub_topic_name, PoseWithCovarianceStamped, self.get_actual_pose)  
        self.threshold = threshold

    def is_valid(self, goal):
        status = False
        if ((goal.target_position.y - self.threshold) < self.amcl_pose.y < (goal.target_position.y + self.threshold)) and \
            ((goal.target_position.x - self.threshold) < self.amcl_pose.x < (goal.target_position.x + self.threshold)):
            status = True
        print(status)
        return status

    
    def get_actual_pose(self, data):
        self.amcl_pose = data.pose.pose.position
        print(self.amcl_pose)


    def execute(self, goal):
        success = False
        vel_twist = goal.vel

        while not self.is_valid(goal):
            self.vel_pub.publish(vel_twist)
            self.rate.sleep()
        vel_twist.linear.x = 0.0
        self.vel_pub.publish(vel_twist)

        # Update the result
        result = MoveBaseResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_base_server')
    server = MoveBaseServer('r1/movebase', '/cmd_vel', '/amcl_pose')
    print('success')
    rospy.spin()
