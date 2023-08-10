#!/usr/bin/env python3

import rospy
import actionlib

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseResult
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point

class MoveBaseServer:
    def __init__(self, ns, pub_topic_name, sub_topic_name, threshold = 0.05, pub_freq = 10): # Change back the threshold to 0.05 once testing is done
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
        # if ((goal.target_position.y - self.threshold) < self.amcl_pose.y < (goal.target_position.y + self.threshold)) and \
        #     ((goal.target_position.x - self.threshold) < self.amcl_pose.x < (goal.target_position.x + self.threshold)):
        if ((goal.target_position.x - self.threshold) < self.amcl_pose.x < (goal.target_position.x + self.threshold)):
            status = True
        print(status)
        return status

    
    def get_actual_pose(self, data):
        self.amcl_pose = data.pose.pose.position
        print(self.amcl_pose)


    def execute(self, goal):
        start_time = rospy.get_rostime()
        rospy.loginfo("Execute time movebase: %i  secs and %i nsecs", start_time.secs, start_time.nsecs)
        success = False
        vel_twist = goal.vel
        now_time = rospy.get_rostime()
        while now_time.to_sec()-start_time.to_sec() < goal.duration:
            if not self.is_valid(goal):
                self.vel_pub.publish(vel_twist)
                self.rate.sleep()
            elif self.is_valid(goal):
                success = True
                break
            now_time = rospy.get_rostime()

        vel_twist.linear.x = 0.0
        self.vel_pub.publish(vel_twist)

        # Update the result
        result = MoveBaseResult()
        result.success = success
        result.time_taken = now_time.to_sec()-start_time.to_sec()
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_base_server')
    # server = MoveBaseServer('TBwOM_1/movebase', 'TBwOM_1/cmd_vel', 'TBwOM_1/amcl_pose')
    # now = rospy.get_rostime()
    # rospy.loginfo("Current time movebase 1: %i  secs and %i nsecs", now.secs, now.nsecs)

    server = MoveBaseServer('TBwOM_1/movebase', 'TBwOM_1/cmd_vel', 'TBwOM_1/amcl_pose')
    now = rospy.get_rostime()
    rospy.loginfo("Current time movebase 1: %i  secs and %i nsecs", now.secs, now.nsecs)
    rospy.spin()
