#!/usr/bin/env python3

import rospy
import actionlib

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal, MoveArmAction, MoveArmGoal
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray

"""Simple solution for concurency test."""

def move_base_client():
    client = actionlib.SimpleActionClient('/r1/movebase', MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_position = Point(0.0,0.8,0.0)  # Target position of the joint in radians
    goal.vel = Twist(linear=Point(-0.03,0.0,0.0),angular=Point(0.0,0.0,0.0))  # Velocity of the joint in radians per second
    now = rospy.get_rostime()
    rospy.loginfo('base goal sent timestamp: %i secs %i nsecs', now.secs, now.nsecs)
    client.send_goal(goal)
    return client

def move_base_result(client):
    client.wait_for_result()
    result = client.get_result()
    now = rospy.get_rostime()
    rospy.loginfo('base result timestamp: %i secs %i nsecs', now.secs, now.nsecs)
    if result.success:
        rospy.loginfo("Action completed successfully.")
    else:
        rospy.loginfo("Action failed.")

def move_arm_client():
    client = actionlib.SimpleActionClient('/r1/movearm', MoveArmAction)
    client.wait_for_server()
    
    goal = MoveArmGoal()
    data = [0.0, -0.0015339808305725455, -1.2701361179351807, 1.023165225982666, 0.23623304069042206, \
            0.25301703000000003, -0.001580357939315339, -1.2951209516803035, 1.0403951278715782, 0.26654234925480946, \
            0.356466198, -0.0016267350480581324, -1.3201057854254263, 1.0576250297604906, 0.29685165781919687, \
            0.43584899400000005, -0.0016731121568009256, -1.3450906191705494, 1.0748549316494027, 0.3271609663835843,\
            0.503548329, -0.001719489265543719, -1.3700754529156722, 1.0920848335383149, 0.3574702749479717, \
            0.566692722, -0.0017658663742865124, -1.395060286660795, 1.1093147354272272, 0.38777958351235914, \
            0.6298371150000001, -0.0018122434830293058, -1.4200451204059181, 1.1265446373161394, 0.41808889207674654, \
            0.692981508, -0.0018586205917720993, -1.445029954151041, 1.1437745392050516, 0.448398200641134, \
            0.7561259010000001, -0.0019049977005148925, -1.4700147878961638, 1.161004441093964, 0.47870750920552135, \
            0.823154946, -0.0019513748092576861, -1.4949996216412866, 1.178234342982876, 0.5090168177699088, \
            0.9017517730000001, -0.0019977519180004793, -1.5199844553864095, 1.1954642448717883, 0.5393261263342962, \
            1.004641131, -0.002044129026743273, -1.5449692891315325, 1.2126941467607006, 0.5696354348986836, \
            1.257658161, -0.002090506135486066, -1.5699541228766554, 1.2299240486496128, 0.599944743463071]
    goal.target_joint_trajectory = Float64MultiArray(data=data)  # Target joint trajectory in radians
    now = rospy.get_rostime()
    rospy.loginfo('arm goal sent timestamp: %i secs %i nsecs', now.secs, now.nsecs)
    client.send_goal(goal)
    return client

def move_arm_result(client):
    client.wait_for_result()
    
    result = client.get_result()
    now = rospy.get_rostime()
    rospy.loginfo('arm result timestamp: %i secs %i nsecs', now.secs, now.nsecs)
    if result.success:
        rospy.loginfo("Action completed successfully.")
    else:
        rospy.loginfo("Action failed.")

if __name__ == '__main__':
    rospy.init_node('move_mobile_manipulator_client')
    base_client = move_base_client()
    arm_client = move_arm_client()
    move_base_result(base_client)
    move_arm_result(arm_client)