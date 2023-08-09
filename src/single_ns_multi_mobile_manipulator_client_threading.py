#!/usr/bin/env python3

import rospy
import actionlib
import threading

from multi_mobile_manipulation_control.msg import MoveBaseAction, MoveBaseGoal, MoveArmAction, MoveArmGoal
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64MultiArray

"""Using Threading for concurency test."""

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

# Note: to start, we are going to assume each robot starts with a position of (0,0,0) and send distances based off of that
def move_base_client(action_name, y_goal): # Made a quick modification to allow for custom y-position
    client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.duration = 2.0

    # Ultimately, robot 1 should end up at global position (4, 2) and robot 2 should end up at (4, 2.5)
    rospy.loginfo("test")
    goal.target_position = Point(4.0, 0.0, 0.0)  # Target position of the joint in radians
    goal.vel = Twist(linear=Point(0.1,0.0,0.0), angular=Point(0.0,0.0,0.0))  # Velocity of the joint in radians per second
    now = rospy.get_rostime()
    rospy.loginfo('base goal sent timestamp %s: %i secs %i nsecs', action_name, now.secs, now.nsecs)
    return client, goal

def move_arm_client(action_name):
    client = actionlib.SimpleActionClient(action_name, MoveArmAction)
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

    goal.duration = 1.42
    # goal.duration = 35

    goal.target_joint_trajectory = Float64MultiArray(data=data)  # Target joint trajectory in radians
    now = rospy.get_rostime()
    rospy.loginfo('arm goal sent timestamp %s: %i secs %i nsecs', action_name, now.secs, now.nsecs)
    return client, goal
    

# For the first test, we only care about the base. So, I'm commenting out any arm-related goals.
# For the next test, we'll send the base along the same trajectory as before, and the arm will execute
# a looped trajectory. Just to start, we'll send a test trajectory to just the arms, no base.
if __name__ == '__main__':
    rospy.init_node('move_mobile_manipulator_client')

    # client_base_1, goal_base_1 = move_base_client('/TBwOM_1/movebase', 2.0)
    # client_arm_1, goal_arm_1 = move_arm_client('/TBwOM_1/movearm')
    # client_base_2, goal_base_2 = move_base_client('/TBwOM_2/movebase', 2.0)
    client_arm_2, goal_arm_2 = move_arm_client('/TBwOM_2/movearm')


    # executor_base_1 = ActionClientExecutor(client_base_1, goal_base_1, 'base_1')
    # executor_arm_1 = ActionClientExecutor(client_arm_1, goal_arm_1, 'arm_1')
    # executor_base_2 = ActionClientExecutor(client_base_2, goal_base_2, 'base_2')
    executor_arm_2 = ActionClientExecutor(client_arm_2, goal_arm_2, 'arm_2')

    # executor_base_1.start()
    # executor_arm_1.start()
    # executor_base_2.start()
    executor_arm_2.start()

    # Wait for both action clients to finish
    # executor_base_1.join()
    # executor_arm_1.join()
    # executor_base_2.join()
    executor_arm_2.join()

    # Process the results - action 1

    # result_base_1 = executor_base_1.result
    # result_arm_1 = executor_arm_1.result

    # Process the results - action 2
    # result_base_2 = executor_base_2.result
    result_arm_2 = executor_arm_2.result